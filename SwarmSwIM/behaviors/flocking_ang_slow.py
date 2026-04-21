"""flocking_ang_slow.py — Velocity-aware flocking with a solidarity mechanism.

Architecture
============
Built on the Couzin ZOR/ZOO/ZOA model from flocking_nav.py, extended with
two velocity-solidarity rules:

  1. Inverse-speed weighting
     Every neighbour's contribution to the orientation (ZOO) and attraction
     (ZOA) vectors is scaled by  w = 1 / (spd + SPD_EPS
     A slower neighbour therefore steers the flock more than a fast one —
     the swarm naturally bends toward the direction the slowest members can
     sustain.

  2. Solidarity spotlight
     Any neighbour whose speed falls below V_SLOW is flagged as a "slow"
     agent.  An additional pull vector (W_SOLIDARITY) is added directly
     toward each such neighbour, independent of zone.  This prevents fast
     agents from out-running laggards entirely.

Speed is already part of the communicated pose dict ("spd" key in posebox),
so no additional communication protocol is required.

A CollisionAvoidance layer is retained on top of flocking output, same as
flocking_nav.py.

Metrics column "spd_spread" is added: the std-dev of agent speeds, giving a
measure of how dispersed the velocity distribution is at each sample step.
"""

import os
from pathlib import Path
import numpy as np
import csv
from datetime import datetime

from SwarmSwIM import Simulator
from ..sail_extension.wind_plotter import WindPlotter
from ..sensors.comm import NeighborhoodComm
from ..sensors.collision_avoidance import CollisionAvoidance

# ── Tunable parameters ──────────────────────────────────────────────────────

SHOW_STATUS = os.environ.get("SWARM_SHOW_STATUS", "1").strip().lower() in ("1", "true", "yes", "on")

SEED            = int  (os.environ.get("SWARMSWIM_SEED",     "142"  ))
NEIGHBOR_RADIUS = float(os.environ.get("SWARM_COMM_RADIUS",   "20.0"))
DOMAIN_SIZE     = float(os.environ.get("SWARMSWIM_DOMAIN",    "30.0"))

# Collision avoidance
SAFE_RADIUS    = float(os.environ.get("SWARM_CA_SAFE_RADIUS", "4.0"))
CPA_HORIZON    = float(os.environ.get("SWARM_CA_HORIZON",     "12.0"))
AVOID_BIAS_MAX = float(os.environ.get("SWARM_CA_BIAS_MAX",    "90.0"))
CA_TIE_MODE    = os.environ.get("SWARM_CA_TIE_MODE", "colregs").lower()
CA_DWELL       = float(os.environ.get("SWARM_CA_DWELL", "3.0"))

# Couzin zones
ZOR = float(os.environ.get("SWARM_ZOR", "4.0"))    # repulsion radius
ZOO = float(os.environ.get("SWARM_ZOO", "10.0"))   # orientation radius
ZOA = float(os.environ.get("SWARM_ZOA", "18.0"))   # attraction radius

# Standard flocking weights
W_ORI = float(os.environ.get("SWARM_W_ORI", "1.0"))
W_ATT = float(os.environ.get("SWARM_W_ATT", "0.6"))

NOISE_STD_DEG = float(os.environ.get("SWARM_FLOCK_NOISE", "3.0"))

# ── Velocity-solidarity parameters ─────────────────────────────────────────
# Speed threshold below which an agent is considered "slow"
V_SLOW        = float(os.environ.get("SWARM_V_SLOW",        "4.3"))   # m/s
# Extra attraction weight toward each slow neighbour
W_SOLIDARITY  = float(os.environ.get("SWARM_W_SOLIDARITY",  "2.0"))
# Small constant added to speed before inversion (prevents division by zero)
SPD_EPS       = float(os.environ.get("SWARM_SPD_EPS",       "0.05"))
# Set to "0" to disable inverse-speed weighting (falls back to equal weights)
USE_SPD_WT    = os.environ.get("SWARM_USE_SPD_WT", "1").strip().lower() in ("1", "true", "yes", "on")

# Boundary handling
BOUNDARY_GAIN   = float(os.environ.get("SWARM_BOUND_GAIN",   "1.2"))
BOUNDARY_MARGIN = float(os.environ.get("SWARM_BOUND_MARGIN", "5.0"))
TOROIDAL        = os.environ.get("SWARM_TOROIDAL", "0").strip().lower() in ("1", "true", "yes", "on")

T_MAX          = float(os.environ.get("SWARM_T_MAX",         "300.0"))
METRICS_PERIOD = float(os.environ.get("SWARM_METRICS_PERIOD",  "1.0"))
HEADLESS       = os.environ.get("SWARM_HEADLESS", "0").strip().lower() in ("1", "true", "yes", "on")

METRICS_FILE = os.environ.get(
    "SWARM_METRICS_FILE",
    f"results/flocking_slow/flock_slow_metrics_seed_{SEED}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
)
os.makedirs(os.path.dirname(METRICS_FILE) or ".", exist_ok=True)

# ── Metrics ─────────────────────────────────────────────────────────────────

_metrics_writer = None

def init_metrics_logger():
    global _metrics_writer
    f = open(METRICS_FILE, "w", newline="")
    _metrics_writer = csv.writer(f)
    _metrics_writer.writerow(["time", "area", "polarisation", "spd_spread", "avg_wind_speed", "collisions"])


def convex_hull_area(points: np.ndarray) -> float:
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 3:
        return 0.0
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(tuple(p))

    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(tuple(p))

    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        return 0.0
    hx = np.array([p[0] for p in hull])
    hy = np.array([p[1] for p in hull])
    return 0.5 * float(abs(np.dot(hx, np.roll(hy, -1)) - np.dot(hy, np.roll(hx, -1))))


def polarisation(agents) -> float:
    """Magnitude of mean heading unit vector in [0, 1]. 1 = perfectly aligned."""
    if not agents:
        return 0.0
    thetas = np.deg2rad(np.array([a.psi for a in agents], dtype=float))
    return float(np.hypot(np.cos(thetas).mean(), np.sin(thetas).mean()))


def speed_spread(agents) -> float:
    """Std-dev of agent speeds; 0 = all agents at same speed."""
    speeds = [float(getattr(a, "vel", 0.0)) for a in agents]
    return float(np.std(speeds)) if speeds else 0.0


# ── Agent initialisation ────────────────────────────────────────────────────

def _initialize_random_agent_states(sim, rng, domain_half=DOMAIN_SIZE):
    m = domain_half - 2.0
    for a in sim.agents:
        a.pos[0] = float(rng.uniform(-m, m))
        a.pos[1] = float(rng.uniform(-m, m))
        a.psi    = float(rng.uniform(0.0, 360.0))
        a.last_msg = None
        if hasattr(a, "nav") and hasattr(a.nav, "wp") and hasattr(a.nav.wp, "clear"):
            a.nav.wp.clear()


# ── Velocity-aware flocking controller ─────────────────────────────────────

class VelocityAwareFlockingController:
    """Couzin-style flocking extended with velocity-solidarity rules.

    Orientation and attraction contributions from each neighbour are scaled by
    the neighbour's inverse speed (USE_SPD_WT=True, the default).  A separate
    solidarity pull is added toward any neighbour slower than V_SLOW.

    Result: the flock coheres around the heading that slow-moving agents can
    sustain rather than averaging over all speeds equally.
    """

    def __init__(
        self,
        comm,
        rng,
        zor=ZOR,
        zoo=ZOO,
        zoa=ZOA,
        w_ori=W_ORI,
        w_att=W_ATT,
        w_solidarity=W_SOLIDARITY,
        v_slow=V_SLOW,
        spd_eps=SPD_EPS,
        use_spd_wt=USE_SPD_WT,
        noise_std_deg=NOISE_STD_DEG,
        domain_half=DOMAIN_SIZE,
        boundary_gain=BOUNDARY_GAIN,
        boundary_margin=BOUNDARY_MARGIN,
        toroidal=TOROIDAL,
    ):
        self.comm          = comm
        self.rng           = rng
        self.zor           = float(zor)
        self.zoo           = float(zoo)
        self.zoa           = float(zoa)
        self.w_ori         = float(w_ori)
        self.w_att         = float(w_att)
        self.w_solidarity  = float(w_solidarity)
        self.v_slow        = float(v_slow)
        self.spd_eps       = float(spd_eps)
        self.use_spd_wt    = bool(use_spd_wt)
        self.noise_std_deg = float(noise_std_deg)
        self.domain_half   = float(domain_half)
        self.bound_gain    = float(boundary_gain)
        self.bound_margin  = float(boundary_margin)
        self.toroidal      = bool(toroidal)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _boundary_vec(self, pos):
        if self.toroidal:
            return None
        x, y   = float(pos[0]), float(pos[1])
        inner  = self.domain_half - self.bound_margin
        if inner <= 0.0:
            return None
        fx = fy = 0.0
        if   x >  inner: fx -= (x - inner) / self.bound_margin
        elif x < -inner: fx += (-inner - x) / self.bound_margin
        if   y >  inner: fy -= (y - inner) / self.bound_margin
        elif y < -inner: fy += (-inner - y) / self.bound_margin
        if fx == 0.0 and fy == 0.0:
            return None
        v = np.array([fx, fy], dtype=float)
        n = np.linalg.norm(v)
        return v / n if n > 1e-6 else None

    @staticmethod
    def _heading_vec(hdg_deg):
        rad = np.deg2rad(hdg_deg)
        return np.array([np.cos(rad), np.sin(rad)], dtype=float)

    @staticmethod
    def _norm(v):
        n = np.linalg.norm(v)
        return v / n if n > 1e-6 else None

    # ── Main control law ─────────────────────────────────────────────────────

    def suggest_heading(self, agent):
        """Return a desired heading [0, 360) degrees, or None (keep current).

        Velocity-solidarity rules applied on top of standard Couzin:
        - Orientation and attraction contributions are weighted by 1/(spd+eps).
        - Neighbours slower than V_SLOW generate an extra solidarity pull.
        """
        my_pose = {
            "x":   float(agent.pos[0]),
            "y":   float(agent.pos[1]),
            "hdg": float(agent.psi),
            "spd": float(getattr(agent, "vel", 0.0)),
        }

        nb_data = self.comm.read_neighbor_poses(agent)
        n_nbrs  = len(nb_data)

        if hasattr(agent, "nav") and hasattr(agent.nav, "status"):
            agent.nav.status = f"FLOCK_S n={n_nbrs}"

        rep_vec      = np.zeros(2, dtype=float)
        ori_vec      = np.zeros(2, dtype=float)
        att_vec      = np.zeros(2, dtype=float)
        slow_vec     = np.zeros(2, dtype=float)   # solidarity spotlight
        has_repulsion = False

        px, py = my_pose["x"], my_pose["y"]

        for nbr, st in nb_data:
            dx = st["x"] - px
            dy = st["y"] - py

            # Toroidal minimum-image wrap
            if self.toroidal:
                L = self.domain_half
                if dx >  L: dx -= 2.0 * L
                elif dx < -L: dx += 2.0 * L
                if dy >  L: dy -= 2.0 * L
                elif dy < -L: dy += 2.0 * L

            d = float(np.hypot(dx, dy))
            if d < 1e-6:
                continue

            r_hat = np.array([dx, dy], dtype=float) / d
            spd_j = max(0.0, float(st.get("spd", 0.0)))

            # Inverse-speed weight: slower neighbour → larger influence
            if self.use_spd_wt:
                w = 1.0 / (spd_j + self.spd_eps)
            else:
                w = 1.0

            if d < self.zor:
                # Repulsion zone — no speed weighting; safety is unconditional
                rep_vec -= r_hat
                has_repulsion = True

            elif d < self.zoo:
                # Orientation zone — align with neighbour, weighted by their slowness
                ori_vec += w * self._heading_vec(st["hdg"])

            elif d < self.zoa:
                # Attraction zone — pull toward neighbour, weighted by their slowness
                att_vec += w * r_hat

            # Solidarity spotlight: regardless of zone, add pull toward slow agents
            if spd_j < self.v_slow:
                slow_vec += r_hat   # direction only; W_SOLIDARITY applied below

        bvec = self._boundary_vec((px, py))

        # Priority: repulsion > (orientation + attraction + solidarity)
        if has_repulsion and np.linalg.norm(rep_vec) > 1e-6:
            v = rep_vec
        else:
            v = (self.w_ori        * ori_vec
               + self.w_att        * att_vec
               + self.w_solidarity * slow_vec)

        if np.linalg.norm(v) < 1e-6:
            if bvec is not None:
                v = self.bound_gain * bvec
            elif n_nbrs > 0:
                v = self._heading_vec(my_pose["hdg"])
            else:
                return None
        else:
            if bvec is not None:
                v = v + self.bound_gain * bvec

        v = self._norm(v)
        if v is None:
            return None

        base_heading = float(np.degrees(np.arctan2(v[1], v[0])) % 360.0)

        if self.noise_std_deg > 0.0:
            noise        = float(self.rng.normal(0.0, self.noise_std_deg))
            base_heading = (base_heading + noise) % 360.0

        return base_heading


# ── Entry point ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    base = Path(__file__).resolve().parent.parent

    sim = Simulator(
        1 / 24,
        sim_xml=str(base / "sail_extension" / "regatta.xml"),
    )

    rng = np.random.default_rng(SEED)
    init_metrics_logger()
    _initialize_random_agent_states(sim, rng, domain_half=DOMAIN_SIZE)

    comm = NeighborhoodComm(sim, NEIGHBOR_RADIUS)

    flock = VelocityAwareFlockingController(
        comm=comm,
        rng=rng,
        zor=ZOR,
        zoo=ZOO,
        zoa=ZOA,
        w_ori=W_ORI,
        w_att=W_ATT,
        w_solidarity=W_SOLIDARITY,
        v_slow=V_SLOW,
        spd_eps=SPD_EPS,
        use_spd_wt=USE_SPD_WT,
        noise_std_deg=NOISE_STD_DEG,
        domain_half=DOMAIN_SIZE,
        boundary_gain=BOUNDARY_GAIN,
        boundary_margin=BOUNDARY_MARGIN,
        toroidal=TOROIDAL,
    )

    ca = CollisionAvoidance(
        comm,
        safe_radius=SAFE_RADIUS,
        horizon=CPA_HORIZON,
        bias_max_deg=AVOID_BIAS_MAX,
        tie_mode=CA_TIE_MODE,
        dwell=CA_DWELL,
        seed=SEED,
    )

    plotter = None
    if not HEADLESS:
        plotter = WindPlotter(
            simulator=sim,
            wind_field=sim.wind_field,
            SIZE=int(DOMAIN_SIZE),
            show_wind=True,
            show_waypoints=False,
            show_agent_targets=False,
            show_agent_status=SHOW_STATUS,
        )

    metrics_last_time = [-1e9]

    def simulation_callback():
        t_now = getattr(sim, "time", 0.0)
        if t_now >= T_MAX:
            if not HEADLESS:
                try:
                    import matplotlib.pyplot as plt
                    plt.close("all")
                except Exception:
                    pass
            return

        sim.tick()
        t = getattr(sim, "time", 0.0)

        # Share poses (includes "spd") so VelocityAwareFlockingController can read them
        for a in sim.agents:
            comm.broadcast_pose(a, {
                "x":   float(a.pos[0]),
                "y":   float(a.pos[1]),
                "hdg": float(a.psi),
                "spd": float(getattr(a, "vel", 0.0)),
            })

        for agent in sim.agents:
            wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)
            agent.update(wind_vec)

            desired = flock.suggest_heading(agent)
            base_heading = desired if desired is not None else float(getattr(agent, "cmd_heading", agent.psi))

            # Collision-avoidance bias on top of flocking heading
            agent.cmd_heading = ca.suggest_heading(agent, base_heading)

            msg = str(agent.nav) if hasattr(agent, "nav") else ""
            if not hasattr(agent, "last_msg") or agent.last_msg != msg:
                agent.last_msg = msg

        # ── Collision counting ────────────────────────────────────────────────
        ca.scan_collisions(sim.agents)
        if plotter is not None:
            plotter.set_info(f"Collisions: {ca.collision_count}")

        # ── Metrics ──────────────────────────────────────────────────────────
        if _metrics_writer is not None and (t - metrics_last_time[0]) >= METRICS_PERIOD:
            positions = np.array([[a.pos[0], a.pos[1]] for a in sim.agents], dtype=float)
            area      = convex_hull_area(positions)
            pol       = polarisation(sim.agents)
            spd_std   = speed_spread(sim.agents)

            wind_speeds = []
            for a in sim.agents:
                wx, wy = sim.wind_field.get_wind_at_position(
                    [a.pos[0], a.pos[1], 0.0], t
                )
                wind_speeds.append(float(np.hypot(wx, wy)))
            avg_wind = float(np.mean(wind_speeds)) if wind_speeds else 0.0

            _metrics_writer.writerow(
                [f"{t:.3f}", f"{area:.6f}", f"{pol:.6f}", f"{spd_std:.6f}", f"{avg_wind:.6f}", ca.collision_count]
            )
            metrics_last_time[0] = t

        # ── Boundary handling ─────────────────────────────────────────────────
        for a in sim.agents:
            if TOROIDAL:
                if a.pos[0] >  DOMAIN_SIZE: a.pos[0] -= 2.0 * DOMAIN_SIZE
                elif a.pos[0] < -DOMAIN_SIZE: a.pos[0] += 2.0 * DOMAIN_SIZE
                if a.pos[1] >  DOMAIN_SIZE: a.pos[1] -= 2.0 * DOMAIN_SIZE
                elif a.pos[1] < -DOMAIN_SIZE: a.pos[1] += 2.0 * DOMAIN_SIZE
            else:
                a.pos[0] = float(np.clip(a.pos[0], -DOMAIN_SIZE, DOMAIN_SIZE))
                a.pos[1] = float(np.clip(a.pos[1], -DOMAIN_SIZE, DOMAIN_SIZE))

    if HEADLESS:
        while getattr(sim, "time", 0.0) < T_MAX:
            simulation_callback()
    else:
        plotter.update_plot(callback=simulation_callback)
