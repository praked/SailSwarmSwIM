import os
from pathlib import Path
import numpy as np
import csv
from datetime import datetime

from SwarmSwIM import Simulator
from .sail_extension.wind_plotter import WindPlotter
from .sensors.comm import NeighborhoodComm
from .sensors.collision_avoidance import CollisionAvoidance

# --- User-tunable parameters via environment variables ---

SHOW_STATUS = os.environ.get("SWARM_SHOW_STATUS", "1").strip().lower() in ("1", "true", "yes", "on")

SEED = int(os.environ.get("SWARMSWIM_SEED", "142"))
NEIGHBOR_RADIUS = float(os.environ.get("SWARM_COMM_RADIUS", "20.0"))       # meters (sim units)
DOMAIN_SIZE = float(os.environ.get("SWARMSWIM_DOMAIN", "35.0"))           # +/- range drawn by WindPlotter

# Collision avoidance
SAFE_RADIUS = float(os.environ.get("SWARM_CA_SAFE_RADIUS", "4.0"))
CPA_HORIZON = float(os.environ.get("SWARM_CA_HORIZON", "12.0"))
AVOID_BIAS_MAX = float(os.environ.get("SWARM_CA_BIAS_MAX", "90.0"))       # deg
CA_TIE_MODE = os.environ.get("SWARM_CA_TIE_MODE", "colregs").lower()
CA_DWELL = float(os.environ.get("SWARM_CA_DWELL", "3.0"))

# Couzin-style zones (all <= NEIGHBOR_RADIUS makes sense)
ZOR = float(os.environ.get("SWARM_ZOR", "4.0"))       # Repulsion radius
ZOO = float(os.environ.get("SWARM_ZOO", "10.0"))      # Orientation radius
ZOA = float(os.environ.get("SWARM_ZOA", "18.0"))      # Attraction radius

# Flocking weights & noise
W_ORI = float(os.environ.get("SWARM_W_ORI", "1.0"))   # Weight for alignment
W_ATT = float(os.environ.get("SWARM_W_ATT", "0.6"))   # Weight for attraction


NOISE_STD_DEG = float(os.environ.get("SWARM_FLOCK_NOISE", "3.0"))  # angular noise std dev

# Max simulation time (seconds)
T_MAX = float(os.environ.get("SWARM_T_MAX", "300.0"))


# Metrics logging period (seconds)
METRICS_PERIOD = float(os.environ.get("SWARM_METRICS_PERIOD", "1.0"))

# Headless / batch mode (no interactive matplotlib window)
HEADLESS = os.environ.get("SWARM_HEADLESS", "0").strip().lower() in ("1", "true", "yes", "on")

# Domain boundary steering
BOUNDARY_GAIN = float(os.environ.get("SWARM_BOUND_GAIN", "1.2"))      # strength of "push" back inside
BOUNDARY_MARGIN = float(os.environ.get("SWARM_BOUND_MARGIN", "5.0"))  # soft wall thickness in meters

METRICS_FILE = os.environ.get(
    "SWARM_METRICS_FILE",
    f"flocking_metrics_seed_{SEED}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
)

_metrics_writer = None
def init_metrics_logger():
    global _metrics_writer
    f = open(METRICS_FILE, "w", newline="")
    _metrics_writer = csv.writer(f)
    _metrics_writer.writerow(["time", "area", "polarisation_sd", "avg_wind_speed"])

def convex_hull_area(points: np.ndarray) -> float:
    """
    Compute area of convex hull of 2D points using monotone chain.
    points: (N, 2) array.
    Returns 0.0 if fewer than 3 points.
    """
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 3:
        return 0.0

    # Sort by x, then y
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    # Build lower hull
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(tuple(p))

    # Build upper hull
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(tuple(p))

    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        return 0.0

    # Shoelace formula
    hx = np.array([p[0] for p in hull])
    hy = np.array([p[1] for p in hull])
    return 0.5 * float(abs(np.dot(hx, np.roll(hy, -1)) - np.dot(hy, np.roll(hx, -1))))


def heading_spread_sd(agents) -> float:
    """
    Polarisation-like metric:
    SD of headings (in radians) around their circular mean.
    Smaller value => better aligned flock.
    """
    if not agents:
        return 0.0

    thetas = np.deg2rad(np.array([a.psi for a in agents], dtype=float))
    # circular mean
    mean_angle = np.arctan2(np.sin(thetas).mean(), np.cos(thetas).mean())
    # wrap differences into [-pi, pi]
    diffs = np.angle(np.exp(1j * (thetas - mean_angle)))
    return float(np.std(diffs))
# --- Helper functions used in init/state handling ---

def _initialize_random_agent_states(sim, rng, domain_half=DOMAIN_SIZE):
    """Randomize agent positions/headings and clear any existing waypoint plans."""
    m = domain_half - 2.0
    for a in sim.agents:
        a.pos[0] = float(rng.uniform(-m, m))
        a.pos[1] = float(rng.uniform(-m, m))
        a.psi = float(rng.uniform(0.0, 360.0))

        # Reset last-msg debug
        if not hasattr(a, "last_msg"):
            a.last_msg = None
        else:
            a.last_msg = None

        # Clear any previous waypoint plans from tacking nav
        if hasattr(a, "nav") and hasattr(a.nav, "wp") and hasattr(a.nav.wp, "clear"):
            a.nav.wp.clear()


# --- Flocking controller (Couzin-inspired) ---

class FlockingController:
    """
    Local flocking control inspired by Couzin et al.:

    - ZOR: repulsion (collision avoidance at behavior level; physics-level CA still runs).
    - ZOO: orientation (align headings).
    - ZOA: attraction (move toward neighbors).
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
        noise_std_deg=NOISE_STD_DEG,
        domain_half=DOMAIN_SIZE,
        boundary_gain=BOUNDARY_GAIN,
        boundary_margin=BOUNDARY_MARGIN,
    ):
        self.comm = comm
        self.rng = rng
        self.zor = float(zor)
        self.zoo = float(zoo)
        self.zoa = float(zoa)
        self.w_ori = float(w_ori)
        self.w_att = float(w_att)
        self.noise_std_deg = float(noise_std_deg)
        # Soft domain boundaries
        self.domain_half = float(domain_half)
        self.bound_gain = float(boundary_gain)
        self.bound_margin = float(boundary_margin)
    def _boundary_vec(self, pos):
        """
        Soft-wall steering vector that points back toward the interior when
        the agent approaches the square domain boundary.

        pos: (x, y) tuple or array.
        Returns a unit vector or None if safely inside.
        """
        x = float(pos[0])
        y = float(pos[1])

        # Start applying steering when closer than (domain_half - margin)
        inner = self.domain_half - self.bound_margin
        if inner <= 0.0:
            return None

        fx = 0.0
        fy = 0.0

        if x > inner:
            fx -= (x - inner) / self.bound_margin
        elif x < -inner:
            fx += (-inner - x) / self.bound_margin

        if y > inner:
            fy -= (y - inner) / self.bound_margin
        elif y < -inner:
            fy += (-inner - y) / self.bound_margin

        if fx == 0.0 and fy == 0.0:
            return None

        v = np.array([fx, fy], dtype=float)
        n = np.linalg.norm(v)
        if n < 1e-6:
            return None
        return v / n

    @staticmethod
    def _heading_vec(hdg_deg):
        """Unit vector for a compass heading (deg, 0°=+x, 90°=+y)."""
        rad = np.deg2rad(hdg_deg)
        return np.array([np.cos(rad), np.sin(rad)], dtype=float)

    @staticmethod
    def _norm(v):
        n = np.linalg.norm(v)
        if n < 1e-6:
            return None
        return v / n

    def suggest_heading(self, agent):
        """
        Compute a desired heading for this agent based on local neighbors.
        Returns heading in degrees [0,360) or None (fallback to current cmd/psi).
        """
        # Use pose-sharing data from NeighborhoodComm
        from .sensors.collision_avoidance import CollisionAvoidance as _CA
        my_pose = _CA._pose_of(agent)

        nb_data = self.comm.read_neighbor_poses(agent)
        n_nbrs = len(nb_data)

        # Update label text if nav has a status field (used by WindPlotter)
        if hasattr(agent, "nav") and hasattr(agent.nav, "status"):
            agent.nav.status = f"FLOCK n={n_nbrs}"

        rep_vec = np.zeros(2, dtype=float)
        ori_vec = np.zeros(2, dtype=float)
        att_vec = np.zeros(2, dtype=float)
        has_repulsion = False

        px, py = my_pose["x"], my_pose["y"]

        for nbr, st in nb_data:
            dx = st["x"] - px
            dy = st["y"] - py
            d = float(np.hypot(dx, dy))
            if d < 1e-6:
                continue

            r_hat = np.array([dx, dy], dtype=float) / d

            if d < self.zor:
                # Repulsion zone
                rep_vec -= r_hat
                has_repulsion = True
            elif d < self.zoo:
                # Orientation zone
                ori_vec += self._heading_vec(st["hdg"])
            elif d < self.zoa:
                # Attraction zone
                att_vec += r_hat
            else:
                # Outside ZOA, ignore
                pass

        # Soft boundary steering vector (toward interior) based on current position
        bvec = self._boundary_vec((px, py))

        if has_repulsion and np.linalg.norm(rep_vec) > 1e-6:
            v = rep_vec
        else:
            # Orientation + attraction
            v = self.w_ori * ori_vec + self.w_att * att_vec

        if np.linalg.norm(v) < 1e-6:
            # If social forces cancel or there are no neighbors, use boundary if needed.
            if bvec is not None:
                v = self.bound_gain * bvec
            elif n_nbrs > 0:
                # Keep current heading if we have neighbors but net vector is zero.
                v = self._heading_vec(my_pose["hdg"])
            else:
                # No neighbors and safely inside domain -> no change requested.
                return None
        else:
            # Mix social vector with boundary steering if near edge.
            if bvec is not None:
                v = v + self.bound_gain * bvec

        v = self._norm(v)
        if v is None:
            return None

        base_heading = np.degrees(np.arctan2(v[1], v[0])) % 360.0

        # Add small angular noise to prevent lattice locking / symmetry artifacts
        if self.noise_std_deg > 0.0:
            noise = float(self.rng.normal(0.0, self.noise_std_deg))
            base_heading = (base_heading + noise) % 360.0

        return base_heading


# --- Optional: nice printing helpers if you want to dump states to console ---

def _format_wp(wp):
    if wp is None:
        return "None"
    if isinstance(wp, dict):
        nm = wp.get("name", "?")
        x = wp.get("x", float("nan"))
        y = wp.get("y", float("nan"))
        return f"{nm}@({x:.1f},{y:.1f})"
    try:
        return f"({wp[0]:.1f},{wp[1]:.1f})"
    except Exception:
        return "?"


def _agent_state_line(a):
    x, y = a.pos[0], a.pos[1]
    h = a.psi
    v = getattr(a, "vel", 0.0)
    wp = None
    dist = float("nan")
    if hasattr(a, "nav") and hasattr(a.nav, "wp"):
        wp = a.nav.wp.target_waypoint
        try:
            dist = a.nav.wp.get_distance_to_current_waypoint(a.pos)
        except Exception:
            dist = float("nan")
    return f"{a.name} pos=({x:.1f},{y:.1f}) H={h:.0f}° v={v:.2f} m/s -> WP {_format_wp(wp)} d={dist:.1f}"


# --- Main script entry point ---

if __name__ == "__main__":
    base = Path(__file__).resolve().parent

    # Build simulator from sailing regatta xml
    sim = Simulator(
        1 / 24,
        sim_xml=str(base / "sail_extension" / "regatta.xml"),
    )

    rng = np.random.default_rng(SEED)
    init_metrics_logger()
    # Spawn agents at random, deterministically from SEED
    _initialize_random_agent_states(sim, rng, domain_half=DOMAIN_SIZE)

    # Neighborhood communication
    comm = NeighborhoodComm(sim, NEIGHBOR_RADIUS)

    # Flocking controller
    flock = FlockingController(
        comm=comm,
        rng=rng,
        zor=ZOR,
        zoo=ZOO,
        zoa=ZOA,
        w_ori=W_ORI,
        w_att=W_ATT,
        noise_std_deg=NOISE_STD_DEG,
        domain_half=DOMAIN_SIZE,
        boundary_gain=BOUNDARY_GAIN,
        boundary_margin=BOUNDARY_MARGIN,
    )

    # Collision avoidance module
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
            show_waypoints=False,        # no global aggregation goal here
            show_agent_targets=False,
            show_agent_status=SHOW_STATUS,
        )

    # Optional periodic printing
    PRINT_PERIOD = float(os.environ.get("SWARM_PRINT_PERIOD", "1.0"))
    state_print_last = [-1e9]

    # Metrics sampling (once per METRICS_PERIOD seconds)
    metrics_last_time = [-1e9]

    def simulation_callback():
        # Stop simulation after T_MAX seconds
        t_now = getattr(sim, "time", 0.0)
        if t_now >= T_MAX:
            if not HEADLESS:
                try:
                    import matplotlib.pyplot as plt
                    plt.close('all')
                except Exception:
                    pass
            return

        # Step dynamics (positions/heading updated according to previous cmd_heading)
        sim.tick()
        t = getattr(sim, "time", 0.0)

        # Share current poses for flocking + collision avoidance
        from .sensors.collision_avoidance import CollisionAvoidance as _CA
        for a in sim.agents:
            comm.broadcast_pose(a, _CA._pose_of(a))

        # Per-agent control
        for agent in sim.agents:
            # Local wind
            wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)

            # Update sail physics + internal nav clock; but we override nav's heading
            agent.update(wind_vec)

            # Flocking heading suggestion (Couzin-style)
            desired = flock.suggest_heading(agent)
            if desired is None:
                base_heading = getattr(agent, "cmd_heading", agent.psi)
            else:
                base_heading = desired

            # Apply collision-avoidance bias on top
            agent.cmd_heading = ca.suggest_heading(agent, float(base_heading))

            # Cache status string if you want to use it elsewhere
            msg = str(agent.nav) if hasattr(agent, "nav") else ""
            if not hasattr(agent, "last_msg") or agent.last_msg != msg:
                agent.last_msg = msg
        
        # --- Metrics: flock area + polarisation-like spread + avg wind speed ---
        # Only log once per METRICS_PERIOD seconds
        if _metrics_writer is not None and (t - metrics_last_time[0]) >= METRICS_PERIOD:
            positions = np.array([[a.pos[0], a.pos[1]] for a in sim.agents], dtype=float)
            area = convex_hull_area(positions)
            pol_sd = heading_spread_sd(sim.agents)  # smaller => more aligned

            # Average wind speed at agent positions
            wind_speeds = []
            for a in sim.agents:
                wx, wy = sim.wind_field.get_wind_at_position(
                    [a.pos[0], a.pos[1], 0.0],
                    t,
                )
                wind_speeds.append(np.hypot(wx, wy))
            avg_wind = float(np.mean(wind_speeds)) if wind_speeds else 0.0

            _metrics_writer.writerow(
                [f"{t:.3f}", f"{area:.6f}", f"{pol_sd:.6f}", f"{avg_wind:.6f}"]
            )
            metrics_last_time[0] = t
        # Hard clamp: keep agents inside square [-DOMAIN_SIZE, DOMAIN_SIZE]
        for a in sim.agents:
            if a.pos[0] > DOMAIN_SIZE:
                a.pos[0] = DOMAIN_SIZE
            elif a.pos[0] < -DOMAIN_SIZE:
                a.pos[0] = -DOMAIN_SIZE
            if a.pos[1] > DOMAIN_SIZE:
                a.pos[1] = DOMAIN_SIZE
            elif a.pos[1] < -DOMAIN_SIZE:
                a.pos[1] = -DOMAIN_SIZE

        # Optional: periodic state dump
        # (uses t defined earlier in this callback)
        # if t - state_print_last[0] >= PRINT_PERIOD:
        #     print(f"t={t:.1f}s | agent states (flocking):")
        #     for a in sim.agents:
        #         print("  " + _agent_state_line(a))
        #     state_print_last[0] = t

    if HEADLESS:
        # Headless / batch mode: no plotting, just advance the simulation
        while getattr(sim, "time", 0.0) < T_MAX:
            simulation_callback()
    else:
        # Start interactive plot loop
        plotter.update_plot(callback=simulation_callback)