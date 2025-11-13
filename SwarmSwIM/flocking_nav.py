import os
from pathlib import Path
import numpy as np

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

# Domain boundary steering
BOUNDARY_GAIN = float(os.environ.get("SWARM_BOUND_GAIN", "1.2"))      # strength of "push" back inside
BOUNDARY_MARGIN = float(os.environ.get("SWARM_BOUND_MARGIN", "5.0"))  # soft wall thickness in meters


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

    def simulation_callback():
        # Step dynamics (positions/heading updated according to previous cmd_heading)
        sim.tick()

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
        t = getattr(sim, "time", 0.0)
        if t - state_print_last[0] >= PRINT_PERIOD:
            print(f"t={t:.1f}s | agent states (flocking):")
            for a in sim.agents:
                print("  " + _agent_state_line(a))
            state_print_last[0] = t

    # Start interactive plot loop
    plotter.update_plot(callback=simulation_callback)