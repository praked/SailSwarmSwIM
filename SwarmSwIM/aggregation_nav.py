import os
from pathlib import Path
import numpy as np
from SwarmSwIM import Simulator
import csv
from datetime import datetime
from .sail_extension.wind_plotter import WindPlotter

from .sensors.comm import NeighborhoodComm
from .sensors.collision_avoidance import CollisionAvoidance


SHOW_STATUS = os.environ.get("SWARM_SHOW_STATUS", "1").strip().lower() in ("1","true","yes","on")
# --- User-tunable parameters via environment variables ---
SEED = int(os.environ.get("SWARMSWIM_SEED", "142"))
NEIGHBOR_RADIUS = float(os.environ.get("SWARM_COMM_RADIUS", "20"))    # meters (sim units)
BROADCAST_PERIOD = float(os.environ.get("SWARM_BCAST_PERIOD", "1.0")) # seconds
DOMAIN_SIZE = float(os.environ.get("SWARMSWIM_DOMAIN", "10"))         # +/- range drawn by WindPlotter
STANDOFF_RADIUS = float(os.environ.get("SWARM_STANDOFF_RADIUS", "3.0"))

PRINT_PERIOD = float(os.environ.get("SWARM_PRINT_PERIOD", "1.0"))

# --- Aggregation metrics + headless/batch support ---

# Metrics logging period (seconds)
METRICS_PERIOD = float(os.environ.get("SWARM_METRICS_PERIOD", "1.0"))

# Distance threshold for connectivity in aggregation metrics (d_c)
# Defaults to the communication/neighbor radius.
DC_THRESH = float(os.environ.get("SWARM_AGG_DC", str(NEIGHBOR_RADIUS)))

# Max simulation time (seconds) in batch mode
T_MAX = float(os.environ.get("SWARM_T_MAX", "300.0"))

# Headless / batch mode (no interactive matplotlib window)
HEADLESS = os.environ.get("SWARM_HEADLESS", "0").strip().lower() in ("1", "true", "yes", "on")

METRICS_FILE = os.environ.get(
    "SWARM_METRICS_FILE",
    f"aggregation_metrics_seed_{SEED}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
)

_metrics_writer = None
def init_metrics_logger():
    global _metrics_writer
    f = open(METRICS_FILE, "w", newline="")
    _metrics_writer = csv.writer(f)
    # time [s], largest cluster size, normalized hull area, mean wind speed
    _metrics_writer.writerow(["time", "c_max", "a_hull_norm", "w_mean"])

# --- Collision Avoidance Parameters ---
SAFE_RADIUS = float(os.environ.get("SWARM_CA_SAFE_RADIUS", "3.0"))
CPA_HORIZON = float(os.environ.get("SWARM_CA_HORIZON", "25.0"))
AVOID_BIAS_MAX = float(os.environ.get("SWARM_CA_BIAS_MAX", "180.0"))  # deg


class GoalConsensus:
    """Consensus on a single aggregation point via periodic broadcast polling.

    Preference rule: higher version wins; ties break by lexicographically
    smallest proposer name. All agents rebroadcast their belief periodically.
    """
    def __init__(self, sim, comm, rng, domain_half=DOMAIN_SIZE):
        self.sim = sim
        self.comm = comm
        self.rng = rng
        self.domain_half = domain_half
        # Per-agent belief: {name: {"goal": (x,y), "version": int, "proposer": str}}
        self.belief = {}
        self._init_proposals()

    def _random_point(self):
        m = self.domain_half - 2.0
        return (
            float(self.rng.uniform(-m, m)),
            float(self.rng.uniform(-m, m)),
        )

    def _init_proposals(self):
        for a in self.sim.agents:
            self.belief[a.name] = {
                "goal": self._random_point(),
                "version": 1,
                "proposer": a.name,
            }
            # Seed mailboxes so first tick already has something to read
            self.comm.mailbox[a.name] = self.belief[a.name]

    @staticmethod
    def _prefer(x, y):
        if x["version"] != y["version"]:
            return x["version"] > y["version"]
        return x["proposer"] < y["proposer"]

    def _choose_consensus(self, agent):
        best = self.belief[agent.name]
        for _, st in self.comm.read_neighbor_states(agent):
            if self._prefer(st, best):
                best = st
        return best

    def tick(self, dt):
        # 1) comm timers
        self.comm.step_time(dt)

        # 2) poll neighbors and update beliefs
        for a in self.sim.agents:
            best = self._choose_consensus(a)
            if best is not self.belief[a.name]:
                self.belief[a.name] = {
                    "goal": best["goal"],
                    "version": best["version"],
                    "proposer": best["proposer"],
                }
        # 3) periodic broadcast
        for a in self.sim.agents:
            if self.comm.can_broadcast(a, BROADCAST_PERIOD):
                self.comm.broadcast(a, self.belief[a.name])

        # 4) pick a single current consensus for plotting and navigation
        current = None
        for st in self.belief.values():
            if current is None or self._prefer(st, current):
                current = st

        if current is not None:
            gx, gy = current["goal"]

            self.sim.waypoints = [{"name": "Agg. Goal", "x": gx, "y": gy}]

            # Assign each agent a unique station on a ring around the goal so
            # their loiter paths don't share the same center and cross each other.
            n = max(1, len(self.sim.agents))
            # Ring radius: ensure adjacent stations are at least SAFE_RADIUS apart
            ring_r = SAFE_RADIUS / (2.0 * np.sin(np.pi / n)) if n > 1 else STANDOFF_RADIUS
            ring_r = max(ring_r, STANDOFF_RADIUS)
            ring_r = min(ring_r, self.domain_half - 2.0)

            for idx, a in enumerate(self.sim.agents):
                angle = 2.0 * np.pi * idx / n
                sx = gx + ring_r * np.cos(angle)
                sy = gy + ring_r * np.sin(angle)
                station = {"x": sx, "y": sy, "name": f"Agg_{a.name}"}

                if hasattr(a, "nav"):
                    # Set loiter center directly so station_keep orbits this
                    # station rather than the shared goal center
                    if hasattr(a.nav, "loiter_center"):
                        a.nav.loiter_center = station
                    # Set waypoint for agents not yet in loiter mode
                    a.nav.set_waypoint_sequence([station])

    


def _initialize_random_agent_states(sim, rng, domain_half=DOMAIN_SIZE):
    m = domain_half - 2.0
    for a in sim.agents:
        a.pos[0] = float(rng.uniform(-m, m))
        a.pos[1] = float(rng.uniform(-m, m))
        a.psi = float(rng.uniform(0.0, 360.0))
        # reset labels and nav plan
        if not hasattr(a, "last_msg"):
            a.last_msg = None
        else:
            a.last_msg = None
        if hasattr(a, "nav") and hasattr(a.nav, "wp") and hasattr(a.nav.wp, "clear"):
            a.nav.wp.clear()

def _compute_standoff_point(center_xy, agent_xy, radius):
    cx, cy = center_xy
    ax, ay = agent_xy
    vx, vy = ax - cx, ay - cy
    n = np.hypot(vx, vy)
    if n < 1e-6:
        # Agent exactly at center: pick a deterministic angle
        theta = 0.0
        return cx + radius * np.cos(theta), cy + radius * np.sin(theta)
    s = radius / n
    return cx + vx * s, cy + vy * s

def _build_boundary_points(center_xy, radius, num=22):
    cx, cy = center_xy
    pts = []
    for k in range(num):
        th = 2 * np.pi * k / num
        pts.append({
            "name":f"{k}",   # non unique
            "x": cx + radius * np.cos(th),
            "y": cy + radius * np.sin(th),
        })
    return pts

# --- Aggregation metric helpers ---

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

def largest_cluster_size(positions: np.ndarray, dc: float) -> int:
    """
    Compute C_max: size of the largest connected component in the
    neighbor graph where edges connect agents with distance < dc.
    """
    pts = np.asarray(positions, dtype=float)
    n = pts.shape[0]
    if n == 0:
        return 0
    if n == 1:
        return 1

    # Build adjacency list
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if np.hypot(*(pts[i] - pts[j])) < dc:
                adj[i].append(j)
                adj[j].append(i)

    visited = [False] * n
    max_size = 1

    for i in range(n):
        if visited[i]:
            continue
        # DFS
        stack = [i]
        visited[i] = True
        size = 0
        while stack:
            u = stack.pop()
            size += 1
            for v in adj[u]:
                if not visited[v]:
                    visited[v] = True
                    stack.append(v)
        if size > max_size:
            max_size = size

    return max_size

# --- Agent state/waypoint formatting for console printing ---
def _format_wp(wp):
    if wp is None:
        return "None"
    if isinstance(wp, dict):
        nm = wp.get("name", "?")
        x = wp.get("x", float("nan"))
        y = wp.get("y", float("nan"))
        return f"{nm}@({x:.1f},{y:.1f})"
    # assume array-like [x,y]
    try:
        return f"({wp[0]:.1f},{wp[1]:.1f})"
    except Exception:
        return "?"

def _agent_state_line(a):
    x, y = a.pos[0], a.pos[1]
    h = a.psi
    v = getattr(a, 'vel', 0.0)
    wp = None
    dist = float('nan')
    if hasattr(a, 'nav') and hasattr(a.nav, 'wp'):
        wp = a.nav.wp.target_waypoint
        try:
            dist = a.nav.wp.get_distance_to_current_waypoint(a.pos)
        except Exception:
            dist = float('nan')
    return f"{a.name} pos=({x:.1f},{y:.1f}) H={h:.0f}° v={v:.2f} m/s -> WP {_format_wp(wp)} d={dist:.1f}"

if __name__ == "__main__":
    base = Path(__file__).resolve().parent

    # Build simulator from sailing regatta xml
    sim = Simulator(
        1 / 24,
        sim_xml=str(base / "sail_extension" / "regatta.xml"),
    )

    rng = np.random.default_rng(SEED)

    # Spawn agents at random, deterministically from SEED
    #_initialize_random_agent_states(sim, rng, domain_half=DOMAIN_SIZE) #uncomment to bring back the random agents

    # Initialize aggregation metrics logger
    init_metrics_logger()

    # Neighborhood comm + consensus
    comm = NeighborhoodComm(sim, NEIGHBOR_RADIUS)
    consensus = GoalConsensus(sim, comm, rng, domain_half=DOMAIN_SIZE)

    # Collision avoidance module
    ca = CollisionAvoidance(
        comm,
        safe_radius=SAFE_RADIUS,
        horizon=CPA_HORIZON,
        bias_max_deg=AVOID_BIAS_MAX,
        seed=SEED,
    )

    plotter = None
    if not HEADLESS:
        plotter = WindPlotter(
            simulator=sim,
            wind_field=sim.wind_field,
            SIZE=int(DOMAIN_SIZE),
            show_wind=True,
            show_waypoints=True,
            show_agent_targets=True,
            show_agent_status=SHOW_STATUS,
        )

    # Metrics sampling (once per METRICS_PERIOD seconds)
    metrics_last_time = [-1e9]

    def simulation_callback():
        # Navigation consensus and physics step
        consensus.tick(sim.Dt if hasattr(sim, "Dt") else 1 / 24)
        sim.tick()

        # Current simulation time
        t = getattr(sim, "time", 0.0)

        # Share current poses via comm so neighbors can react
        for a in sim.agents:
            comm.broadcast_pose(a, CollisionAvoidance._pose_of(a))

        for agent in sim.agents:
            wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)
            agent.update(wind_vec)

            # Apply collision-avoidance heading bias
            if hasattr(agent, 'cmd_heading'):
                agent.cmd_heading = ca.suggest_heading(agent, float(agent.cmd_heading))

            msg = str(agent.nav)
            if not hasattr(agent, "last_msg") or agent.last_msg != msg:
                agent.last_msg = msg

        # --- Aggregation metrics: C_max(t), normalized hull area A_hull(t)/(2L_h)^2, and mean wind speed ---
        if _metrics_writer is not None and (t - metrics_last_time[0]) >= METRICS_PERIOD:
            positions = np.array([[a.pos[0], a.pos[1]] for a in sim.agents], dtype=float)
            c_max = largest_cluster_size(positions, DC_THRESH)
            hull_area = convex_hull_area(positions)
            arena_area = (2.0 * DOMAIN_SIZE) ** 2
            a_norm = hull_area / arena_area if arena_area > 0.0 else 0.0

            # Compute spatially averaged wind speed experienced by the swarm
            wind_speeds = []
            for a in sim.agents:
                w_vec = sim.wind_field.get_wind_at_position(a.pos, sim.time)
                w_arr = np.asarray(w_vec, dtype=float)
                if w_arr.size >= 2:
                    speed = float(np.hypot(w_arr[0], w_arr[1]))
                else:
                    speed = float(np.linalg.norm(w_arr))
                wind_speeds.append(speed)
            w_mean = float(np.mean(wind_speeds)) if wind_speeds else 0.0

            _metrics_writer.writerow(
                [f"{t:.3f}", int(c_max), f"{a_norm:.6f}", f"{w_mean:.6f}"]
            )
            metrics_last_time[0] = t
        # Periodic state dump (optional)
        # if t - state_print_last[0] >= PRINT_PERIOD:
        #     print(f"t={t:.1f}s | agent states (aggregation):")
        #     for a in sim.agents:
        #         print("  " + _agent_state_line(a))
        #     state_print_last[0] = t

    if HEADLESS:
        # Batch mode: run without interactive plotting up to T_MAX seconds
        while getattr(sim, "time", 0.0) < T_MAX:
            simulation_callback()
    else:
        # Interactive mode with WindPlotter
        plotter.update_plot(callback=simulation_callback)