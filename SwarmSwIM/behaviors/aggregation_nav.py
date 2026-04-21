import os
from pathlib import Path
import numpy as np
from SwarmSwIM import Simulator
from .sail_extension.wind_plotter import WindPlotter

from .sensors.comm import NeighborhoodComm
from .sensors.collision_avoidance import CollisionAvoidance


SHOW_STATUS = os.environ.get("SWARM_SHOW_STATUS", "1").strip().lower() in ("1","true","yes","on")
# --- User-tunable parameters via environment variables ---
SEED = int(os.environ.get("SWARMSWIM_SEED", "142"))
NEIGHBOR_RADIUS = float(os.environ.get("SWARM_COMM_RADIUS", "20"))    # meters (sim units)
BROADCAST_PERIOD = float(os.environ.get("SWARM_BCAST_PERIOD", "1.0")) # seconds
DOMAIN_SIZE = float(os.environ.get("SWARMSWIM_DOMAIN", "35"))         # +/- range drawn by WindPlotter
STANDOFF_RADIUS = float(os.environ.get("SWARM_STANDOFF_RADIUS", "15.0"))
PRINT_PERIOD = float(os.environ.get("SWARM_PRINT_PERIOD", "1.0"))

# --- Collision Avoidance Parameters ---
SAFE_RADIUS = float(os.environ.get("SWARM_CA_SAFE_RADIUS", "4.0"))
CPA_HORIZON = float(os.environ.get("SWARM_CA_HORIZON", "12.0"))
AVOID_BIAS_MAX = float(os.environ.get("SWARM_CA_BIAS_MAX", "90.0"))  # deg


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
        #print(best)
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

            # Plot center + circular boundary
            center_wp = [{"name": "Agg. Goal", "x": gx, "y": gy}]
            #ring = _build_boundary_points((gx, gy), STANDOFF_RADIUS, num=32)
            self.sim.waypoints = center_wp #self.sim.waypoints = center_wp + ring

            # Per-agent standoff target on the circle, then hold
            for a in self.sim.agents:
                # sx, sy = _compute_standoff_point((gx, gy), a.pos[0:2], STANDOFF_RADIUS)
                # standoff_wp = [{"name": f"Standoff_{a.name}", "x": sx, "y": sy}]
                if hasattr(a, "nav") and hasattr(a.nav, "wp"):
                    if hasattr(a.nav.wp, "set_hold"):
                        a.nav.wp.set_hold(True)
                    else:
                        a.nav.wp.hold_at_target = True
                #a.nav.set_waypoint_sequence(standoff_wp)
                a.nav.set_waypoint_sequence(center_wp)

    


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
    _initialize_random_agent_states(sim, rng, domain_half=DOMAIN_SIZE)

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

    plotter = WindPlotter(
        simulator=sim,
        wind_field=sim.wind_field,
        SIZE=int(DOMAIN_SIZE),
        show_wind=True,
        show_waypoints=True,
        show_agent_targets=True,
        show_agent_status=SHOW_STATUS,
    )

    # --- Periodic agent state printing ---
    state_print_last = [-1e9]  # mutable container for closure

    def execute_control(agent):
        wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)
        agent.update(wind_vec)

    def simulation_callback():
        # Navigation consensus and physics step
        consensus.tick(sim.Dt if hasattr(sim, "Dt") else 1 / 24)
        sim.tick()
        # Share current poses via comm so neighbors can react
        for a in sim.agents:
            comm.broadcast_pose(a, CollisionAvoidance._pose_of(a))
        for agent in sim.agents:
            execute_control(agent)
            # Apply collision-avoidance heading bias for next tick
            if hasattr(agent, 'cmd_heading'):
                agent.cmd_heading = ca.suggest_heading(agent, float(agent.cmd_heading))
                
            msg = str(agent.nav)
            if not hasattr(agent, "last_msg") or agent.last_msg != msg:
                agent.last_msg = msg

        # Periodic state dump
        # t = getattr(sim, 'time', 0.0)
        # if t - state_print_last[0] >= PRINT_PERIOD:
        #     print(f"t={t:.1f}s | agent states:")
        #     for a in sim.agents:
        #         print("  "+_agent_state_line(a))
        #     state_print_last[0] = t

    plotter.update_plot(callback=simulation_callback)