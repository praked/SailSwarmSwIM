import os
from pathlib import Path
import numpy as np
from SwarmSwIM import Simulator
from .sail_extension.wind_plotter import WindPlotter

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
# --- CA symmetry-breaking and hysteresis ---
CA_TIE_MODE = os.environ.get("SWARM_CA_TIE_MODE", "deterministic").lower()  # deterministic|random|colregs
CA_DWELL    = float(os.environ.get("SWARM_CA_DWELL", "3.0"))                # s, keep chosen side per encounter

class NeighborhoodComm:
    """One-to-many broadcast within a distance-limited neighborhood.

    Not a full network model. Each tick it exposes the last broadcast
    from any neighbor within NEIGHBOR_RADIUS.
    """
    def __init__(self, sim, radius):
        self.sim = sim
        self.radius = radius
        self.mailbox = {a.name: None for a in sim.agents}
        self.time_since_last_tx = {a.name: 0.0 for a in sim.agents}
        self.posebox = {a.name: None for a in sim.agents}

    def neighbors_of(self, agent):
        nbrs = []
        for other in self.sim.agents:
            if other is agent:
                continue
            d = np.linalg.norm(other.pos[0:2] - agent.pos[0:2])
            if d <= self.radius:
                nbrs.append(other)
        return nbrs

    def can_broadcast(self, agent, period):
        return self.time_since_last_tx[agent.name] >= period

    def broadcast(self, agent, state):
        """Sender places its state; neighbors 'poll' by reading mailbox."""
        self.mailbox[agent.name] = state
        self.time_since_last_tx[agent.name] = 0.0

    def step_time(self, dt):
        for k in self.time_since_last_tx:
            self.time_since_last_tx[k] += dt

    def read_neighbor_states(self, agent):
        out = []
        for nbr in self.neighbors_of(agent):
            st = self.mailbox.get(nbr.name)
            if st is not None:
                out.append((nbr, st))
        return out

    def broadcast_pose(self, agent, pose_state):
        """Always-available instantaneous pose broadcast (not rate-limited)."""
        self.posebox[agent.name] = pose_state

    def read_neighbor_poses(self, agent):
        out = []
        for nbr in self.neighbors_of(agent):
            st = self.posebox.get(nbr.name)
            if st is not None:
                out.append((nbr, st))
        return out

# --- Collision Avoidance Helper ---
class CollisionAvoidance:
    """Heading bias based on Closest-Point-of-Approach to neighbors, with symmetry-breaking and hysteresis."""
    def __init__(self, comm, safe_radius=4.0, horizon=12.0, bias_max_deg=35.0,
                 tie_mode=CA_TIE_MODE, dwell=CA_DWELL, seed=SEED):
        self.comm = comm
        self.safe_radius = float(safe_radius)
        self.horizon = float(horizon)
        self.bias_max = np.deg2rad(float(bias_max_deg))
        self.tie_mode = tie_mode
        self.dwell = float(dwell)
        self.seed = int(seed)
        # Remember chosen side per pair to avoid flip-flop
        # key: (me, other) with lexical order of names; value: {"sign": +/-1, "expires": time}
        self.encounters = {}

    @staticmethod
    def _pose_of(agent):
        spd = getattr(agent, 'vel', 0.0)
        return {"x": float(agent.pos[0]), "y": float(agent.pos[1]), "hdg": float(agent.psi), "spd": float(spd)}

    @staticmethod
    def _vel_xy(pose):
        vx = pose["spd"] * np.cos(np.deg2rad(pose["hdg"]))
        vy = pose["spd"] * np.sin(np.deg2rad(pose["hdg"]))
        return np.array([vx, vy], dtype=float)

    def _cpa(self, my_pose, nb_pose):
        p_i = np.array([my_pose["x"], my_pose["y"]], dtype=float)
        v_i = self._vel_xy(my_pose)
        p_j = np.array([nb_pose["x"], nb_pose["y"]], dtype=float)
        v_j = self._vel_xy(nb_pose)
        r = p_j - p_i
        v = v_j - v_i
        v2 = float(np.dot(v, v))
        if v2 < 1e-6:
            return 0.0, np.linalg.norm(r)
        t_cpa = - float(np.dot(r, v)) / v2
        t_cpa = max(0.0, min(self.horizon, t_cpa))
        d_cpa = np.linalg.norm(r + v * t_cpa)
        return t_cpa, d_cpa

    @staticmethod
    def _heading_diff_deg(h1, h2):
        d = abs((h1 - h2 + 180.0) % 360.0 - 180.0)
        return d

    @staticmethod
    def _dot(a, b):
        return float(a[0]*b[0] + a[1]*b[1])

    def _pair_key(self, me_name, nb_name):
        return tuple(sorted((str(me_name), str(nb_name))))

    def _is_head_on_or_following(self, my_pose, nb_pose):
        # Head-on: headings nearly opposite
        dh = self._heading_diff_deg(my_pose["hdg"], nb_pose["hdg"])
        head_on = dh > 160.0
        # Following: nearly same heading and neighbor is ahead along my velocity vector
        same_dir = dh < 20.0
        vi = self._vel_xy(my_pose)
        r = np.array([nb_pose["x"] - my_pose["x"], nb_pose["y"] - my_pose["y"]], dtype=float)
        ahead = self._dot(vi, r) > 0.0
        following = same_dir and ahead
        return head_on or following

    def _deterministic_side(self, me_name, nb_name):
        # Consistent per pair: lower name turns left (+1), higher right (-1)
        a, b = self._pair_key(me_name, nb_name)
        return +1.0 if me_name == a else -1.0

    def _random_side(self, me_name, nb_name):
        a, b = self._pair_key(me_name, nb_name)
        h = hash((a, b, self.seed))
        return +1.0 if (h & 1) == 0 else -1.0

    def _colregs_side(self, my_pose, nb_pose):
        # Simple COLREGs-inspired: treat as head-on → both go right (negative sign)
        return -1.0

    def _avoid_sign(self, my_pose, nb_pose, me_name, nb_name, now):
        # Base from left/right pass using cross product
        r = np.array([nb_pose["x"] - my_pose["x"], nb_pose["y"] - my_pose["y"]], dtype=float)
        v_i = self._vel_xy(my_pose)
        z = r[0]*v_i[1] - r[1]*v_i[0]  # 2D cross product z-component
        base_sign = 1.0 if z < 0 else -1.0

        # Check cached decision for this pair
        key = (me_name, nb_name)
        skey = self._pair_key(me_name, nb_name)
        cached = self.encounters.get((key, skey))
        if cached is not None and now <= cached["expires"]:
            return cached["sign"]

        # Symmetric situations: head-on or following → break tie with configured rule
        if self._is_head_on_or_following(my_pose, nb_pose) or abs(z) < 1e-3:
            if self.tie_mode.startswith("deterministic"):
                chosen = self._deterministic_side(me_name, nb_name)
            elif self.tie_mode.startswith("random"):
                chosen = self._random_side(me_name, nb_name)
            else:  # colregs
                chosen = self._colregs_side(my_pose, nb_pose)
        else:
            chosen = base_sign

        # Store with dwell time to prevent flip-flop
        self.encounters[(key, skey)] = {"sign": chosen, "expires": now + self.dwell}
        return chosen

    def suggest_heading(self, agent, desired_heading_deg):
        my_pose = self._pose_of(agent)
        conflicts = []
        for nbr, st in self.comm.read_neighbor_poses(agent):
            t, d = self._cpa(my_pose, st)
            if 0.0 <= t <= self.horizon and d < self.safe_radius:
                conflicts.append((nbr, st, t, d))
        if not conflicts:
            return desired_heading_deg
        # Most urgent by smallest d at CPA (then shortest time)
        nbr, st, t, d = min(conflicts, key=lambda x: (x[3], x[2]))
        # Choose side using symmetry-breaking and hysteresis
        now = getattr(agent, 'internal_clock', 0.0)
        sign = self._avoid_sign(my_pose, st, agent.name, nbr.name, now)
        # Severity weighting by both distance and time
        k_d = max(0.0, min(1.0, (self.safe_radius - d) / self.safe_radius))
        k_t = max(0.0, min(1.0, (self.horizon - t) / self.horizon))
        k = max(k_d, 0.5*k_t)
        bias = sign * (k * self.bias_max)
        return (desired_heading_deg + np.rad2deg(bias)) % 360.0


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

            # Plot center + circular boundary
            center_wp = [{"name": "Agg. Goal", "x": gx, "y": gy}]
            ring = _build_boundary_points((gx, gy), STANDOFF_RADIUS, num=32)
            self.sim.waypoints = center_wp + ring

            # Per-agent standoff target on the circle, then hold
            for a in self.sim.agents:
                sx, sy = _compute_standoff_point((gx, gy), a.pos[0:2], STANDOFF_RADIUS)
                standoff_wp = [{"name": f"Standoff_{a.name}", "x": sx, "y": sy}]
                if hasattr(a, "nav") and hasattr(a.nav, "wp"):
                    if hasattr(a.nav.wp, "set_hold"):
                        a.nav.wp.set_hold(True)
                    else:
                        a.nav.wp.hold_at_target = True
                a.nav.set_waypoint_sequence(standoff_wp)

    


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
        tie_mode=CA_TIE_MODE,
        dwell=CA_DWELL,
        seed=SEED,
    )

    plotter = WindPlotter(
        simulator=sim,
        wind_field=sim.wind_field,
        SIZE=int(DOMAIN_SIZE),
        show_wind=True,
        show_waypoints=True,
        show_agent_targets=True,
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