"""agg_nav_v2.py — Swarm aggregation via behavioural force fields.

Architecture
============
Each agent computes a desired heading from three concurrent behavioural forces:

  F_sep  — separation: repulsion from agents closer than D_SEP metres.
            This replaces the dedicated CollisionAvoidance layer; the same
            social-force physics that produces aggregation also keeps agents
            from colliding.

  F_coh  — cohesion: attraction toward the centroid of all comm-range
            neighbours.  Keeps the swarm connected even before a goal
            is agreed upon.

  F_agg  — goal attraction: normalised pull toward the distributed-consensus
            goal.  Active at any range.

The combined force vector F = w_sep*F_sep + w_coh*F_coh + w_agg*F_agg is
converted to a virtual waypoint LOOKAHEAD metres ahead of the agent, then
passed to TackingNavigation.calculate_heading() so wind-tacking rules are
respected automatically.

Communication
=============
Agents share their pose (x, y, hdg, spd) every tick via the shared posebox.
Goal consensus uses version-based voting (same protocol as aggregation_nav.py):
higher version wins; ties broken by lexicographically smallest proposer name.

No CollisionAvoidance object is used.
"""

import os
from pathlib import Path

import numpy as np
from SwarmSwIM import Simulator
import csv
from datetime import datetime

from ..sail_extension.wind_plotter import WindPlotter
from ..sensors.comm import NeighborhoodComm

# ── Tunable parameters (all overridable via environment variables) ──────────

SHOW_STATUS      = os.environ.get("SWARM_SHOW_STATUS",   "1"   ).strip().lower() in ("1", "true", "yes", "on")
SEED             = int  (os.environ.get("SWARMSWIM_SEED",    "142"  ))
DOMAIN_SIZE      = float(os.environ.get("SWARMSWIM_DOMAIN",   "120"  ))

# Communication
NEIGHBOR_RADIUS  = float(os.environ.get("SWARM_COMM_RADIUS",   "20.0"))
BROADCAST_PERIOD = float(os.environ.get("SWARM_BCAST_PERIOD",   "1.0"))

# Behavioural weights  (tune these first when something looks wrong)
W_AGG  = float(os.environ.get("SWARM_W_AGG",  "1.0"))   # goal-attraction weight
W_COH  = float(os.environ.get("SWARM_W_COH",  "1.2"))   # cohesion weight
W_SEP  = float(os.environ.get("SWARM_W_SEP",  "5.0"))   # separation weight

# Behavioural distances (metres)
D_SEP  = float(os.environ.get("SWARM_D_SEP",  "4.0"))   # repulsion onset
D_COH  = float(os.environ.get("SWARM_D_COH",  "15.0"))  # cohesion neighbourhood

# Navigation
LOOKAHEAD = float(os.environ.get("SWARM_LOOKAHEAD", "10.0"))  # virtual-waypoint distance (m)

# Batch / headless
HEADLESS  = os.environ.get("SWARM_HEADLESS", "0").strip().lower() in ("1", "true", "yes", "on")
T_MAX     = float(os.environ.get("SWARM_T_MAX", "300.0"))

# Metrics
METRICS_PERIOD = float(os.environ.get("SWARM_METRICS_PERIOD", "1.0"))
DC_THRESH      = float(os.environ.get("SWARM_AGG_DC", str(NEIGHBOR_RADIUS)))
METRICS_FILE   = os.environ.get(
    "SWARM_METRICS_FILE",
    f"results/aggregation_v2/agg_v2_metrics_seed_{SEED}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
)
os.makedirs(os.path.dirname(METRICS_FILE) or ".", exist_ok=True)

# ── Metrics logger ──────────────────────────────────────────────────────────

_metrics_writer = None

def init_metrics_logger():
    global _metrics_writer
    f = open(METRICS_FILE, "w", newline="")
    _metrics_writer = csv.writer(f)
    _metrics_writer.writerow(["time", "c_max", "a_hull_norm", "w_mean"])


# ── Metric helpers ──────────────────────────────────────────────────────────

def convex_hull_area(points: np.ndarray) -> float:
    """Area of the convex hull of 2-D points (monotone-chain algorithm).
    Returns 0.0 for fewer than 3 points.
    """
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


def largest_cluster_size(positions: np.ndarray, dc: float) -> int:
    """Size of the largest connected component in the proximity graph."""
    pts = np.asarray(positions, dtype=float)
    n   = pts.shape[0]
    if n == 0:
        return 0
    if n == 1:
        return 1

    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if np.hypot(*(pts[i] - pts[j])) < dc:
                adj[i].append(j)
                adj[j].append(i)

    visited  = [False] * n
    max_size = 1
    for i in range(n):
        if visited[i]:
            continue
        stack, visited[i], size = [i], True, 0
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


# ── Distributed goal consensus ──────────────────────────────────────────────

class GoalConsensus:
    """Version-based consensus on a single aggregation goal point.

    Agents vote for the highest-version goal proposal; ties are broken by
    the lexicographically smallest proposer name.  Goal coordinates are
    shared via the existing NeighborhoodComm mailbox.

    This class is intentionally navigation-free: it only tracks the winning
    goal (gx, gy).  All movement decisions are made by SwarmBehavior.
    """

    def __init__(self, sim, comm, rng, domain_half=DOMAIN_SIZE):
        self.sim         = sim
        self.comm        = comm
        self.rng         = rng
        self.domain_half = domain_half
        # {agent_name: {"goal": (x, y), "version": int, "proposer": str}}
        self.belief = {}
        self._init_proposals()

    def _random_point(self):
        m = self.domain_half - 2.0
        return float(self.rng.uniform(-m, m)), float(self.rng.uniform(-m, m))

    def _init_proposals(self):
        for a in self.sim.agents:
            self.belief[a.name] = {
                "goal":     self._random_point(),
                "version":  1,
                "proposer": a.name,
            }
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
        """Advance communication timers, update beliefs, and rebroadcast."""
        self.comm.step_time(dt)

        for a in self.sim.agents:
            best = self._choose_consensus(a)
            if best is not self.belief[a.name]:
                self.belief[a.name] = {
                    "goal":     best["goal"],
                    "version":  best["version"],
                    "proposer": best["proposer"],
                }

        for a in self.sim.agents:
            if self.comm.can_broadcast(a, BROADCAST_PERIOD):
                self.comm.broadcast(a, self.belief[a.name])

    def current_goal(self):
        """Return (gx, gy) of the current winning proposal, or None.

        Also writes the goal to sim.waypoints so the plotter can render it.
        """
        best = None
        for st in self.belief.values():
            if best is None or self._prefer(st, best):
                best = st
        if best is None:
            return None
        gx, gy = best["goal"]
        self.sim.waypoints = [{"name": "Agg.Goal", "x": gx, "y": gy}]
        return gx, gy


# ── Swarm behavioural force model ───────────────────────────────────────────

class SwarmBehavior:
    """Per-agent force computation: separation + cohesion + goal attraction.

    Forces are computed from poses stored in comm.posebox (shared every tick).
    No external collision-avoidance module is needed: the separation force
    acts as a distributed, implicit collision avoidance mechanism.

    Force directions are unit vectors; magnitude is weighted by W_* constants.
    The combined force magnitude is NOT normalised so that a stronger combined
    pull produces a more decisive heading command.
    """

    def __init__(self, sim, comm):
        self.sim  = sim
        self.comm = comm

    def _neighbor_positions(self, agent):
        """Return list of (x, y) tuples for all comm-range neighbours."""
        mx, my = float(agent.pos[0]), float(agent.pos[1])
        out = []
        for nbr in self.sim.agents:
            if nbr is agent:
                continue
            st = self.comm.posebox.get(nbr.name)
            if st is None:
                continue
            if np.hypot(st["x"] - mx, st["y"] - my) <= self.comm.radius:
                out.append((st["x"], st["y"]))
        return out

    def compute_force(self, agent, goal_xy):
        """Return (fx, fy) combined behavioural force for *agent*.

        F = W_SEP * F_sep  +  W_COH * F_coh  +  W_AGG * F_agg

        Separation (F_sep):
            For each neighbour j within D_SEP metres:
              push away with strength proportional to (D_SEP - d) / D_SEP.
            At contact (d → 0) the push is maximum; it fades linearly to 0 at d = D_SEP.

        Cohesion (F_coh):
            Unit vector from self toward the centroid of all neighbours within
            D_COH metres.  Zero when no neighbours are in range.

        Goal attraction (F_agg):
            Unit vector from self toward the consensus goal.
            Zero if no goal has been agreed upon yet.
        """
        px, py = float(agent.pos[0]), float(agent.pos[1])
        nbrs   = self._neighbor_positions(agent)

        # 1. Separation ─────────────────────────────────────────────────────
        sep_x = sep_y = 0.0
        for nx, ny in nbrs:
            dx, dy = px - nx, py - ny
            d = np.hypot(dx, dy)
            if 0.0 < d < D_SEP:
                strength = (D_SEP - d) / D_SEP    # linear ramp: 1 at contact, 0 at D_SEP
                sep_x   += strength * dx / d
                sep_y   += strength * dy / d

        # 2. Cohesion ───────────────────────────────────────────────────────
        coh_x = coh_y = 0.0
        coh_nbrs = [(nx, ny) for nx, ny in nbrs
                    if np.hypot(nx - px, ny - py) < D_COH]
        if coh_nbrs:
            cx = sum(nx for nx, _ in coh_nbrs) / len(coh_nbrs) - px
            cy = sum(ny for _, ny in coh_nbrs) / len(coh_nbrs) - py
            dc = np.hypot(cx, cy)
            if dc > 1e-6:
                coh_x = cx / dc
                coh_y = cy / dc

        # 3. Goal attraction ────────────────────────────────────────────────
        agg_x = agg_y = 0.0
        if goal_xy is not None:
            gx, gy = goal_xy
            dx, dy = gx - px, gy - py
            dg = np.hypot(dx, dy)
            if dg > 1e-6:
                agg_x = dx / dg
                agg_y = dy / dg

        fx = W_SEP * sep_x + W_COH * coh_x + W_AGG * agg_x
        fy = W_SEP * sep_y + W_COH * coh_y + W_AGG * agg_y
        return fx, fy


# ── Entry point ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    base = Path(__file__).resolve().parent.parent

    sim = Simulator(
        1 / 24,
        sim_xml=str(base / "sail_extension" / "regatta.xml"),
    )

    rng = np.random.default_rng(SEED)

    init_metrics_logger()

    comm      = NeighborhoodComm(sim, NEIGHBOR_RADIUS)
    consensus = GoalConsensus(sim, comm, rng, domain_half=DOMAIN_SIZE)
    behavior  = SwarmBehavior(sim, comm)

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

    metrics_last_time = [-1e9]

    def simulation_callback():
        dt = sim.Dt if hasattr(sim, "Dt") else 1 / 24
        consensus.tick(dt)
        sim.tick()

        t       = getattr(sim, "time", 0.0)
        goal_xy = consensus.current_goal()   # also updates sim.waypoints for plotter

        # Share current poses so SwarmBehavior can read neighbour positions
        for a in sim.agents:
            comm.broadcast_pose(a, {
                "x":   float(a.pos[0]),
                "y":   float(a.pos[1]),
                "hdg": float(a.psi),
                "spd": float(getattr(a, "vel", 0.0)),
            })

        for agent in sim.agents:
            wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)
            wind_arr = np.asarray(wind_vec, dtype=float)

            # Physics update.  nav.tick() sets cmd_heading via waypoint/loiter
            # logic; we override it below with the swarm-force heading.
            agent.update(wind_vec)

            # Compute combined swarm force
            fx, fy  = behavior.compute_force(agent, goal_xy)
            f_mag   = np.hypot(fx, fy)

            if f_mag > 1e-6:
                # Project force direction LOOKAHEAD metres ahead → virtual waypoint
                vwp = {
                    "x":    agent.pos[0] + LOOKAHEAD * fx / f_mag,
                    "y":    agent.pos[1] + LOOKAHEAD * fy / f_mag,
                    "name": "SwarmForce",
                }
                agent.nav.wp.set_transient_target(vwp)
                agent.nav.wp.hold_at_target = True
                # Clear any captured loiter center so calculate_heading uses
                # the transient waypoint rather than the previous orbit focus
                agent.nav.loiter_center = None
                heading = agent.nav.calculate_heading(agent, wind_arr)
                if heading is not None:
                    agent.cmd_heading = float(heading % 360)

            msg = str(agent.nav)
            if not hasattr(agent, "last_msg") or agent.last_msg != msg:
                agent.last_msg = msg

        # ── Aggregation metrics ─────────────────────────────────────────────
        if _metrics_writer is not None and (t - metrics_last_time[0]) >= METRICS_PERIOD:
            positions  = np.array([[a.pos[0], a.pos[1]] for a in sim.agents], dtype=float)
            c_max      = largest_cluster_size(positions, DC_THRESH)
            hull_area  = convex_hull_area(positions)
            arena_area = (2.0 * DOMAIN_SIZE) ** 2
            a_norm     = hull_area / arena_area if arena_area > 0.0 else 0.0

            wind_speeds = []
            for a in sim.agents:
                w_vec = sim.wind_field.get_wind_at_position(a.pos, sim.time)
                w_arr = np.asarray(w_vec, dtype=float)
                speed = float(np.hypot(w_arr[0], w_arr[1])) if w_arr.size >= 2 \
                        else float(np.linalg.norm(w_arr))
                wind_speeds.append(speed)
            w_mean = float(np.mean(wind_speeds)) if wind_speeds else 0.0

            _metrics_writer.writerow(
                [f"{t:.3f}", int(c_max), f"{a_norm:.6f}", f"{w_mean:.6f}"]
            )
            metrics_last_time[0] = t

    if HEADLESS:
        while getattr(sim, "time", 0.0) < T_MAX:
            simulation_callback()
    else:
        plotter.update_plot(callback=simulation_callback)
