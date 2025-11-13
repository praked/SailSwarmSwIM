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


class GoalConsensus:
    """Consensus on a single flocking point via periodic broadcast polling.

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
            goal_wp = [{"name": "FlockGoal", "x": gx, "y": gy}]
            self.sim.waypoints = goal_wp
            for a in self.sim.agents:
                a.nav.set_waypoint_sequence(goal_wp)


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

    plotter = WindPlotter(
        simulator=sim,
        wind_field=sim.wind_field,
        SIZE=int(DOMAIN_SIZE),
        show_wind=True,
        show_waypoints=True,
    )

    def execute_control(agent):
        wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)
        agent.update(wind_vec)

    def simulation_callback():
        # Navigation consensus and physics step
        consensus.tick(sim.Dt if hasattr(sim, "Dt") else 1 / 24)
        sim.tick()
        for agent in sim.agents:
            execute_control(agent)
            msg = str(agent.nav)
            if not hasattr(agent, "last_msg") or agent.last_msg != msg:
                agent.last_msg = msg

    plotter.update_plot(callback=simulation_callback)