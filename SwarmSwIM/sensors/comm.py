import numpy as np

class NeighborhoodComm:
    """Distance-limited one-to-many broadcast, plus pose sharing."""
    def __init__(self, sim, radius):
        self.sim = sim
        self.radius = float(radius)
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

    # Pose sharing used by collision avoidance
    def broadcast_pose(self, agent, pose_state):
        self.posebox[agent.name] = pose_state

    def read_neighbor_poses(self, agent):
        out = []
        for nbr in self.neighbors_of(agent):
            st = self.posebox.get(nbr.name)
            if st is not None:
                out.append((nbr, st))
        return out