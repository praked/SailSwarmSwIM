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
NEIGHBOR_RADIUS = float(os.environ.get("SWARM_COMM_RADIUS", "20.0"))
DOMAIN_SIZE = float(os.environ.get("SWARMSWIM_DOMAIN", "35.0"))

BOUNDARY_GAIN = float(os.environ.get("SWARM_BOUND_GAIN", "1.2"))
BOUNDARY_MARGIN = float(os.environ.get("SWARM_BOUND_MARGIN", "5.0"))


def _initialize_agents(sim):
    """Clear any existing waypoint plans."""
    # Positions set to random via XML

    for a in sim.agents:

        # Reset last-msg debug
        if not hasattr(a, "last_msg"):
            a.last_msg = None
        else:
            a.last_msg = None

        # Clear any previous waypoint plans from tacking nav
        if hasattr(a, "nav") and hasattr(a.nav, "wp") and hasattr(a.nav.wp, "clear"):
            a.nav.wp.clear()


# --- Flocking controller (Reynolds' algorithm) ---

class BoidsFlockingController:
    """
    Boids flocking algorithm, inspired by Craig Reynolds

    - sep_vec: Separation: steer to avoid crowding local flockmates
    - alg_vec: Alignment: steer towards the average heading of local flockmates
    - coh_vec: Cohesion: steer to move toward the average position of local flockmates
    """

    def __init__(
        self,
        comm,
        domain_half=DOMAIN_SIZE,
        boundary_gain=BOUNDARY_GAIN,
        boundary_margin=BOUNDARY_MARGIN,
    ):
        self.comm = comm
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
    def _norm_vec(vec):
        """Noramlize given vector"""
        norm = np.linalg.norm(vec)
        if norm > 1e-6:
            vec /= norm
        return vec


    def desired_heading(self, agent):
        """
        Compute a desired heading for this agent based on local neighbors.
        Returns heading in degrees [0,360).
        """

        # Use pose-sharing data from NeighborhoodComm
        from .sensors.collision_avoidance import CollisionAvoidance as _CA
        my_pose = _CA._pose_of(agent)

        nb_data = self.comm.read_neighbor_poses(agent) # nb_data contains nbr object and st dictionary: {'x', 'y', 'hdg', 'spd'}
        n_nbrs = len(nb_data)

        # Update label text if nav has a status field (used by WindPlotter)
        if hasattr(agent, "nav") and hasattr(agent.nav, "status"):
            agent.nav.status = f"BOID n={n_nbrs}"

        # Three vectors as described for Reynold's algorithm 
        sep_vec = np.zeros(2, dtype=float) # separation
        alg_vec = np.zeros(2, dtype=float) # alignment
        coh_vec = np.zeros(2, dtype=float) # cohesion

        avg_pos = np.zeros(2, dtype=float) # average position of nbrs (for cohesion)

        px, py = my_pose["x"], my_pose["y"]

        for nbr, st in nb_data:
            dx = st["x"] - px
            dy = st["y"] - py
            d = float(np.hypot(dx, dy))

            if d < 1e-6:
                continue

            # alignment vector
            alg_vec += self._heading_vec(st["hdg"])

            # average position (cohesion)
            avg_pos[0] += st["x"]
            avg_pos[1] += st["y"]

            # separation vector
            diff = np.array([px - st["x"], py - st["y"]])
            repulsion_factor = d*d      # factor of repulsion d*d: inverse proportional to distance
            diff /= repulsion_factor
            sep_vec[0] += diff[0]
            sep_vec[1] += diff[1]

        if n_nbrs > 0:
            # normalize vectors
            alg_vec /= n_nbrs

            avg_pos /= n_nbrs
            coh_vec[0] = avg_pos[0] - px
            coh_vec[1] = avg_pos[1] - py

        des_vec = np.add(alg_vec, coh_vec)
        des_vec = np.add(des_vec, sep_vec)

        # Soft boundary steering vector (toward interior) based on current position
        bvec = self._boundary_vec((px, py))
        if bvec is not None:
            des_vec = np.add(des_vec, bvec)

        desired_heading = np.degrees(np.arctan2(des_vec[1], des_vec[0])) % 360
        return desired_heading


# --- Main script entry point ---

if __name__ == "__main__":
    sim = Simulator(
        1 / 24,
        sim_xml="sail_extension/flocking.xml",
    )
    """
    Simulation Object
    - timeSubdivision: (float), unit in seconds, time interval used for each simulation step.
    - sim_xml: (string) name of XML file describing the simulation parameters
    """

    _initialize_agents(sim)

    # Neighborhood communication
    comm = NeighborhoodComm(sim, NEIGHBOR_RADIUS)

    boid = BoidsFlockingController(
        comm=comm,
        domain_half=DOMAIN_SIZE,
        boundary_gain=BOUNDARY_GAIN,
        boundary_margin=BOUNDARY_MARGIN,
    )

    plotter = WindPlotter(
        simulator=sim,
        wind_field=sim.wind_field,
        SIZE=int(DOMAIN_SIZE),
        show_wind=True,
        show_waypoints=False,
        show_agent_status=SHOW_STATUS,
    )

    def simulation_callback():
        sim.tick()

        # Share current poses for flocking + collision avoidance
        from .sensors.collision_avoidance import CollisionAvoidance as _CA
        for a in sim.agents:
            comm.broadcast_pose(a, _CA._pose_of(a))


        for agent in sim.agents:
            execute_control(agent)

            agent_nav_msg = str(agent.nav)
            if agent_nav_msg != agent.last_msg:
                agent.last_msg = agent_nav_msg

    def execute_control(agent):
        wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)

        agent.update(wind_vec)

        # desired heading for flocking (Boids algorithm)
        desired_heading = boid.desired_heading(agent)
        if desired_heading is None:
            base_heading = getattr(agent, "cmd_heading", agent.psi)
        else:
            base_heading = desired_heading

        # TODO: Colision avoidance could be applied here

        agent.cmd_heading = base_heading



    plotter.update_plot(callback=simulation_callback)