import os
from pathlib import Path
import numpy as np

from SwarmSwIM import Simulator
from .sail_extension.wind_plotter import WindPlotter
from .sensors.comm import NeighborhoodComm
from .sensors.collision_avoidance import CollisionAvoidance

# PARAMETERS
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

class DispersionController:
    def __init__(self, sim, comm, domain_size):
        self.sim = sim
        self.comm = comm
        self.domain_size = float(domain_size)

        self.targets = self._generate_grid_targets(len(sim.agents), self.domain_size)

        # Format: { cell_idx: {'cost': infinity, 'winner': None} }
        self.agent_memory = {}
        for a in sim.agents:
            self.agent_memory[a.name] = {
                i : {'cost': float('inf'), 'winner': None}
                for i in range(len(self.targets))
            }

    def _generate_grid_targets(self, n_agents, d_size):
        side = np.sqrt(n_agents)
        rows = int(np.floor(side))
        cols = int(np.ceil(n_agents / rows))
        while rows * cols < n_agents:
            rows += 1

        x_space = np.linspace(-d_size, d_size, cols * 2 + 1)[1::2]
        y_space = np.linspace(-d_size, d_size, rows * 2 + 1)[1::2]

        targets = []
        for y in y_space:
            for x in x_space:
                targets.append(np.array([x, y]))

        return targets

    def step(self, agent):
        my_mem = self.agent_memory[agent.name]
        my_pos = agent.pos[0:2]

        # PHASE 1: read and update internal memory
        for nbr, msg in self.comm.read_neighbor_states(agent):
            neighbor_auction = msg.get('auction_data')
            if not neighbor_auction:
                continue
            
            for cell_id, nbr_bid in neighbor_auction.items():
                curr_bid = my_mem[cell_id]

                # Neighbor has lower bid
                if nbr_bid['cost'] < curr_bid['cost'] - 1e-6:
                    my_mem[cell_id] = nbr_bid.copy()
                # Bids are (nearly) equal -> Tiebreaking by name (lexicographic)
                elif abs(nbr_bid['cost'] - curr_bid['cost']) < 1e-6:
                    if nbr_bid['winner'] is not None and (curr_bid['winner'] is None or nbr_bid['winner'] < curr_bid['winner']):
                        my_mem[cell_id] = nbr_bid.copy()

        # PHASE 2: bidding
        # Check if winning a cell
        my_cell_idx = None
        for cell_id, bid in my_mem.items():
            if bid['winner'] == agent.name:
                my_cell_idx = cell_id
                dist = np.linalg.norm(self.targets[cell_id] - my_pos)
                my_mem[cell_id]['cost'] = dist
                break
                # Currently using break -> selecting the first cell if there are multiple cells where the same agent is winner
                # TODO: Celect cell by cost (dist) or cell with lowest count of other bidders

        # Bidding fails (lost bid or no cell available)
        if my_cell_idx is None:
            best_cell = None
            best_bid_margin = -1.0

            # Evaluating bids of all cells (compare with bids of agents which are no nbrs)
            for cell_id, bid in my_mem.items():
                target_pos = self.targets[cell_id]
                dist = np.linalg.norm(target_pos - my_pos)

                if dist < bid['cost']:
                    margin = bid['cost'] - dist
                    if margin > best_bid_margin:
                        best_bid_margin = margin
                        best_cell = cell_id

            # take best_cell and bid
            if best_cell is not None:
                dist = np.linalg.norm(self.targets[best_cell] - my_pos)
                my_mem[best_cell] = {'cost': dist, 'winner': agent.name}
                my_cell_idx = best_cell

        # PHASE 3: broadcasting results
        self.comm.broadcast(agent, {'auction_data': my_mem})

        # PHASE 4: output target
        if my_cell_idx is not None:
            return self.targets[my_cell_idx]
        return None
        # TODO: Select any cell if no result from above for iterations


# --- Main script entry point ---

if __name__ == "__main__":
    base = Path(__file__).resolve().parent.parent

    sim = Simulator(
        1 / 24,
        sim_xml=str(base / "SwarmSwIM" / "sail_extension" / "flocking.xml"),
    )
    """
    Simulation Object
    - timeSubdivision: (float), unit in seconds, time interval used for each simulation step.
    - sim_xml: (string) name of XML file describing the simulation parameters
    """

    # Clear waypoints
    _initialize_agents(sim)

    # Initialize modules
    comm = NeighborhoodComm(
        sim=sim,
        radius=NEIGHBOR_RADIUS,
    )

    dispersion = DispersionController(
        sim=sim,
        comm=comm,
        domain_size=DOMAIN_SIZE,
    )

    plotter = WindPlotter(
        simulator=sim,
        wind_field=sim.wind_field,
        SIZE=int(DOMAIN_SIZE),
        show_wind=True,
    )

    def simulation_callback():
        sim.tick()

        comm.step_time(sim.Dt)

        for agent in sim.agents:
            #execute_control(agent)

            target_pos = dispersion.step(agent)

            if target_pos is not None:
                loiter_radius = agent.nav.wp.waypoint_tolerance
                dist_to_target = np.linalg.norm(target_pos - agent.pos[0:2])

                # agent arrived at cell; stop setting transient waypoint and loiter
                if dist_to_target <= loiter_radius:
                    agent.nav.status = f"AUC: LOITERING ({dist_to_target:.1f}m)"

                # agent ha not yet arrived; continue setting transient waypoint
                else:
                    agent.nav.wp.set_transient_target({
                        "x": target_pos[0],
                        "y": target_pos[1],
                        "name": "Cell"      #TODO: evt naming of cells
                    })

                    agent.viz_target = {"x": target_pos[0],"y": target_pos[1]}
                    agent.nav.status = f"AUC: {target_pos[0]:.0f},{target_pos[1]:.0f}"
            else:
                # Standard behaviour if no cell found/still bidding; continue path
                #agent.cmd_heading = getattr(agent, "cmd_heading", agent.psi)
                agent.nav.status = "AUC: Bidding..."

            agent_nav_msg = str(agent.nav)
            if agent_nav_msg != agent.last_msg:
                agent.last_msg = agent_nav_msg

            wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)
            agent.update(wind_vec)



    """def execute_control(agent):
        target_pos = dispersion.step(agent)

        if target_pos is not None:
            agent.nav.wp.set_transient_target({
                "x": target_pos[0],
                "y": target_pos[1],
                "name": "Cell"      #TODO: evt naming of cells
            })

            agent.viz_target = {"x": target_pos[0],"y": target_pos[1]}
            agent.nav.status = f"AUC: {target_pos[0]:.0f},{target_pos[1]:.0f}"
        else:
            # Standard behaviour if no cell found/still bidding; continue path
            agent.cmd_heading = getattr(agent, "cmd_heading", agent.psi)
            agent.nav.status = "AUC: Bidding..."


        wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)
        agent.update(wind_vec)"""

    plotter.update_plot(callback=simulation_callback)

    