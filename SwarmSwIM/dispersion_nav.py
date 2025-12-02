import os
from pathlib import Path
import numpy as np
import random

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
    """TODO: Update description of phases
    Simple dispersion controller dividing the domain into cells.
    Each agent holds a list of all cells, targets are selected and
    negotiated with neighbors via a bidding-system.

    PHASE 1: Read all incoming messages and update winners in internal list
    PHASE 2: Select target cell (currently first cell that is won).
        Fallback: evaluate all cells and take best (nearest with no known winner)
    PHASE 3: Broadcast internal list ("locally updated" or fallback)
    PHASE 4: Return selected target
    """

    def __init__(self, sim, comm, domain_size):
        self.sim = sim
        self.comm = comm
        self.domain_size = float(domain_size)

        self.targets = self._generate_grid_targets(len(sim.agents), self.domain_size)

        # Store loitering cell
        self.loiter_cell = {}

        # Priority initially 0
        self.agent_priority = {}

        # Memory-Format: { cell_idx: {'cost': infinity, 'winner': None, 'winners_prio': 0} }
        self.agent_memory = {}
        for a in sim.agents:
            self.agent_memory[a.name] = {
                i : {'cost': float('inf'), 'winner': None, 'winners_prio': float(0), 'loitering': False}
                for i in range(len(self.targets))
            }
            self.agent_priority[a.name] = float(0)
            self.loiter_cell[a.name] = int(-1)



    def _generate_grid_targets(self, n_agents, d_size):
        """Divide domain into grid-cells, depending on amount of agents.
        Set center of the cells as targets for dispersion"""

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
    
    def _take_cell(self, agent, cell_id, loitering):
        my_mem = self.agent_memory[agent.name]
        my_pos = agent.pos[0:2]
        my_dist = np.linalg.norm(self.targets[cell_id] - my_pos)
        my_prio = float(self.agent_priority[agent.name])

        my_mem[cell_id] = {
            'cost': my_dist - my_prio,
            'winner': agent.name,
            'winners_prio': my_prio,
            'loitering': loitering
            }
        
    def _clear_cell(self, agent, cell_id):
        my_mem = self.agent_memory[agent.name]

        my_mem[cell_id] = {
            'cost': float('inf'),
            'winner': None,
            'winners_prio': float(0),
            'loitering': False
            }

    def step(self, agent):
        """
        """
        my_mem = self.agent_memory[agent.name]
        my_pos = agent.pos[0:2]
        my_prio = float(self.agent_priority[agent.name])
        self_loitering = agent.nav.is_loitering
        loiter_cell = self.loiter_cell[agent.name]
        has_cell = False

        # PHASE 0: update own memory with own distances, update NOT if there is information about a nearer neighbor

        """if not self_loitering:
            for cell_id, bid in my_mem.items():

                if bid['winner'] == agent.name:
                    self._take_cell(agent, cell_id, loitering=False)
                    #has_cell = True

            #if not has_cell:
                #best_cell = None
                #best_cost = float('inf')

                #for cell_id, bid in my_mem.items():
                    #my_dist = np.linalg.norm(self.targets[cell_id] - my_pos)
                    #if (my_dist < best_cost) and bid['winner'] is None:
                        #best_cell = cell_id
                
                #if best_cell is not None:
                    #self._take_cell(agent, best_cell, loitering=False)

        else:
            for cell_id, bid in my_mem.items():
                if bid['winner'] == agent.name:
                    if cell_id == loiter_cell:
                        print("take cell")
                        self._take_cell(agent, cell_id, loitering=True)    # keep loitering here
                    else:
                        print(agent.name, " clearing cell ", cell_id, "though loiter_cell is ", loiter_cell)
                        self._clear_cell(agent, cell_id)"""


        # LESS AGGRESSIVE
        best_cell = None
        best_cost = float('inf')


        if not self_loitering:
            # Currenly not loitering around any cell (update for all entries)
            # Update NOT if there is information about a nearer neighbor, or neighbor loitering in a cell
            for cell_id, bid in my_mem.items():
                my_dist = np.linalg.norm(self.targets[cell_id] - my_pos)

                # Update own bid entry (especially also when my_dist increased)
                if bid['winner'] == agent.name:
                    self._take_cell(agent, cell_id, loitering=False)
                    has_cell = True

                # Keep note of the best cell available to take if I don't have already one
                elif bid['winner'] is None:
                    if my_dist < best_cost - 1e-6:
                        best_cell = cell_id
                        best_cost = my_dist


                # No known winner for this cell -> assume self as winner
                #elif bid['winner'] is None and not has_cell:
                    #self._take_cell(agent, cell_id, loitering=False)
                    #has_cell = True

                """elif my_dist < bid['cost'] - 1e-6 and not bid['loitering']:
                    # Known winner, but with higher cost (and not already loitering in that cell)
                    self._take_cell(agent, cell_id, loitering=False)
                    has_cell = True"""# switch with elif bid['winner]
            
            # If I don't own a cell after cycling through all -> take the closest available one
            if not has_cell:
                if best_cell is not None:
                    self._take_cell(agent, best_cell, loitering=False)
                    has_cell = True
                
        else:
            # Loitering around one cell (update this cell's entry, free up all other OWN entries)
            for cell_id, bid in my_mem.items():

                # Alter only own cells
                if bid['winner'] == agent.name:
                    # Update only loitering cell
                    if cell_id == loiter_cell:
                        self._take_cell(agent, cell_id, loitering=True)    #keep loitering here
                    # Discard all other owned cells
                    else:
                        self._clear_cell(agent, cell_id)
        """
        for cell_id, bid in my_mem.items():
            my_dist = np.linalg.norm(self.targets[cell_id] - my_pos)
            effective_dist = max(my_dist - my_prio, 0) # distance updated with priority

            if bid['winner'] is None:
                # No known winner for this cell -> assume self as winner
                my_mem[cell_id] = {'cost': effective_dist, 'winner': agent.name, 'winners_prio': my_prio}
            elif effective_dist < bid['cost'] - 1e-6:
                # Known winner, but with higher cost
                my_mem[cell_id] = {'cost': effective_dist, 'winner': agent.name, 'winners_prio': my_prio}"""
        

        # PHASE 1: Update internal memory with nbr data
        for nbr, msg in self.comm.read_neighbor_states(agent):
            neighbor_auction = msg.get('auction_data')
            if not neighbor_auction:
                continue
            
            for cell_id, nbr_bid in neighbor_auction.items():
                curr_bid = my_mem[cell_id]

                # Skip own cell if loitering (Relay-Mode)
                if self_loitering:
                    if cell_id == loiter_cell:
                        continue

                # Accept entry if neighbor already loiters (and nbr is not me)
                if nbr_bid['loitering'] and (nbr_bid['winner'] != agent.name):
                    my_mem[cell_id] = nbr_bid.copy()

                # Accept neighbor clearing itself from cell
                # -> nbr is closest in our entry, but has 'winner': None in his entry
                elif curr_bid['winner'] == nbr.name and nbr_bid['winner'] is None:
                    my_mem[cell_id] = nbr_bid.copy()

                # Neighbor has lower bid (since I'm not loitering here) and nbr is NOT me
                elif (nbr_bid['cost'] < curr_bid['cost'] - 1e-6) and nbr_bid['winner'] != agent.name:
                    my_mem[cell_id] = nbr_bid.copy()
                # Bids are (nearly) equal and nbr is NOT me -> Tiebreaking by priority -> Fallback: random choice
                elif (abs(nbr_bid['cost'] - curr_bid['cost']) < 1e-6) and nbr_bid['winner'] != agent.name:
                    nbr_prio = nbr_bid['winners_prio']
                    if my_prio < nbr_prio:
                        my_mem[cell_id] = nbr_bid.copy()
                    elif my_prio == nbr_prio:
                        my_mem[cell_id] = random.choice((my_mem[cell_id], nbr_bid.copy()))

                """# If I'm loitering don't accept other cells won by me (to propagate clearance)
                elif self_loitering:
                    # Neighbor has lower bid (since I'm not loitering here) and nbr is NOT me
                    if (nbr_bid['cost'] < curr_bid['cost'] - 1e-6) and nbr_bid['winner'] != agent.name:
                        my_mem[cell_id] = nbr_bid.copy()
                    # Bids are (nearly) equal and nbr is NOT me -> Tiebreaking by priority -> Fallback: random choice
                    elif (abs(nbr_bid['cost'] - curr_bid['cost']) < 1e-6) and nbr_bid['winner'] != agent.name:
                        nbr_prio = nbr_bid['winners_prio']
                        if my_prio < nbr_prio:
                            my_mem[cell_id] = nbr_bid.copy()
                        elif my_prio == nbr_prio:
                            my_mem[cell_id] = random.choice((my_mem[cell_id], nbr_bid.copy()))
                            
                else:
                    # Neighbor has lower bid (since I'm not loitering here) and nbr is NOT me
                    if (nbr_bid['cost'] < curr_bid['cost'] - 1e-6):
                        my_mem[cell_id] = nbr_bid.copy()
                    # Bids are (nearly) equal and nbr is NOT me -> Tiebreaking by priority -> Fallback: random choice
                    elif (abs(nbr_bid['cost'] - curr_bid['cost']) < 1e-6):
                        nbr_prio = nbr_bid['winners_prio']
                        if my_prio < nbr_prio:
                            my_mem[cell_id] = nbr_bid.copy()
                        elif my_prio == nbr_prio:
                            my_mem[cell_id] = random.choice((my_mem[cell_id], nbr_bid.copy()))"""

                """# Neighbor has lower bid (since I'm not loitering here)
                if nbr_bid['cost'] < curr_bid['cost'] - 1e-6:
                    my_mem[cell_id] = nbr_bid.copy()
                # Bids are (nearly) equal -> Tiebreaking by priority -> Fallback: random choice
                elif abs(nbr_bid['cost'] - curr_bid['cost']) < 1e-6:
                    nbr_prio = nbr_bid['winners_prio']
                    if my_prio < nbr_prio:
                        my_mem[cell_id] = nbr_bid.copy()
                    elif my_prio == nbr_prio:
                        my_mem[cell_id] = random.choice((my_mem[cell_id], nbr_bid.copy()))"""

                    
                    
                       
                    


                """# Update entry if neighbor has lower bid
                if nbr_bid['cost'] < curr_bid['cost'] - 1e-6:
                    my_mem[cell_id] = nbr_bid.copy()
                # Bids are (nearly) equal -> Tiebreaking by priority -> Fallback: random choice
                elif abs(nbr_bid['cost'] - curr_bid['cost']) < 1e-6:
                    nbr_prio = nbr_bid['winners_prio']
                    if my_prio < nbr_prio:
                        my_mem[cell_id] = nbr_bid.copy()
                    elif my_prio == nbr_prio:
                        my_mem[cell_id] = random.choice((my_mem[cell_id], nbr_bid.copy()))
                    #if nbr_bid['winner'] is not None and (curr_bid['winner'] is None or nbr_bid['winner'] < curr_bid['winner']):
                        #my_mem[cell_id] = nbr_bid.copy()"""

        # PHASE 2: Find best winning cell
        my_cells = [c for c, b in my_mem.items() if b['winner'] == agent.name]
        my_cell_idx = None

        # If at least one cell is won, select as target
        if my_cells:
            my_cell_idx = min(my_cells, key=lambda c: my_mem[c]['cost'])
        
        #if best_cell is not None:
            #print(agent.name, " taking cell ", best_cell)
            #self._take_cell(agent, best_cell, loitering=self_loitering)
            #self.loiter_cell[agent.name] = best_cell

        
        # PHASE 2b: Mark chosen target as loitering
        if my_cell_idx is not None:
            self._take_cell(agent, my_cell_idx, loitering=self_loitering)
            self.loiter_cell[agent.name] = my_cell_idx

        # PHASE 3: Bidding fails (lost bid or no cell available)
        if my_cell_idx is None:
            self.agent_priority[agent.name] += 1 #increase priority

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
                self._take_cell(agent, best_cell, loitering=False)
                my_cell_idx = best_cell

        # PHASE 4: broadcasting results
        self.comm.broadcast(agent, {'auction_data': my_mem})
        #print("Agent: ", agent.name, "Memory:\n", my_mem)
        #print("going to: ", my_cell_idx, "loitering there: ", self_loitering)
        #print("loiter-memory: ", self.loiter_cell)

        # PHASE 5: return target
        if my_cell_idx is not None:
            return self.targets[my_cell_idx]
        return None


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

    # Visualise center of gridcells as waypoints
    sim.waypoints = [
        {"x": float(t[0]), "y": float(t[1]), "name": f"{i}"} 
        for i, t in enumerate(dispersion.targets)
    ]

    plotter = WindPlotter(
        simulator=sim,
        wind_field=sim.wind_field,
        SIZE=int(DOMAIN_SIZE),
        show_wind=True,
        show_waypoints=True,
        show_agent_targets=True,
        show_agent_status=True,
    )

    def simulation_callback():

        for agent in sim.agents:

            wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)
            agent.update(wind_vec)

            target_pos = dispersion.step(agent)

            if target_pos is not None:
                loiter_radius = agent.nav.wp.waypoint_tolerance
                dist_to_target = np.linalg.norm(target_pos - agent.pos[0:2])

                # agent arrived at cell; stop setting transient waypoint and loiter
                if dist_to_target <= loiter_radius and agent.nav.is_loitering:
                    if hasattr(agent, "nav") and hasattr(agent.nav, "status"):
                        agent.nav.status = f"LOITERING [{agent.nav.pattern}]({dist_to_target:.1f}m)"

                # agent ha not yet arrived; continue setting transient waypoint
                else:
                    agent.nav.is_loitering = False
                    agent.nav.wp.set_transient_target({
                        "x": target_pos[0],
                        "y": target_pos[1],
                        "name": f"Cell [{target_pos}]"      #TODO: evt naming of cells
                    })

                    agent.viz_target = {"x": target_pos[0],"y": target_pos[1]}
                    if hasattr(agent, "nav") and hasattr(agent.nav, "status"):
                        agent.nav.status = f"Transit: {target_pos[0]:.0f},{target_pos[1]:.0f}"

            elif hasattr(agent, "nav") and hasattr(agent.nav, "status"):
                # Standard behaviour if no cell found/still bidding; continue path
                agent.cmd_heading = getattr(agent, "cmd_heading", agent.psi)
                agent.nav.status = "Fallback: Bidding..."

            agent_nav_msg = str(agent.nav)
            if agent_nav_msg != agent.last_msg:
                agent.last_msg = agent_nav_msg

        sim.tick()

        comm.step_time(sim.Dt)

    plotter.update_plot(callback=simulation_callback)

    