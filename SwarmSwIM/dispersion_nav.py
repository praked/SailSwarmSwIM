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
NEIGHBOR_RADIUS = float(os.environ.get("SWARM_COMM_RADIUS", "5.0"))
DOMAIN_SIZE = float(os.environ.get("SWARMSWIM_DOMAIN", "70.0"))

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

class SocialSteeringController:
    def __init__(
        self,
        domain_size,
        boundary_gain,
        boundary_margin
    ):
        # Domain boundaries
        self.domain_size = float(domain_size)
        self.bound_gain = float(boundary_gain)
        self.bound_margin = float(boundary_margin)

    @staticmethod
    def _norm(v):
        n = np.linalg.norm(v)
        if n < 1e-6:
            return None
        return v / n
    
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
        inner = self.domain_size - self.bound_margin
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
    
    def applyBoundary(self, agent):
        cmd_rad = np.radians(agent.cmd_heading)
        cmd_vec = np.array([np.cos(cmd_rad), np.sin(cmd_rad)])
        my_pos = agent.pos

        # Apply soft-boundary steering if necessary
        bvec = socSteering._boundary_vec(my_pos)     #boundary_gain already applied

        if bvec is not None:
            cmd_vec = cmd_vec + bvec * self.bound_gain
            if hasattr(agent, "nav") and hasattr(agent.nav, "status"):
                        agent.nav.status = f"Avoiding boundary: +{np.degrees(np.arctan2(bvec[1], bvec[0])) % 360.0}°"

        cmd_vec = self._norm(cmd_vec)

        if cmd_vec is None:
            return agent.cmd_heading
        
        soc_heading = np.degrees(np.arctan2(cmd_vec[1], cmd_vec[0])) % 360.0
        

        return soc_heading



class DispersionController:
    """
    Simple dispersion controller dividing the domain into cells.
    Each agent holds a list of all cells, targets are selected and
    negotiated with neighbors via a bidding-system in the step()-method
    """

    def __init__(
        self,
        sim,
        comm,
        domain_size,
    ):
        self.sim = sim
        self.comm = comm
        self.domain_size = float(domain_size)

        # Generate cells dividing the domain for the agents to disperse in (target:= center of a cell)
        self.targets = self._generate_grid_targets(len(sim.agents), self.domain_size)

        # Store the cell one agent wants to loiter around
        self.loiter_cell = {}

        # Priority initially 0 (increases when agent fails to win any cell)
        self.agent_priority = {}

        # Memory-Format: { cell_idx: {'cost': infinity/float, 'winner': None/agent.name, 'winners_prio': int, 'loitering': Bool} }
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
        """Enter specific cell as own cell in own memory"""
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
        """Clear self from cell (reset cell entry) in own memory"""
        my_mem = self.agent_memory[agent.name]

        my_mem[cell_id] = {
            'cost': float('inf'),
            'winner': None,
            'winners_prio': float(0),
            'loitering': False
            }

    def step(self, agent, stepcount=0):
        """
        Compute a target cell for this agent based on current position and neighbor communication.
        Each agent keeps a list of every target cell and its occupant and tries to constantly update it.
        A cell gets won by a bidding system; winner is the agent with the lowest cost (i.e. distance to cell)
        Fallback is a recursion of this step-function (with increased priority and updated internal memory), as well
        as hardcore overwrite and random selection as further fallbacks (should never happen).
        Phases:
            - PHASE 0: Update this agents memory with own distances, own at least one (closest) cell at the end of this step
            - PHASE 1: Bidding: Update internal memory with nbr data (here an agent can loose its selected cell or gain new ones)
            - PHASE 2: Find the nearest won cell and select it as target
            - PHASE 3: Fallback if lost all cells during bidding
            - PHASE 4: Broadcast results (perspective on whole list of cells) via comm module
            - PHASE 5: Return target (or target-cell-id if in recursion)
        """
        my_mem = self.agent_memory[agent.name]
        my_pos = agent.pos[0:2]
        my_prio = float(self.agent_priority[agent.name])
        self_loitering = agent.nav.is_loitering
        loiter_cell = self.loiter_cell[agent.name]
        has_cell = False

        # PHASE 0: Update own memory with own distances, own at least one (closest) cell at the end of this step
        # LESS AGGRESSIVE -> take maximum one cell (has_cell)
        best_cell = None
        best_cost = float('inf')


        if not self_loitering:
            # Currenly not loitering around any cell (update for all entries)
            for cell_id, bid in my_mem.items():
                my_dist = np.linalg.norm(self.targets[cell_id] - my_pos)

                # Update own bid entry (especially also when my_dist increased)
                if bid['winner'] == agent.name:
                    self._take_cell(agent, cell_id, loitering=self_loitering)
                    has_cell = True

                # Keep note of the best cell available to take if I don't have already one
                elif bid['winner'] is None:
                    if my_dist < best_cost - 1e-6:
                        best_cell = cell_id
                        best_cost = my_dist
            
            # If I don't own a cell after cycling through all -> take the closest available one
            if not has_cell:
                if best_cell is not None:
                    self._take_cell(agent, best_cell, loitering=self_loitering)
                    has_cell = True        
        else:
            # Loitering around one cell (update this cell's entry, free up all other OWN entries)
            for cell_id, bid in my_mem.items():

                # Alter only own cells
                if bid['winner'] == agent.name:
                    # Update only loitering cell
                    if cell_id == loiter_cell:
                        self._take_cell(agent, cell_id, loitering=self_loitering)    #keep loitering here
                    # Discard all other owned cells
                    else:
                        self._clear_cell(agent, cell_id)
        

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


        # PHASE 2: Find best winning cell
        my_cells = [c for c, b in my_mem.items() if b['winner'] == agent.name]
        my_cell_idx = None

        # If at least one cell is won, select as target
        if my_cells:
            my_cell_idx = min(my_cells, key=lambda c: my_mem[c]['cost'])

        # Mark chosen target as loitering target
        if my_cell_idx is not None:
            self._take_cell(agent, my_cell_idx, loitering=self_loitering)
            self.loiter_cell[agent.name] = my_cell_idx


        # PHASE 3 (FALLBACK): Bidding fails (lost bid or no cell available)
        if my_cell_idx is None:
            self.agent_priority[agent.name] += 1 #increase priority

            # Do a recursion of step(), effective because own memory is now updated (affects PHASE 0)
            if stepcount < 3:
                # Do a maximum of 3 recursions
                new_stepcount = stepcount + 1
                my_cell_idx = self.step(agent, new_stepcount)

            else:
                # Hardcore-Fallback: overtake a cell though it has a better bidder
                # Goal: overtake the cell where I was closest to winning
                print("Fallback")

                best_cell = None
                best_bid_margin = -1.0 #margin:= diff between best cost and my cost

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
                    self._take_cell(agent, best_cell, loitering=self_loitering)
                    my_cell_idx = best_cell
                else:
                    # Extreme Fallback: choose a random cell to overwrite
                    my_cell_idx = random.randint(0, len(self.targets) - 1 )
                    self._take_cell(agent, my_cell_idx, loitering=self_loitering)
                    print(agent.name, "random choosing", my_cell_idx)


        # PHASE 4: Broadcasting results
        self.comm.broadcast(agent, {'auction_data': my_mem})


        # PHASE 5: Return target (/target-cell-id if in recursion)
        if stepcount == 0:
            if my_cell_idx is not None:
                return self.targets[my_cell_idx]
            return None
        else:
            return my_cell_idx


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

    socSteering = SocialSteeringController(
        domain_size=DOMAIN_SIZE,
        boundary_gain=BOUNDARY_GAIN,
        boundary_margin=BOUNDARY_MARGIN,        
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
            
            # Local wind
            wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)

            # Update sail physics + internal nav clock; but we override nav's heading
            agent.update(wind_vec)

            # Gain target from dispersion controller
            target_pos = dispersion.step(agent, 0)

            if target_pos is not None:
                dist_to_target = np.linalg.norm(target_pos - agent.pos[0:2])

                # agent arrived at cell; stop setting transient waypoint and loiter
                if agent.nav.is_loitering:
                    if hasattr(agent, "nav") and hasattr(agent.nav, "status"):
                        agent.nav.status = f"LOITERING [{agent.nav.pattern}]({dist_to_target:.1f}m)"

                # agent ha not yet arrived; continue setting target as transient waypoint
                else:
                    agent.nav.is_loitering = False
                    agent.nav.wp.set_transient_target({
                        "x": target_pos[0],
                        "y": target_pos[1],
                        "name": f"Cell [{target_pos}]"
                    })

                    agent.viz_target = {"x": target_pos[0],"y": target_pos[1]}
                    if hasattr(agent, "nav") and hasattr(agent.nav, "status"):
                        agent.nav.status = f"Transit: {target_pos[0]:.0f},{target_pos[1]:.0f}"

            elif hasattr(agent, "nav") and hasattr(agent.nav, "status"):
                # Standard behaviour if no cell found/still bidding; continue path
                agent.cmd_heading = getattr(agent, "cmd_heading", agent.psi)
                agent.nav.status = "Fallback: Bidding..."

            # Alter command heading by social steering, eg: if near boundary
            agent.cmd_heading = socSteering.applyBoundary(agent)

            agent_nav_msg = str(agent.nav)
            if agent_nav_msg != agent.last_msg:
                agent.last_msg = agent_nav_msg

        sim.tick()

        comm.step_time(sim.Dt)

    plotter.update_plot(callback=simulation_callback)

    