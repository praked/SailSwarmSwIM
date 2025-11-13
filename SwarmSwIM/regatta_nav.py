import numpy as np
from .sail_extension.wind_plotter import WindPlotter
from SwarmSwIM import Simulator
from pathlib import Path

if __name__ == "__main__":
    base = Path(__file__).resolve().parent
    sim = Simulator(
        1 / 24,
        sim_xml=str(base / "sail_extension" / "regatta.xml"),
    )

    plotter = WindPlotter(
        simulator=sim,
        wind_field=sim.wind_field,
        SIZE=35,
        show_wind=True,
        show_waypoints=True,
    )

    # Opposite course
    #wp = sim.waypoints
    # sim.agents[0].nav.set_waypoint_sequence(sim.waypoints)
    # sim.agents[1].nav.set_waypoint_sequence([wp[0], wp[3], wp[2], wp[1], wp[4]])
    # sim.agents[0].nav.set_waypoint_sequence([wp[0], wp[3], wp[2], wp[1], wp[4]])

    # Parallel course
    for agent in sim.agents:
        if sim.waypoints and len(sim.waypoints) > 0:
            agent.nav.set_waypoint_sequence(sim.waypoints)

    for agent in sim.agents:
        agent.last_msg = None

    def simulation_callback():
        sim.tick()

        for agent in sim.agents:
            execute_control(agent)

            agent_nav_msg = str(agent.nav)
            if agent_nav_msg != agent.last_msg:
                agent.last_msg = agent_nav_msg

    def execute_control(agent):
        wind_vec = sim.wind_field.get_wind_at_position(agent.pos, sim.time)

        agent.update(wind_vec)

        # agent.cmd_fhd(wind_vec, 180, 0)
        # agent.cmd_yawrate = 10

    plotter.update_plot(callback=simulation_callback)
