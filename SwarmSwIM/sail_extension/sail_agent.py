import numpy as np
import os
from SwarmSwIM.agent_class import Agent
from .physics import *


class SailAgent(Agent):
    """Extended class for wind propelled agents"""

    def __init__(
        self,
        name,
        Dt=0.1,
        initialPosition=np.array([0.0, 0.0, 0.0]),
        initialHeading=0.0,
        agent_xml="sail_extension/sail_agent_default.xml",
        rng=None,
    ):

        super().__init__(name, Dt, initialPosition, initialHeading, agent_xml, rng)

        # Behavior
        self.nav = TackingNavigation(Dt)  # TODO: make xml config

        # Physics
        self.physics = SimplifiedSailingMechanics()  # TODO: make xml config
        # self.physics = RealisticSailingMechanics()  # TODO: make xml config


        # Heading control for yawrate mode
        # self.max_yawrate = 30.0  # degrees per second
        # self.heading_kp = 2.0  # proportional gain for heading control

    def __str__(self):
        return f"""{self.name}: {self.vel:.1f}m/s | H:{self.psi:.0f}°\n
        {self.nav.status}"""

    def update(self, true_wind):
        # TODO overload Agent.tick but with true wind calculation?
        super().tick()
        self.physics.calculate_speed(self, true_wind)
        self.physics.calculate_turn_rate(self, true_wind)
        self.nav.tick(self, true_wind)


class WaypointPlanner:
    """Manages sequential waypoint navigation and completion tracking"""

    def __init__(self, waypoint_tolerance=15.0):
        """
        Args:
            waypoint_tolerance: distance in meters where an agent is said to have completed a waypoint
        """
        self.waypoint_tolerance = waypoint_tolerance
        self.waypoints = []
        self.current_waypoint_index = 0
        self.viz_target = None

        # Do not advance to next waypoint when True
        self.hold_at_target = False

        # Loitering waypoint (will be set to last waypoint in sequence)
        #self.loitering_waypoint = None

        # Go into loitering behaviour when True
        #self.arrived = False

    @property
    def target_waypoint(self):
        # Problem: no differentiation between no waypoints set and arrived at last waypoint
        if (
            len(self.waypoints) == 0
            or len(self.waypoints) <= self.current_waypoint_index
        ):
            return None
        #if len(self.waypoints) == 0:
            #return None

        #elif len(self.waypoints) <= self.current_waypoint_index:
            #self.arrived = True
            # Set last waypoint as loitering waypoint
            #self.loitering_waypoint = self.waypoints[self.current_waypoint_index - 1]
            #return None

        return self.waypoints[self.current_waypoint_index]

    @target_waypoint.setter
    def target_waypoint(self, waypoint):
        """Set the target waypoint and update current index if needed"""
        if waypoint in self.waypoints:
            self.current_waypoint_index = self.waypoints.index(waypoint)
        else:
            # If waypoint not in list, add it and set as target
            self.waypoints.append(waypoint)
            self.current_waypoint_index = len(self.waypoints) - 1

    def update(self, agent_pos):
        """Readjust target waypoint based on agent position"""
        if self.get_distance_to_current_waypoint(agent_pos) < self.waypoint_tolerance:
            if not self.hold_at_target:
                self.current_waypoint_index += 1


    def set_hold(self, hold: bool):
        """Enable/disable holding at current target waypoint."""
        self.hold_at_target = bool(hold)
    
    def set_waypoint_sequence(self, waypoints):
        """Set a complete sequence of waypoints to navigate through"""
        self.waypoints = waypoints.copy()
        self.current_waypoint_index = 0

    def set_transient_target(self, waypoint):
        """Replace the current sequence with a single, transient target.
        Used by station-keeping to drive a moving virtual waypoint.
        """
        self.waypoints = [waypoint]
        self.current_waypoint_index = 0

    def append_waypoint(self, waypoint):
        """Add a single waypoint to the sequence"""
        self.waypoints.append(waypoint)

    def get_distance_to_current_waypoint(self, agent_pos):
        """Calculate distance from agent to current waypoint"""
        if self.target_waypoint is None:
            return float("inf")

        # Handle both dictionary waypoints and array positions
        if isinstance(self.target_waypoint, dict):
            target_x = self.target_waypoint["x"]
            target_y = self.target_waypoint["y"]
        else:
            # Assume it's an array-like object [x, y, ...]
            target_x = self.target_waypoint[0]
            target_y = self.target_waypoint[1]

        dx = target_x - agent_pos[0]
        dy = target_y - agent_pos[1]
        return np.sqrt(dx**2 + dy**2)

    def clear(self):
        """Clear all waypoints and reset to initial state"""
        self.waypoints = []
        self.current_waypoint_index = 0
        #self.target_waypoint = None

    def __str__(self):
        if self.target_waypoint is not None:
            return f"Navigating to {self.target_waypoint['name']}: ({self.current_waypoint_index}/{len(self.waypoints)} waypoints complete)"
        elif len(self.waypoints) > 0:
            return "All waypoints completed"
        else:
            return "No waypoints set"


class SailNavigation:
    """Navigation behavior tree for waypoint following, station-keeping, etc."""

    def __init__(self, waypoint_tolerance=15.0):
        """Navigation system using composition with WaypointPlanner

        Args:
            waypoint_tolerance: distance in meters where an agent is said to have completed a waypoint
        """
        self.wp = WaypointPlanner(waypoint_tolerance)
        self.status = ""  # Additional text displayed on agent label

    def __str__(self):
        if self.wp.target_waypoint is not None and self.wp.waypoints is not None:
            # Check if target_waypoint is a dictionary with "name" key
            if (
                isinstance(self.wp.target_waypoint, dict)
                and "name" in self.wp.target_waypoint
            ):
                target_name = self.wp.target_waypoint["name"]
            else:
                target_name = "Position"  # Fallback for non-dict targets
            return f"""Navigating to {target_name}: ({self.wp.current_waypoint_index}/{len(self.wp.waypoints)} waypoints complete)"""
        else:
            return f"""Idle: Station keeping"""

    def tick(self, agent, true_wind):
        """Update Navigation decision-making"""
        raise NotImplementedError()

    def set_waypoint_sequence(self, waypoints):
        """Set a sequence of waypoints to navigate through"""
        self.wp.set_waypoint_sequence(waypoints)

    def navigate_to_waypoint(self, agent, true_wind):
        """
        Navigate towards the target waypoint considering wind direction with proper tacking

        Args:
            true_wind: np.array([wind_north, wind_east]) in m/s
        """
        self.wp.update(agent.pos)

        if self.wp.target_waypoint is None:
            return

        return


class TackingNavigation(SailNavigation):
    def __init__(self, Dt=0.1, waypoint_tolerance=15.0, tack_duration=20.0):
        """Tacking navigation

        Args:
            Dt: time subdivision
            waypoint_tolerance: distance in meters where an agent is said to have completed a waypoint
            tack_duration: time in consecutive seconds between tacks
        """
        self.Dt = Dt
        self.current_tack = 1  # 1 for starboard, -1 for port
        self.tack_timer = 0.0
        self.tack_duration = tack_duration
        self.tack_angle = 45.0  # nearest sail direction from oncoming wind

        # --- Station-keeping (figure-eight) parameters ---
        self.loiter_r_min = float(os.environ.get('SWARM_LOITER_R_MIN', '3.0'))
        self.loiter_r_max = float(os.environ.get('SWARM_LOITER_R_MAX', '10.0'))
        self.loiter_wind_gain = float(os.environ.get('SWARM_LOITER_WIND_GAIN', '0.5'))  # m per (m/s)
        self.loiter_omega = float(os.environ.get('SWARM_LOITER_OMEGA', '0.4'))  # rad/s
        self.loiter_center = None  # dict with keys x,y
        self.is_loitering = False # locks agent into loitering mode


        super().__init__(waypoint_tolerance)

    def _capture_last_target_coords(self):
        """Capture loiter target coordinates from last waypoint and return as loiter_center"""

        if len(self.wp.waypoints) > 0:
            last_wp_index = self.wp.current_waypoint_index - 1 #last waypoint
            if last_wp_index >= 0:
                last_wp = self.wp.waypoints[last_wp_index]
                if isinstance(last_wp, dict):
                    return {"x": float(last_wp["x"]), "y": float(last_wp["y"]), "name": "LoiterCenter"}
                return {"x": float(last_wp[0]), "y": float(last_wp["y"]), "name": "LoiterCenter"}
        return None

    def tick(self, agent, true_wind):
        """Update Navigation decision-making"""
        # Ensure waypoint logic is up to date
        self.wp.update(agent.pos)

        # LOITERING logic
        # If loitering, stay in loiter mode
        if self.is_loitering:
            self.station_keep(agent, true_wind)
            return

        # Coming from transient target (sequence of length >= 1) and with no waypoint set into loiter mode
        if self.wp.target_waypoint is None and len(self.wp.waypoints) >= 1:
            center_coords = self._capture_last_target_coords()

            if center_coords is not None:
                # Commit into loitering mode and clear all waypoints
                self.loiter_center = center_coords
                self.is_loitering = True
                self.wp.waypoints = []

                self.station_keep(agent, true_wind)
                return
        
        if self.wp.target_waypoint is not None:
            # Continue waypoint navigation
            self.navigate_to_waypoint(agent, true_wind)
        else:
            # Fallback
            # Maintain current position as target until a real waypoint appears.
            # Store as a dict instead of a numpy array to avoid ambiguous truth-value
            # comparisons inside WaypointPlanner (np.array == np.array -> array of bools).
            self.wp.set_transient_target({
                "x": float(agent.pos[0]),
                "y": float(agent.pos[1]),
                "name": f"{agent.name}_hold",
            })
            agent.viz_target = {"x": float(agent.pos[0]), "y": float(agent.pos[1])}
            return

    def station_keep(self, agent, true_wind):
        """Station-keeping around a FIXED center.

        - The loiter center is captured once (outside) and never moved here.
        - Follow a circle or figure-eight around that center.
        - Radius scales with wind but is clamped.
        - If a leash distance is exceeded, return to center, then resume.
        """
        if self.loiter_center is None:
            return

        # Fixed center (do not modify here)
        cx = float(self.loiter_center["x"]) if isinstance(self.loiter_center, dict) else float(self.loiter_center[0])
        cy = float(self.loiter_center["y"]) if isinstance(self.loiter_center, dict) else float(self.loiter_center[1])

        # Wind-based radius selection
        wnx, wny = float(true_wind[0]), float(true_wind[1])
        wmag = np.hypot(wnx, wny)
        r = self.loiter_r_min + self.loiter_wind_gain * wmag
        r = max(self.loiter_r_min, min(self.loiter_r_max, r))

        # Choose path pattern
        pattern = os.environ.get("SWARM_STATION_PATTERN", "circling").lower()
        t = agent.internal_clock
        theta = self.loiter_omega * t
        if pattern.startswith("circ"):
            # Circle
            x_local = r * np.cos(theta)
            y_local = r * np.sin(theta)
        else:
            # Figure-eight (Gerono lemniscate)
            x_local = r * np.cos(theta)
            y_local = 0.6 * r * np.sin(2.0 * theta)

        # Align loop with wind direction for realism
        wind_dir = np.arctan2(wny, wnx) if wmag > 1e-6 else 0.0
        c, s = np.cos(wind_dir), np.sin(wind_dir)
        dx = c * x_local - s * y_local
        dy = s * x_local + c * y_local

        target_x = cx + dx
        target_y = cy + dy

        # Hard leash: keep vehicle from wandering too far from center
        leash_mult = float(os.environ.get("SWARM_LOITER_LEASH_MULT", "1.75"))
        max_drift = leash_mult * r
        d_center = np.hypot(agent.pos[0] - cx, agent.pos[1] - cy)
        if d_center > max_drift:
            target_x, target_y = cx, cy
            target_name = "Return"
        else:
            target_name = "Loiter"

        # Update visualization target for plotter (non-waypoint overlay)
        agent.viz_target = {"x": float(target_x), "y": float(target_y)}

        # Drive the agent using a transient target, but DO NOT change center
        self.wp.set_transient_target({"name": target_name, "x": target_x, "y": target_y})
        self.status = f"LOITER[{pattern}] r={r:.1f} leash={max_drift:.1f} -> ({target_x:.1f},{target_y:.1f})"

        # Force hold the target, because waypoint_tolerance > Radii
        self.wp.set_hold(True)

        # Command heading toward the transient waypoint using standard tacking logic
        desired_heading = self.calculate_heading(agent, np.array([wnx, wny]))
        if desired_heading is not None:
            agent.cmd_heading = desired_heading % 360


    def calculate_heading(self, agent, true_wind):
        self.wp.update(agent.pos)

        if self.wp.target_waypoint is None:
            return

        # Handle both dictionary waypoints and array positions
        if isinstance(self.wp.target_waypoint, dict):
            target_x = self.wp.target_waypoint["x"]
            target_y = self.wp.target_waypoint["y"]
        else:
            # Assume it's an array-like object [x, y, ...]
            target_x = self.wp.target_waypoint[0]
            target_y = self.wp.target_waypoint[1]

        dx = target_x - agent.pos[0]
        dy = target_y - agent.pos[1]
        desired_heading = np.rad2deg(np.arctan2(dy, dx))

        aw_mag, aw_dir = relative_wind(agent, true_wind)
        # Convert [0, 360] to [0, 180] where 0 = downwind, 180=headwind, 90=perpendicular (of agent's current heading)
        relative_wind_angle = aw_dir
        if relative_wind_angle > 180:
            relative_wind_angle = abs(relative_wind_angle - 360)
        relative_wind_angle = relative_wind_angle % 180

        # [0, 180] (of agent's desired heading angle relative to wind)
        # wind_to_desired_angle = (relative_wind_angle - desired_heading) % 180
        wind_direction = np.rad2deg(np.arctan2(true_wind[1], true_wind[0])) % 360
        wind_to_desired_angle = abs(
            (desired_heading - wind_direction + 180) % 360 - 180
        )

        # Check if sailing upwind (trying to sail within 45deg of upwind direction)
        # Only tack if trying to sail into the wind (upwind), not with the wind (downwind)
        upwind_direction = (wind_direction + 180) % 360  # Direction wind is coming FROM
        upwind_angle = abs((desired_heading - upwind_direction + 180) % 360 - 180)

        if upwind_angle < self.tack_angle:  # Too close to upwind, need to tack
            # Calculate both possible tack headings relative to upwind direction
            port_tack = (upwind_direction + self.tack_angle) % 360
            starboard_tack = (upwind_direction - self.tack_angle) % 360

            # Calculate which tack gets us closer to desired heading
            port_diff = abs((port_tack - desired_heading + 180) % 360 - 180)
            starboard_diff = abs((starboard_tack - desired_heading + 180) % 360 - 180)

            # Choose the better tack and update current tack state
            if port_diff < starboard_diff:
                self.current_tack = 1  # Port tack
                chosen_heading = port_tack
                tack_name = "PORT"
            else:
                self.current_tack = -1  # Starboard tack
                chosen_heading = starboard_tack
                tack_name = "STAR"

            self.status = f"TACK {tack_name} -> {chosen_heading:.0f}° (target: {desired_heading:.0f}°)"
            return chosen_heading
        else:
            # Can sail more directly towards waypoint
            self.status = f"DIRECT -> {desired_heading:.0f}°"
            return desired_heading

    def navigate_to_waypoint(self, agent, true_wind):
        """
        Navigate towards the target waypoint considering wind direction with proper tacking

        Args:
            true_wind: np.array([wind_north, wind_east]) in m/s
        """
        self.wp.update(agent.pos)

        if self.wp.target_waypoint is None:
            return

        desired_heading = self.calculate_heading(agent, true_wind)
        # Expose current nav target for visualization without touching sim.waypoints
        if isinstance(self.wp.target_waypoint, dict):
            agent.viz_target = {
                "x": float(self.wp.target_waypoint["x"]),
                "y": float(self.wp.target_waypoint["y"]),
            }
        else:
            agent.viz_target = {
                "x": float(self.wp.target_waypoint[0]),
                "y": float(self.wp.target_waypoint[1]),
            }
        agent.cmd_heading = desired_heading % 360


def bearing_to(pt1, pt2):
    """Calculate the bearing from pt1 to pt2

    Args:
        pt1 (_type_): [x, y]
        pt2 (_type_): [x, y]
    """
    dx = pt1[0] - pt2[0]
    dy = pt1[1] - pt2[1]
    bearing = np.rad2deg(np.arctan2(dy, dx)) % 360
    return bearing


def bearing_to_target(agent, target_position):
    """Return the relative angle to the target position

    Args:
        agent (_type_): _description_
        target_position (_type_): _description_

    Returns:
        _type_: _description_
    """
    dx = target_position["x"] - agent.pos[0]
    dy = target_position["y"] - agent.pos[1]
    bearing = np.rad2deg(np.arctan2(dy, dx)) % 360
    return bearing


# python3 -m SwarmSwIM.SwarmSwIM.sail_extension.sail_agent
if __name__ == "__main__":
    pass
