# Classes for calculating the physical forces/disturbances applied to agents in the sim
import numpy as np
import math
from abc import ABC, abstractmethod
from ..agent_class import Agent
from .. import sim_functions
from .sailswarm_toy_forces import (
    SailPolarParams,
    sail_forces,
    calm_water_quadratic_drag,
    added_resistance_waves_simple,
    morison_inline_force,
    total_longitudinal_force,
    AIR_DENSITY,
    WATER_DENSITY,
    GRAVITY,
)


class WindGust:
    """Wind gust generator"""

    def __init__(
        self,
        amplitude=1.0,
        frequency=0.1,
        speed=10.0,
        duration=5.0,
        wavelength=50.0,
    ):
        """
        Initialize wind gust generator

        Args:
            amplitude: Additional wind gust speed in m/s
            speed: Propagation speed in 2-d plane & Additional wind gust speed in m/s
            frequency: Average frequency of gust events in seconds #TODO: correct units
            duration: Duration of gust in seconds # TODO: correct units
            wavelength: Span of gust in meters
        """
        self.amplitude = amplitude
        self.speed = speed
        self.frequency = frequency
        self.duration = duration
        self.wavelength = wavelength

    def get_gust_wind(self, position, time, base_direction=0.0):
        """
        Calculate gust wind vector at a given position and time

        Args:
            position: Position [x, y] in meters
            time: Current simulation time in seconds
            base_direction: Base wind direction in degrees

        Returns:
            gust_vector: np.array([x, y]) in m/s
        """
        if self.amplitude == 0 or self.frequency == 0 or self.duration == 0:
            return np.array([0.0, 0.0])

        # Based on sim_functions:local_waves()
        k = 2 * np.pi / self.wavelength
        w = k * self.speed
        versor = [
            np.cos(np.deg2rad(base_direction)),
            np.sin(np.deg2rad(base_direction)),
        ]
        pos = position[0] * versor[0] + position[1] * versor[1]
        force = self.amplitude * np.sin(w * time - k * pos)
        current = np.array([force * versor[0], force * versor[1]])
        return current


class WindField:
    """Wind field generator with spatial and temporal variations"""

    def __init__(
        self,
        base_wind_speed=5.0,
        base_wind_direction=0.0,
        turbulence_intensity=0.05,
        temporal_frequency=10.0,
        wind_gust=WindGust(
            amplitude=0.0,
        ),
        rng=np.random.default_rng(),
    ):
        """
        Args:
            base_wind_speed: Base wind speed in m/s
            base_wind_direction: Base wind direction in degrees NED convention
            turbulence_intensity: Standard deviation in speed and direction of wind
            temporal_frequency: Rate of change in wind speed/direction in seconds
            wind_gust (optional): WindGust instance
            rng: Random seed
        """
        self.base_wind_speed = base_wind_speed  # m/s
        self.base_wind_direction = base_wind_direction  # degrees
        self.base_wind = vector_from_components(
            self.base_wind_speed, self.base_wind_direction
        )
        self.turbulence_intensity = turbulence_intensity
        self.temporal_frequency = temporal_frequency  # seconds
        self.wind_gust = wind_gust
        self.rng = rng

    def get_wind_at_position(self, position, time):
        """
        Get wind vector at a specific position and time

        Args:
            position: Position [x, y] in meters
            time: Current simulation time in seconds

        Returns:
            wind_vector: np.array([x, y]) in m/s
        """
        if len(position) > 2 and position[2] < 0:
            return np.array(
                [0.0, 0.0]
            )  # no wind force applied to agents below sea level

        # Add spatial variations using vortex field
        # TODO: add vortex component using sim_class.py:VortexField:current_vortex_calculate(agent)
        turbulent_wind = np.array([0.0, 0.0])

        # Add temporal noise
        temporal_noise = self.rng.normal(0, self.turbulence_intensity, 2)
        temporal_noise *= np.sin(2 * np.pi * time / (self.temporal_frequency * 0.3))

        total_wind = (
            self.base_wind
            + turbulent_wind
            + temporal_noise
            + self.wind_gust.get_gust_wind(position, time, self.base_wind_direction)
        )

        return total_wind


class SailingMechanics(ABC):
    """Abstract base class for sailing mechanics calculations"""

    @staticmethod
    @abstractmethod
    def calculate_speed(agent, true_wind):
        """
        Calculate sailing speed based on wind conditions

        Args:
            agent: Agent class
            true_wind: np.array([wind_east, wind_north]) in m/s localized at agent

        Returns:
            float: Target sailing speed in m/s
        """
        raise NotImplementedError()

    @staticmethod
    @abstractmethod
    def calculate_turn_rate(agent, true_wind):
        """
        Calculate maximum turn rate based on sailing conditions

        Args:
            agent: Agent class
            true_wind: np.array([wind_east, wind_north]) in m/s localized at agent

        Returns:
            float: Maximum turn rate in degrees/second
        """
        raise NotImplementedError()


class RealisticSailingMechanics(SailingMechanics):

    @staticmethod
    def calculate_speed(agent, true_wind):
        """WIP: Pranav-based realistic sail force calc"""
        # TODO: move vars to xml instead of hardcode
        # BOAT PARAMETERS
        A_sail = 5  # m^2 (robotic boat)
        A_ref = 0.05  # m^2, proxy frontal area
        C_D_hull = 0.2  # Drag coefficient on hull??

        # BOAT/SIM STATE

        # Get wind magnitude and direction
        wind_magnitude = np.linalg.norm(true_wind)
        wind_direction = np.rad2deg(np.arctan2(true_wind[1], true_wind[0]))
        relative_wind_angle = abs((wind_direction % 360) - (agent.psi % 360))
        relative_wind_angle = min(relative_wind_angle, 360 - relative_wind_angle)
        d = math.dist(
            agent.last_step_planar,
            [agent.pos[0], agent.pos[1]],
        )
        # print(f"{agent.last_step_planar} -> {agent.pos}")
        vel = d / agent.Dt

        V_tw = float(wind_magnitude)  # m/s true wind
        beta_tw = np.deg2rad(relative_wind_angle)  # from starboard bow
        V_boat = vel  # m/s
        V_app, beta_app = apparent_wind(V_tw, beta_tw, V_boat, np.deg2rad(agent.psi))

        alpha = np.deg2rad(8.0)
        CE_h = 0.7
        (Fx, Fy), Mheel = sail_forces(A_sail, AIR_DENSITY, V_app, alpha, beta_app, CE_h)

        # Hydrodynamics
        U = V_boat

        # Simple waves
        a = 0.15  # wave amplitude [m]
        lam = 8.0  # wavelength [m]
        beta_wave_rel = 0.0  # head seas
        RAW = added_resistance_waves_simple(
            U,
            WATER_DENSITY,
            A_waterplane=0.25,
            wave_amp=a,
            wave_length=lam,
            wave_heading_rel=beta_wave_rel,
            k_raw=0.6,
        )
        # RAW = 0.0  # TODO: Temp ignore wave resistance

        F_net = total_longitudinal_force(Fx, U, WATER_DENSITY, C_D_hull, A_ref, RAW)
        agent.cmd_fhd(F_net, agent.psi, 0)

    @staticmethod
    def calculate_turn_rate(agent, true_wind):
        SimplifiedSailingMechanics.calculate_turn_rate(agent, true_wind)


class SimplifiedSailingMechanics(SailingMechanics):
    """Simplified sailing mechanics
    - Speed is based on the relative angle to wind
    - Turn rate is based on speed"""

    @staticmethod
    def calculate_speed(agent, true_wind):
        # TODO: make better formula accounting for sail angle, sail area, drag, etc.
        base_speed = 0.2  # TODO: replace with sail area mod

        wind_magnitude, wind_direction = components_from_vector(true_wind)

        aw = apparent_wind(
            wind_magnitude, np.deg2rad(wind_direction), agent.vel, np.deg2rad(agent.psi)
        )
        aw_mag, aw_dir = aw[0], aw[1]

        # Convert apparent wind direction to relative angle in [0, 180]:
        #   0   = dead downwind (tailwind)
        #   90  = beam reach
        #   180 = dead upwind (irons / in-irons)
        # apparent_wind() returns angle in [0, 360] where 0=downwind, 180=irons.
        # Fold the starboard side [180, 360] onto the port side [0, 180] symmetrically.
        relative_wind_angle = aw[1]
        if relative_wind_angle > 180:
            relative_wind_angle = 360.0 - relative_wind_angle  # mirror to [0, 180]

        # Piecewise speed multiplier based on relative angle
        if relative_wind_angle > 135:  # Irons
            speed_mult_from_wind = max(0.1, 0.8 - ((relative_wind_angle - 135) * 0.04))
        elif relative_wind_angle > 90:  # Perpendicular
            speed_mult_from_wind = 0.8
        elif relative_wind_angle > 45:  # Roughly downwind
            speed_mult_from_wind = 1.0
        else:  # Full down-wind
            speed_mult_from_wind = 0.9

        effective_speed = max(base_speed, (speed_mult_from_wind * aw_mag))
        # print(f"SPEED: {effective_speed}, MULT:{speed_mult_from_wind}, RW: {relative_wind_angle}, AW: {aw[1]}")

        # Update agent pos
        vel_x = effective_speed * np.cos(np.deg2rad(agent.psi))
        vel_y = effective_speed * np.sin(np.deg2rad(agent.psi))

        dX = vel_x * agent.Dt
        dY = vel_y * agent.Dt
        d = math.sqrt(dX**2 + dY**2)
        agent.pos[0] += dX
        agent.pos[1] += dY

    @staticmethod
    def calculate_turn_rate(agent, true_wind):
        # TODO: make better formula
        # Add rudder authority config?
        # Add velocity drag penalty from turning?
        # TODO: this overwrites .xml yawrate_limit, set as % of max instead
        turn_rate = 5 + agent.vel * 5
        agent.yawrate_limit = turn_rate


def vector_from_components(magnitude, direction):
    """Convert magnitude and direction (degrees) of a force to a vector"""
    direction_rad = np.deg2rad(direction)
    return magnitude * np.array(
        [np.cos(direction_rad), np.sin(direction_rad)]  # X component  # Y component
    )


def components_from_vector(vector):
    """Convert a vector [x, y] to a magnitude and direction (degrees)"""
    magnitude = np.linalg.norm(vector)
    if magnitude == 0:
        return 0.0, 0.0

    direction_rad = np.arctan2(vector[1], vector[0])
    direction_deg = np.rad2deg(direction_rad) % 360

    return magnitude, direction_deg


def mathematical_to_nautical_degrees(deg):
    """Shift 0/360-deg East (x-positive), 90-deg North (y-positive) CCW-positive
    to 0/360-deg North (y-positive), 90-deg East (x-positive) CW-positive"""
    return (deg - 90) % 360


def nautical_to_mathematical_degrees(deg):
    return (deg + 90) % 360


def apparent_wind(V_tw: float, beta_tw: float, V_boat: float, psi_boat: float):
    """
    Compute apparent wind speed and direction in the *boat-fixed* frame.

    Args:
        V_tw: true wind speed [m/s] (over ground)
        beta_tw: true wind direction [rad] in earth frame, 0 = +x (east), CCW positive
        V_boat: boat speed over ground [m/s]
        psi_boat: boat heading [rad] in earth frame, 0 = +x (east), CCW positive

    Returns:
        (V_app, beta_app): apparent wind speed [m/s] and angle [deg] relative to boat x-axis (bow).
                           beta_app = 0 -> headwind from straight ahead; +CCW (port).
    """
    # Earth-frame wind vector
    Wx = V_tw * math.cos(beta_tw)
    Wy = V_tw * math.sin(beta_tw)
    # Earth-frame boat velocity
    Bx = V_boat * math.cos(psi_boat)
    By = V_boat * math.sin(psi_boat)
    # Apparent wind in earth frame: W - B
    Ax = Wx - Bx
    Ay = Wy - By
    # Rotate into boat frame (x-forward, y-port)
    c, s = math.cos(psi_boat), math.sin(psi_boat)
    Axb = c * Ax + s * Ay
    Ayb = -s * Ax + c * Ay
    V_app = math.hypot(Axb, Ayb)
    beta_app = math.atan2(Ayb, Axb)  # angle of incoming flow relative to boat x-axis

    # Wrap angle to [-pi, pi]
    x = (beta_app + math.pi) % (2.0 * math.pi)
    x = x - math.pi

    return V_app, np.rad2deg(x) % 360


def relative_wind(agent: Agent, true_wind):
    """
    Compute apparent wind speed and direction in the *boat-fixed* frame.

    Args:
        agent: Agent instance
        true_wind: np.array([wind_north, wind_east]) in m/s

    Returns:
        (V_app, beta_app): apparent wind speed [m/s] and angle [deg] relative to boat x-axis (bow).
                           beta_app = 0 -> headwind from straight ahead; +CCW (port).
    """
    # TODO: deprecate apparent_wind() or this
    V_tw, beta_tw = components_from_vector(true_wind)

    aw = apparent_wind(V_tw, np.deg2rad(beta_tw), agent.vel, np.deg2rad(agent.psi))

    # print(f"TW: {V_tw:.1f}m/s {beta_tw:.1f}deg | AW: {aw[0]:.1f}m/s {aw[1]:.1f}deg")

    # Get wind direction
    aw_mag, aw_dir = aw[0], aw[1]
    return aw_mag, aw_dir

    # Convert [0, 360] to [0, 180] where 0 = downwind, 180=headwind, 90=perpendicular (of agent's current heading)
    # relative_wind_angle = aw_dir
    # if relative_wind_angle > 180:
    #     relative_wind_angle = abs(relative_wind_angle - 360)
    # relative_wind_angle = relative_wind_angle % 180

    # return relative_wind_angle


if __name__ == "__main__":
    pass
