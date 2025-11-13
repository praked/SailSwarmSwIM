import os
import numpy as np

# Defaults via env, but can be overridden by constructor args
DEFAULT_SAFE_RADIUS = float(os.environ.get("SWARM_CA_SAFE_RADIUS", "4.0"))
DEFAULT_CPA_HORIZON = float(os.environ.get("SWARM_CA_HORIZON", "12.0"))
DEFAULT_AVOID_BIAS_MAX = float(os.environ.get("SWARM_CA_BIAS_MAX", "90.0"))  # deg
DEFAULT_TIE_MODE = os.environ.get("SWARM_CA_TIE_MODE", "colregs").lower()  # deterministic|random|colregs
DEFAULT_DWELL = float(os.environ.get("SWARM_CA_DWELL", "3.0"))

class CollisionAvoidance:
    """Heading bias from Closest-Point-of-Approach with symmetry-breaking and hysteresis."""
    def __init__(self, comm,
                 safe_radius=DEFAULT_SAFE_RADIUS,
                 horizon=DEFAULT_CPA_HORIZON,
                 bias_max_deg=DEFAULT_AVOID_BIAS_MAX,
                 tie_mode=DEFAULT_TIE_MODE,
                 dwell=DEFAULT_DWELL,
                 seed=None):
        self.comm = comm
        self.safe_radius = float(safe_radius)
        self.horizon = float(horizon)
        self.bias_max = np.deg2rad(float(bias_max_deg))
        self.tie_mode = tie_mode
        self.dwell = float(dwell)
        self.seed = int(seed) if seed is not None else 142
        self.encounters = {}

    @staticmethod
    def _pose_of(agent):
        spd = getattr(agent, 'vel', 0.0)
        return {"x": float(agent.pos[0]), "y": float(agent.pos[1]),
                "hdg": float(agent.psi), "spd": float(spd)}

    @staticmethod
    def _vel_xy(pose):
        vx = pose["spd"] * np.cos(np.deg2rad(pose["hdg"]))
        vy = pose["spd"] * np.sin(np.deg2rad(pose["hdg"]))
        return np.array([vx, vy], dtype=float)

    def _cpa(self, my_pose, nb_pose):
        p_i = np.array([my_pose["x"], my_pose["y"]], dtype=float)
        v_i = self._vel_xy(my_pose)
        p_j = np.array([nb_pose["x"], nb_pose["y"]], dtype=float)
        v_j = self._vel_xy(nb_pose)
        r = p_j - p_i
        v = v_j - v_i
        v2 = float(np.dot(v, v))
        if v2 < 1e-6:
            return 0.0, np.linalg.norm(r)
        t_cpa = - float(np.dot(r, v)) / v2
        t_cpa = max(0.0, min(self.horizon, t_cpa))
        d_cpa = np.linalg.norm(r + v * t_cpa)
        return t_cpa, d_cpa

    @staticmethod
    def _heading_diff_deg(h1, h2):
        return abs((h1 - h2 + 180.0) % 360.0 - 180.0)

    @staticmethod
    def _dot(a, b):
        return float(a[0]*b[0] + a[1]*b[1])

    @staticmethod
    def _pair_key(a, b):
        return tuple(sorted((str(a), str(b))))

    def _is_head_on_or_following(self, my_pose, nb_pose):
        dh = self._heading_diff_deg(my_pose["hdg"], nb_pose["hdg"])
        head_on = dh > 160.0
        same_dir = dh < 20.0
        vi = self._vel_xy(my_pose)
        r = np.array([nb_pose["x"] - my_pose["x"], nb_pose["y"] - my_pose["y"]], dtype=float)
        ahead = self._dot(vi, r) > 0.0
        return head_on or (same_dir and ahead)

    def _deterministic_side(self, me_name, nb_name):
        a, b = self._pair_key(me_name, nb_name)
        return +1.0 if me_name == a else -1.0

    def _random_side(self, me_name, nb_name):
        a, b = self._pair_key(me_name, nb_name)
        h = hash((a, b, self.seed))
        return +1.0 if (h & 1) == 0 else -1.0

    def _colregs_side(self):
        """Fallback: power-driven style head-on -> both alter to starboard."""
        return -1.0

    @staticmethod
    def _norm_deg(x):
        return x % 360.0

    @staticmethod
    def _signed_delta(a, b):
        """Return signed smallest angle a-b in degrees in [-180,180]."""
        return ((a - b + 180.0) % 360.0) - 180.0

    @staticmethod
    def _bearing_deg(dx, dy):
        return (np.degrees(np.arctan2(dy, dx)) + 360.0) % 360.0

    def _wind_flow_dir_deg(self, pose):
        """Direction the wind is blowing TOWARD at pose (degrees)."""
        if not hasattr(self.comm, 'sim') or not hasattr(self.comm.sim, 'wind_field'):
            return None
        wx, wy = self.comm.sim.wind_field.get_wind_at_position(
            [pose["x"], pose["y"], 0.0], getattr(self.comm.sim, 'time', 0.0)
        )
        return self._bearing_deg(wx, wy)

    def _upwind_dir_deg(self, pose):
        d = self._wind_flow_dir_deg(pose)
        if d is None:
            return None
        return self._norm_deg(d + 180.0)

    def _tack_side(self, pose):
        """Return 'port' if wind comes from port side, 'starboard' otherwise."""
        up = self._upwind_dir_deg(pose)
        if up is None:
            return None
        delta = self._signed_delta(up, pose["hdg"])  # positive => wind from port
        return 'port' if delta > 0.0 else 'starboard'

    def _is_me_overtaking(self, my_pose, nb_pose):
        """Am I approaching neighbor from abaft their beam (>22.5° abaft)?"""
        brg_nb_to_me = self._bearing_deg(my_pose["x"] - nb_pose["x"], my_pose["y"] - nb_pose["y"])
        diff = abs(self._signed_delta(brg_nb_to_me, nb_pose["hdg"]))
        return diff > 112.5  # within ~135° stern sector

    def _is_nb_overtaking_me(self, my_pose, nb_pose):
        brg_me_to_nb = self._bearing_deg(nb_pose["x"] - my_pose["x"], nb_pose["y"] - my_pose["y"])
        diff = abs(self._signed_delta(brg_me_to_nb, my_pose["hdg"]))
        return diff > 112.5

    def _sail_colregs_side(self, my_pose, nb_pose):
        """Sailing Rule 12 + Rule 13 override.
        Returns:
            -1.0 for starboard alteration (give-way),
             0.0 for stand-on (no alteration),
             None if wind data unavailable.
        """
        up_me = self._upwind_dir_deg(my_pose)
        up_nb = self._upwind_dir_deg(nb_pose)
        if up_me is None or up_nb is None:
            return None

        # Overtaking has priority over sail cross rules
        if self._is_me_overtaking(my_pose, nb_pose):
            return -1.0  # I am give-way
        if self._is_nb_overtaking_me(my_pose, nb_pose):
            return 0.0   # Stand-on

        my_tack = self._tack_side(my_pose)
        nb_tack = self._tack_side(nb_pose)

        if my_tack is None or nb_tack is None:
            return None

        if my_tack != nb_tack:
            # Port gives way to starboard
            return -1.0 if my_tack == 'port' else 0.0

        # Same tack: windward gives way to leeward
        # Determine downwind axis
        flow_dir = self._wind_flow_dir_deg(my_pose)
        ux = np.cos(np.radians(flow_dir))
        uy = np.sin(np.radians(flow_dir))
        r = np.array([my_pose["x"] - nb_pose["x"], my_pose["y"] - nb_pose["y"]], dtype=float)
        down_proj = r[0]*ux + r[1]*uy  # >0 => I am more leeward (downwind)
        if abs(down_proj) < 1e-3:
            # Ambiguous: default to starboard alteration
            return -1.0
        return 0.0 if down_proj > 0.0 else -1.0

    def _avoid_sign(self, my_pose, nb_pose, me_name, nb_name, now):
        r = np.array([nb_pose["x"] - my_pose["x"], nb_pose["y"] - my_pose["y"]], dtype=float)
        v_i = self._vel_xy(my_pose)
        z = r[0]*v_i[1] - r[1]*v_i[0]
        base_sign = 1.0 if z < 0 else -1.0

        key = (me_name, nb_name)
        skey = self._pair_key(me_name, nb_name)
        cached = self.encounters.get((key, skey))
        if cached is not None and now <= cached["expires"]:
            return cached["sign"]

        # Sailing COLREGS first (Rule 12/13)
        sail_choice = self._sail_colregs_side(my_pose, nb_pose)
        if sail_choice is not None:
            chosen = sail_choice
        elif self._is_head_on_or_following(my_pose, nb_pose) or abs(z) < 1e-3:
            if self.tie_mode.startswith("deterministic"):
                chosen = self._deterministic_side(me_name, nb_name)
            elif self.tie_mode.startswith("random"):
                chosen = self._random_side(me_name, nb_name)
            else:
                chosen = self._colregs_side()
        else:
            chosen = base_sign

        self.encounters[(key, skey)] = {"sign": chosen, "expires": now + self.dwell}
        return chosen

    def suggest_heading(self, agent, desired_heading_deg):
        my_pose = self._pose_of(agent)
        conflicts = []
        for nbr, st in self.comm.read_neighbor_poses(agent):
            t, d = self._cpa(my_pose, st)
            if 0.0 <= t <= self.horizon and d < self.safe_radius:
                conflicts.append((nbr, st, t, d))
        if not conflicts:
            return desired_heading_deg

        nbr, st, t, d = min(conflicts, key=lambda x: (x[3], x[2]))
        now = getattr(agent, 'internal_clock', 0.0)
        sign = self._avoid_sign(my_pose, st, agent.name, nbr.name, now)

        k_d = max(0.0, min(1.0, (self.safe_radius - d) / self.safe_radius))
        k_t = max(0.0, min(1.0, (self.horizon - t) / self.horizon))
        k = max(k_d, 0.5 * k_t)
        bias = sign * (k * self.bias_max)
        return (desired_heading_deg + np.rad2deg(bias)) % 360.0