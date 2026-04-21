import os
import numpy as np

# Defaults via env, but can be overridden by constructor args
DEFAULT_SAFE_RADIUS = float(os.environ.get("SWARM_CA_SAFE_RADIUS", "4.0"))
DEFAULT_CPA_HORIZON = float(os.environ.get("SWARM_CA_HORIZON", "12.0"))
DEFAULT_AVOID_BIAS_MAX = float(os.environ.get("SWARM_CA_BIAS_MAX", "90.0"))  # deg
DEFAULT_TIE_MODE = os.environ.get("SWARM_CA_TIE_MODE", "colregs").lower()  # deterministic|random|colregs
DEFAULT_DWELL = float(os.environ.get("SWARM_CA_DWELL", "3.0"))
DEFAULT_CA_RADIUS = os.environ.get("SWARM_CA_RADIUS", None)  # None → use comm radius
# Physical collision threshold — matches SWARM_NEAR_RADIUS used by the visual indicator
# so scan_collisions() and the red-agent display fire on the same condition.
_nr = os.environ.get("SWARM_NEAR_RADIUS")
DEFAULT_COLLISION_RADIUS = float(_nr) if _nr else DEFAULT_SAFE_RADIUS / 2.0

class CollisionAvoidance:
    """Heading bias from Closest-Point-of-Approach with symmetry-breaking and hysteresis."""
    def __init__(self, comm,
                 safe_radius=DEFAULT_SAFE_RADIUS,
                 horizon=DEFAULT_CPA_HORIZON,
                 bias_max_deg=DEFAULT_AVOID_BIAS_MAX,
                 tie_mode=DEFAULT_TIE_MODE,
                 dwell=DEFAULT_DWELL,
                 seed=None,
                 ca_radius=None,
                 collision_radius=DEFAULT_COLLISION_RADIUS):
        self.comm = comm
        self.safe_radius = float(safe_radius)
        self.collision_radius = float(collision_radius)  # threshold for scan_collisions()
        self.horizon = float(horizon)
        self.bias_max = np.deg2rad(float(bias_max_deg))
        self.tie_mode = tie_mode
        self.dwell = float(dwell)
        self.seed = int(seed) if seed is not None else 142
        # CA sensing radius: defaults to comm radius if not set
        _env = DEFAULT_CA_RADIUS
        _arg = ca_radius if ca_radius is not None else (_env and float(_env))
        self.ca_radius = float(_arg) if _arg is not None else None
        self.encounters = {}
        self.collision_count = 0            # cumulative unique collision events
        self._colliding_pairs: set = set()  # pairs currently overlapping

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

    def _read_all_poses(self, agent):
        """Return (neighbor, pose) pairs within ca_radius (falls back to comm.radius)."""
        radius = self.ca_radius if self.ca_radius is not None else self.comm.radius
        mx, my = agent.pos[0], agent.pos[1]
        out = []
        for nbr in self.comm.sim.agents:
            if nbr is agent:
                continue
            st = self.comm.posebox.get(nbr.name)
            if st is None:
                continue
            if np.hypot(st["x"] - mx, st["y"] - my) <= radius:
                out.append((nbr, st))
        return out

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
            if sail_choice == 0.0:
                # Stand-on vessel: COLREGS Rule 17(b) — when it becomes apparent
                # the give-way vessel is not acting, the stand-on vessel SHALL take
                # action.  Apply a reduced cooperative bias in the geometry-dictated
                # direction so both vessels open the gap, not just one.
                chosen = base_sign * 0.4
            else:
                chosen = sail_choice  # -1.0: give-way, full avoidance
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
        now = getattr(agent, 'internal_clock', 0.0)
        total_bias_rad = 0.0
        any_conflict = False

        for nbr, st in self._read_all_poses(agent):
            t, d = self._cpa(my_pose, st)
            if not (0.0 <= t <= self.horizon and d < self.safe_radius):
                continue
            any_conflict = True

            # Urgency from predicted CPA distance
            k_d = max(0.0, min(1.0, (self.safe_radius - d) / self.safe_radius))
            # Urgency from current separation (handles already-overlapping agents)
            cur_d = np.hypot(st["x"] - my_pose["x"], st["y"] - my_pose["y"])
            k_now = max(0.0, min(1.0, (self.safe_radius - cur_d) / self.safe_radius)) \
                if cur_d < self.safe_radius else 0.0
            # Time urgency: full when imminent, half at the planning horizon
            k_t = max(0.0, 1.0 - t / self.horizon)
            k = max(k_d, k_now) * (0.5 + 0.5 * k_t)

            sign = self._avoid_sign(my_pose, st, agent.name, nbr.name, now)
            total_bias_rad += sign * k * self.bias_max

        if not any_conflict:
            return desired_heading_deg

        # Clamp aggregate bias to [-bias_max, +bias_max]
        total_bias_rad = max(-self.bias_max, min(self.bias_max, total_bias_rad))
        return (desired_heading_deg + np.rad2deg(total_bias_rad)) % 360.0

    def scan_collisions(self, agents):
        """Detect inter-agent collisions (separation < collision_radius).

        Uses collision_radius (physical overlap threshold, same as SWARM_NEAR_RADIUS)
        rather than safe_radius (the CA avoidance bubble) so the counter matches
        the visual red-agent indicator exactly.
        Counts each pair's first frame inside collision_radius as one event.
        Call once per simulation tick after positions are updated.
        Returns the cumulative collision_count.
        """
        agent_list = list(agents)
        now_colliding = set()
        for i in range(len(agent_list)):
            for j in range(i + 1, len(agent_list)):
                a, b = agent_list[i], agent_list[j]
                d = float(np.hypot(a.pos[0] - b.pos[0], a.pos[1] - b.pos[1]))
                if d < self.collision_radius:
                    key = self._pair_key(a.name, b.name)
                    now_colliding.add(key)
                    if key not in self._colliding_pairs:
                        self.collision_count += 1
        self._colliding_pairs = now_colliding
        return self.collision_count