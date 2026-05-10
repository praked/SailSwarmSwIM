# Flocking-Ang-Slow: Mathematical description

A description of the [`flocking_ang_slow`](SwarmSwIM/behaviors/flocking_ang_slow.py) behavior, contrasted with the baseline [`flocking_nav`](SwarmSwIM/behaviors/flocking_nav.py) Couzin implementation. Both run on top of the same `SailAgent` simulator and share collision-avoidance, boundary-steering, and waypoint infrastructure; the difference is purely in how each agent decides its desired *heading* and *speed* given its neighbours.

---

## 1. Notation

Per agent $i$ at simulator tick $t$:

| Symbol | Meaning |
|---|---|
| $\mathbf{x}_i \in \mathbb{R}^2$ | planar position (m) |
| $\psi_i$ | heading (°), measured CCW from $+x$ axis |
| $v_i$ | scalar speed (m/s), exposed as `agent.cur_speed` |
| $s_i \in [0,1]$ | sail-trim factor, exposed as `agent.sail_trim` |
| $\hat{\mathbf{r}}_{ij}$ | unit vector from $i$ to $j$, $\hat{\mathbf{r}}_{ij} = (\mathbf{x}_j - \mathbf{x}_i)/\lVert\mathbf{x}_j - \mathbf{x}_i\rVert$ |
| $d_{ij}$ | distance $\lVert\mathbf{x}_j - \mathbf{x}_i\rVert$ |
| $\mathcal{N}_i$ | set of neighbours of $i$ within `comm radius` |

The three Couzin zones are nested radii $\text{ZOR} < \text{ZOO} < \text{ZOA}$ (repulsion / orientation / attraction).

---

## 2. Baseline — `flocking_nav` (classical Couzin)

Each agent computes a desired heading once per tick. For neighbour $j \in \mathcal{N}_i$ a single contribution is selected by zone:

$$
\mathbf{r}_i = -\!\!\sum_{\substack{j \in \mathcal{N}_i \\ d_{ij} < \text{ZOR}}}\!\hat{\mathbf{r}}_{ij}
\qquad
\mathbf{o}_i = \!\!\sum_{\substack{j \in \mathcal{N}_i \\ \text{ZOR} \le d_{ij} < \text{ZOO}}}\!\hat{\mathbf{e}}(\psi_j)
\qquad
\mathbf{a}_i = \!\!\sum_{\substack{j \in \mathcal{N}_i \\ \text{ZOO} \le d_{ij} < \text{ZOA}}}\!\hat{\mathbf{r}}_{ij}
$$

with $\hat{\mathbf{e}}(\psi) = (\cos\psi, \sin\psi)$.

The social vector is

$$
\mathbf{v}_i^{\text{social}} =
\begin{cases}
\mathbf{r}_i & \text{if } \lVert\mathbf{r}_i\rVert > \varepsilon\ \text{(repulsion overrides everything)} \\
w_{\text{ori}}\, \mathbf{o}_i + w_{\text{att}}\, \mathbf{a}_i & \text{otherwise}
\end{cases}
$$

A boundary-steering term $\mathbf{b}_i$ (a smooth push toward the interior of the square domain) is added when an agent is near a wall:

$$
\mathbf{v}_i = \mathbf{v}_i^{\text{social}} + g_b \, \mathbf{b}_i
$$

The desired heading is

$$
\psi_i^{\star} = \operatorname{atan2}\!\big(v_{i,y},\, v_{i,x}\big) + \mathcal{N}(0, \sigma_\psi^2)
$$

(with a small angular noise to break lattice symmetries), and is then post-processed by a CPA-based collision-avoidance bias before being written to `agent.cmd_heading`. **Speed is not commanded** — the underlying `SimplifiedSailingMechanics` chooses it from the apparent wind alone.

This is the classical Couzin et al. (2002) flocking model, with one wall term added.

---

## 3. `flocking_ang_slow` — heading side: inverse-speed weighting

The orientation and attraction sums are reweighted so that **slower neighbours influence direction more** than faster ones. For each neighbour $j$ define

$$
w_{ij} \;=\; \frac{k}{(v_j + \epsilon)^p}
$$

with three behaviour-level parameters:

| Symbol | Env-var | Meaning |
|---|---|---|
| $k$ | `SWARM_SPD_WT_K` | overall gain (default 1.0) |
| $p$ | `SWARM_SPD_WT_P` | denominator exponent (default 1.0) |
| $\epsilon$ | `SWARM_SPD_EPS` | regulariser preventing $v_j \to 0$ blow-up (default 0.1) |

The orientation and attraction sums become

$$
\mathbf{o}_i = \!\!\sum_{\substack{j \in \mathcal{N}_i \\ \text{ZOR} \le d_{ij} < \text{ZOO}}}\! w_{ij}\, \hat{\mathbf{e}}(\psi_j),
\qquad
\mathbf{a}_i = \!\!\sum_{\substack{j \in \mathcal{N}_i \\ \text{ZOO} \le d_{ij} < \text{ZOA}}}\! w_{ij}\, \hat{\mathbf{r}}_{ij}
$$

The repulsion term keeps unit weights — safety must not depend on neighbour speed.

### Geometric meaning of `k` and `p`

- `p = 0`: weights collapse to a constant; behaviour matches uniform-weight Couzin.
- `p = 1` (default): weight is inversely proportional to neighbour speed — the original "ang-slow" idea.
- `p > 1`: drop-off is steeper; slow neighbours dominate fast ones much more sharply.

`k` is a uniform multiplier on every $w_{ij}$. Because the social vector is normalised before steering ($\mathbf{v}_i \mapsto \mathbf{v}_i/\lVert\mathbf{v}_i\rVert$), a constant $k$ cancels — *except* in the mix with the boundary term $\mathbf{b}_i$. There $k$ effectively controls the relative authority of "stay with the flock" vs "stay inside the domain":
- $k \to 0$: boundary dominates → flock disperses near walls.
- $k \to \infty$: social vector dominates → flock holds together against the boundary.

The blend with the wall term is otherwise identical to the baseline:

$$
\mathbf{v}_i = \mathbf{v}_i^{\text{social}} + g_b \, \mathbf{b}_i
\qquad\text{(repulsion priority and noise as in §2)}
$$

---

## 4. `flocking_ang_slow` — speed side: sail-trim controller

Where flocking_nav does **nothing** to an agent's speed, flocking_ang_slow adds a closed-loop controller that drives a fast agent's actual boat speed toward the local average. The mechanism uses the sail-trim hook in the physics layer (§5).

### Target speed

For agent $i$, take the mean speed of its neighbours:

$$
\bar{v}_i = \frac{1}{|\mathcal{N}_i|} \sum_{j \in \mathcal{N}_i} v_j
\qquad (\text{undefined when }\mathcal{N}_i = \varnothing)
$$

(Neighbours' speeds are read from `cur_speed` in the broadcast pose; no extra communication required.)

### Proportional + rate-limited update

Let $s_i^{(t)}$ be the agent's current sail trim and $K_p$, $\Delta_{\max}$ the controller gains:

$$
\text{err} = v_i - \bar{v}_i, \qquad
s^{\star} =
\begin{cases}
s_i^{(t)} - K_p \dfrac{\text{err}}{\max(\bar{v}_i,\ 10^{-3})} & \mathcal{N}_i \neq \varnothing \\[4pt]
1.0 & \mathcal{N}_i = \varnothing
\end{cases}
$$

$$
\delta = \operatorname{clip}\!\big(s^{\star} - s_i^{(t)},\ -\Delta_{\max},\ \Delta_{\max}\big)
$$

$$
s_i^{(t+1)} = \operatorname{clip}\!\big(s_i^{(t)} + \delta,\ 0,\ 1\big)
$$

| Symbol | Env-var | Default | Role |
|---|---|---|---|
| $K_p$ | `SWARM_TRIM_KP` | 0.4 | controller gain |
| $\Delta_{\max}$ | `SWARM_TRIM_RATE` | 0.05 | per-tick rate limit |

### Why this converges to the slowest sustainable pace

`sail_trim` can only depower (never overpower above the natural max wind-driven speed). So:
- An agent with $v_i > \bar{v}_i$ has $\text{err} > 0$ → trim decreases → boat slows.
- An agent with $v_i < \bar{v}_i$ has $\text{err} < 0$ → trim relaxes toward 1 but is capped there.

The mean drifts down each tick because fast agents are slowing and slow agents cannot speed up beyond their max. The fleet converges to the slowest sustainable pace — the same self-organised behaviour that real-world flotillas exhibit when the leader can't hold the pace.

---

## 5. The sail-trim physics knob

The controller above is meaningless without a way for `agent.sail_trim` to actually alter the boat's speed. This is implemented in [`SimplifiedSailingMechanics.calculate_speed`](SwarmSwIM/sail_extension/physics.py) (the active speed model).

Let $\beta \in [0°, 180°]$ be the relative wind angle in the boat's frame ($0°$ = downwind tailwind, $180°$ = head-on / in irons), and $|w|$ the apparent-wind magnitude. Define:

$$
\text{sensitivity}(\beta) = \left(\frac{\beta}{180°}\right)^{2}
$$

$$
\text{depower}(\beta, s) = 1 - \text{sensitivity}(\beta) \cdot (1 - s)
$$

The effective speed used to advance the agent's position becomes

$$
v_{\text{eff}} = \max\!\Big(\,v_{\text{base}},\ \mu(\beta) \cdot |w| \cdot \text{depower}(\beta, s)\,\Big)
$$

where $\mu(\beta)$ is the existing piecewise polar multiplier (≈ 0.8 / 1.0 / 0.9 / 0.1 for beam reach / broad reach / dead downwind / irons) and $v_{\text{base}} = 0.2$ m/s is a floor that prevents complete stall.

### Why the depower curve is angle-dependent

This mirrors real sailing physics:

| Wind angle | Mechanism | Effect of easing |
|---|---|---|
| Downwind ($\beta \approx 0°$) | Drag-driven | Sail catches wind regardless of fine trim — sensitivity ≈ 0, depower factor ≈ 1.0 even at $s=0$. |
| Beam reach ($\beta \approx 90°$) | Mixed lift/drag | Easing partially luffs the sail — sensitivity ≈ 0.25. |
| Upwind ($\beta \approx 180°$) | Lift-driven | Easing fully luffs the sail — sensitivity ≈ 1.0, $s=0 \Rightarrow$ depower factor 0. |

Concrete depower factors at a few sample $(\beta, s)$ pairs:

| $\beta$ | $s = 1.0$ | $s = 0.5$ | $s = 0.0$ |
|---|---|---|---|
| 0° | 1.000 | 1.000 | 1.000 |
| 45° | 1.000 | 0.969 | 0.938 |
| 90° | 1.000 | 0.875 | 0.750 |
| 135° | 1.000 | 0.719 | 0.438 |
| 180° | 1.000 | 0.500 | 0.000 |

Setting `agent.sail_trim = 1.0` (the default for any agent that doesn't write the attribute) reproduces the unmodified physics exactly, so flocking_nav and any other behaviour that ignores `sail_trim` continues to work as before.

---

## 6. Differences from the baseline at a glance

| Aspect | `flocking_nav` (Couzin) | `flocking_ang_slow` |
|---|---|---|
| Orientation weight | $1$ | $w_{ij} = k / (v_j + \epsilon)^p$ |
| Attraction weight | $1$ | $w_{ij} = k / (v_j + \epsilon)^p$ |
| Repulsion weight | $1$ | $1$ (unchanged — safety invariant) |
| Speed actuation | none — `cmd_heading` only | closed-loop sail-trim controller drives $v_i \to \bar{v}_i$ |
| Physics dependency | uses `SimplifiedSailingMechanics` as-is | uses the same model **plus** the angle-dependent `depower(β, s)` factor |
| Extra state per agent | — | `sail_trim ∈ [0, 1]`, `cur_speed` |
| Extra communication | — | none — `cur_speed` already lives in the existing pose-broadcast |
| Env-var knobs | `SWARM_ZOR/ZOO/ZOA`, `W_ORI`, `W_ATT`, `FLOCK_NOISE`, … (shared with ang-slow) | + `SPD_WT_K`, `SPD_WT_P`, `SPD_EPS`, `TRIM_KP`, `TRIM_RATE` |

### Behavioural intuition

- **Heading-side change** (`w_{ij}`) tilts the swarm's *direction* toward what slow neighbours can sustain. By itself it doesn't slow anyone down; it just bends the flock's course.
- **Speed-side change** (sail-trim controller + angle-dependent depower) actually *slows fast agents down* so the formation stays intact. Crucially, a fast agent on a beat (close-hauled) gets strongly de-powered when it eases the sheet, while a fast agent on a run (downwind) barely changes speed at all — matching how a real sailor would respond.

Together the two mechanisms produce a flock that aligns gently around the slowest neighbours' direction *and* paces itself to their speed — but only when the wind angle gives the trim controller authority to do so.

---

## 7. Parameter cheat-sheet

| Env var | Default | Lives in | Effect |
|---|---|---|---|
| `SWARM_ZOR` | 4.0 m | shared | repulsion radius |
| `SWARM_ZOO` | 10.0 m | shared | orientation radius |
| `SWARM_ZOA` | 18.0 m | shared | attraction radius |
| `SWARM_W_ORI` | 1.0 | shared | orientation weight $w_{\text{ori}}$ |
| `SWARM_W_ATT` | 0.6 | shared | attraction weight $w_{\text{att}}$ |
| `SWARM_FLOCK_NOISE` | 3.0° | shared | heading-noise std-dev $\sigma_\psi$ |
| `SWARM_BOUND_GAIN` | 1.2 | shared | wall-push gain $g_b$ |
| `SWARM_BOUND_MARGIN` | 5.0 m | shared | thickness of the soft wall band |
| `SWARM_SPD_WT_K` | 1.0 | ang_slow only | gain $k$ on inverse-speed weight |
| `SWARM_SPD_WT_P` | 1.0 | ang_slow only | exponent $p$ on $(v + \epsilon)$ |
| `SWARM_SPD_EPS` | 0.1 | ang_slow only | regulariser $\epsilon$ |
| `SWARM_TRIM_KP` | 0.4 | ang_slow only | trim controller gain $K_p$ |
| `SWARM_TRIM_RATE` | 0.05 | ang_slow only | per-tick trim rate limit $\Delta_{\max}$ |
| `SWARM_NEAR_RADIUS` | safe_radius/2 | shared (CA) | physical collision threshold for the counter |

Setting `SWARM_SPD_WT_K = SWARM_SPD_WT_P = 1`, `SWARM_TRIM_KP = SWARM_TRIM_RATE = 0` and `SWARM_USE_SPD_WT = 0` reduces flocking_ang_slow to the same heading dynamics as flocking_nav (the trim controller becomes a no-op too). Useful for ablations.

---

## 8. References

- Couzin, I. D., Krause, J., James, R., Ruxton, G. D., & Franks, N. R. (2002). *Collective memory and spatial sorting in animal groups.* Journal of Theoretical Biology, 218(1), 1–11.
- Source: [`SwarmSwIM/behaviors/flocking_nav.py`](SwarmSwIM/behaviors/flocking_nav.py), [`SwarmSwIM/behaviors/flocking_ang_slow.py`](SwarmSwIM/behaviors/flocking_ang_slow.py), [`SwarmSwIM/sail_extension/physics.py`](SwarmSwIM/sail_extension/physics.py).
