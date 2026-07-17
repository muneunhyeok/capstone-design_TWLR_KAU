# Transformation Matrices - TWLR

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | link1_right |
| $L_{2}$ | link3_right |
| $L_{3}$ | link1_left |
| $L_{4}$ | wheel_right |
| $L_{5}$ | link3_left |
| $L_{6}$ | wheel_left |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | hip_right | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | knee_right | continuous (rad) | $L_{1}$ | $L_{2}$ |
| $q_{3}$ | hip_left | continuous (rad) | $L_{1}$ | $L_{3}$ |
| $q_{4}$ | wheel_right | continuous (rad) | $L_{2}$ | $L_{4}$ |
| $q_{5}$ | knee_left | continuous (rad) | $L_{3}$ | $L_{5}$ |
| $q_{6}$ | wheel_left | continuous (rad) | $L_{5}$ | $L_{6}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [continuous] hip_right (q1)
      L1: link1_right
        |-- [continuous] knee_right (q2)
        |   L2: link3_right
        |     +-- [continuous] wheel_right (q4)
        |         L4: wheel_right
        +-- [continuous] hip_left (q3)
            L3: link1_left
              +-- [continuous] knee_left (q5)
                  L5: link3_left
                    +-- [continuous] wheel_left (q6)
                        L6: wheel_left
```

## Transforms

## hip_right

$L_{0}$ **base_link** -> $L_{1}$ **link1_right** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0.12337, -0.038877, 0.160846) m
- **origin rpy**: (-0.619267, 0, 0) rad
- **axis**: (1, 0, 0)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.12337 \\
0 & 0.814304 & 0.580438 & -0.038877 \\
0 & -0.580438 & 0.814304 & 0.160846 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{1} & -s_{1} & 0 \\
0 & s_{1} & c_{1} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## knee_right

$L_{1}$ **link1_right** -> $L_{2}$ **link3_right** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0, 0.163205, -0.117321) m
- **origin rpy**: (-2.089403, 0, 0) rad
- **axis**: (1, 0, 0)

### Local Transform

$T^{1}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & -0.495671 & 0.868511 & 0.163205 \\
0 & -0.868511 & -0.495671 & -0.117321 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{2} & -s_{2} & 0 \\
0 & s_{2} & c_{2} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## hip_left

$L_{1}$ **link1_right** -> $L_{3}$ **link1_left** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (0.060327, 0, 0) rad
- **axis**: (1, 0, 0)

### Local Transform

$T^{1}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0.998181 & -0.060291 & 0 \\
0 & 0.060291 & 0.998181 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{3} & -s_{3} & 0 \\
0 & s_{3} & c_{3} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## wheel_right

$L_{2}$ **link3_right** -> $L_{4}$ **wheel_right** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (0.039494, -0.006035, 0.007648) m
- **origin rpy**: (-3.141593, 0.991862, 1.570796) rad
- **axis**: (0.309017, 0, -0.951057)

### Local Transform

$T^{2}_{4}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 1 & 0 & 0.039494 \\
0.547132 & 0 & -0.837046 & -0.006035 \\
-0.837046 & 0 & -0.547132 & 0.007648 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & s_{4} & 0 & 0 \\
-s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## knee_left

$L_{3}$ **link1_left** -> $L_{5}$ **link3_left** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (0, 0.155835, -0.126947) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (1, 0, 0)

### Local Transform

$$
T^{3}_{5}(q_{5}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{5} & -s_{5} & 0.155835 \\
0 & s_{5} & c_{5} & -0.126947 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## wheel_left

$L_{5}$ **link3_left** -> $L_{6}$ **wheel_left** (continuous)
  Variable: $q_{6}$

- **origin xyz**: (-0.303192, 0.024704, -0.014133) m
- **origin rpy**: (3.141593, 0, -1.570796) rad
- **axis**: (-0.309017, 0, -0.951057)

### Local Transform

$T^{5}_{6}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & -0.303192 \\
-1 & 0 & 0 & 0.024704 \\
0 & 0 & -1 & -0.014133 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & s_{6} & 0 & 0 \\
-s_{6} & c_{6} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2})\quad (L_0 \to L_{2}: \text{link3_right})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{3})\quad (L_0 \to L_{3}: \text{link1_left})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{4}(q_{4})\quad (L_0 \to L_{4}: \text{wheel_right})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{3}) \cdot T^{3}_{5}(q_{5})\quad (L_0 \to L_{5}: \text{link3_left})$$

$$T^{0}_{6} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{3}) \cdot T^{3}_{5}(q_{5}) \cdot T^{5}_{6}(q_{6})\quad (L_0 \to L_{6}: \text{wheel_left})$$

