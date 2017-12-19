**Model**
===================




## Intrinsic
-------------
$$
P_{e} =  \begin{bmatrix}
-r*cos(\alpha)*sin(\beta) \\
r*cos(\alpha)*cos(\beta) \\
r*sin(\alpha) \\
1
\end{bmatrix}
$$

$$
T_{1} = \begin{bmatrix}
1 & 0 & 0 & L_{2x} \\
0 & cos(tilt + tilt_{offset}) & -sin(tilt+ tilt_{offset}) & L_{2y} \\
0 & sin(tilt + tilt_{offset}) & cos(tilt+tilt_{offset}) & L_{2z} \\
\end{bmatrix}
$$

$$
T_{2} = \begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & L_{1z} \\
\end{bmatrix}
$$

> **Where,** “r” is the radial distance of the beam, “alpha” is multi-layer angle, “beta” is the horizontal angle of the beam, tilt is the angle of the PTU around “x”, “L2x,y,z” are the distances from the scanning point to the tilt joint and “L1z” represents the distance from. The tilt joint to the base of the system.


## Extrinsic
-------------

$$
T_{3} = \begin{bmatrix}
cos(roll) & 0 & sin(roll) & 0 \\
0 & 1 & 0 & 0 \\
-sin(roll) & 0 & cos(roll) & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
T_{4} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & cos(pitch ) & -sin(pitch) & 0 \\
0 & sin(pitch ) & cos(pitch) &  0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
T_{5} = \begin{bmatrix}
cos(yaw) &  -sin(yaw) & 0 & 0 \\
sin(yaw) &  cos(yaw) & 0 & 0 \\
0 &  0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
T_{6} = \begin{bmatrix}
1 &  0 & 0 & Lo_{x} \\
0 & 1 & 0 & Lo_{y} \\
0 & 0 & 1 & Lo_{z} \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
T_{7} = \begin{bmatrix}
cos(\psi) & -sin(\psi) & 0 & off_{x}\\
sin(\psi) &  cos(\psi) & 0 & off_{y} \\
0 &  0 & 1 & 0 \\
0 &  0 & 0 & 1 \\
\end{bmatrix}
$$

> **Where,** “pitch, roll and yaw” represent the possible angular deviation of the PTU respect to the rover (“minnie”) structure around “x”, “y” and “z” respectively.  “Lox”, “Loy” and “Loz” represent the translation distances between the PTU and the reference of the rover. The matrix transformation T7 contents the position information of the rover to fuse different measurements in the same reference frame axis, “offx” and “offy” represent the displacement of the rover and Psi the turns calculated from quaternions of gyroscope.

Finally, the position of the scanned point i, can be calculated as follows:
$$
Point_{(i,x,y,z)} = T_{7}*T_{6}*T_{5}*T_{4}*T_{3}*T_{2}*T_{1}*P_{e}
$$


## Author
* Harold F MURCIA
