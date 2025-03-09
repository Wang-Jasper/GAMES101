# GAMES101
# Lecture Notes
- Rodrigues' Rotation Formula : $$Rotation\ by\ angle\ \alpha\ around\ axis\ \mathbf{n}$$

$$\mathbf{R}(\mathbf{n}, \alpha) = \cos(\alpha) \mathbf{I} + (1 - \cos(\alpha)) \mathbf{n} \mathbf{n}^T + \sin(\alpha) \underbrace{\left( \begin{array}{ccc}0 & -n\_z & n\_y \\\\n\_z & 0 & -n\_x \\\\-n\_y & n\_x & 0\end{array}\right)}_{\mathbf{N}}$$
- The rotation matrix is orthogonal
## MVP: 
- Model Transform
- View Transform
- Projection Transform:
    - Orthographic Projection
        - cuboid to "canonical" cube [-1, 1]<sup>3<sup>
    - Pespective Projection : Human eye, most common, Furthur objects are smaller
        - frustum to "canonical" cube
## Rasterization