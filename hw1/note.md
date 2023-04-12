# 1. 旋转
1. 旋转矩阵：有9个自由度，但实际用到的只有三个，而且无实际意义
2. 欧拉角: R = Rz(angle) * Ry(angle) * Rx(angle)，分别沿着三个轴进行旋转，默认坐标系与物体绑定(就是坐标系随着物体转), x' = R*x
3. 旋转向量：$(\vec{u}, \theta)$ 来表示旋转 $\vec{x'} = \vec{x} + (\sin{\theta}) \vec{u}*\vec{x} + (1-\cos{\theta} \vec{u}*(\vec{u} * \vec{x})$
4. 四元数：
# 2. get_model_matrix
1. 获取的时按照世界坐标的旋转，因为model本身就是将物体对应道世界坐标空间的变化
2. 使用欧拉角实现依据不同的轴进行旋转
3. 根据**任意旋转轴**和**旋转角**得到旋转矩阵：
根据旋转轴和旋转角度推导旋转矩阵的方法是通过罗德里格斯公式（Rodrigues' rotation formula）来实现的。该公式是将旋转轴和旋转角度转化为旋转矩阵的一种方法。假设旋转轴为 $\vec{u} = (u_x, u_y, u_z)$，旋转角度为 $\theta$，则旋转矩阵 $R$ 可以按如下方式计算：
$$R = \cos \theta \cdot I + (1 - \cos \theta) \cdot \vec{u}\vec{u}^T + \sin \theta \cdot
\begin{bmatrix}
0 & -u_z & u_y \\
u_z & 0 & -u_x \\
-u_y & u_x & 0
\end{bmatrix}
$$
   1. 其中 $I$ 是 $3\times 3$ 的单位矩阵，$\vec{u}\vec{u}^T$ 是 $3\times 3$ 的外积矩阵。
   1. 通过罗德里格斯公式，我们可以将旋转轴和旋转角度转化为旋转矩阵，从而方便地进行计算和应用。