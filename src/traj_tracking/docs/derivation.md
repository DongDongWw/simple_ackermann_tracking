# Trajectory Tracking using MPC of Ackermann Model

## 1 建模
### 1.1 控制变量 & 状态变量
$$
\begin{align*}
& \mathbf{x} = (x, y, v, \theta) \\
& \mathbf{u} = (a, \omega)
\end{align*}
$$
### 1.2 MPC范式
- 代价函数：一般代价和终端代价，一般代价由 $x, y, v, \theta$与参考状态的误差项与$a, \omega$的二次项构成，终端代价由终端代价的误差项构成
- 约束
  - 等式约束：运动学模型（线性模型，简化对运动学模型的线性化处理）+ $\omega, v$到左右前轮转角的转换
  - 不等式约束：速度范围，加速度范围，左右前轮转角范围
$$
\begin{align*}
& \mathbf{J} = (x_N-x_{r,N})^TP_N(x_N-x_{r,N}) + \sum_{i=0}^{N-1}{\{(x_i-x_{r,i})^TP_i(x_i-x_{r,i})+u_i^TQ_iu_i\}} \\
& \begin{align*}
    \mathrm{s.t.} &\\
    &\begin{align}
        & \qquad \begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{v} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} v\cos{\theta} \\ v\sin{\theta} \\ a \\ \omega \end{bmatrix} \hspace{4cm} \\
        & \qquad \tan{\varphi_l} = \frac{2\omega l}{2v-b\omega} \\
        & \qquad \tan{\varphi_r} = \frac{2\omega l}{2v+b\omega} \\
        & \qquad l_{left} \le \tan{\varphi_l} \le u_{left} \\
        & \qquad l_{right} \le \tan{\varphi_r} \le u_{right} \\
        & \qquad v_{lb} \le v \le v_{ub} \\
        & \qquad a_{lb} \le a \le a_{ub} \\
    \end{align}
\end{align*}
\end{align*}
$$

### 1.3 线性化离散化
运动学公式离散化：
$$
\begin{align*}
& \begin{bmatrix} x_{k+1} \\ y_{k+1} \\ \theta_{k + 1} \\ v_{k + 1} \end{bmatrix} =
 (E+\Delta{t}\frac{\partial{f}}{\partial{x}}^T)x_k + \Delta{t}\frac{\partial{f}}{\partial{u}}^Tu_k +
 \Delta{t}(f(x_{r,k}, u_{r,k}) - \frac{\partial{f}}{\partial{x}}^Tx_{r,k}-\frac{\partial{f}}{\partial{u}}^Tu_{r,k}) \\
& \frac{\partial{f}}{\partial{x}} =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
-\Delta{t}v_{r,k}\sin{\theta_{r,k}} & \Delta{t}v_{r,k}\cos{\theta_{r,k}} & 1 & 0 \\
\Delta{t}\cos{\theta_{t,k}} & \Delta{t}\sin{\theta_{t,k}} & 0 & 1
\end{bmatrix} \\
& \frac{\partial{f}}{\partial{u}} =
\begin{bmatrix}
0 & 0 & \Delta{t} & 0 \\
0 & 0 & 0 & \Delta{t}
\end{bmatrix}
\end{align*}
$$
左右前轮旋转角度约束：
$$
\begin{align*}
\begin{bmatrix}
0 & 0 & 0 & 2l_{left} \\
0 & 0 & 0 & -2u_{left} \\
0 & 0 & 0 & 2l_{right} \\
0 & 0 & 0 & -2u_{right} \\
\end{bmatrix} \mathrm{x_k} +
\begin{bmatrix}
-(2l_{left}+bl_{left}) & 0 \\
(2u_{left}+bu_{left}) & 0 \\
-(2l_{right}-bl_{right}) & 0 \\
(2u_{right}-bu_{right}) & 0 \\
\end{bmatrix}\mathrm{u_k} \le 0
\end{align*}
$$

## 2 求解
使用tinyMPC求解器对该问题进行求解，tinyMPC结合了ADMM和LQR方法，适用于嵌入式计算平台。它可以处理输入、状态范围和二阶锥约束的问题，因此需要将左右前轮转角约束通过引入松弛变量转换成标准形式




