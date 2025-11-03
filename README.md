
Usage of Jacobi Matrix

| # | Purpose                     | Equation                       | Example Outcome                                            |
| - | --------------------------- | ------------------------------ | ---------------------------------------------------------- |
| 1 | Velocity mapping            | ( \dot{x} = J \dot{q} )        | Joint speeds → (–0.655, 0.533) m/s                         |
| 2 | Inverse velocity kinematics | ( \dot{q} = J^{+} \dot{x} )    | End-effector (0.2, 0.1) m/s → joints (0.013, –0.542) rad/s |
| 3 | Force/torque mapping        | ( \tau = J^T F )               | 10 N in X → (–9.83, –4.84) Nm                              |
| 4 | Singularity detection       | ( \det(J) = L_1 L_2 \sin q_2 ) | Singular at ( q_2 = 0°, 180° )                             |
| 5 | Dynamics coupling           | ( J^T F_{ext} )                | 5 N in X → extra torques (–4.9, –2.4) Nm                   |



| Aspect          | **Velocity-Based (Differential IK)**                 | **Position-Based (IK Solver)**                   |
| :-------------- | :--------------------------------------------------- | :----------------------------------------------- |
| **Input goal**  | Cartesian **velocity** or continuous position stream | Cartesian **pose** (target position/orientation) |
| **Output**      | Joint **velocities** (integrated to positions)       | Joint **angles** directly                        |
| **Math**        | ( \dot{x} = J(q)\dot{q} )  → pseudoinverse           | ( x = f(q) ) nonlinear → iterative solver        |
| **Speed**       | Real-time, continuous updates (fast)                 | Slower (iterative convergence)                   |
| **Usage**       | Cartesian servoing, path following                   | Point-to-point Cartesian motion                  |
| **Robustness**  | Sensitive to integration drift                       | Robust to drift (solves absolute pose)           |
| **Precision**   | Good for local motions                               | Good for exact goal reaching                     |
| **Typical use** | Teleoperation, impedance control                     | Planning, Cartesian goal moves                   |



| Layer                               | Uses                            | Model info   | PID used for                     | Typical Output |
| ----------------------------------- | ------------------------------- | ------------ | -------------------------------- | -------------- |
| **PID only**                        | servo joints                    | none         | regulate individual errors       | τ or q         |
| **Kinematic + PID**                 | geometry-based Cartesian motion | Jacobian     | smooth position/velocity control | q̇             |
| **Dynamic + PID (computed torque)** | physics-based tracking          | (M, C, G)    | linearize error dynamics         | τ              |
| **Task-space Impedance (PID-like)** | compliance                      | (J, M, C, G) | desired stiffness/damping        | τ              |
