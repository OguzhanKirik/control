# 🤖 Jacobian Relations in Robot Kinematics & Dynamics

The **Jacobian matrix** `J(q)` is the bridge between **joint space** (angles, velocities, torques)  
and **task space** (end-effector position, velocity, force).  
It defines how motion and forces transform between the robot’s joints and its tool.


| What you start with                          | Multiply by                   | What you get                          | Direction    | Meaning                                          |
| -------------------------------------------- | ----------------------------- | ------------------------------------- | ------------ | ------------------------------------------------ |
| **Joint velocity** ( \dot{q} )               | ( J(q) )                      | **End-effector velocity** ( \dot{x} ) | Joint → Task | How fast the tool moves if joints move           |
| **End-effector velocity** ( \dot{x} )        | ( J^{+}(q) ) (pseudo-inverse) | **Joint velocity** ( \dot{q} )        | Task → Joint | What joint speeds produce that tool motion       |
| **End-effector wrench** (force/torque) ( F ) | ( J^T(q) )                    | **Joint torques** ( \tau )            | Task → Joint | What torques generate that end-effector force    |
| **Joint torques** ( \tau )                   | ( J^{-T}(q) )                 | **End-effector wrench** ( F )         | Joint → Task | What force the end-effector applies to the world |

| Direction                    | Relation                    | Meaning                            |
| ---------------------------- | --------------------------- | ---------------------------------- |
| **Velocity mapping**         | ( \dot{x} = J \dot{q} )     | joint speed → tool speed           |
| **Force mapping**            | ( \tau = J^T F )            | tool force → joint torque          |
| **Inverse velocity mapping** | ( \dot{q} = J^{+} \dot{x} ) | desired tool motion → joint motion |
| **Inverse force mapping**    | ( F = J^{-T} \tau )         | joint torque → tool force          |

