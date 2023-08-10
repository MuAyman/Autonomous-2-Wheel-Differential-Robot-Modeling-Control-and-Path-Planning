# Autonomous 2-Wheel Differential Robot Modeling Control and Path Planning
 
The autonomous 2-wheel differential robot has emerged as a versatile and flexible platform with numerous applications in diverse fields. Its ability to navigate through complex environments and perform intricate tasks makes it a valuable asset in industrial automation, surveillance, and search and rescue operations. In this project, we present a comprehensive study focused on the modeling, control, and path planning of such a robot using Simulink.

In the first phase of the project, we develop a precise kinematics state-space model of the 2-wheel differential robot. To achieve precise and robust control, we design and implement two distinct control strategies: a state feedback controller and a Lyapunov controller. The state feedback controller relies on a linearized version of the developed model, utilizing feedback information from the robot's actual state error and desired states (position and orientation) to regulate the robot's motion and ensure delivery and stability. Conversely, the Lyapunov controller ensures asymptotic stability by employing developed Lyapunov functions for the nonlinear state space model. Subsequently, simulations with various inputs are conducted to analyze behavior and validate the accuracy and reliability of the controlled robot model.

Furthermore, we integrate an Artificial Potential Field (APF) path planning and obstacle avoidance algorithm into the control system. The APF algorithm empowers the robot to detect and evade obstacles in its environment while planning a collision-free path toward a desired goal. Through the incorporation of this algorithm, the robot demonstrates enhanced autonomy and safety during navigation. To bolster the system's reliability and reject noisy sensor data, we leverage an Extended Kalman Filter (EKF). The EKF conducts sensor fusion and estimation, thereby enabling accurate localization and robust control. By adeptly filtering sensor measurements, the EKF significantly bolsters the overall performance and reliability of the autonomous 2-wheel differential robot.

In summary, this project encompasses the modeling and simulation of a 2-wheel differential robot, the design and implementation of state feedback and Lyapunov controllers, the integration of an APF path planning and obstacle avoidance algorithm, and the utilization of an extended Kalman filter for reliable localization and noise immunity. These contributions collectively aim to elevate the precision, autonomy, and robustness of the autonomous 2-wheel differential robot, thereby facilitating its applications across various real-world scenarios.
