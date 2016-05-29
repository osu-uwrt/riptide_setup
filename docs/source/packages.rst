Autonomy
--------

The State Machine is the decision making tool that allows computer to control its behavior autonomously. SMACH is one of the packages in the ROS that provides these functions to be used in easier form. In SMACH, each state defines each actions or tasks to be performed. This allows vehicle to execute each task and make transitions smoothly, and it also allows team members to monitor vehicle’s state by vehicle publishing each of its decision. One of the classes in SMACH that was particularly useful was the SimpleActionState, which allows state machine to access nodes through action files. This feature was necessary since the state machine manages and operates specific nodes accordingly.



Controls
--------

The vehicle control stack is comprised of three components: the PID controller, thrust mapper, and thrust calibrator. These components begin with an acceleration vector, provided by the navigation stack, and ultimately activate the thrusters in any combination necessary to achieve said acceleration. The modular form of this system allows for control by either high-level mission planning software or direct user input, which has proven useful for testing purposes.

The PID controller constantly monitors the vehicle’s current acceleration, as reported by onboard sensors, and compares that to the desired acceleration set by the navigation stack. This yields an “error” in acceleration, both linear and angular, which is required by the thrust mapper. The PID’s constant monitoring improves control accuracy and allows for smooth motion.

The thrust mapper solves several physics equations to determine what force each thruster must produce in order to make up the difference in the acceleration error. Success of the mapper relies heavily on the accuracy of the error calculated by the PID controller as well as the mathematical representation of the vehicle as a physical body. The mapper succeeds when it finds a solution for the thrust required by each thruster within certain constraints, such as maximum thrust possible. If no solution exists, the mapper throws and error which is then handled by the rest of the system as needed.

Using the thrusts provided by the thrust mapper, the thrust calibrator transmits the PWM signals activating the thrusters as required. The challenge in developing this component lies in finding which PWM signals actually yield the desired thrust. Being the only component in the stack with a physical output, testing its real values requires significantly more effort than the components with software outputs.




Estimation
----------

Ooh, this is new!




Navigation
----------

The outputs from two LORD Microstrain 3DM-GX4-25 inertial measurement units, mono odometry from the bottom-facing camera, stereo odometry from the forward-facing cameras, and a Blue Robotics Bar30 pressure sensor are inputted into a Kalman filter. This filter then produces estimates for the absolute position, velocity, and acceleration of the vehicle. Orientation data is also obtained from the inertial measurement unit located near the center of mass of the vehicle. These data are used to determine the vehicle’s current state which are used in conjunction with a new desired state and several PID controllers to ascertain required X, Y, and Z acceleration values for the vehicle. The PID controllers are implemented separately for linear accelerations, roll, pitch, and yaw in order to provide the most accurate PID parameters for each case. The desired state of the vehicle is determined from the state machine. These set points can also be provided by a joystick from the surface in the case of tethered movement during the testing phase. This capability allows for easy tuning of the PID controllers and stabilization assessment.



Vision
------

What is email? How do I read it?
