# ME-239-278
ME 239 (Robotic Locomotion) &amp; 278 (Design for the Human Body) Project
Lleyton Elliott et al., UC Berkeley, Fall 2025

Code for a group project shared between two graduate classes. This code runs a wearable assistive sit-to-stand device based on a dynamics model, with multimodal sensor input governing the device's action based on the user's inferred phase of motion.

The model uses three quantities:
1. The orientation of the user's torso in the saggital plane (sensor: IMU)
2. The angle between the user's shin and the user's thigh (sensor: potentiometer)
3. The force applied by the user on the seat (sensor: array of resistive pressure sensors)

These measurements, and their derivatives, are used to determine which phase of motion the user is currently in:
1. Sitting
2. Sit-to-Stand
3. Standing
4. Stand-to-Sit

The sensor data is also used to check for an "emergency" state which immediately overrides the motor control if the user's motion is out of a constraint range.

For the scope of the classes, in lieu of a fully robust dynamic model of the human sitting/standing motion, we opted for a predetermined trajectory based on existing data. Each point along the trajectory corresponds to a state vector (the quantities described above and their derivatives) and a recommended/desired torque to help advance the user to the next step along the trajectory. At each time step, the model estimates where the user is along the trajectory using least-squares optimization, and sets the desired torque accordingly once it infers the position. The "torque" in this case is actually a virtual torque applied near the knee joint. The system uses a linear actuator to assist the user, so we derived an equation to calculate the virtual torque based on the current through the linear actuator's motor.

High-Level Code Block Diagram:

![Block Diagram](images/block_diagram.svg)
