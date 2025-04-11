# cse5280_assignment_robotarmforwardkinematics-Akuskea
I initially developed this project using Jupyter Notebook and later converted it into a standalone Python script. The project utilizes the Vedo and NumPy libraries, along with forward kinematics principles, to simulate a robotic arm.

The robotic arm consists of 4 segments and 5 joints, allowing for 4 controllable angles to manipulate its movement. I implemented a custom function called getTransformation, which computes the local transformation matrices containing both rotation and translation components.

In the main kinematics function, I use getTransformation to determine each joint's local coordinate matrix. These are then combined to calculate the global transformation matrices, which provide the precise global positions of the robotic arm's joints.

The Vedo library is used to visually render the arm in 3D space. Additionally, I implemented a loop function that animates the arm's movement by incrementally adjusting each angle by π/20 radians.

Finally, I added a feature to record the arm’s motion as a video, allowing for easy visualization of the animation.

# Youtube Video of the arm robot:
[https://youtu.be/9LMBftHGDlE ](https://youtu.be/Ez1UVE9CWVs)
