# Igus Rebel Arm Real Time Movement

## Robotics 2

This is gonna have some cool info one day....
hi, changes from my linux

Goal: Mimic human movement with robot arm
Sensor: Azure Kinect Camera
Actuator: Igus Rebel Arm

[https://github.com/TemugeB/joint_angles_calculate/blob/main/README.md](https://github.com/TemugeB/joint_angles_calculate/blob/main/README.md)

[https://github.com/CommonplaceRobotics/iRC_ROS](https://github.com/CommonplaceRobotics/iRC_ROS)



Steps: (Camera )

1. Get Camera Data (big)

   1. Basic laptop camera examples from mediapipe (easy)
   2. Setup joint angle calculation (installation) (easy)
2. Transfer Camera Data into joints for one arm (print joints on console)

   1. get keypoints live into joint calculation,
   2. modify calculate_joint_angles.py to acccept list of parameters in external function
   3. modify calculate_joint_angles.py to only look for 3 landmarks (1 arm and 9 parameters total)
   4. shoulder
   5. elbow
   6. hand
   7. know the robot arm format (just read)
   8. Define data/message format ()
3. Create publisher -> camera (easy)
4. **Get Kinect Camera Data (not easy at all lols point)**
5. Test functionality of joints with Kinect (easy)
6. use same publisher (easy)
7. (Make better Plan)

   1. Define tools used so far
   2. Adapt Plan to new parameters and log changes
   3. Refine points
8. Connect to Robot Arm

   1. Create subscriber for Robot Arm
9. 
10. Map Joints
11. then double it for two arms (maybe funky orientation problems)
12. ROMO-A Fragebogen maybe
13. 14.2.2025 Deadline

Proposal bis ende des Monats
context
vorhaben(plan)
