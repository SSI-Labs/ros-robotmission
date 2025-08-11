# ros-robotmission

This project showcases a fully autonomous robot designed to navigate a structured environment, identify and retrieve a specified part from an inventory zone, navigate through a series of obstacles, and deliver the part to a production line dropoff zone. The robot employs a front-mounted Anti-Collision bar that mounts an electromagnet which allows the robot to pick up the part magnetically. The robot begins at its start location, travels through the enclosed space to the inventory zone, and uses the object recognition ROS2 package, [find-object](https://github.com/introlab/find-object), to detect and select the correct target part from a collection of options. The robot navigates to the part, centering it in the camera view, then engages the electromagnnet to pick it up. The robot then navigates to the red dropoff zone using color detection and object avoidance abilities, navigating through both static and dynamic obstacles along the way. After successfully driving to the red dropoff zone, the robot disengages the electromagnet to release the part, and backs up, indicating a successful mission.
Through intelligent navigation, robust part and color recognition, and collision-free transport, the robot demonstrates reliable execution of a real-world material handling task, adapted for use in constrained environments with limited mechanical complexity.


1. Robot identifies object and navigates to it (object recognition)
  - The robot moves forward 2.1 m (adjustable) and stops to scan for the specified object.
  - As it slowly approaches the object, the robot centers it in the camera view and stops 0.2 m from it using LIDAR.

2. Robot turns on electromagnet to pick up object (GPIO pin)
  - The RPi5 GPIO pin sends a low signal through the circuit that controls the electromagnet which turns the magnet on.
  - The robot moves forward for 1 sec to pick up the object (which has a steel bar attached) and backs up to face the obstacles.

3. Robot navigates through a maze/series of obstacles (LIDAR)
  - The robot avoids obstacles that are in the way of the dropoff zone.

4. Robot recognizes landing site and drops off object (color recognition)
  - The robot uses the yahboomcar_astra colorHSV ROS package that comes pre-installed to detect the red dropoffzone (a red folder against a wall), stops 0.2 meters from the folder, and turns the electomagnet off to drop the object.
  - The robot reverses for 1 second.


INSTRUCTIONS TO SETUP AND RUN THE DOCKER CONTAINER
# Allow Docker to connect to your display (ex. RealVNC)
xhost +local:docker

# Run the container with a custom name and image
# Gives the container GPIO pin access, host display access, and mounts the X11 socket to talk to the host
docker run -it --privileged \
  --name my_robot_mission_container \
  --device /dev/gpiochip0 \
  --device /dev/gpiochip1 \
  --device /dev/gpiochip2 \
  --device /dev/gpiochip3 \
  --device /dev/gpiochip4 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  my_robot_mission_image


INSTRUCTIONS TO START THE MISSION
- In three seperate terminals (all in the yahboomcar_ros2_ws/yahboomcar_ws directory) run the following commands:
  Terminal 1
    - ros2 run yahboomcar_astra colorHSV

  Terminal 2
    - colcon build --packages-select find_object_2d
    - source install/setup.bash
    - ros2 run find_object_2d find_object_2d image:=/image_raw --ros-args -p objects_path:=/root/yahboomcar_ros2_ws/yahboomcar_ws/src/find-object/objects

  Terminal 3
    - colcon build --packages-select pkg_robotmission_py
    - source install/setup.bash
    - ros2 run pkg_robotmission_py robotmission
 
