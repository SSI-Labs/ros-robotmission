from rclpy.node import Node
from find_object_2d.msg import ObjectsStamped
from std_msgs.msg import Bool,UInt16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

# Electromagnet imports
import gpiozero.pins.lgpio
import lgpio
from gpiozero import OutputDevice
from time import sleep
# At the top of object_finder.py
from pkg_robotmission_py.utils import stop_robot


def __patched_init(self, chip=None):
    gpiozero.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
    chip = 4  # Use gpiochip4 on Pi5
    self._handle = lgpio.gpiochip_open(chip)
    self._chip = chip
    self.pin_class = gpiozero.pins.lgpio.LGPIOPin

gpiozero.pins.lgpio.LGPIOFactory.__init__ = __patched_init


class ObjectFinder(Node):
    def __init__(self):
        super().__init__('object_finder')

        # Subscribe to object detection
        self.subscription = self.create_subscription(ObjectsStamped, '/objectsStamped', self.listener_callback, 10)

        # Subscribe to laser scan to determine distance
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # Subcribe later after object pickup (used to detect red)
        self.sub_camera = None

        # Publisher to control robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_Buzzer = self.create_publisher(UInt16, '/beep', 1)

        # State tracking
        self.target_id = 7
        self.detection_chunk_size = 12
        self.object_detected = False
        self.safe_distance = 0.2  # meters (ADJUSTABLE)
        self.bridge = CvBridge()
        self.red_detected = False
        self.latest_front_distance = None  # Most recent front distance from LIDAR

        # Last known horizontal position of the object
        self.last_obj_x = None
        
        # Electromagnet setup
        self.RELAY_PIN = 4
        try:
            self.relay = OutputDevice(self.RELAY_PIN, active_high=False, initial_value=True)
            self.get_logger().info("Electromagnet relay initialized.")
            self.magnet_on = False
        except Exception as e:
            self.get_logger().error(f"Failed to initialize electromagnet relay: {e}.")
            self.relay = None
            self.magnet_on = False
            
        self.move_forward_initial_distance()

            
    def move_forward_initial_distance(self, distance=2.1, speed=0.25):
        self.get_logger().info(f"Moving forward {distance} meters before finding object...")
        twist = Twist()
        twist.linear.x = speed
        duration = distance / speed  # time = distance / speed

        # Publish movement command
        self.cmd_vel_pub.publish(twist)
        sleep(duration)

        # Stop robot after moving
        stop_robot(self)
        self.get_logger().info("Initial movement complete. Starting object detection...")


    def listener_callback(self, msg):
        data = msg.objects.data
        found = False

        for i in range(0, len(data), self.detection_chunk_size):
            object_id = int(data[i])
            if object_id == self.target_id:
                found = True
                self.object_detected = True

                chunk = data[i:i+self.detection_chunk_size]
                self.get_logger().info(f'Object chunk data: {chunk}')

                obj_x = chunk[9]  # Horizontal pixel coordinate

                self.get_logger().info(f'Object {self.target_id} detected at x={obj_x:.1f}. Centering...')
                self.move_towards_object(obj_x)
                break

        if not found:
            # If object lost but are already close, do not stop
            if self.latest_front_distance is not None and self.latest_front_distance < (self.safe_distance + 0.03):
                self.get_logger().info(f'Object out of sight but within safe distance ({self.latest_front_distance:.2f} m). Continuing...')
                return

            if self.object_detected:
                self.get_logger().info(f'Lost sight of object {self.target_id}. Stopping.')
                self.object_detected = False
                stop_robot(self)


    def laser_callback(self, msg):
        # Check distance to obstacle directly in front
        ranges = list(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        angle_range = 20  # degrees

        center_indices = [
            i for i in range(len(ranges))
            if abs(math.degrees(angle_min + i * angle_increment)) <= angle_range
        ]

        # Filter out NaNs and zero/near-zero readings
        distances = [ranges[i] for i in center_indices if not math.isnan(ranges[i]) and ranges[i] > 0.1]

        if distances:
            self.latest_front_distance = min(distances)
            self.get_logger().info(f"[LIDAR] Front distance: {self.latest_front_distance:.2f} meters")

            if self.object_detected and self.latest_front_distance < self.safe_distance:
                self.get_logger().info(f'Close to object (distance: {self.latest_front_distance:.2f} m). Stopping.')
                stop_robot(self)
                self.turn_magnet_on()
                self.object_detected = False
                self.last_obj_x = None
                self.latest_front_distance = None  # Reset to avoid stale readings
                
                # Move forward for 1 second
                twist = Twist()
                twist.linear.x = 0.15
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                sleep(1)

                # Stop and wait for 2 seconds
                stop_robot(self)
                sleep(2)

                # Move backward for 4.5 seconds
                twist.linear.x = -0.17
                twist.angular.z = 0.4
                self.cmd_vel_pub.publish(twist)
                sleep(4.5)

                # Final stop
                stop_robot(self)
                
                # Subscribe to image_raw to get camera image
                if self.sub_camera is None:
                    self.sub_camera = self.create_subscription(Image, "/image_raw", self.process_camera_image, 2)
                
            else:
                self.latest_front_distance = None


    def move_towards_object(self, obj_x):
        twist = Twist()
        image_center = 330  # Half of 640 px width
        error_x = obj_x - image_center  # Positive if object is right of center

        # Proportional control gain for steering (ADJUSTABLE)
        Kp_angular = 0.0033

        # Deadband: if error is within this range, stop turning
        deadband = 14  # pixels (ADJUSTABLE)
        
        # Adjust angular velocity based on the deadband
        if abs(error_x) > deadband:
            twist.angular.z = -Kp_angular * error_x
            # Keep a base forward speed while turning
            twist.linear.x = 0.11
        else:
            # If within the deadband, stop turning and move straight
            twist.angular.z = 0.0
            # Increase forward speed to move more confidently in a straight line
            twist.linear.x = 0.13

        self.cmd_vel_pub.publish(twist)
        
    
    '''def process_camera_image(self, image_data):
        print('Processing camera image...')
        if not isinstance(image_data, Image):
            return
        try:
            np_img = self.bridge.imgmsg_to_cv2(image_data, desired_encoding="bgr8")
            hsv_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2HSV)
            hsv_img = cv2.GaussianBlur(hsv_img, (5, 5), 0)

            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

            red_pixels = cv2.countNonZero(mask)
            self.red_detected = red_pixels > 500

            if self.red_detected:
                print("Red detected in camera image.")
                self.turn_magnet_off()
                print("BEEP")
                beep = UInt16()
                beep.data = 1
                self.pub_Buzzer.publish(beep)
            else:
                print("No red!")
                beep = UInt16()
                beep.data = 0
                self.pub_Buzzer.publish(beep)
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")
'''
    
    # IF RED IS SPOTTED, CALL THIS METHOD    
    def dropoff_zone(self):
        # go towards the red sign, use LIDAR to stop a certain distance away from it
        # release the elctromagnet to drop object
        # retreat by backing up a few inches
        self.get_logger().info("Checking for red dropoff zone...")

        if self.red_detected:
            self.get_logger().info("Red detected. Turning off electromagnet.")
            self.turn_magnet_off()
        else:
            self.get_logger().warn("Red not detected. Holding object.")

    
    def turn_magnet_on(self):
        if self.relay is not None and not self.magnet_on:
            self.relay.off()
            self.magnet_on = True
            self.get_logger().info("Electromagnet ON.")
        elif self.relay is None:
            self.get_logger().warn("Electromagnet relay not initialized. Cannot turn on.")


    def turn_magnet_off(self):
        if self.relay is not None and self.magnet_on:
            self.relay.on()
            self.magnet_on = False
            self.get_logger().info("Electromagnet OFF.")
        elif self.relay is None:
            self.get_logger().warn("Electromagnet relay not initialized. Cannot turn off.")
