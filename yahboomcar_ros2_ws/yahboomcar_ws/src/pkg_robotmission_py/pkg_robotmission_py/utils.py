from geometry_msgs.msg import Twist

def stop_robot(node):
    twist = Twist()
    node.cmd_vel_pub.publish(twist)
    node.get_logger().info("Robot stopped.")

def destroy_node(node):
    node.get_logger().info("Shutting down. Stopping robot and turning off magnet.")
    stop_robot(node)
    node.turn_magnet_off()
    super(type(node), node).destroy_node()
