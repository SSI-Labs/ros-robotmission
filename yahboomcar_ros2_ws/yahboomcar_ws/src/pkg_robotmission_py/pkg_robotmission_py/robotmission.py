import rclpy
from pkg_robotmission_py import ObjectFinder, destroy_node


def main():
    rclpy.init()
    object_finder = ObjectFinder()
    try:
        rclpy.spin(object_finder)
    except KeyboardInterrupt:
        pass
    finally:
        destroy_node(object_finder)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
