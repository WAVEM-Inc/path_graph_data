from .service.pathService import PathService
import rclpy


def main(args=None):
    rclpy.init(args=args)

    service = PathService()
    rclpy.spin(service)

    service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
