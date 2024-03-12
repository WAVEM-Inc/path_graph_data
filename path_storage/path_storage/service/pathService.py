from rclpy.node import Node
from ..controller import path as pathCtrl
from ..controller import graph as graphCtrl
from .ResponseService import ResponseService as rspCtrl

from path_graph_msgs.srv import Path
from path_graph_msgs.srv import Graph
from ..config.path_config import PathConfig


class PathService(Node):
    """
    ROS node class for managing route information

    Attributes:
        srv_path: Service Server for path information service
        srv_graph: Service Server for graph information service

    """

    def __init__(self):
        super().__init__("PathService")

        self.get_logger().info("Start storage path data management service")

        conf = PathConfig()
        path_topic = conf.config["CONFIG"]["path_topic"]
        self.srv_path = self.create_service(Path, path_topic, self.get_path_callback)

        graph_topic = conf.config["CONFIG"]["graph_topic"]
        self.srv_graph = self.create_service(
            Graph, graph_topic, self.get_graph_callback
        )

    def get_path_callback(self, request, response):
        """
        Callback function for route information service request

        Args:
            request : service request
            response : service response

        Returns:
            response

        Raises:

        """
        self.get_logger().info(
            "Incoming request latitude: %s longitude: %s start_node: %s end_node: %s "
            % (
                request.position.latitude,
                request.position.longitude,
                request.start_node,
                request.end_node,
            )
        )

        try:
            data, mapId, version = pathCtrl.PathController(self).get_task_path(
                request.position, request.start_node, request.end_node
            )

            if data is None:
                return response

            self.get_logger().info(
                "*GET PATH DATA* "
                + "path id : "
                + data.id
                + ", name : "
                + data.name
                + ", len : "
                + str(len(data.nodeList))
            )

            data = rspCtrl(self).convertResponse(data, response)

            if data is None:
                return response

            response.map_id = mapId
            self.get_logger().info(
                "*RESPONSE DATA* "
                + "path id : "
                + response.path.id
                + ", name : "
                + response.path.name
                # + "\n"
                # + json.dumps(response.path.nodelist)
            )

        except Exception as e:
            self.get_logger().error(
                (
                    "Exception occurred while code execution(get_path_callback): "
                    + str(e)
                )
            )

        return response

    def get_graph_callback(self, request, response):
        """
        Callback function for graph(map) information service request

        Args:
            request : service request
            response : service response

        Returns:
            response

        Raises:

        """
        self.get_logger().info("Incoming request send_id: %s" % (request.send_id))
        try:
            data = graphCtrl.GraphController().get_graph()
            if data is None:
                return response

            jsonStr = rspCtrl(self).makeGraphJson(request.send_id, data)

            response.graph_list = jsonStr

            self.get_logger().info("*RESPONSE DATA* \n" + response.graph_list)

        except Exception as e:
            self.get_logger().error(
                ("Exception occurred while code execution: " + str(e))
            )

        return response


# def main(args=None):
#     rclpy.init(args=args)

#     service = PathService()

#     rclpy.spin(service)

#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
