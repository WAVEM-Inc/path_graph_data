from rclpy.node import Node

from ..controller import path as pathCtrl
from ..controller import graph as graphCtrl
from .ResponseService import ResponseService as rspCtrl

from path_graph_msgs.srv import Path
from path_graph_msgs.srv import Graph


class PathService(Node):

    def __init__(self):
        super().__init__("PathService")
        self.get_logger().info("start PathService")
        self.srv = self.create_service(
            Path, "path_graph_msgs/path", self.get_path_callback
        )
        self.srv = self.create_service(
            Graph, "path_graph_msgs/graph", self.get_graph_callback
        )

    def get_path_callback(self, request, response):

        self.get_logger().info(
            "Incoming request latitude: %s longitude: %s end_node: %s "
            % (request.position.latitude, request.position.longitude, request.end_node)
        )

        try:
            data, mapId, version = pathCtrl.PathController().get_task_path(
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

            data = rspCtrl().convertResponse(data, response)

            if data is None:
                return response

            response.map_id = mapId
            self.get_logger().info(
                "*RESPONSE DATA* "
                + "path id : "
                + response.path.id
                + ", name : "
                + response.path.name
                + ", len : "
                + str(len(response.path.nodelist))
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
        self.get_logger().info("Incoming request\nsend_id: %s" % (request.send_id))
        try:
            data = graphCtrl.GraphController().get_graph()
            if data is None:
                return response

            jsonStr = rspCtrl().makeGraphJson(request.send_id, data)

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
