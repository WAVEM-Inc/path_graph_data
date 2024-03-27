from rclpy.node import Node as Logback
from ..entity.path import Path
from ..entity.graphSet import GraphSet
from ..entity.graph_list import GraphList
from ..entity.graph_node import GraphNode
from ..entity.graph import Graph
from ..config.path_config import PathConfig

from route_msgs.msg import Node
from route_msgs.msg import Position
from route_msgs.msg import DetectionRange


class ResponseService:
    """
    Class for generating service response information

    Attributes:
        logger: Object for logging, ROS node class.

    """

    def __init__(self, logback: Logback):
        super().__init__()
        self.logger = logback

    def convertResponse(self, data: Path, response):
        """
        Converts route information into service response format.

        Args:
            data : route information.
            response : service response

        Returns:
            response : Service response information with route information

        Raises:

        """
        response.path.id = data.id
        response.path.name = data.name

        try:
            conf = PathConfig()
            nodeCode = conf.config["DEFINE"]["waitng_node"]

            nodelist = []
            for node in data.nodeList:
                rangelist = []
                if node.kind == nodeCode:
                    for range in node.detectionRange:
                        rangelist.append(
                            DetectionRange(
                                offset=range.offset,
                                # position=Position(
                                #     latitude=range.position.latitude,
                                #     longitude=range.position.longitude,
                                # ),
                                width_left=range.widthLeft,
                                width_right=range.widthRight,
                                height=range.height,
                                action_code=range.actionCode,
                            )
                        )
                if rangelist:
                    nodelist.append(
                        Node(
                            nodeid=node.nodeId,
                            position=Position(
                                latitude=node.position.latitude,
                                longitude=node.position.longitude,
                            ),
                            type=node.type,
                            kind=node.kind,
                            heading=node.heading,
                            direction=node.direction,
                            driving_option=node.drivingOption,
                            rangelist=rangelist,
                        )
                    )
                else:
                    nodelist.append(
                        Node(
                            nodeid=node.nodeId,
                            position=Position(
                                latitude=node.position.latitude,
                                longitude=node.position.longitude,
                            ),
                            type=node.type,
                            kind=node.kind,
                            heading=node.heading,
                            direction=node.direction,
                            driving_option=node.drivingOption,
                        )
                    )
            response.path.nodelist = nodelist

        except Exception as e:
            self.logger.get_logger().error(
                ("Exception occurred while code execution(convertResponse): " + str(e))
            )
            response = None
        return response

    def makeGraphJson(self, send_id, data: GraphSet):
        """
        Convert graph information to JSON string format.

        Args:
            send_id : Elements of response data
            data : graph data

        Returns:
            response : Service response information with route information

        Raises:

        """

        nodelist = []

        try:
            if data.node:
                nodelist = list(
                    map(
                        lambda m: (
                            GraphNode(
                                node_id=m.nodeId,
                                node_name=m.nodeName,
                                x=m.position.longitude,
                                y=m.position.latitude,
                                node_type=m.type,
                                heading=m.heading,
                                critical=False,
                            )
                            if m.nodeName
                            else GraphNode(
                                node_id=m.nodeId,
                                node_name=m.nodeId,
                                x=m.position.longitude,
                                y=m.position.latitude,
                                node_type=m.type,
                                heading=m.heading,
                                critical=False,
                            )
                        ),
                        data.node,
                    )
                )

            graphlist = []
            graphlist.append(
                Graph(
                    map_id=data.mapId,
                    version=data.version,
                    is_indoor=False,
                    node_list=nodelist,
                )
            )

            rbt_graph_list = GraphList(send_id=send_id, graph=graphlist)

        except Exception as e:
            self.logger.get_logger().error(
                ("Exception occurred while code execution: " + str(e))
            )
            return None

        return rbt_graph_list.model_dump_json()

    def makeGraphNodeLsit(self, nodelist, m):
        return GraphNode(
            node_id=m.nodeId,
            node_name=m.nodeId,
            x=m.position.longitude,
            y=m.position.latitude,
            node_type=m.type,
            heading=0,
            critical=False,
        )
