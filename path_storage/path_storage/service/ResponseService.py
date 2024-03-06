from rclpy.node import Node as Logback
from ..entity.path import Path
from ..entity.graphSet import GraphSet
from ..entity.graph_list import GraphList
from ..entity.graph_node import GraphNode
from ..entity.graph_edge import GraphEdge
from ..entity.graph import Graph


from path_graph_msgs.msg import Node
from path_graph_msgs.msg import Position
from path_graph_msgs.msg import DetectionRange


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
            nodelist = []
            rangelist = []
            for node in data.nodeList:
                if node.kind == "waiting":
                    for range in node.detectionRange:
                        rangelist.append(
                            DetectionRange(
                                position=Position(
                                    latitude=range.position.latitude,
                                    longitude=range.position.longitude,
                                ),
                                width=range.width,
                                height=range.height,
                                code=range.actionCode,
                            )
                        )
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
                        rangelist=rangelist,
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
        edgelist = []
        try:
            if data.node is not None:
                nodelist = list(
                    map(
                        lambda m: GraphNode(
                            node_id=m.nodeId,
                            node_name=m.nodeId,
                            x=m.position.longitude,
                            y=m.position.latitude,
                            node_type=m.type,
                            heading=0,
                            critical=False,
                        ),
                        data.node,
                    )
                )

            if data.link is not None:
                edgelist = list(
                    map(
                        lambda m: GraphEdge(
                            source=m.stNode, target=m.edNode, directional=True
                        ),
                        data.link,
                    )
                )

            graphlist = []
            graphlist.append(
                Graph(
                    map_id=data.mapId,
                    version=data.version,
                    is_indoor=False,
                    node_list=nodelist,
                    edge_list=edgelist,
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
