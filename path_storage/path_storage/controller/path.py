from rclpy.node import Node
from ..config.path_config import PathConfig
from ..entity.pathSet import PathSet
from ..entity.graphSet import GraphSet
from ..repository.map_load import MapLoad
from ..common import geo
from std_msgs.msg import String


class PathController:
    """
    Class that searches for a path matching a request

    Attributes:
        logger: Object for logging, ROS node class.

    """

    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.FIRST_NODE = "start"
        self.END_NODE = "end"

    def get_task_path(self, startPos, startNode, endNode):
        """Find a route that matches the origin and destination..

        1) Extract route information suitable for the requested task from among the vehicle driving routes created in the route editor.
        2) Returns the saved map ID and version.

        Args:
            startPos: origin node.(longitude/latitude coordinates)
            startNode : origin node.(node id)
            endNode: destination node(node id)

        Returns:
            pathObj : path object ( Class Path )
            mapId : map id
            version : map version

        Raises:

        """
        try:
            if startNode == "" and (
                startPos is None
                or startPos.latitude == 0.0
                or startPos.longitude == 0.0
            ):
                startNode = self._getWorkplaceNodeId()
                self.node.get_logger().info("Searched start node Id : " + startNode)

            # 파일에서 조회한 경로 정보 에서 start,end가 매핑되는 경로를 찾는다.
            data = MapLoad.load_path_file()
            # error code 를 topic으로 전송
            if data is None:
                self._publish(self.node, self.node.err_code_map)
                return None, None, None

            path_info = PathSet(**data)  # path,node,link

            mapId = path_info.mapId
            version = path_info.version
            # 1. end node가 없고 start node의 kind가 "endpoint" 인 경우 마지막 노드가 대기장소인 경로를 조회한다.
            # 1.1  경로 리스트에서 시작노드와 일치하는 경로 리스트를 필터링 한다.
            if endNode == "":
                #  1.2.마지막 노드가 대기장소인 경로를 필터링한다.
                tempPathList = list(
                    filter(
                        lambda pl: self._IsWorkplacePoint(pl.nodeList[-1]),
                        path_info.path,
                    )
                )

                if startNode != "":  # start node id가 있으면
                    tempPathList = self._find_maching_pathlist_byNodeId(  # 1.1.1경로 리스트에서 시작 노드id가 첫번째 노드와 일치 하는 것을 조회
                        startNode, tempPathList, self.FIRST_NODE
                    )
                else:  # start node pos가 있으면
                    tempPathList = self._find_maching_pathlist_byLocation(  # 1.1.2경로 리스트에서 시작 좌표가 첫번째 노드와 일치 하는 것을 조회
                        startPos, tempPathList
                    )

                # if tempPathList:
                #     #  1.2.마지막 노드가 대기장소인 경로를 필터링한다.
                #     tempPathList = list(
                #         filter(
                #             lambda pl: self._IsWorkplacePoint(pl.nodeList[-1]),
                #             tempPathList,
                #         )
                #     )

            else:  # 2.end node가 있으면
                # 2.1.경로 리스트에서 마지막노드와 일치하는 경로 리스트를 필터링 한다.
                tempPathList = self._find_maching_pathlist_byNodeId(
                    endNode, path_info.path, self.END_NODE
                )

                if tempPathList:
                    # 2.2.1. 옵션이 GPS 우선이 아니고 start node id가 있으면
                    if (
                        self._IsGpsPriority() is False and startNode != ""
                    ):  # gps 우선이 아니고 start node id 가 있으면 node id로 시작 노드를 검색
                        tempPathList = self._find_maching_pathlist_byNodeId(
                            startNode, tempPathList, self.FIRST_NODE
                        )
                    else:  # 2.2.2. start positon 이 있으면
                        tempPathList = self._find_maching_pathlist_byLocation(
                            startPos, tempPathList
                        )

        except Exception as e:
            self.node.get_logger().error(  # logger.error(
                ("Exception occurred while code execution(get_task_path): " + str(e))
            )
            # error code 를 topic으로 전송
            self._publish(self.node, self.node.err_code_path)
            return None, None, None

        if not tempPathList:
            self.node.get_logger().error(  # logger.error()
                "No matching route information was found. node number : " + endNode
            )
            # error code 를 topic으로 전송
            self._publish(self.node, self.node.err_code_pos)
            return None, None, None

        # 3. 경로 오브젝트
        pathObj = tempPathList.pop()

        return pathObj, mapId, version

    def _find_maching_pathlist_byNodeId(self, nodeId, pathlist, type):
        """Find paths with matching node IDs in the nodes on the path.

        Finds and returns a route that matches the current location of the vehicle in the route list..

        Args:
            nodeId: start node id.
            pathlist: Path lists for matching searches
            type : Search node type (first node or last node)

        Returns:
            Searched path information(Class Path)

        Raises:

        """
        if type == self.END_NODE:
            result = list(filter(lambda x: self._getLastNodeId(x) == nodeId, pathlist))
        elif type == self.FIRST_NODE:
            result = list(filter(lambda x: self._getFirstNodeId(x) == nodeId, pathlist))
        else:
            self.node.get_logger().error(  # logger.error()
                ("invalide parameter(find_maching_nodeId): type - " + str(type))
            )
            return None

        if not result:
            self.node.get_logger().error(  # logger.error()
                "The path containing the node ID could not be found. "
                + str(type)
                + " node number : "
                + nodeId
            )
            return None

        return result

    def _find_maching_pathlist_byLocation(self, pos, pathlist):
        """Finds the path with coordinates among the nodes in the path.

        Finds and returns a path in the route list whose starting node matches the vehicle's current location.
        Find a path that has the nearest node while remaining within the limit.

        Args:
            pos: location of vehicle.
            pathlist: Path lists for matching searches

        Returns:
            Searched path information(Class Path)

        Raises:

        """

        limit = PathConfig().config["CONFIG"].getfloat("matching_limit")
        limit = 2.0 if limit is None else limit

        distance_min = 10000
        pathId = None
        for pathObj in pathlist:
            distance = geo.getDistanceBetweenPoints(
                pos.latitude,
                pos.longitude,
                pathObj.nodeList[0].position.latitude,
                pathObj.nodeList[0].position.longitude,
            )
            if distance < limit and distance < distance_min:
                pathId = pathObj.id
                distance_min = distance

            # result = list(
            #     filter(
            #         lambda x: geo.getDistanceBetweenPoints(
            #             pos.latitude,
            #             pos.longitude,
            #             x.position.latitude,
            #             x.position.longitude,
            #         )
            #         < limit,
            #         pathObj.nodeList,
            #     )
            # )
            # pathId = pathObj.id
            # if not result:
            #     self.node.get_logger().error(
            #         (
            #             "A point on the route that matches the vehicle's coordinates could not be found. : "
            #             + str(pos)
            #         )
            #     )
            #     return None
        if pathId is None:
            self.node.get_logger().error(
                (
                    "No node matching the coordinates could be found in the path list: "
                    + str(pos.latitude)
                    + "/"
                    + str(pos.longitude)
                )
            )
        else:
            self.node.get_logger().info(
                ("pathId : " + pathId + ",  min distance : " + str(distance_min))
            )
        result = list(filter(lambda x: x.id == pathId, pathlist))

        return result

    def _getLastNodeId(self, pathObj):
        """Find the last node number in the node list.

        Args:
            nodeList: Node list of driving path.

        Returns:
            This is the node ID of the search result for node list data.
            The returned node ID is always a string.

        Raises:

        """
        nodelist = list(map(lambda x: x.nodeId, pathObj.nodeList))
        node = nodelist[-1]
        return node

    def _getFirstNodeId(self, pathObj):
        """Find the first node number in the node list.

        Args:
            nodeList: Node list of driving path.

        Returns:
            This is the node ID of the search result for node list data.
            The returned node ID is always a string.

        Raises:

        """
        nodelist = list(map(lambda x: x.nodeId, pathObj.nodeList))
        node = nodelist[0]
        return node

    def _IsCompletePoint(self, node):
        """Check whether the node property is “work node”

        Args:
            node: Node

        Returns:
            if “work node” then True otherwise false

        Raises:

        """
        nodeCode = "work_node"
        # 환경 파일에 있으면 그것을 사용한다.
        try:
            conf = PathConfig()
            nodeCode = conf.config["DEFINE"]["work_node"]
        except Exception as e:
            self.node.get_logger().error(
                (
                    "The value defined in the configuration file cannot be found. "
                    + str(e)
                )
            )

        if node.type == nodeCode:
            return True

        return False

    def _IsWorkplacePoint(self, node):
        """Check whether the node property is “work place”

        Args:
            node: Node

        Returns:
            if “end point” then True otherwise false

        Raises:

        """

        # 환경 파일에 있으면 그것을 사용한다.
        try:
            conf = PathConfig()
            nodeCode = conf.config["DEFINE"]["workplace_node"]
        except Exception as e:
            self.node.get_logger().error(
                (
                    "The value defined in the configuration file cannot be found. "
                    + str(e)
                )
            )

        if node.type == nodeCode:
            return True

        return False

    def _IsGpsPriority(self):
        """Check whether the node property is “work place”

        Args:
            node: Node

        Returns:
            if “end point” then True otherwise false

        Raises:

        """
        # 환경 파일에 있으면 그것을 사용한다.
        try:
            conf = PathConfig()
            isGpsPriority = conf.config["CONFIG"].getboolean("priority_gps")
        except Exception as e:
            self.node.get_logger().error(
                (
                    "The value defined in the configuration file cannot be found. "
                    + str(e)
                )
            )

        return isGpsPriority

    def _getWorkplaceNodeId(self):
        """find WorkPalce Node ID

        Args:


        Returns:
            work place ndoe id

        Raises:

        """
        result = ""
        try:
            conf = PathConfig()
            nodeCode = conf.config["DEFINE"]["workplace_node"]

            data = MapLoad.load_path_file()
            graph_info = GraphSet(**data)  # path,node,link

            # 경로 정보 에서 노드 헤딩 값을 찾아서 넣는다.
            nlist = list(filter(lambda nd: nd.type == nodeCode, graph_info.node))
            if nlist:
                result = nlist.pop().nodeId

        except Exception as e:
            self.node.get_logger().error(("eror _getWorkplaceNodeID. " + str(e)))

        return result

    def _isSame(self, val1, val2):

        self.node.get_logger().info("_isSame: " + val1 + val2)
        if val1 == val2:
            return True
        else:
            return False

    def _publish(self, node, code):

        msg = String()
        msg.data = code
        node.topic_publisher.publish(msg)
