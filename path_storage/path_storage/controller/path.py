from rclpy.node import Node
from ..config.path_config import PathConfig
from ..entity.pathSet import PathSet
from ..repository.map_load import MapLoad
from ..common import geo


class PathController:
    """
    Class that searches for a path matching a request

    Attributes:
        logger: Object for logging, ROS node class.

    """

    def __init__(self, node: Node):
        super().__init__()
        self.logger = node
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

            if startNode != "" and startPos is None:
                self.logger.get_logger().error(  # logger.error(
                    (
                        "invalide parameter(get_task_path): startNode - "
                        + str(startNode)
                        + ", startPos - "
                        + str(startPos)
                    )
                )
                return None, None, None

            # 파일에서 조회한 경로 정보 에서 start,end가 매핑되는 경로를 찾는다.
            data = MapLoad.load_path_file()
            path_info = PathSet(**data)  # path,node,link

            mapId = path_info.mapId
            version = path_info.version
            # 1. end node가 없고 start node의 kind가 "endpoint" 인 경우 마지막 노드가 대기장소인 경로를 조회한다.
            # 1.1  경로 리스트에서 시작노드와 일치하는 경로 리스트를 필터링 한다.
            if endNode == "":
                if startNode != "":  # start node id가 있으면
                    tempPathList = self.find_maching_nodeId(  # 1.1.1경로 리스트에서 시작 노드id가 첫번째 노드와 일치 하는 것을 조회
                        startNode, path_info.path, self.FIRST_NODE
                    )
                else:  # start node pos가 있으면
                    tempPathList = self.find_maching_node(  # 1.1.2경로 리스트에서 시작 좌표가 첫번째 노드와 일치 하는 것을 조회
                        startPos, path_info.path
                    )

                if (
                    tempPathList is None
                    or len(tempPathList) == 0
                    or self.IsCompletePoint(tempPathList[0].nodeList[0]) is False
                ):  # 시작 노드가 end point 가 아니면
                    self.logger.get_logger().error(  # logger.error()
                        "No matching route information was found. node number : "
                        + startNode
                        + "/"
                        + endNode
                    )
                    return None, None, None

                #  1.2.마지막 노드가 대기장소인 경로를 필터링한다.
                tempPathList = list(
                    filter(
                        lambda pl: self.IsWorkplacePoint(pl.nodeList[-1]), tempPathList
                    )
                )

            else:  # 2.end node가 있으면
                # 2.1.경로 리스트에서 마지막노드와 일치하는 경로 리스트를 필터링 한다.
                tempPathList = self.find_maching_nodeId(
                    endNode, path_info.path, self.END_NODE
                )

                if tempPathList is None or len(tempPathList) == 0:
                    self.logger.get_logger().error(  # logger.error()
                        "No matching route information was found. node number : "
                        + endNode
                    )
                    return None, None, None

                # 2.2.1. 옵션이 GPS 우선이 아니고 start node id가 있으면
                if (
                    self.IsGpsPriority() is False and startNode != ""
                ):  # gps 우선이 아니고 start node id 가 있으면 node id로 시작 노드를 검색
                    tempPathList = self.find_maching_nodeId(
                        startNode, tempPathList, self.FIRST_NODE
                    )
                else:  # 2.2.2. start positon 이 있으면
                    tempPathList = self.find_maching_node(startPos, tempPathList)

            if tempPathList is None or len(tempPathList) == 0:
                self.logger.get_logger().error(  # logger.error()
                    "No matching route information was found. node number : " + endNode
                )
                return None, None, None

        except Exception as e:
            self.logger.get_logger().error(  # logger.error(
                ("Exception occurred while code execution(get_task_path): " + str(e))
            )
            return None, None, None

        # 3. 경로 오브젝트
        pathObj = tempPathList.pop()

        return pathObj, mapId, version

    def find_maching_nodeId(self, nodeId, pathlist, type):
        """Find paths with matching node IDs in the nodes on the path.

        Finds and returns a route that matches the current location of the vehicle in the route list..

        Args:
            pos: location of vehicle.
            pathlist: Path lists for matching searches
            type : Search node type (first node or last node)

        Returns:
            Searched path information(Class Path)

        Raises:

        """
        if type == self.END_NODE:
            result = list(filter(lambda x: self.getLastNodeId(x) == nodeId, pathlist))
        elif type == self.FIRST_NODE:
            result = list(filter(lambda x: self.getFirstNodeId(x) == nodeId, pathlist))
        else:
            self.logger.get_logger().error(  # logger.error()
                ("invalide parameter(find_maching_nodeId): type - " + str(type))
            )
            return None

        if len(result) == 0:
            self.logger.get_logger().error(  # logger.error()
                "No matching route information was found. node number : " + nodeId
            )
            return None

        return result

    def find_maching_node(self, pos, pathlist):
        """Finds the path with coordinates among the nodes in the path.

        Finds and returns a route that matches the current location of the vehicle in the route list..

        Args:
            pos: location of vehicle.
            pathlist: Path lists for matching searches

        Returns:
            Searched path information(Class Path)

        Raises:

        """

        limit = float(PathConfig().config["CONFIG"]["matching_limit"])

        pathId = None
        for pathObj in pathlist:
            ret = list(
                filter(
                    lambda x: geo.getDistanceBetweenPoints(
                        pos.latitude,
                        pos.longitude,
                        x.position.latitude,
                        x.position.longitude,
                    )
                    < limit,
                    pathObj.nodeList,
                )
            )
            pathId = pathObj.id
            if len(ret) == 0:
                self.logger.get_logger().error(
                    (
                        "A point on the route that matches the vehicle's coordinates could not be found."
                    )
                )
                return None

        result = list(filter(lambda x: x.id == pathId, pathlist))

        return result

    def isSame(self, x1, x2):
        return x1 == x2

    def getLastNodeId(self, pathObj):
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

    def getFirstNodeId(self, pathObj):
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

    def IsCompletePoint(self, node):
        """Check whether the node property is “end point”

        Args:
            node: Node

        Returns:
            if “end point” then True otherwise false

        Raises:

        """
        nodeCode = "endpoint"
        # 환경 파일에 있으면 그것을 사용한다.
        try:
            conf = PathConfig()
            nodeCode = conf.config["DEFINE"]["complete_node"]
        except Exception as e:
            self.logger.get_logger().error(
                (
                    "The value defined in the configuration file cannot be found. "
                    + str(e)
                )
            )

        if node.kind == nodeCode:
            return True

        return False

    def IsWorkplacePoint(self, node):
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
            self.logger.get_logger().error(
                (
                    "The value defined in the configuration file cannot be found. "
                    + str(e)
                )
            )

        if node.type == nodeCode:
            return True

        return False

    def IsGpsPriority(self):
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
            self.logger.get_logger().error(
                (
                    "The value defined in the configuration file cannot be found. "
                    + str(e)
                )
            )

        return isGpsPriority
