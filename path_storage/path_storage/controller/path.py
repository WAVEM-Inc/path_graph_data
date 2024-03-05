# debug용
from ..config.path_config import PathConfig
from ..entity.pathSet import PathSet
from ..repository.map_load import MapLoad
from ..common import geo

# from ..entity.path import Path

# from config.path_config import PathConfig
# from entity.pathSet import PathSet
# from repository.map_load import MapLoad
# from entity.path import Path
# from common import geo

import logging

logger = logging.getLogger()


class PathController:

    def __init__(self):
        super().__init__()

    def get_task_path(self, startPos, startNode, endNode):

        try:
            # 파일에서 조회한 경로 정보 에서 start,end가 매핑되는 경로를 찾는다.
            data = MapLoad.load_path_file()
            path_info = PathSet(**data)  # path,node,link

            mapId = path_info.mapId
            version = path_info.version
            # 1. drv_path.path에서 마지막 노드가 endNode와 동일한 것을 찾는다.
            templist = list(
                filter(lambda x: self.getLastNodeId(x) == endNode, path_info.path)
            )

            if len(templist) == 0:
                logger.error(
                    "No matching route information was found. node number : " + endNode
                )
                return None, None, None

            # 2. startPos 검색결과 path 리스트의 nodelist에서  매칭되는 path를 찾는다.
            pathObj = self.find_maching_node(startPos, templist)
            if pathObj is None:
                logger.error(
                    (
                        "No matching route information was found. node position : "
                        + str(startPos.latitude)
                        + "/"
                        + str(startPos.longitude)
                    )
                )
                return None, None, None

        except Exception as e:
            logger.error(
                ("Exception occurred while code execution(get_task_path): " + str(e))
            )
            return None, None, None

        return pathObj, mapId, version

    def find_maching_node(self, pos, pathlist):
        """Find the path whose coordinates are in the node list.

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
                logger.info(
                    ("A point matching the vehicle's coordinates could not be found.")
                )
                return None

        result = list(filter(lambda x: x.id == pathId, pathlist)).pop()

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
        val = nodelist[-1]
        return val
