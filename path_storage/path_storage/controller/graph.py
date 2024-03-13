from ..entity.graphSet import GraphSet
from ..entity.pathSet import PathSet
from ..repository.map_load import MapLoad


class GraphController:
    """
    Class to retrieve map graph information

    Attributes:

    """

    def __init__(self):
        super().__init__()

    def get_graph(self):
        """Extract graph information for use by the control center from the created map.

        Args:

        Returns:
            graph_info : graph information

        Raises:

        """
        # 그래프 정보로 변환하여 리턴한다.
        data = MapLoad.load_path_file()
        graph_info = GraphSet(**data)  # path,node,link
        path_info = PathSet(**data)  # path,node,link
        # print(graph_info.dict())

        # 경로 정보 에서 노드 헤딩 값을 찾아서 넣는다.
        nlist = list(map(lambda pl: pl.nodeList, path_info.path))
        result = list(
            map(lambda x: self._map_node_direction(x, sum(nlist, [])), graph_info.node)
        )

        graph_info.node = result
        return graph_info

    def _map_node_direction(self, node, nodelist):
        """Find the direction value (exit direction) that the vehicle should take from the node.

        Args:
            node : graph node
            nodelist : path information

        Returns:
            node : Node with input direction

        Raises:

        """
        nodeList = list(filter(lambda nl: nl.nodeId == node.nodeId, nodelist))

        if nodeList:
            node.heading = nodeList.pop().heading
        else:
            node.heading = 0

        return node
