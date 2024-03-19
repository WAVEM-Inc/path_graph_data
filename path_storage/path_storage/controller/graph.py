from ..config.path_config import PathConfig
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

        workNodeCode = "work_node"
        tempNodeCode = "workplace_node"
        initNodeCode = "initial_node"

        try:
            conf = PathConfig()
            workNodeCode = conf.config["DEFINE"]["work_node"]
            tempNodeCode = conf.config["DEFINE"]["workplace_node"]
            initNodeCode = conf.config["DEFINE"]["initial_node"]
        except Exception as e:
            print(
                (
                    "The value defined in the configuration file cannot be found. "
                    + str(e)
                )
            )
        # 코드값 변환(out 파일의 코드와 내부 사용 코드 값이 다른 경우 변환을 위해 필요함)
        convertValue = {}
        convertValue[tempNodeCode] = initNodeCode
        print(graph_info.node)
        # 작업장,대기장소만 필터
        graphNodeList = list(
            filter(
                lambda nd: nd.type == workNodeCode or nd.type == tempNodeCode,
                graph_info.node,
            )
        )

        print("node type : " + workNodeCode + " / " + tempNodeCode)
        print(graphNodeList)
        # workplace_node를 initial_node로 변환
        graphNodeList = list(
            map(lambda x: self._map_node_conversion(x, convertValue), graphNodeList)
        )

        # 경로 정보 에서 노드 헤딩 값을 찾아서 넣는다.
        nlist = list(map(lambda pl: pl.nodeList, path_info.path))
        result = list(
            # map(lambda x: self._map_node_direction(x, sum(nlist, [])), graph_info.node)
            map(lambda x: self._map_node_direction(x, sum(nlist, [])), graphNodeList)
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

    def _map_node_conversion(self, node, convertValue: dict):
        """Conversion value.

        Args:
            node : graph node

        Returns:
            node : Node

        Raises:

        """
        print("_map_node_conversion")
        print(str(node))
        print(str(convertValue))

        if convertValue.get(node.type) is not None:
            node.type = convertValue[node.type]

        return node
