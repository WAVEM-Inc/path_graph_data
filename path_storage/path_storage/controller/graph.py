from ..entity.graphSet import GraphSet
from ..repository.map_load import MapLoad

import logging

logger = logging.getLogger()


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
        # 파일에서 조회한 경로 정보 에서 start,end가 매핑되는 경로를 찾는다.
        data = MapLoad.load_path_file()
        graph_info = GraphSet(**data)  # path,node,link
        # print(graph_info.dict())

        return graph_info
