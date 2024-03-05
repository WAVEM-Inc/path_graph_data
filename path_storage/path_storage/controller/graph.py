# import json
# from types import SimpleNamespace
# debug용
from ..entity.graphSet import GraphSet
from ..repository.map_load import MapLoad

# from entity.graphSet import GraphSet
# from repository.map_load import MapLoad

import logging

logger = logging.getLogger()


class GraphController:

    def __init__(self):
        super().__init__()

    def get_graph(self):

        # 그래프 정보로 변환하여 리턴한다.
        try:
            # 파일에서 조회한 경로 정보 에서 start,end가 매핑되는 경로를 찾는다.
            data = MapLoad.load_path_file()
            graph_info = GraphSet(**data)  # path,node,link
            # print(graph_info.dict())

        except Exception as e:
            logger.error(("Exception occurred while code execution: " + str(e)))
            return None
        return graph_info
