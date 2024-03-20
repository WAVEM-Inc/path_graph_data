# import pytest
import unittest

# from controller.path import PathController
from controller.graph import GraphController

# from entity.position import Position

# from entity.path import Path
# from repository.map_load import MapLoad
# from service.ResponseService import ResponseService


class MyCalcTest(unittest.TestCase):

    def test_func(self):
        # p = PathController()
        g = GraphController()
        # r = ResponseService()
        # # 경로 조회
        # pos = Position(latitude=37.3923235, longitude=126.938187)
        # pathObj, mapId, version = p.get_task_path(pos, "", "33601099")
        # if pathObj == None:
        #     print("*RESPONSE DATA* \n" + "None")
        #     return
        # print("*RESPONSE DATA* \nmamp id : " + mapId + ", ver : " + version)
        # print(pathObj)
        # # retVal = r.convertResponse(data,response)

        # 그래프
        data = g.get_graph()
        if not data:
            return

        # jsonStr = r.makeGraphJson("TEST001", data)
        # print("*RESPONSE DATA* \n" + jsonStr)


if __name__ == "__main__":
    unittest.main()
