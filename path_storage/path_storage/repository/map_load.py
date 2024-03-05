import json
import logging
import os

# debug용
from ..config.path_config import PathConfig

# from config.path_config import PathConfig

logger = logging.getLogger()


class MapLoad:

    def __init__(self):
        super().__init__("MapLoad")

    def load_path_file(filename=""):

        try:
            home_path = os.path.expanduser("~")
            conf = PathConfig()
            filepath = home_path + conf.config["CONFIG"]["file_path"] + "/"

            if filename == "":
                filename = conf.config["CONFIG"]["file_name"]

            # 경로 맵 파일
            fullpath = filepath + filename

            with open(fullpath, "r") as f:
                path_json = json.load(f)

        except Exception as e:
            logger.error(("Exception occurred while code execution: " + str(e)))

        return path_json

    def get_linklist():
        return 0

    def get_pathlist():
        return 0
