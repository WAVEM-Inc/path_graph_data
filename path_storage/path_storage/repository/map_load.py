import json

from ..config.path_config import PathConfig


class MapLoad:
    """
    Class to read path information file

    Attributes:

    """

    def __init__(self):
        super().__init__("MapLoad")

    def load_path_file(filename=""):
        """
        Reads the file and returns it in json format.

        Args:
            filename : File name to read. If not, use the file name in the configuration file.

        Returns:
            path_json : json format data of the contents of the file

        Raises:

        """
        conf = PathConfig()
        filepath = conf.home_path + conf.config["CONFIG"]["file_path"]

        if filename == "":
            filename = conf.config["CONFIG"]["file_name"]

        # 경로 맵 파일
        fullpath = filepath + "/" + filename

        with open(fullpath, "r") as f:
            path_json = json.load(f)

        return path_json

    def get_linklist():
        return 0

    def get_pathlist():
        return 0
