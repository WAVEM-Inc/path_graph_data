import configparser
import os


class PathConfig:
    """
    Class that manages operating environment information

    Attributes:
        home_path :  Your system's home directory
        config  : Object of class configparser
    """

    def __init__(self):
        self.home_path = os.path.expanduser("~")

        self.config = configparser.ConfigParser()
        self.config.read(self.home_path + "/RobotData/maps/kecd_path/config/config.ini")
