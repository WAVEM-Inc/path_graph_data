import configparser
import os


class PathConfig:

    def __init__(self):
        home_path = os.path.expanduser("~")

        self.config = configparser.ConfigParser()
        self.config.read(home_path + "/RobotData/maps/kecd_path/config/config.ini")
        # self.config.read("config.ini")

        # try:
        #     print(ret)
        # except Exception as e:
        #    print(("Exception occurred while code execution 1: " + str(e)))

        # try:
        #     print(self.config['CONFIG'])
        # except Exception as e:
        #    print(("Exception occurred while code execution 2: " + str(e)))

        # filepath = self.config['CONFIG']['file_path']
        # filepath2 = self.config['CONFIG']['file_path']

        # dir = os.getcwd()
        # print("\ncurrent path : " + dir)

        # # 환경 파일
        # with open(dir +'/config/conf.yaml', 'r') as f:
        #    self.config = yaml.load(f)
        #    print(self.config)
