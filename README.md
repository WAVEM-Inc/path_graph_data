
# path and graph information manager


**`Documentation`** |
------------------- |


[ubuntu](https://ubuntu.com/) 및 [ROS2](https://docs.ros.org) 환경에서 [Python](https://www.python.org/)으로 개발하였다.

KEC 자율주행 차량에서 사용되는 주행 경로 및 관제센터에서 사용하는 그래프 정보를 저장하고 관리 한다.

아래와 같이 서비스 서버 패키지 와 서비스 메시지 패키지로 구성되어 있다.

├── path_graph_msgs  
└── path_storage 

### path_graph_msgs 
- path 서비스와 graph 서비스를 등록한다.  
### path_storage 
- 등록된 서비스에 따라 서비스를 제공하는 서비스 서버이다.  




## Build

소스를 빌드 하기 위해서는 먼저 ROS2 설치 및 워크스페이스가 구성되어 있어야 하고 의존 관계 패키지인 route_msgs가 구성되어 있어야 한다.

```shell
$ colcon build --packages-select path_graph_msgs
```
```shell
$ source install/setup.bash
$ ros2 interface list | grep path
    path_graph_msgs/srv/Graph
    path_graph_msgs/srv/Path
```
```shell
$ colcon build --packages-select path_storage
```

## Run

```shell
$ source install/setup.bash
$ ros2 launch path_storage path_storage.launch.py
```

## Custom Service 
서비스 메시지는 다음과 같이 정의 되어 있으며 메시지 명세는 소스 및 관련 문서를 참고 한다.

    path_graph_msgs/srv/Graph  
    path_graph_msgs/srv/Path

    route_msgs/msg/DetectionRange
    route_msgs/msg/Node
    route_msgs/msg/Path
    route_msgs/msg/Position




## Service guidelines

path_graph_msgs/srv/Graph
*   구성된 맵의 전체 노드와 엣지 정보를 조회 하여 리턴한다.

path_graph_msgs/srv/Path

*   차량의 위치 및 목적지 노드에 맞는 경로를 조회 하여 리턴한다.      

*   Service Field Notes
    
    route_msgs/Position position  
    string start_node  
    string end_node  


주행경로 서비스 요청시 필드(출발지 좌표,출발지 아이디,도착지 아이디)의 optional에 따른 출발/도착 경로 조회 결과는 다음과 같다.
*priority_gps 는 config.ini의 환경변수 이다.

![image](https://github.com/WAVEM-Inc/path_graph_data/assets/87844157/964cf39f-5caf-4250-b761-8dc957634196)



## License


