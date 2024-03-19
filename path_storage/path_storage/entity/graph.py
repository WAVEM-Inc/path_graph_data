from pydantic import BaseModel
from typing import List

# debug용
from .graph_edge import GraphEdge
from .graph_node import GraphNode

# from entity.graph.edge import GraphEdge
# from entity.graph.node import GraphNode


class Graph(BaseModel):
    map_id: str
    version: str
    is_indoor: bool
    node_list: List[GraphNode]
