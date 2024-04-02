from typing import List
from pydantic import BaseModel

from .node import Node


class GraphSet(BaseModel):
    mapId: str
    version: str
    node: List[Node] = None
