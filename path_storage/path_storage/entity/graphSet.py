from typing import List
from pydantic import BaseModel

from .node import Node
from .link import Link


class GraphSet(BaseModel):
    mapId: str
    version: str
    node: List[Node] = None
    link: List[Link] = None
