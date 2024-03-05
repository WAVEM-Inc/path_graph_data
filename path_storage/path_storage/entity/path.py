from typing import List
from pydantic import BaseModel

from .node import Node


class Path(BaseModel):
    id: str
    name: str
    nodeList: List[Node]
