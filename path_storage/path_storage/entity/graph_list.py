from pydantic import BaseModel

from .graph import Graph
from typing import List


class GraphList(BaseModel):
    send_id: str
    graph: List[Graph]
