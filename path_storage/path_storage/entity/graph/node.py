from pydantic import BaseModel


class GraphNode(BaseModel):
    node_id: str
    node_name: str
    x: float
    y: float
    node_type: str
    heading: float
    critical: bool
