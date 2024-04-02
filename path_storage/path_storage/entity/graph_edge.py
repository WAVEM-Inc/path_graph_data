from pydantic import BaseModel


class GraphEdge(BaseModel):
    source: str
    target: str
    # velocity_limit: float = None
    directional: bool
