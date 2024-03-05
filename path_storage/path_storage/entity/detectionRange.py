from pydantic import BaseModel
from .position import Position


class DetectionRange(BaseModel):
    position: Position
    width: float
    height: float
    actionCode: str
