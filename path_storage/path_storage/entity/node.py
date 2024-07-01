from pydantic import BaseModel
from typing import List, Optional

from .detectionRange import DetectionRange
from .position import Position


class Node(BaseModel):
    nodeId: str
    nodeName: str = None
    position: Position
    type: Optional[str] = None
    kind: Optional[str] = None
    heading: Optional[float] = None
    direction: Optional[str] = None
    drivingOption: Optional[str] = None
    arriveOption: Optional[str] = None
    speed: Optional[float] = None
    detectionRange: Optional[List[DetectionRange]] = None
