from pydantic import BaseModel
from typing import List, Optional

from .detectionRange import DetectionRange
from .position import Position


class Node(BaseModel):
    nodeId: str
    position: Position
    type: Optional[str] = None
    kind: Optional[str] = None
    heading: Optional[int] = None
    direction: Optional[str] = None
    detectionRange: Optional[List[DetectionRange]] = None
