from typing import List
from pydantic import BaseModel

from .path import Path


class PathSet(BaseModel):
    mapId: str
    version: str = None
    path: List[Path]
