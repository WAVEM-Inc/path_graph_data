from pydantic import BaseModel


class DetectionRange(BaseModel):
    offset: float
    width: float
    height: float
    actionCode: str
