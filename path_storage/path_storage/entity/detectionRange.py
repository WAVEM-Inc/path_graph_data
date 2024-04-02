from pydantic import BaseModel


class DetectionRange(BaseModel):
    offset: float
    widthLeft: float
    widthRight: float
    height: float
    actionCode: str
