from pydantic import BaseModel


class Link(BaseModel):
    linkId: str
    stNode: str
    edNode: str
