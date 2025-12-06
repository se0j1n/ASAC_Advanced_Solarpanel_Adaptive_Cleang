from fastapi import APIRouter
from app.services.camera_stream import get_camera_stream

router = APIRouter()

@router.get("/livecam")
async def livecam():
    """ 웹에서 <img src='/livecam'> 로 MJPEG 스트림 표시 """
    return get_camera_stream()
