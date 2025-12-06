from fastapi import APIRouter
from fastapi.responses import JSONResponse
from app.services.robot_state import robot_state

router = APIRouter()

@router.get("/api/work_state")
async def work_state():
    return JSONResponse(robot_state)
