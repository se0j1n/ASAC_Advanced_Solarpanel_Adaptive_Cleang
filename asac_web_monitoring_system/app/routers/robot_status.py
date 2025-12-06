# app/routers/robot_status.py

from fastapi import APIRouter, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates

from app.services.robot_pose import get_robot_pose  

router = APIRouter()
templates = Jinja2Templates(directory="app/templates")


# --------------------------
# 1) 로봇 말단(TCP) 좌표 API
#    GET /api/robot_pose
# --------------------------
@router.get("/api/robot_pose")
async def api_robot_pose():
    pose = get_robot_pose()
    if pose is None:
        return {"x": None, "y": None, "z": None}
    return pose


# --------------------------
# 2) 로봇팔 상태 페이지
#    GET /robot-status
# --------------------------
@router.get("/robot-status", response_class=HTMLResponse)
async def robot_status_page(request: Request):
    return templates.TemplateResponse(
        "robot_pose.html",  
        {
            "request": request,
            "page_title": "Robot Arm Status",
            "page_eyebrow": "Robot Monitoring",

        }
    )
