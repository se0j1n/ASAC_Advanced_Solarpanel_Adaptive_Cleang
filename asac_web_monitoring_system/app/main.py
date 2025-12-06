# app/main.py
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
# from app.services.robot_state import test_router as robot_state_test_router #api test용


# -----------------------------
# ROS2 Robot State Subscriber
# -----------------------------
from app.services.robot_state import create_robot_state_node, router as robot_state_router
from rclpy.executors import MultiThreadedExecutor
import threading

# ROS2 노드 생성 + 멀티스레드 실행
robot_node = create_robot_state_node()
executor = MultiThreadedExecutor()
executor.add_node(robot_node)

threading.Thread(target=executor.spin, daemon=True).start()


# -----------------------------
# FastAPI App 초기화
# -----------------------------
app = FastAPI(title="ASAC Control Center")


# -----------------------------
# Static & Templates
# -----------------------------
app.mount("/static", StaticFiles(directory="app/static"), name="static")
templates = Jinja2Templates(directory="app/templates")


# -----------------------------
# Routers 등록
# -----------------------------
from app.routers.livecam import router as livecam_router
from app.routers.dashboard import router as dashboard_router
from app.routers.robot_status import router as robot_status_router  # 로봇 포즈 페이지

app.include_router(livecam_router)
app.include_router(dashboard_router)
app.include_router(robot_status_router)
app.include_router(robot_state_router)  # ★ Work State API 포함



# ============================================================
# 메인 대시보드 페이지: GET "/"
# ============================================================
@app.get("/", response_class=HTMLResponse)
async def dashboard(request: Request):

    print("📌 '/' 대시보드 진입 OK")

    context = {
        "request": request,
        "page_title": "ASAC System Dashboard",
        "page_eyebrow": "Monitoring",

        # 하드코딩된 UI 값 (ROS 연동되면 여기 업데이트)
        "robot_name": "ASAC-01",
        "connection_status": "ONLINE",
        "connection_latency_ms": 18,
        "current_mode": "AUTO CLEANING",

        "job_progress": 47,
        "panel_cleaned": 12,
        "panel_total": 24,

        "alert_count": 2,
        "last_alert": "Wind speed high on Sector B-03",

        "environment": {
            "temp": 32.4,
            "wind": 6.2,
            "solar": 780
        }
    }

    return templates.TemplateResponse("dashboard.html", context)
