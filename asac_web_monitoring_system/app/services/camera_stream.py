import cv2
from fastapi.responses import StreamingResponse

def generate_frames():
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

    # --- 카메라 설정 ---
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # 실제 포맷 확인
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    codec = "".join([chr((fourcc >> 8*i) & 0xFF) for i in range(4)])
    print("현재 사용 중인 포맷:", codec)

    # --- MJPEG 스트림 ---
    while True:
        success, frame = cap.read()
        if not success:
            continue

        # YUYV → BGR 변환 필요할 경우
        if codec == "YUYV":
            frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)

        ret, buffer = cv2.imencode(".jpg", frame)
        frame = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )


def get_camera_stream():
    """FastAPI StreamingResponse (MJPEG)"""
    return StreamingResponse(
        generate_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )
