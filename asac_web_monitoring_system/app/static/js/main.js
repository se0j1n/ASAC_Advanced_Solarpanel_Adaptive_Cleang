/**
 * 메인 대시보드 클라이언트 스크립트
 * 지금은 더미 애니메이션만 넣어두고,
 * 나중에 WebSocket / ROS Bridge 붙일 때 여기 확장하면 됨.
 */

document.addEventListener("DOMContentLoaded", () => {
    // 작업 진행률 가볍게 출렁이는 애니메이션 (데모용)
    const progressFill = document.querySelector(".progress-fill");
    if (progressFill) {
        const initialWidth = parseFloat(progressFill.style.width) || 0;
        let direction = 1;

        setInterval(() => {
            let current = parseFloat(progressFill.style.width) || initialWidth;
            if (current > initialWidth + 6) direction = -1;
            if (current < initialWidth - 4) direction = 1;
            progressFill.style.width = (current + direction * 0.6) + "%";
        }, 2200);
    }

    // 알림이 하나씩 살짝 빛나는 효과
    const alerts = document.querySelectorAll("#alert-list .list-item");
    let activeIdx = 0;

    setInterval(() => {
        alerts.forEach((el, idx) => {
            if (idx === activeIdx) {
                el.style.boxShadow = "0 0 0 1px rgba(250, 204, 21, 0.6), 0 0 22px rgba(250, 204, 21, 0.35)";
            } else {
                el.style.boxShadow = "none";
            }
        });
        activeIdx = (activeIdx + 1) % alerts.length;
    }, 2600);
});