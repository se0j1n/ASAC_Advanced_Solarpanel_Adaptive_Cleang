@웹서버 환경 설정 및 실행방법입니다. 순서대로 터미널에 복붙해주시면 됩니다.
@requirements 읽어보시고 이미 설치된 파일이면 설치 안하셔도 됩니다.
@이미 설치된 항목 다시 install해도 충돌문제는 없습니다. 아 마 도 ?!
------------------------환경구축------------------------
1. pip install -r requirements.txt
2. sudo apt-get update
3. sudo apt-get install -y libgl1 libglib2.0-0

------------------------서버실행------------------------
@app폴더 상위 폴더인 ASAC_WEB_VER1 폴더 안에서 진행해주세요.
cd ~/ASAC_WEB_VER1
uvicorn app.main:app --reload
