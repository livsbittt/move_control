# 웹 탐색 실행 시간 검증

`tools/navigation_session_web_rig.py`: domain 229, localhost, 실제 HTTP/ROS와
WanderNode. 준비 상태와 지도 메타데이터는 합성이며 물리 모델·모터는 없다.

`tools/test_navigation_session_browser.py`: 실제 Edge에서 가까운 곳 우선,
전체 10초, 정체 10초를 선택하고 노드 적용 상태를 확인했다. 페이지를 닫은 뒤
다시 접속해 `time_limit`, `active=false`, `stop:time_limit`을 확인했다.

`events.json`의 ROS 시간은 모두 0이다. `explore_nearest`부터 `stop`까지
실제 시간은 10.1320초이며, 뒤이어 `/cmd_vel_raw=[0,0]`을 관측했다.
처음부터 움직이지 않는 시험이므로 실제 감속·물리 정지 성능의 증명은 아니다.
브라우저 JavaScript 오류는 0개였다.

[실행 중 화면](running.png) · [시간 만료 화면](timeout.png) · [노드 상태](state.json)

첫 브라우저 시도는 시험 서버에서 backend 포트 치환을 빠뜨려 시작 버튼이
비활성 상태였다. 시험 서버의 치환과 결과 조회용 메서드 누락을 수정한 뒤
위 실행을 통과했다.

순수 로직 457 통과/1 건너뜀, 기존 웹 JS 검사 24 통과.
HTTP·Wander·GoalNode ROS 검사 29개 통과 후 GoalNode 전략 전환 검사를 추가했고,
GoalNode 파일의 12개 검사를 다시 실행해 모두 통과했다.
