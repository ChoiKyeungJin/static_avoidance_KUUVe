# static_avoidance_KUUVe

테스트 아직 안 해본 코드

앞으로 수정해야하고 고쳐나가야 할 부분:
<ol>
  <li>정적장애물 두 개 피하기</li>
  <li>마지막 장애물을 회피하고, 본래 차선을 유지하도록 하기</li>
  <li>오른쪽, 왼쪽 장애물 둘 중 어느 것이 와도 장애물을 피하도록 하기</li>
  <li>차선 두 개중 진행차선이 아닌 옆 차선에 장애물이 있을 경우 장애물을 무시하고, 진행하도록 하기</li>
  <li>여러환경에서 실험해보며 박스필터의 범위 수정하기</li>
  <li>Imu의 Yaw값을 이용해서 코드를 수정하기</li>
   영상에서는 Matlab을 사용하지만 원리를 이해하는데 도움이 되는 : https://www.youtube.com/watch?v=hN8dL55rP5I
</ol>
 
-------------------------
<h2>Fuction: innerProduct</h2>

장애물과 차량이 이루는 각도를 계산해 장애물을 지나칠 때 회전할 수 있도록 구현

<div>
<img width="1000" src= "https://user-images.githubusercontent.com/67793181/92998879-99d42b00-f557-11ea-9e38-6387fb5a1fe3.jpg">
</div>
-------------------------
<h2>case : STATUS_GO </h2>

>진행차선에 장애물이 없고, 왼쪽에 장애물이 있을 경우 wayPoint를 진행차선에 위치시킨다.

<div>
<img width="1000" src="https://user-images.githubusercontent.com/67793181/93022332-d40afe80-f623-11ea-84e0-f73164be07e7.jpg">
이후 wayPoint로 접근하다가 wayPoint.x가 일정범위 내로 접근하면 STATUS를 변경합니다.
</div>

:octocat:20.09.13 - 장애물이 raw_obstacle, right_obstacle, left_obstacle 의 세 경우로 박스필터에 의해 구분되는데, 여기서 원하는 데이터를 가져오는게 어려움.

-------------------------
<h2> Left, Right Obstacle Recognition </h2>

2D라이다의 스캔 방향이 반시계방향이므로, 가장먼저 인식 되는것은 오른쪽 장애물이고, 두번째 인식되는 것은 왼쪽 장애물이다.
LMS151의 경우 오른쪽 장애물이 먼저 인식되고, 그 다음에 왼쪽 장애물이 인식됩니다.

생각 중인 코드:
 scan_center[1].y > scan_center[0].y 이 식은 장애물이 왼쪽,오른쪽 어디서부터 시작이건 같을 것이다.
 그러나 scan_center[1].x > scan_center[0].x 이 경우 왼쪽 장애물이 더 멀리 있는 경우이므로 오른쪽,왼쪽 순서대로 장애물이 놓인 것이다. 
 이런식으로 x 좌표를 이용해 장애물을 비교하는 코드를 구현하고싶다.

<div>
<img width="1000" src="https://user-images.githubusercontent.com/67793181/93403667-274eac80-f8c3-11ea-8454-80081fb9c9e6.jpg">
</div>
