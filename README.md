# static_avoidance_KUUVe

테스트 아직 안 해본 코드

앞으로 수정해야하고 고쳐나가야 할 부분:
<ol>
  <li>정적장애물 두 개 피하기</li>
  <li>마지막 장애물을 회피하고, 본래 차선을 유지하도록 하기</li>
  <li>오른쪽, 왼쪽 장애물 둘 중 어느 것이 와도 장애물을 피하도록 하기</li>
  <li>차선 두 개중 진행차선이 아닌 옆 차선에 장애물이 있을 경우 장애물을 무시하고, 진행하도록 하기</li>
  <li>여러환경에서 실험해보며 박스필터의 범위 수정하기</li>
</ol>

-------------------------
<h4>Fuction: innerProduct</h4>

장애물과 차량이 이루는 각도를 계산해 장애물을 지나칠 때 회전할 수 있도록 구현

<div>
<img width="1000" src= "https://user-images.githubusercontent.com/67793181/92998879-99d42b00-f557-11ea-9e38-6387fb5a1fe3.jpg">
</div>
