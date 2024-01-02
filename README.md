# Final Project : 물류배달로봇

# 함께한 사람들

 - 조성호(팀장) : ROS 원격 제어
 - 강병철 : SLAM, Navigation
 - 김창미 : 자율주행
 - 김태헌 : SLAM, Navigation
 - 박한규 : ROS 통신
 - 박성준 : 자율주행
<br>

# 시나리오

취지 : 로봇에 있는 기능들을 최대한 활용해 보자!

  1. 저장된 데이터에 없는 지형의 경우 지형 탐색 시작
	  - SLAM 및 자율주행, Data 및 맵 정보는 DB에
	  - 조건1 : 추후 길찾기 알고리즘 추가를 위해 출발지점부터 도착지점까지 루트를 2~3개
	  - 조건2 : 출발 지점 이후 도착 지점에 도착시 맵을 전부 밝히지 않았더라도 종료.

  3. 맵 데이터가 있는 경우, 출발지점부터 도착지점까지 최단 경로로 주행
	  - 길찾기 알고리즘 [참고](https://www.youtube.com/watch?v=rfOgaPXCADQ)
		  1. A* : 출발지점과 도착지점이 멀어지면 급격하게 느려지지만, 휴리스틱 함수를 사용하여 최단 경로를 찾는데 효율적으로 동작
		  2. JPS : A* 와 구조는 동일하나. 레이더처럼 방사형태로 검색을 하고 충돌정보를 기반으로 분기점이 될 수 있는 지점만  Priority-Query에 넣기 때문에 A* 보다 성능은 좋다만!
		  3. JPS(B), JPS+: +는 상황이 많이 변한다면 성능이 그리 좋지 않음
		  4. Dijkstra : 어느 한 지점에서 다른 한 지점까지의 최단거리를 찾음 ( 가능한 적은 비용으로 가장 빠르게), 모든 경로를 탐색하며 모든 가중치가 동일할 때 가장 효과적
		  5. BFS : 매우 큰 미로일 경우 메모리 사용량이 늘어날 수 있음
		  6. DFS : 한 방향으로 갈 수 있는 최대한 멀리까지 탐색, 미로를 완전히 탐색하는데는 좋을 수도?
		

  ---
  위의 1과 2가 끝났다면 다음을 추가해보면 어떨까?
  3차 프로젝트 때 객체 인식을 했으니 이를 활용해서 특정 지역을 탐색 혹은 이동 중 객체를 인식해서 상황별 행동을 하게 만드는 것이다.

  3. 딜리버버리
	  - 물품 최소 3가지 이상
	  - 물품 보관소를 경유해 이용자가 주문한 물품을 배달
<br>

# 예외상황 발생시

  1. 바닥의 융기 혹은 장애물이 있을 경우 
	  - 사전에 감지할 수 있다면 : 장애물의 존재에 대한 통신 및 장애물을 피해 이동할 수 있는지 판단, 안 될 경우 Buzzer
	  - 사전에 감지가 안된다면 : 이동 중인 상황에서 엔코더의 좌표값이 일정시간 (약 1~2초)동안 변하지 않는다면 장애물에 걸린 것으로 판단, 일시적으로 뒤로 간 뒤 해당 상황에 대한 알람

  2. 최단거리 이동 중 길이 막혔을 경우
	  - 다른 최단 루트로 변경
	  - 만약 전부다 막혀있다면 알람

  3. 특정 상황에서는 사용자가 직접 조종해야할 일이 생길 수 있음.
	  - 원격으로 조종할 수 있는 방법을 생각해보자
<br>

# 역할배정

 - [ ] ROS2 SLAM
 - [ ] 자율 주행 
 - [ ] 원격 조종
 - [ ] DB 구축
 - [ ] 사용자와 로봇, DB간 통신 (이때 사용자는 rviz2 or gazebo or gui)
 - [ ] 길찾기 알고리즘 구현 (총 이동 거리 및 이동 시간 DB에 저장?)
 - [ ] opencv를 ros2로 구현
 - [ ] Model Training
 - [ ] 사용자  GUI를 별도로 만들 것인가? (해당 부분은 프로젝트 진행하면서 판단해야 할 듯)
<br>

# 기능리스트

  - 카메라 객체 인식 지원
  - 수동 및 자율모드
    
     1. 수동모드
        - Raspi4 ROS 터미널에서 키보드 입력을 받아 시리얼 전송
        - Arduino에서 시리얼 입력을 통해 센서 제어
           - 종류 : 방향키 (WASD)
                   방향지시등 (J, L)
                   경적 (K)
                   거리측정 (H)
                   속도 증감 (U, O)
                   정지 (Y)
        - 센서 정보는 DB에 기록되어 상태 조회 가능
       
     2. 자율모드
        - 자율주행모드
            - 목적지(좌표값)이 입력되면 해당 위치까지 이동 (이동 중 객체 인식 및 장애물 조우시 회피기동 or 정지)
         <br>

# 시스템 구성도

![Screenshot from 2024-01-02 10-35-01](https://github.com/addinedu-ros-3rd/ros-repo-2/assets/91608731/34e27c25-aaee-4f26-bf90-26cb6279da76)
