# KDAS2026

<div align="center">
  <img src="images/kdasmain.gif" alt="KDAS Main" width="800"/>
</div>

# KDAS – 국민대학교 자율주행 시스템

## Introduction

안녕하세요.  
본 저장소는 **국민대학교 자율주행 팀**이 수행한 자율주행 시스템 연구 및 개발 결과를 정리한  
메인 프로젝트 레포지토리입니다.

본 프로젝트는 **Quanser Autonomous Car Competition (ACC)** 제출을 목표로 수행되었으며,  
실제 자율주행 차량의 작동 방식을 기준으로  
**인지(Perception) – 위치추정(Localization) – 계획(Planning) – 제어(Control)** 단계를 분리하여  
시스템을 구성하였습니다.

각 단계는 명확한 입력과 출력을 갖도록 설계되었으며,  
각 모듈의 설계 의도, 구현 방식, 그리고 실험 결과를  
독립적인 폴더 단위로 정리하였습니다.

---

## Whole Project

본 프로젝트에서 저희 팀은 자율주행 시스템을  
실제 자율주행 차량의 시스템 구조에 맞추어 단계적으로 구현하였습니다.

- 인지 모듈은 주행 환경으로부터 필요한 정보를 추출하고  
- 위치추정 모듈은 차량의 현재 위치와 자세를 안정적으로 추정하며  
- 계획 모듈은 해당 정보를 바탕으로 주행 경로 및 전략을 결정하고  
- 제어 모듈은 결정된 경로를 따라 차량이 실제로 주행할 수 있도록 조향·제동 명령을 생성합니다.

아래 GIF들은 각 모듈에서 실제로 구현한 기능과 그 결과를 요약적으로 보여줍니다.  
각 항목의 링크를 클릭하면, 해당 모듈의 상세 문서와 구현 결과를 확인할 수 있습니다.

<table>
  <tr>
    <td align="center">
      <img src="images/YOLOReadme1.gif" width="320"><br>
      <b>YOLO – 객체 인식 (1)</b><br>
      <a href="YOLO/">폴더 바로가기</a>
    </td>
    <td align="center">
      <img src="images/YOLOReadme2.gif" width="320"><br>
      <b>YOLO – 객체 인식 (2)</b><br>
      <a href="YOLO/">폴더 바로가기</a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="images/SCNNmodel.gif" width="320"><br>
      <b>SCNN – 차선 인식</b><br>
      <a href="SCNN/">폴더 바로가기</a>
    </td>
    <td align="center">
      <img src="images/ControlMat.gif" width="320"><br>
      <b>Control – 조향 및 제동 제어</b><br>
      <a href="Control/">폴더 바로가기</a>
    </td>
  </tr>
</table>

---

## Project Modules Overview

아래는 본 프로젝트를 구성하는 주요 모듈과,  
각 모듈이 자율주행 시스템 내에서 수행하는 역할에 대한 개요입니다.  
모듈 순서는 실제 데이터 흐름을 기준으로 정렬되어 있습니다.

---

### [1) Localization](Localization/Cartographer/)

**역할**  
Localization 모듈은 차량의 현재 위치와 자세를 추정하여,  
자율주행 시스템 전반에서 공통으로 사용되는 **기준 좌표계 및 위치 정보**를 제공하는 역할을 수행합니다.

Planning 및 Control 모듈은 모두 Localization 결과를 기반으로 동작하며,  
주행 경로 추종 및 장애물 회피와 같은 모든 제어 동작은  
해당 위치 추정 결과를 기준으로 수행됩니다.

**구현 개요**  
- LiDAR 센서를 활용한 2D SLAM 기반 위치 추정  
- Cartographer를 이용한 지도 생성 및 위치 추정 파이프라인 구성  
- ROS2 TF 트리를 통한 좌표계 관리  
- 시뮬레이션 환경에서의 위치 추정 안정성 검증  

**산출물**  
- Cartographer 기반 SLAM 및 Localization 설정 파일  
- 지도 생성 및 위치 추정 결과  
- Localization–Planning–Control 연동 구조 설명 문서  

---

### [2) Helper](Helper/)

**역할**  
Helper 모듈은 Planning 단계에서 처리·생성된 경로 정보를 가공하여,  
Control 모듈이 직접 사용할 수 있는 형태로 전달하는 **중간 연결 모듈**입니다.

즉, Planning과 Control 사이에서  
경로 표현을 정리하고 필요한 정보를 추출하여  
제어기가 안정적으로 동작할 수 있도록 입력을 제공하는 역할을 수행합니다.

**구현 개요**
- Planning 결과로 생성된 waypoint 경로 수신  
- Look-Ahead Distance 기준 목표점(target point) 계산  
- 주행 상황에 따라 제어기에 전달할 경로 정보 구성  
- Pure Pursuit 및 Obstacle Avoidance 모듈에 공통 입력 제공  

Helper 모듈은 주행 전략을 결정하지 않으며,  
Planning 결과를 제어 단계로 **연결·중계하는 역할**에 집중합니다.

---

### [3) Control](Control/)

**역할**  
Control 모듈은 Planning 단계에서 생성된 주행 경로 및 판단 결과를 입력으로 받아,  
차량이 실제로 주행할 수 있도록 **조향 및 제동 명령을 생성하는 최종 제어 단계**를 담당합니다.

본 프로젝트의 Control 모듈은 주행 상황에 따라  
정상 주행과 장애물 회피 상황을 구분하여 동작하도록 구성되어 있습니다.

---

#### [3-1) Pure Pursuit Control](Control/Matlab&Simulink/Pure%20Pursuit/)

**역할**  
Pure Pursuit 모듈은 장애물이 없는 정상 주행 상황에서,  
Planning 단계에서 전달된 waypoint 경로를 따라  
차량이 안정적으로 주행하도록 조향각을 계산하는 **경로 추종 제어 모듈**입니다.

**구현 개요**  
- Look-Ahead Distance 기반 목표점 추종 방식 적용  
- 차량 위치와 목표점 간의 기하학적 관계를 이용한 조향각 계산  
- Simulink 기반 제어기 설계 및 ROS2 노드 형태로 연동  

해당 모듈은 전역 및 국소 경로 추종을 담당하며,  
시스템의 기본적인 주행 안정성을 결정하는 핵심 제어기로 사용됩니다.

---

#### [3-2) Obstacle Avoidance Control](Control/Obstacle%20Avoidance/)

**역할**  
Obstacle Avoidance 모듈은 주행 경로 상에 장애물이 존재하는 경우,  
충돌을 방지하기 위해 **횡방향 회피 기동과 차선 복귀 동작을 수행하는 제어 모듈**입니다.

**구현 개요**  
- 전방 장애물과의 거리 기반 회피 기동 필요 여부 판단  
- LiDAR(또는 Depth Camera)를 이용한 장애물 크기 추정  
- 차량 폭과 안전 여유 거리를 고려한 최소 횡방향 회피 거리 계산  
- 기존 waypoint를 기반으로 한 국소적 경로 수정 방식 적용  
- 일정 곡률을 이용한 회피 기동 수행 및 동일 곡률 기반 차선 복귀  

해당 모듈은 Pure Pursuit 제어기와 연동되어,  
회피 상황에서도 조향 입력의 연속성과 주행 안정성을 유지하도록 설계되었습니다.

---

### [4) SCNN – Lane Detection](Perception/SCNN/)

**역할**  
주행 환경에서 차선을 인식하고,  
차량이 추종할 수 있는 **차선 구조 및 기준선 정보**를 생성하는 인지 모듈입니다.

**구현 개요**  
- SCNN 모델 기반 차선 인식 파이프라인 구성  
- 입력 영상 전처리 및 출력 후처리 설계  
- ROS2 메시지 형태로 차선 정보 전달  

**산출물**  
- 차선 인식 모델 및 추론 구조 설명  
- 차선 인식 결과 시각화(GIF/이미지)  
- 시스템 통합 관련 문서 정리  

---

### [5) YOLO – Object Detection](Perception/YOLO/)

**역할**  
교통 표지판 및 신호와 같은 객체를 실시간으로 인식하여,  
자율주행 판단 및 제어 모듈에 필요한 정보를 제공하는 인지 모듈입니다.

**구현 개요**  
- YOLO 기반 객체 인식 모델 적용  
- 실시간 추론 결과를 ROS2 토픽으로 송신  
- 제어 로직과 연동 가능한 인터페이스 설계  

**산출물**  
- 객체 인식 모델 및 추론 파이프라인 설명  
- 객체 인식 결과 GIF  
- ROS2 연동 구조 및 메시지 정의 문서  

---

## 정리

본 레포지토리는 ACC 예선 제출을 목표로 한  
자율주행 시스템의 전체 구조와 구현 결과를  
실제 시스템 흐름에 맞추어 단계별·모듈별로 정리한 문서화 공간입니다.

각 모듈은 독립적으로 이해 및 검증 가능하도록 구성되었으며,  
동시에 하나의 통합된 자율주행 시스템으로 동작하도록 설계되었습니다.








Core Principles of Self-Driving (ACC Criteria)


각 모듈별 산물물을 정리하는 것 대신 아래처럼 산출물만 모아서 정리하기??????????!!!!!!!!!!!!?????????


본 프로젝트는 ACC에서 제시한 Core Principles of Self-Driving을
시스템 설계 전반에 걸쳐 반영하였습니다.

1) Data Collection

자율주행 알고리즘은 다양한 센서로부터 데이터를 수집합니다.

Camera 기반 영상 데이터 (차선, 표지판, 객체 인식)

LiDAR 기반 거리 및 환경 정보

차량 상태 정보 (속도, 위치, 시스템 상태)

수집된 원시 데이터는 각 인지 모듈에서 전처리 및 필터링 과정을 거쳐
의미 있는 정보로 변환됩니다.

2) Interpretation

수집된 데이터는 시스템 내부 및 외부 상황을 해석하는 데 사용됩니다.

차선 구조 및 주행 가능 영역 추론

교통 표지판 및 신호의 의미 해석

전방 장애물 존재 여부 및 회피 필요성 판단

Localization 정보를 통한 차량 상태 및 위치 해석

3) Control Systems

해석된 결과를 바탕으로,
차량은 선택된 주행 전략을 정확하게 실행합니다.

차선 유지 및 경로 추종

교차로 및 곡선 구간 주행

장애물 회피 및 차선 복귀

목표 속도 유지 및 감속/정지 수행

4) Localization & Path Planning

차량은 지도 상에서 자신의 위치를 이해하고,
목표 지점까지의 경로를 계획 및 수정할 수 있어야 합니다.

LiDAR 기반 SLAM을 이용한 위치 추정

Global / Local 경로 기반 주행

주행 중 환경 변화에 따른 경로 재계획
