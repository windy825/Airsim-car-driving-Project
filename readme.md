# 가상 주행 프로젝트 <small>22.06.15 ~ 22.06.28</small>

<br>
## 목차

#### 1. 기본 (on this page)

#### 2. [주행 파트](https://github.com/windy825/Airsim-car-driving-Project/blob/master/1.%20%EC%A3%BC%ED%96%89%20%ED%8C%8C%ED%8A%B8.md)

#### 3. [장애물 극복](https://github.com/windy825/Airsim-car-driving-Project/blob/master/2.%20%EC%9E%A5%EC%95%A0%EB%AC%BC%20%EA%B7%B9%EB%B3%B5.md)

#### 4. [API 명세](https://github.com/windy825/Airsim-car-driving-Project/blob/master/3.%20API%20%EB%AA%85%EC%84%B8.md)

<br>

<br>

**이 프로젝트는 Airsim 시뮬레이터를 이용하여, 가상현실에 구현된 차량을**

**주행 데이터와 프로그래밍 언어를 기반으로 안정적이고 빠르게 주행하는 것이 목표입니다.**

```
사용 언어 : Python

연결 형태 : 시뮬레이터 (Unreal Engine 기반) 
            API       (주행정보를 수신, 주행조작을 송신하는 데이터 통신)
            Python    (받아온 데이터를 이용하여 주행조작 설정)

참 가 자  : 홍성목(주행)
        임진현(장애물 극복)
```

<br>

### Airsim 이란?

AirSim is a simulator for drones, cars and more, built on [Unreal Engine](https://www.unrealengine.com/) (we now also have an experimental [Unity](https://unity3d.com/) release). It is open-source, cross platform, and supports software-in-the-loop simulation with popular flight controllers such as PX4 & ArduPilot and hardware-in-loop with PX4 for physically and visually realistic simulations. It is developed as an Unreal plugin that can simply be dropped into any Unreal environment. Similarly, we have an experimental release for a Unity plugin.

Our goal is to develop AirSim as a platform for AI research to experiment with deep learning, computer vision and reinforcement learning algorithms for autonomous vehicles. For this purpose, AirSim also exposes APIs to retrieve data and control vehicles in a platform independent way. ( 출처 : Microsoft.github.io/Airsim/ )

```
위 내용처럼 드론, 차 그리고 그 이상의 것들을 실험하는 시뮬레이터 입니다.
기반 엔진은 언리얼 이고 현재 유니티 기반의 작동도 가능하게 되었습니다.

현 오픈소스 프로젝트 형태로 진행중인 Airsim은 현실에서의 자율주행 탑승장비의 인공지능 연구 (딥러닝, 컴퓨터 비젼, 강화학습)를 위한 플랫폼이 목표입니다.
```

<br>

### 자율주행 구현 목표

**[기본 요구사항]** 

도로의 형태에 따라 알맞은 주행을 통해, 대상 차량의 자율주행에 안정성을 확보한다.

<br>

**[추가 요구사항]**  

여러 정보들(API를 통해 받아오는 데이터)을 종합적으로 판단하여 안전하게 주행한다.

판단해야 할 요소는 아래와 같습니다.

1. 장애물
2. 도로의 각도 (완만 or 급격한 커브도로)
3. 다른 차 정보
4. 도로의 경계선, 중앙선
