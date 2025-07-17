**Guidance Library (C++)**


2‑D 직선·원형 경로 유도 알고리즘 구현

get_angle · line_guidance · circle_guidance

**개요**

이 레포지토리는 이동체(UGV/UAV 등)의 2‑차원 경로 추종을 위해 자주 사용되는 L1(Guidance) 방식 의 핵심 로직을 C++로 구현한 예시입니다.
외부 제어기·펌웨어(PX4, ROS 노드 등)에 쉽게 포함할 수 있도록 헤더 한 장(guidance.h) 과 guidance.cpp으로 분리했습니다.

| 함수                                                                        | 설명                                     | 반환          |
| ------------------------------------------------------------------------- | -------------------------------------- | ----------- |
| `double get_angle(start, end)`                                            | +Y 축(북)을 기준으로 **시계방향**으로 잰 각도 (radian) |  `0 ~ 2π`   |
| `std::vector<double> line_guidance(start, end, local, step)`              | 직선 경로를 따라 **다음 set‑point** 계산          |  `{dx, dy}` |
| `std::vector<double> circle_guidance(center, radius, direc, local, step)` | 원 궤도를 따라 **다음 set‑point** 계산           |  `{dx, dy}` |


각 함수는 현 위치 → 목표까지 이동할 상대 변위 (dx, dy)를 반환하므로, 호출 측에서 세계 좌표로 변환해 사용.

좌표계·각도 규칙

          +Y (0 rad)
            ↑
            │   시계방향(CW)으로 증가
            │
 -X (−π/2) ├──► +X (π/2)
            │
            │
          −Y (π = 180°)
          
atan2(y, x) 대신 atan2(x, y) 를 사용해 +Y 기준 시계방향 각도를 직접 얻습니다.

psi_control(조향각)은 −π ~ +π 범위로 정규화해 반환합니다.

**빌드 & 사용 예시**

**1. 의존성**

표준 C++17 이상( <vector> <cmath> )만 사용.
다만 다음 유틸 함수·상수가 별도 정의되어 있어야 합니다.



double norm(const std::vector<double>& v);           // 2‑노름
std::vector<double> eminus(a, b);                    // b → a  차 벡터
constexpr double KP = 1.0;                           // 횡방향 오차 ↦ 조향 이득
enum Turn { CW = 1, CCW = -1 };


**3. 코드 스니펫**

#include "guidance.h"
#include "utils.h"

// 직선 예시
std::vector<double> start{0, 0}, end{10, 0};
std::vector<double> local{2, -1};
double step = 0.5;

auto delta = line_guidance(start, end, local, step);
// delta = {dx, dy} : 현 위치에서 dx,dy 만큼 이동하면 됨

**디렉터리 구조**


├── guidance.h 

├── guidance.cpp   

└── utils.{h,cpp}   

