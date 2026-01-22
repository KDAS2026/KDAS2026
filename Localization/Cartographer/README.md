
## 1.1 SLAM Implementation: Cartographer

완전한 SLAM 시스템을 바닥부터 구현하는 대신, 검증된 오픈소스 솔루션을 **채택 및 최적화**하여 개발 효율성을 높였습니다.

###  Solution Selection
| Solution | Description | Decision |
| :--- | :--- | :---: |
| **AMCL** | 사전에 구축된 정밀 지도가 필요 (Map-based Localization) |X|
| **Cartographer** | **LiDAR** 데이터를 기반으로 매핑과 위치 추정을 실시간 수행 (Graph-based SLAM) |O|

**결정:** 매핑과 위치 추정이 모두 필요한 폐쇄 루프(Closed-loop) 실내 트랙 환경에 최적화된 **Cartographer**를 선정하여, 센서 데이터만으로 비교적 정확한 위치 추정을 달성했습니다.

---

## 1.2 Final Solution: Practical Stabilization 

SLAM 알고리즘을 통해 얻은 Pose 데이터는 정확도는 높지만, 센서 특유의 고주파 노이즈가 포함되어 있었습니다. 우리는 이론적으로 복잡한 필터를 억지로 적용하는 대신, **단순하지만 강력한(Robust) 실용적 접근법**을 선택하여 제어 안정성을 확보했습니다.

###  Decimal Truncation (소수점 절삭)
* **Method:** Pose 데이터의 불필요한 과도 정밀도(소수점 아래 하위 비트)를 버림으로써 데이터의 유효 숫자를 최적화.
* **Effect:** SLAM의 미세한 노이즈로 인해 조향 장치가 좌우로 떨리는 **과민 반응(Oscillation)을 효과적으로 억제**.
* **Conclusion:** 추가적인 모델링 가정이나 연산 부하 없이, **매우 안정적이고 부드러운 주행 거동**을 확보했습니다.

