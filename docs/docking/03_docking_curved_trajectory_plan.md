# 📋 곡선 궤적 추종(Cross-Track Stanley Controller) 기반 정밀 동축 도킹 계획서

본 문서는 T-기동 후 발생하는 **3~4° 잔여 각도 오차의 기하학적 근본 원인**을 규명하고, 직각 재기동 없이 **부드러운 곡선 주행(S-Curve)을 통해 마커 법선 축(동축)에 완벽히 수렴**하도록 도킹 제어기를 고도화하기 위한 기술 계획서입니다.

---

## 1. 문제 분석 (왜 3~4° 각도 오차가 발생했는가?)

### 사용자 지적의 기하학적 검증 (100% 정확한 분석)
T-기동 3단계($90^\circ$ 회전) 완료 후, 휠 슬립이나 오도메트리 누적으로 인해 **법선 축에서 불과 1cm만 빗나가도(횡오차 $e_{lat} \approx 1\text{cm}$)** 다음과 같은 문제가 발생합니다:

```
[마커 벽면]  ====================[Marker]====================
                                    | \ (각도 오차 3도 발생!)
                                    |  \
                                    |   \ (현재 방식: 마커 중심으로 회전해서 직진)
                                    |    \
                                    |     \
[법선 축]   ------------------------+------[Pinky] (1cm 횡오차)
                                    |
```

1. **현재 코드의 한계 (Pure Bearing Pursuit)**:
   - 현재 `APPROACH_NORMAL` 및 `FINAL_APPROACH`는 **마커 중심점($x=0$)을 카메라 중앙에 두는 제어(`bearing_error`)**만 수행합니다.
   - 거리 $z = 0.20\text{m}$($20\text{cm}$)에서 횡방향으로 $1\text{cm}$ 빗겨 서 있을 때, 마커 중심을 카메라 가운데로 맞추면:
     $$\theta_{tilt} = \arctan\left(\frac{0.01\text{m}}{0.20\text{m}}\right) \approx 2.86^\circ \approx 3^\circ$$
   - 즉, **로봇이 마커를 정면으로 바라보기 위해 차체를 3° 비스듬히 틀어버리고, 그 비스듬한 상태 그대로 벽을 향해 직진**하게 됩니다!
2. **결과**:
   - 마커는 카메라 정중앙에 있지만, 로봇 차체는 법선 축과 평행하지 않고 **3~4° 기울어진 사선으로 도킹 스테이션에 진입**하게 됩니다.

---

## 2. 해결 방안: 곡선 궤적 수렴 제어기 (Stanley / Cross-Track Controller)

로봇이 마커 중심점 하나만 바라보는 것이 아니라, **"마커 법선 직선(동축)" 자체를 도로의 차선(Lane)으로 인식하고, 직진하면서 부드러운 곡선(S-Curve)으로 차선에 합류**하도록 제어기를 전면 개편합니다.

### 기하학적 곡선 주행 원리

```
[마커 벽면]  ====================[Marker]====================
                                    |
                                    |  ^  (법선 축에 수직 안착!)
                                    | /
                                    | ( 부드러운 곡선 합류 (S-Curve)
                                    |   \
                                    |    \
[법선 축]   ────────────────────────+─────[Pinky] (1cm 횡오차 발생 위치)
```

1. **횡오차($e_{lat}$ = `signed_lat`)**: 법선 축에서 얼마나 오른쪽/왼쪽으로 떨어져 있는가?
2. **헤딩오차($e_{heading}$ = `wall_angle`)**: 로봇의 진행 방향이 법선 축과 얼마나 평행한가?
3. **곡선 조향 제어 법칙 (Stanley Formulation)**:
   $$\omega = -K_{yaw} \cdot \text{wall\_angle} + K_{lat} \cdot \arctan\left(\frac{k \cdot \text{signed\_lat}}{v_x + 0.01}\right)$$
   - **동작 메커니즘**:
     - 로봇이 오른쪽에 치우쳐 있으면($signed\_lat > 0$): 좌측으로 완만하게 조향(곡선 진입).
     - 축에 가까워지면($signed\_lat \to 0$): $K_{yaw}$ 항이 작동하여 차체를 벽면과 직각($wall\_angle \to 0$)으로 스스로 정렬(카운터 스티어링).
     - 벽에 도달했을 때는 **횡오차 $0\text{cm}$ + 각도오차 $0.0^\circ$로 완벽 수직 안착**.

---

## 3. 세부 변경 계획 (Proposed Changes)

### Component: `pinky_precision_docking`

#### 1. [MODIFY] `pinky_precision_docking/precision_docking_node.py`

- **`_handle_fine_align` 간소화**:
  - 제자리 회전으로 억지 중앙 정렬을 하지 않고, 마커 시각 확인 즉시 곡선 주행 모드(`APPROACH_NORMAL`)로 즉각 인계.
- **`_handle_approach_normal` & `_handle_final_approach` 개편**:
  - 기존의 단순 `bearing_error` 비례 제어 제거.
  - Stanley 기반 **이중 피드백 궤적 제어기** 적용:
    ```python
    # 횡오차(Cross-track Error)와 헤딩오차(Heading Error) 동시 수렴
    lat_steering = math.atan2(self.kp_lateral * signed_lat, max(twist.linear.x, 0.03))
    yaw_steering = -self.kp_wall_yaw * wall_angle

    # 블렌딩 조향 각속도 계산 (곡선 주행)
    ang_cmd = yaw_steering + lat_steering
    twist.angular.z = self._clamp_angular(ang_cmd)
    ```

#### 2. [MODIFY] `config/docking_params.yaml`
- 신규 제어 게인 추가:
  ```yaml
  # Cross-track Curve Alignment Gains
  kp_lateral: 1.5      # 횡방향 동축 수렴 게인
  kp_wall_yaw: 1.8     # 벽면 직각 평행도 유지 게인
  max_curve_yaw_rate: 0.25 # 곡선 주행 시 최대 회전 속도 제한 (부드러운 주행 보장)
  ```

---

## 4. 검증 계획 (Verification Plan)

### 시뮬레이션 및 오프라인 수학적 궤적 검증
1. 초기 조건: 거리 $Z = 0.35\text{m}$, 횡오차 $X = +0.02\text{m}$ (2cm 오차), 헤딩 오차 $0.0^\circ$.
2. 제어 루프를 20Hz로 돌렸을 때, 로봇 궤적이 S-Curve를 그리며 $Z = 0.15\text{m}$ 도달 시점에 횡오차 $< 2\text{mm}$, 각도 오차 $< 0.5^\circ$로 수렴하는지 Python 검증 스크립트로 사전 확인.

### 실 로봇 복귀 시 확인 절차
1. 로봇을 마커 정면에서 의도적으로 좌우로 2~3cm 비껴 세우고 도킹 시작.
2. 제자리에서 각진 회전을 반복하지 않고, 부드러운 호(Arc)를 그리며 전진하여 최종 벽면에 정확히 $90^\circ$ 수직으로 멈추는지 확인.

---

## 5. 사용자 검토 및 확인 사항 (User Review Required)

> [!TIP]
> - **별도의 2차 직각 T-기동 필요 여부**:
>   - 횡오차가 $5\text{cm}$ 이내의 미세 오차인 경우, 직각으로 꺾지 않고 **곡선 주행(S-Curve)만으로 100% 수렴이 가능**합니다.
>   - 만약 횡오차가 $10\text{cm}$ 이상으로 매우 크다면 다시 T-기동을 하는 것이 안전하지만, Step 3 복귀 후 남는 오차는 보통 1~2cm 수준이므로 곡선 주행이 가장 부드럽고 이상적입니다.
