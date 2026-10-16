# 📌 Pinky Pro ArUco 정밀 자동 도킹 시스템 개발 요약

> **일자**: 2026-10-15  
> **환경**: Pinky Pro (Differential Drive) / ROS 2 Jazzy / OpenCV 4.x  
> **목표**: ArUco 마커 기반 광축-법선 벡터 일치(동축 정렬) 및 초근접(<3cm) 자동 도킹

---

## 1. 핵심 성과 & 현재 정밀도 지표

- **횡방향 거리 오차 (Lateral Error)**: **1cm 이내 달성** (마커 법선 축 정렬 완료)
- **각도 정렬 오차 (Angular Error)**: **약 3~4° 내외** (벽면 기울기 환경에서 직각 진입 성공)
- **최종 도킹 성공률**: 마커가 경사져 부착되어도 T-기동을 통해 법선 축 위로 자동 진입 후 직진 안착

---

## 2. 주요 개발 & 해결 내역

### ① ArUco 5x5 전용 탐색 & Auto-Lock
- 5x5 사전(`DICT_5X5_250`) 전용 필터링 및 ID 자동 고정(`target_marker_id: -1`)
- 타 마커 난입 시 무시하고 기존 타깃 마커만 추적

### ② 바닥 평면 투영 기하학 (Roll-Invariant Geometry)
- 마커가 옆으로 붙거나 회전되어도 영향받지 않도록 3차원 법선 벡터 $\mathbf{n}_c = R[:, 2]$를 바닥 평면$(X_c, Z_c)$에 투영
- 실제 벽면 각도(`wall_angle`) 및 법선 직선까지의 최단 거리 수선의 발($P_{closest}$) 정확 산출

### ③ 4단계 직교 T-기동 (Collinear T-Maneuver)
로봇과 마커의 광축을 일치시키기 위한 전용 기동 알고리즘 완성:
1. **Step 1 (`T_TURN_PARALLEL`)**: 벽면과 평행(마커 법선과 $90^\circ$)이 되도록 $n^\circ$만 회전
2. **Step 2 (`T_DRIVE_LATERAL`)**: 벽과 평행하게 마커 법선 축($P_{closest}$)까지 직선 이동
3. **Step 3 (`T_RETURN_FACING`)**: 마커를 정면으로 바라보도록 정확히 $90^\circ$ 회전하여 축 일치
4. **Step 4 (`FINE_ALIGN` $\rightarrow$ `APPROACH_NORMAL`)**: 광축 중심 직진 주행

### ④ 초근접 블라인드 푸시 (`BLIND_PUSH`)
- 거리 14cm 이하 근접 시 마커가 카메라 시야를 벗어나는 문제 해결
- 14cm 도달 즉시 Odometry 데드레코닝으로 전환하여 목표 거리(<3cm)까지 저속 밀착 후 완벽 정지

### ⑤ 비상 정지 및 OSD UI 개선
- **Ctrl+C 즉시 정지**: 노드 종료 시 0-속도 명령을 연속 플러시하여 로봇 폭주 방지
- **OSD 3단 디버그 뷰**: `/docking/debug_image` 및 `/docking/debug_image/compressed` 동시 발행, 글씨 겹침 없이 실시간 거리/각도/오차 표시

---

## 3. 핵심 상태 다이어그램 (FSM)

```
[SEARCH] ──(5x5 마커 감지)──> [ALIGN_BEARING]
                                     │
           ┌─────────────────────────┴─────────────────────────┐
    (각도/횡오차 허용치 초과)                               (정면 정렬 상태)
           ▼                                                   ▼
 [T_TURN_PARALLEL] (n도 회전)                          [APPROACH_NORMAL]
           ▼                                                   │
 [T_DRIVE_LATERAL] (횡이동)                                     │
           ▼                                                   │
 [T_RETURN_FACING] (90도 회전)                                 │
           ▼                                                   │
   [FINE_ALIGN]                                                │
           └─────────────────────────┬─────────────────────────┘
                                     ▼
                              [FINAL_APPROACH]
                                     │ (< 14cm 진입)
                                     ▼
                                [BLIND_PUSH] (Odom 제어)
                                     │ (< 3cm 완료)
                                     ▼
                                  [DOCKED]
```

---

## 4. 향후 과제 (Next Steps)

1. **오차 추가 압축**:
   - 휠 슬립(Slip)으로 인한 Odometry 누적 오차 보정 $\rightarrow$ 잔여 각도(3~4°)를 1~2° 이내로 축소
2. **하드웨어 도킹 스테이션 연동**:
   - 최종 3cm 안착 후 충전 단자 접촉 감지 및 전원 제어 연계
