# 모델 파일

ACT 정책 (100 에피소드로 학습, 120,000 스텝)

- 다운로드: https://github.com/kernel-robot-9th/kernel-robot-repo-1/releases/download/omx-act-120000/act_120000_pretrained_model.tgz
- 작업(task): Packaging the box
- 입력: left_wrist, right_wrist, head 카메라(640x480@30fps)와 관절 상태 12차원, 출력: 행동 12차원

받아서 풀기 (저장소 루트에서):

```bash
mkdir -p omx/models/act_120000
tar xzf act_120000_pretrained_model.tgz -C omx/models/act_120000
```

그다음 `omx/scripts/run_act.sh`의 `POLICY_PATH`를 `omx/models/act_120000/pretrained_model`로 바꿉니다.
