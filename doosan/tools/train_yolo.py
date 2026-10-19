"""진열대 상품 YOLO 학습.

  ~/venv/yolo/bin/python ~/couping_ws/tools/train_yolo.py [--model yolo11s.pt] [--epochs 100] [--data ...] [--name ...]

데이터: ~/couping_ws/dataset/yolo/data.yaml (make_yolo_dataset.py 로 생성)
결과:   ~/couping_ws/runs/shelf/<이름>/weights/best.pt

선택 이유
- YOLO11s: 클래스 6개·물체 크기가 작음(1280 화면에서 컵이 약 60~80px) → n 보다 정확하고 m 보다 가볍다.
- imgsz 1280: 기본 640 으로 줄이면 컵이 30~40px 로 작아져 안의 그림이 뭉개짐.
- fliplr 0: 'Tide', 'Bath Tissue' 글자가 좌우 반전되지 않게.
- hsv_h 작게: 당근·토마토처럼 색이 중요한 클래스 보존.
"""
import argparse
import os

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='yolo11s.pt')
    ap.add_argument('--epochs', type=int, default=100)
    ap.add_argument('--imgsz', type=int, default=1280)
    ap.add_argument('--batch', type=int, default=8)
    ap.add_argument('--name', default='yolo11s_1280')
    ap.add_argument('--data', default='~/couping_ws/dataset/yolo/data.yaml')
    ap.add_argument('--patience', type=int, default=30)
    args = ap.parse_args()

    data = os.path.expanduser(args.data)
    project = os.path.expanduser('~/couping_ws/runs/shelf')
    model = YOLO(args.model)  # COCO 사전학습 가중치에서 시작 (전이학습)
    model.train(
        data=data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
        project=project, name=args.name, exist_ok=True,
        patience=args.patience,  # 검증 성능이 이 epoch 수 동안 안 오르면 조기 종료
        fliplr=0.0, flipud=0.0,  # 좌우·상하 반전 없음 (글자 반전, 진열대가 뒤집힐 일 없음)
        degrees=3.0,            # 약한 회전만
        hsv_h=0.005, hsv_s=0.5, hsv_v=0.4,
        mosaic=1.0, close_mosaic=15,
        workers=4, seed=0, plots=True,
    )
    metrics = model.val(data=data, imgsz=args.imgsz, split='val')
    print(f'검증 mAP50={metrics.box.map50:.3f}  mAP50-95={metrics.box.map:.3f}')
    print('가중치:', os.path.join(project, args.name, 'weights', 'best.pt'))


if __name__ == '__main__':
    main()
