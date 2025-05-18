import os
import cv2

# 1. 영상이 저장된 폴더 (output)
video_dir = 'output'
frame_output_root = 'output_frames'

# 2. output 폴더 내 모든 mp4 파일 순회
video_files = [f for f in os.listdir(video_dir) if f.endswith('.mp4')]

if not video_files:
    print(f"[❌] No .mp4 files found in {video_dir}")
    exit(1)

for video_filename in video_files:
    video_name = os.path.splitext(video_filename)[0]
    video_path = os.path.join(video_dir, video_filename)

    # 3. 프레임 저장할 디렉토리 생성
    frame_output_dir = os.path.join(frame_output_root, video_name)
    os.makedirs(frame_output_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[⚠️] Cannot open video: {video_path}")
        continue

    frame_idx = 0
    print(f"[▶️] Processing: {video_filename}")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_filename = os.path.join(
            frame_output_dir,
            f'frame_{video_name}_{frame_idx:04d}.png'
        )
        cv2.imwrite(frame_filename, frame)
        frame_idx += 1

    cap.release()
    print(f"[✅] {frame_idx} frames saved to: {frame_output_dir}")

print("[🏁] All videos processed.")