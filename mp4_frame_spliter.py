import os
import cv2

# 1. 분할 대상 mp4 파일 경로 (output 폴더 안에 있는 파일 이름 지정)
video_filename = 'seperator_tanger.mp4'  # 예시: 앞 코드로 생성된 파일 이름
video_path = os.path.join('output', video_filename)

# 2. 출력 프레임 저장 폴더 생성
frame_output_dir = os.path.join('output_frames', os.path.splitext(video_filename)[0])
os.makedirs(frame_output_dir, exist_ok=True)

# 3. 비디오 파일 열기
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print(f"[❌] Cannot open video: {video_path}")
    exit(1)

frame_idx = 0

# 4. 프레임 단위로 저장
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_filename = os.path.join(frame_output_dir, f'frame_{frame_idx:04d}.png')
    cv2.imwrite(frame_filename, frame)
    frame_idx += 1

cap.release()

print(f"[✅] Finished extracting {frame_idx} frames to {frame_output_dir}")