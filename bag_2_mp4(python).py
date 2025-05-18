import os
import cv2
import sqlite3
import numpy as np
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# 1. .bag 폴더 경로 지정
bag_path = '/home/chanwoo/seperator(05.18)/seperator_semi_tanger'

# 경로 후보
# seperator_ripe_godoo
# seperator_ripe_no
# seperator_ripe_yeolgwa
# seperator_semi_no
# seperator_semi_tanger
# seperator_tanger

topic_name = '/camera/color/image_raw'  # 원하는 이미지 토픽

# 2. .db3 파일 자동 탐색
db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
if not db_files:
    print(f"[❌] No .db3 file found in: {bag_path}")
    exit(1)

db_filename = db_files[0]
db_path = os.path.join(bag_path, db_filename)
print(f"[🔍] Using bag file: {db_path}")

# 3. .db3 이름 기반 mp4 출력 파일 이름 생성
base_name = os.path.splitext(db_filename)[0]

# 🔄 현재 실행 디렉토리 기준으로 output 폴더 생성
current_dir = os.getcwd()
output_dir = os.path.join(current_dir, 'output')
os.makedirs(output_dir, exist_ok=True)

# 🔄 output.mp4 경로 생성
output_file = os.path.join(output_dir, f'{base_name}.mp4')

# 4. SQLite DB 연결
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# 5. 토픽 ID 및 메시지 타입 확인
cursor.execute("SELECT id, type FROM topics WHERE name=?", (topic_name,))
result = cursor.fetchone()
if not result:
    print(f"[❌] Topic '{topic_name}' not found in bag.")
    exit(1)

topic_id, msg_type_str = result
msg_type = get_message(msg_type_str)

# 6. 메시지 읽기
cursor.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic_id,))
messages = cursor.fetchall()

bridge = CvBridge()
video_writer = None
frame_count = 0
fps = 30  # 원하는 FPS 설정

# 7. 이미지 → 영상 저장
for row in messages:
    data = row[0]
    msg = deserialize_message(data, msg_type)
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        print(f"[⚠️] cv_bridge error: {e}")
        continue

    if video_writer is None:
        height, width, _ = cv_image.shape
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))
        print(f"[✅] Video writing started: {output_file}")

    video_writer.write(cv_image)
    frame_count += 1

# 8. 정리
if video_writer:
    video_writer.release()

print(f"[🎞️] Video writing finished. Total frames: {frame_count}")
