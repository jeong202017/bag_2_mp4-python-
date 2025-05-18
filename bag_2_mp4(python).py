import os
import cv2
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3
import numpy as np

bag_path = '/home/chanwoo/seperator(05.18)/seperator_tanger'  # ros2 bag 기록된 폴더 경로
output_file = 'output.mp4'
topic_name = '/camera/color/image_raw'  # 저장된 이미지 토픽 이름

# bag_path 내에서 .db3 파일 자동 탐색
db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
if not db_files:
    print(f"No .db3 file found in: {bag_path}")
    exit(1)

db_path = os.path.join(bag_path, db_files[0])  # 첫 번째 .db3 파일 사용
print(f"Using bag file: {db_path}")

# SQLite DB 접근
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# 토픽 ID 가져오기
cursor.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
topic_id_result = cursor.fetchone()
if not topic_id_result:
    print(f"Topic '{topic_name}' not found in bag.")
    exit(1)
topic_id = topic_id_result[0]

# 메시지 타입 가져오기
cursor.execute("SELECT type FROM topics WHERE name=?", (topic_name,))
msg_type_str = cursor.fetchone()[0]
msg_type = get_message(msg_type_str)

# 메시지 읽기
cursor.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic_id,))
messages = cursor.fetchall()

bridge = CvBridge()
video_writer = None
frame_count = 0
fps = 30  # 또는 원하는 프레임 속도

for row in messages:
    data = row[0]
    msg = deserialize_message(data, msg_type)
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        print(f"cv_bridge error: {e}")
        continue

    if video_writer is None:
        height, width, _ = cv_image.shape
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))
        print(f"Video writing started: {output_file}")

    video_writer.write(cv_image)
    frame_count += 1

print(f"Video writing finished. Total frames: {frame_count}")
if video_writer:
    video_writer.release()