# bag_2_mp4-python-

다음은 ROS2 `.bag` 파일에서 이미지 토픽을 추출해 `.mp4` 영상으로 변환하는 과정을 설명하는 **README.md 예시**입니다. 아래 내용은 `bag_2_mp4(python).py` 파일을 사용하는 전체 과정, 실행 방법, 환경 설정 등을 포함하고 있습니다.

---

## 📦 ROS2 Bag to MP4 변환 스크립트

ROS2 `.bag` 파일에서 특정 이미지 토픽을 추출하여 `.mp4` 비디오로 저장하는 Python 스크립트입니다. 이 스크립트는 ROS2 환경과 Python 3.8에서 작동하도록 설계되었습니다.

---

### 📋 요구사항

* ROS2 Foxy (또는 Python 3.8 기반 ROS2)
* Python 3.8
* `cv_bridge`, `opencv-python`, `numpy`
* `.bag` 기록 폴더 (예: `/camera/color/image_raw`와 같은 이미지 토픽 포함)

---

### ⚙️ 환경 설정

#### 1. ROS2 Foxy 환경 활성화

```bash
source /opt/ros/foxy/setup.bash
```

#### 2. Python 3.8 설치 (필요한 경우)

```bash
sudo apt update
sudo apt install python3.8 python3.8-venv python3.8-dev
```

#### 3. 가상환경 생성 및 패키지 설치 (권장)

```bash
python3.8 -m venv ~/ros2_py38_env
source ~/ros2_py38_env/bin/activate

pip install -U pip
pip install opencv-python numpy
```

> `cv_bridge`는 ROS2 설치와 함께 제공되므로 별도 설치가 필요하지 않습니다.

---

### 📁 파일 구조 예시

```
project_folder/
├── bag_2_mp4(python).py
└── your_ros2_bag_folder/
    ├── metadata.yaml
    ├── ...
    └── your_bag_file.db3
```

---

### 🏃 실행 방법

1. Python 스크립트에 사용할 `.bag` 폴더 경로를 수정합니다.

```python
# bag_2_mp4(python).py 내부
bag_path = '/absolute/path/to/your_ros2_bag_folder'
topic_name = '/camera/color/image_raw'  # 필요한 이미지 토픽 이름
```

2. 실행

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_py38_env/bin/activate  # 가상환경 사용 시

python3.8 bag_2_mp4\(python\).py
```

> `zsh` 사용자는 경로에 괄호가 있는 경우 `\`로 escape 해야 하며, `()` 대신 `_`로 리네이밍하는 것도 추천합니다.

---

### 🎞️ 출력 결과

* 영상 파일은 `.bag` 폴더 내부에 `.db3` 파일과 같은 이름으로 `.mp4` 형식으로 저장됩니다.
* 예: `your_bag_file.db3` → `your_bag_file.mp4`

---

### ✅ 참고 사항

* 다수의 `.db3` 파일이 있는 경우, 첫 번째 파일만 사용됩니다.
* 영상 저장 시 기본 프레임레이트는 30 FPS이며 필요 시 수정 가능합니다.
* `.bag` 내에 지정된 이미지 토픽이 존재하지 않으면 오류가 발생합니다.

---
