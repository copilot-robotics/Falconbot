# Falconbot — 鹰隼机器人控制界面

基于 Flask 的 Web 控制界面，用于驱动一台仿鹰隼形态的舵机机器人。支持多关节同步控制、行为动作（情绪动作 + 叫声联动）、正弦运动、摄像头、音频播放/录制、人脸检测等功能。

---

## 项目结构

```
web-servo-control/
├── app.py                      # Flask 后端主程序（所有 API 路由 + 舵机控制逻辑）
├── templates/
│   ├── index.html              # 控制台前端单页（关节控制、行为动作、摄像头、音频、LLM）
│   └── login.html              # 登录/注册页
├── static/
│   └── sounds/                 # 鹰隼叫声 WAV/MP3 文件（cry / cackle / whistle）
├── face_detection/
│   └── face_detector.py        # 人脸检测模块（OpenCV Haar 级联）
├── instance/
│   └── users.db                # SQLite 用户数据库
└── README.md
```

### 后端模块（app.py）

| 模块 | 说明 |
|------|------|
| **用户认证** | 注册/登录/短信验证码/Session 管理 |
| **舵机控制** | 串口连接、同步写位置、单舵机移动、紧急停止、位置轮询 |
| **正弦运动** | 可配置频率/幅值/相位的关节周期运动 |
| **行为动作引擎** | 4 种情绪动作（Idle/Happy/Calm/Enthusiastic），每种对应不同的关节运动参数和鹰隼叫声循环 |
| **摄像头** | 实时视频流、录像录制与回放 |
| **音频系统** | 鹰隼叫声播放、麦克风录音、音量控制 |
| **人脸检测** | 开关式人脸检测叠加显示 |
| **LLM 配置** | 大语言模型接口参数在线配置 |

### 舵机 ID — 关节映射

| ID | 关节 |
|----|------|
| 2  | 尾巴 Tail |
| 3  | 右腿 Hip |
| 4  | 右腿 Knee |
| 5  | 右腿 Ankle |
| 6  | 左腿 Hip |
| 7  | 左腿 Knee |
| 8  | 左腿 Ankle |
| 9  | 头部 Yaw |
| 10 | 头部 Pitch |
| 11 | 嘴巴开合 Mouth |

---

## 快速启动

### 环境要求

- Python 3.8+
- 依赖：Flask, Flask-SQLAlchemy, pyserial, opencv-python, numpy, scikit-learn
- 系统工具：`aplay`（音频播放）、`arecord`（录音）、`ffmpeg`（音频转码）
- 硬件：USB 转串口（TTL）， Feetech SCS 系列舵机

### 安装与运行

```bash
cd web-servo-control
pip install flask flask-sqlalchemy pyserial opencv-python numpy scikit-learn
python3 app.py
```

浏览器访问 `http://<本机IP>:5000/`，登录账号 `admin` / 密码 `admin123`。

---

## GUI 操作使用流程

### 1. 登录

打开网页后输入用户名和密码登录（首次使用可注册新账号，支持短信验证码）。

### 2. 连接舵机

1. 在控制台顶部 **Serial Connection** 区域选择串口设备（如 `/dev/ttyUSB0`）
2. 选择波特率（默认 1000000）
3. 点击 **Connect**，成功后状态指示灯变绿，关节位置滑块开始显示实时位置

### 3. 手动控制关节

- **关节控制面板**：每个舵机 ID 对应一个滑块，拖动滑块即可设置目标位置（0–4095）
- **移动速度/加速度**：可在移动时指定速度和加速度
- **紧急停止**：点击 **Emergency Stop** 按钮立即禁用所有舵机扭矩

### 4. 正弦周期运动

1. 在 **Sin Motion** 面板配置频率、幅值、相位
2. 点击 **Start** 开始周期运动，**Stop** 停止

### 5. 行为动作（Emotion Actions）

这是本项目的核心功能面板，位于动作按钮下方：

| 按钮 | 动作 | 叫声 | 说明 |
|------|------|------|------|
| 💤 Idle | 全身微动 | 无 | 小幅度随机运动模拟活体动物待机 |
| 😊 Happy | 全身运动 | Cackle 咯咯声 | 所有关节大幅度动作 |
| 😌 Calm | 头+嘴 | Whistle 哨声 | 仅头部和嘴巴缓慢动作 |
| 🤗 Enthusiastic | 腿部 | Cry 鸣叫 | 仅双腿行进步态 |

**操作方式**：
1. 点击任意动作按钮，机器人开始执行该动作
2. 右侧开关切换 **Animation Only（仅动画）** / **Direct Drive（直接驱动机器人）**
   - 仅动画：不连接串口也可观看杆状动画
   - 直接驱动：需先连接串口，机器人与动画同步运动
3. 杆状动画面板实时显示 10 个关节的位置变化，当前动作涉及的关节高亮
4. 动作进行时循环播放对应叫声，点击 **Stop** 后动作和叫声同时停止，舵机回中位

### 6. 摄像头

- **Camera** 面板点击 **Start** 开启实时视频流
- 可录制视频片段并回放/下载
- 开启 **Face Detection** 可在视频上叠加人脸框

### 7. 音频测试

- **Speaker & Mic Test** 面板：
  - 点击鹰隼叫声按钮播放对应叫声
  - 点击 **Record** 录制麦克风音频，可回放和下载
  - 可调节扬声器音量

### 8. LLM 配置

- 在 **LLM Config** 面板中配置大语言模型的 API 地址和密钥
- 用于扩展对话式交互功能

---

## API 接口概览

| 路由 | 方法 | 功能 |
|------|------|------|
| `/api/auth/login` | POST | 用户登录 |
| `/api/auth/register` | POST | 用户注册 |
| `/api/connect` | POST | 连接串口 |
| `/api/disconnect` | POST | 断开串口 |
| `/api/move` | POST | 移动单个舵机 |
| `/api/emergency_stop` | POST | 紧急停止 |
| `/api/sin_motion/start` | POST | 启动正弦运动 |
| `/api/emotion/start` | POST | 启动行为动作（含叫声循环） |
| `/api/emotion/stop` | POST | 停止行为动作 |
| `/api/emotion/status` | GET | 查询动作状态和关节位置 |
| `/api/camera/start` | POST | 开启摄像头 |
| `/api/camera/record/start` | POST | 开始录像 |
| `/api/audio/play/<animal>` | POST | 播放鹰隼叫声 |
| `/api/audio/record` | POST | 麦克风录音 |
| `/api/face/detection` | POST | 开关人脸检测 |
| `/api/llm/config` | GET/POST | 查看/更新 LLM 配置 |
