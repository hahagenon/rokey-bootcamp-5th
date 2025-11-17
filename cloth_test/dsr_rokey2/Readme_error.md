# 옷 접기 로봇 Firebase 모니터링 시스템

두산 DSR01 M0609 협동로봇을 Firebase와 연동하여 웹/모바일에서 실시간으로 모니터링하고 제어하는 시스템

---

## 📋 목차

- [시스템 개요](#시스템-개요)
- [시스템 구성](#시스템-구성)
- [필요 파일](#필요-파일)
- [설치 방법](#설치-방법)
- [실행 방법](#실행-방법)
- [사용 방법](#사용-방법)
- [문제 해결](#문제-해결)
- [기능 설명](#기능-설명)

---

## 시스템 개요

### 프로젝트 목적
- 소매점(유니클로, 탑텐 등)에서 티셔츠 자동 접기 시스템
- 판매 개수와 로봇 작업 횟수를 실시간 모니터링
- 모바일/웹에서 원격으로 로봇 제어 및 상태 확인

### 주요 기능
- **판매 등록**: 바코드 리더 시뮬레이션 (현재 버튼 방식, 추후 바코드 연동)
- **로봇 제어**: 웹/모바일에서 로봇 작동 명령
- **실시간 모니터링**: 판매 개수, 로봇 완료 개수, 수동 포장 개수
- **관리자 기능**: 데이터 초기화

---

## 시스템 구성

```
┌─────────────────┐         ┌──────────────┐         ┌─────────────────┐
│  웹/모바일      │ ←────→  │   Firebase   │ ←────→  │  로봇 컴퓨터    │
│  (dashboard)    │         │  (클라우드)  │         │  (ROS2 + 로봇) │
└─────────────────┘         └──────────────┘         └─────────────────┘
   버튼 클릭                    실시간 동기화              로봇 제어
   상태 표시                    데이터 저장                Firebase 폴링
```

### 기술 스택
- **로봇 제어**: Python 3 + ROS2 + 두산 로보틱스 SDK
- **클라우드**: Firebase Realtime Database
- **웹/모바일**: HTML5 + JavaScript + Firebase Web SDK
- **통신 방식**: Firebase 폴링 (0.5초 간격)

---

## 필요 파일

### 1. `python_control.py` (로봇 컴퓨터)
- 로봇 제어 및 Firebase 연동
- 위치: `~/cobot1_ws/src/doosan-robot2/dsr_rokey2/dsr_rokey2/`

### 2. `dashboard.html` (웹 서버)
- 모니터링 대시보드
- 위치: `~/cobot1_ws/src/doosan-robot2/dsr_rokey2/web/`

### 3. Firebase 키 파일
- `rokey-550f7-firebase-adminsdk-fbsvc-eba1fa0ef4.json`
- 위치: `python_control.py`와 같은 폴더

---

## 설치 방법

### 1. Python 의존성 설치

```bash
# pip3 설치 (없는 경우)
sudo apt install python3-pip

# Firebase Admin SDK 설치
pip3 install --user firebase-admin
```

### 2. 파일 배치

```bash
# 프로젝트 폴더 구조
~/cobot1_ws/src/doosan-robot2/dsr_rokey2/
├── dsr_rokey2/
│   ├── python_control.py                              # 로봇 제어
│   └── rokey-550f7-firebase-adminsdk-fbsvc-*.json     # Firebase 키
└── web/
    └── dashboard.html                                  # 대시보드
```

### 3. Firebase 프로젝트 설정 확인

Firebase 콘솔에서 Realtime Database 활성화 확인:
- URL: `https://rokey-550f7-default-rtdb.asia-southeast1.firebasedatabase.app`
- 데이터베이스 경로: `/robot`

---

## 실행 방법

### 로봇 컴퓨터에서

#### 1단계: 로봇 제어 프로그램 실행

```bash
cd ~/cobot1_ws/src/doosan-robot2/dsr_rokey2/dsr_rokey2/
python3 python_control.py
```

**정상 실행 시 출력:**
```
============================================================
Firebase 초기화 시작...
============================================================
✅ Firebase 초기화 완료!
✅ Firebase 경로 설정: /robot

############################################################
로봇 초기화 시작:
  ROBOT_ID: dsr01
  ROBOT_MODEL: m0609
  ROBOT_TCP: GripperDA
  ROBOT_TOOL: Tool Weight
############################################################
✅ 로봇 초기화 완료!

============================================================
🚀 로봇 Firebase 폴링 시작!
📊 현재 상태:
   - 판매: 0개
   - 완료: 0개
   - 상태: 대기 중

💡 0.5초마다 Firebase 확인 중...
   HTML에서 [로봇 작동] 버튼을 눌러보세요!
============================================================
```

#### 2단계: 웹 서버 실행 (새 터미널)

```bash
cd ~/cobot1_ws/src/doosan-robot2/dsr_rokey2/web/
python3 -m http.server 8000 --bind 0.0.0.0
```

**정상 실행 시 출력:**
```
Serving HTTP on 0.0.0.0 port 8000 (http://0.0.0.0:8000/) ...
```

#### 3단계: IP 주소 확인

```bash
# 무선 네트워크 IP 확인
ip addr show wlp1s0 | grep "inet "
```

**예시 출력:**
```
inet 172.16.0.220/24 ...
```
→ IP 주소: `172.16.0.220`

---

### 웹/모바일에서 접속

#### 같은 컴퓨터에서:
```
http://localhost:8000/dashboard.html
```

#### 다른 기기(폰, 태블릿)에서:
```
http://172.16.0.220:8000/dashboard.html
```
(IP 주소는 3단계에서 확인한 것 사용)

---

## 사용 방법

### 일반 사용 (직원)

#### 1. 판매 등록
```
[판매 +1] 버튼 클릭
→ 판매 개수 증가
→ 바코드 리더 시뮬레이션
```

#### 2. 로봇 작동
```
1. 티셔츠를 로봇에 올리기
2. [로봇 작동 ▶] 버튼 클릭
3. 로봇이 1개 접기 (약 45초)
4. 완료 후 "로봇 완료" 개수 자동 증가
```

#### 3. 실시간 모니터링
```
- 오늘 판매: 총 판매된 개수
- 🤖 로봇 완료: 로봇이 접은 개수
- 👤 수동 포장: 판매 - 로봇 완료 (바지 등)
- 로봇 상태: 🟢 대기 중 / 🟡 작동 중
```

---

### 관리자 사용

#### 데이터 초기화
```
1. 화면 하단 [⚙️ 관리자 설정] 클릭
2. [🔄 오늘 데이터 초기화] 버튼 나타남
3. 클릭 → 확인창 → "확인" 선택
4. 모든 카운터가 0으로 리셋
```

**확인창 예시:**
```
정말 초기화하시겠습니까?

• 판매: 15개 → 0개
• 로봇 완료: 10개 → 0개

이 작업은 되돌릴 수 없습니다.
```

---

## 문제 해결

### 문제 1: `ModuleNotFoundError: No module named 'firebase_admin'`

**원인:** Firebase Admin SDK가 설치되지 않음

**해결:**
```bash
pip3 install --user firebase-admin
```

---

### 문제 2: `AttributeError: 'Reference' object has no attribute 'on'`

**원인:** Python Firebase Admin SDK는 `.on()` 실시간 리스너를 지원하지 않음

**해결:** 이미 폴링 방식으로 수정된 코드 사용 (0.5초마다 Firebase 확인)

**코드 확인:**
```python
# 잘못된 방식 (에러)
robot_ref.on('value', callback)  # ❌

# 올바른 방식 (현재 코드)
while True:
    data = robot_ref.get()  # ✅
    # 명령 확인 및 처리
    time.sleep(0.5)
```

---

### 문제 3: 폰에서 대시보드 접속 안 됨

#### 3-1. 404 에러 (File not found)

**원인:** dashboard.html 파일이 없는 폴더에서 웹 서버 실행

**해결:**
```bash
# 올바른 폴더로 이동
cd ~/cobot1_ws/src/doosan-robot2/dsr_rokey2/web/

# 파일 확인
ls dashboard.html

# 웹 서버 실행
python3 -m http.server 8000 --bind 0.0.0.0
```

**로그 확인:**
```
172.16.0.220 - - [...] "GET /dashboard.html HTTP/1.1" 200  ✅ 성공
172.16.0.220 - - [...] "GET /dashboard.html HTTP/1.1" 404  ❌ 파일 없음
```

---

#### 3-2. 무한 로딩 / 연결 불가

**원인 1: 다른 네트워크에 연결**

**확인:**
```bash
# 컴퓨터 IP
ip addr show wlp1s0 | grep "inet "
# 예: 172.16.0.220

# 폰 IP (폰 설정에서 확인)
# 예: 172.16.0.154

# 앞 3자리가 같아야 함!
# 172.16.0.XXX ✅ 같은 네트워크
# 192.168.0.XXX ❌ 다른 네트워크
```

**해결:** 컴퓨터와 폰을 같은 WiFi에 연결

---

**원인 2: 방화벽 차단**

**확인:**
```bash
sudo ss -tlnp | grep 8000
```

**올바른 출력:**
```
LISTEN 0  5  0.0.0.0:8000  0.0.0.0:*  ✅ 모든 IP에서 접속 가능
LISTEN 0  5  172.16.0.220:8000  0.0.0.0:*  ❌ 특정 IP만
```

**해결:**
```bash
# 방화벽 포트 열기
sudo ufw allow 8000

# 웹 서버 재시작 (0.0.0.0 바인딩)
python3 -m http.server 8000 --bind 0.0.0.0
```

---

**원인 3: WiFi AP Isolation (공용 WiFi)** ⭐ 가장 흔한 문제

**증상:**
- 같은 WiFi에 연결됨
- 방화벽 설정 완료
- ping도 안 됨
```bash
ping 172.16.0.154  # 폰 IP
# Request timeout ❌
```

**원인:** 학교/회사/공용 WiFi는 보안상 기기 간 통신을 차단

**해결 방법 1: 폰 핫스팟 사용** (추천) ⭐

```bash
# 1. 폰에서 개인 핫스팟 켜기
# 2. 컴퓨터를 폰 핫스팟에 연결
# 3. 컴퓨터 IP 다시 확인
ip addr show | grep "inet "

# 4. 웹 서버 실행
cd ~/cobot1_ws/src/doosan-robot2/dsr_rokey2/web/
python3 -m http.server 8000 --bind 0.0.0.0

# 5. 폰 브라우저에서 접속 (새 IP 사용)
# http://192.168.43.100:8000/dashboard.html (예시)
```

**해결 방법 2: 온라인 호스팅**

- dashboard.html을 GitHub Pages, Firebase Hosting, Netlify 등에 업로드
- 인터넷을 통해 어디서든 접속 가능
- 로봇과 같은 Firebase DB 사용

---

### 문제 4: 로봇 버튼 눌러도 반응 없음

**확인 사항:**

**1. Python 실행 중인가?**
```bash
# 터미널에서 확인
python3 python_control.py

# "💡 0.5초마다 Firebase 확인 중..." 메시지 있어야 함
```

**2. Firebase 연결 확인**
- Firebase 콘솔 접속: https://console.firebase.google.com/
- Realtime Database → `/robot` 경로 확인
- `robot_command: "start"` 값이 보이는가?

**3. 터미널 로그 확인**
```
[로봇 작동] 버튼 클릭 시:

🔔🔔🔔🔔🔔🔔🔔🔔🔔🔔
로봇 시작 명령 수신!
🔔🔔🔔🔔🔔🔔🔔🔔🔔🔔
```
→ 이 메시지가 나와야 정상

---

### 문제 5: `pip3: command not found`

**해결:**
```bash
sudo apt install python3-pip
```

---

### 문제 6: `netstat: command not found`

**대체 명령어 사용:**
```bash
sudo ss -tlnp | grep 8000
```

**또는 설치:**
```bash
sudo apt install net-tools
```

---

## 기능 설명

### Firebase 데이터 구조

```json
/robot
  ├── sales_count: 15              // 판매 개수
  ├── completed_count: 10          // 로봇 완료 개수
  ├── robot_status: "waiting"      // 로봇 상태 (waiting/working/error)
  ├── robot_command: "idle"        // 명령 (idle/start/processing)
  └── last_completed_time: 1700... // 마지막 완료 시간 (timestamp)
```

### 로봇 작동 흐름

```
1. [로봇 작동] 버튼 클릭 (HTML)
   ↓
2. Firebase 업데이트: robot_command = "start"
   ↓
3. Python 폴링으로 명령 감지 (0.5초 이내)
   ↓
4. robot_status = "working" 업데이트
   ↓
5. 로봇 작업 수행 (약 45초)
   ↓
6. completed_count + 1
   ↓
7. robot_status = "waiting", robot_command = "idle"
```

### 폴링 방식 작동 원리

**왜 폴링 방식인가?**
- Python Firebase Admin SDK는 실시간 리스너(`.on()`)를 지원하지 않음
- 대신 주기적으로 Firebase를 확인하는 폴링 방식 사용

**성능:**
- 확인 주기: 0.5초 (500ms)
- 반응 속도: 버튼 클릭 후 최대 0.5초 이내
- CPU 사용량: 매우 낮음

**조정 방법:**
```python
# python_control.py 내부
check_interval = 0.5  # ← 이 값 변경

# 예시:
# 0.1 = 더 빠른 반응, CPU 사용 약간 증가
# 1.0 = 느린 반응, CPU 사용 감소
```

---

## 시스템 요구사항

### 하드웨어
- 두산 DSR01 M0609 협동로봇
- 그리퍼 (디지털 출력 1, 2번 사용)
- 로봇 제어 컴퓨터 (Ubuntu 24)
- WiFi 연결

### 소프트웨어
- Ubuntu 24.04
- Python 3.x
- ROS2 (Humble 또는 Foxy)
- 두산 로보틱스 ROS2 패키지
- Firebase Admin SDK
- 웹 브라우저 (Chrome, Safari 등)

---

## 향후 개선 계획

### 1단계 (현재) ✅
- [x] Firebase 연동
- [x] 판매/완료 카운팅
- [x] 로봇 원격 제어
- [x] 관리자 초기화 기능


---

## 라이선스

이 프로젝트는 교육 및 연구 목적으로 개발되었습니다.

---

## 문의

문제가 발생하거나 질문이 있으시면 이슈를 등록해주세요.

---

**마지막 업데이트:** 2025-11-16