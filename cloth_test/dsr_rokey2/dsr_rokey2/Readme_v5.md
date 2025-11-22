# 🤖 Firebase 기반 로봇 제어 시스템

Firebase Realtime Database를 활용한 두산로보틱스 협동로봇(M0609) 원격 제어 시스템입니다.

## 📋 개요

웹 인터페이스(HTML)에서 버튼 클릭 → Firebase 명령 전달 → ROS2 기반 로봇 동작 실행

### 주요 기능
- 실시간 Firebase 폴링 (0.5초 주기)
- 웹 기반 원격 로봇 제어
- 작업 완료 카운터 자동 업데이트
- 그리퍼 제어 및 복합 동작 시퀀스

## 🗂️ 버전 정보

- **v2**: 기본 Firebase 폴링 방식 (이 문서 기준)
- **v5**: 개선된 버전 (별도 문서 참조)

## 🔧 시스템 요구사항

### 하드웨어
- 두산로보틱스 M0609 협동로봇
- ROS2 실행 환경 (Ubuntu 22.04 권장)
- 그리퍼 (디지털 I/O 1, 2번 사용)

### 소프트웨어
```bash
# ROS2 Humble
# Python 3.10+
# Firebase Admin SDK
# DR_init, DSR_ROBOT2 라이브러리
```

## 📦 설치

### 1. 의존성 설치
```bash
pip install firebase-admin
```

### 2. Firebase 설정
1. Firebase Console에서 프로젝트 생성
2. 서비스 계정 키 다운로드 (`rokey-550f7-firebase-adminsdk-fbsvc-eba1fa0ef4.json`)
3. Realtime Database 생성 (위치: asia-southeast1)
4. 키 파일을 프로젝트 루트에 배치

### 3. Firebase Database 구조
```json
{
  "robot": {
    "robot_status": "waiting",
    "robot_command": "idle",
    "sales_count": 0,
    "completed_count": 0,
    "last_completed_time": 0
  }
}
```

## 🚀 실행 방법

### ROS2 환경 설정
```bash
source /opt/ros/humble/setup.bash
source ~/your_workspace/install/setup.bash
```

### 로봇 제어 프로그램 실행
```bash
python3 robot_firebase_v2.py
```

### 실행 시 출력 예시
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
  VELOCITY: 100
  ACC: 100
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

## 🎮 제어 방법

### 웹 인터페이스에서 제어
1. HTML 페이지에서 **[로봇 작동]** 버튼 클릭
2. Firebase에 `robot_command: "start"` 전송
3. 로봇이 자동으로 작업 시퀀스 수행
4. 완료 후 `completed_count` 자동 증가

### Firebase 직접 제어 (테스트용)
Firebase Console에서 수동 업데이트:
```json
{
  "robot_command": "start"
}
```

## 📊 상태 모니터링

### 로봇 상태 (`robot_status`)
- `waiting`: 대기 중
- `working`: 작업 수행 중
- `error`: 오류 발생

### 명령 상태 (`robot_command`)
- `idle`: 대기
- `start`: 시작 명령 (외부에서 설정)
- `processing`: 처리 중

## 🔄 동작 시퀀스

코드에 정의된 로봇 작업 순서:
1. 그리퍼 열기
2. 픽업 위치로 이동 (movej)
3. 물체 파지 (closegripper)
4. 배치 위치로 이동 (movec, movel)
5. 그리퍼 열기
6. 다음 작업 수행
7. 홈 위치 복귀

## ⚙️ 설정 변경

### 코드 상단 상수 수정
```python
# 로봇 설정
ROBOT_ID = "dsr01"          # 로봇 ID
ROBOT_MODEL = "m0609"        # 로봇 모델

# 속도 설정
VELOCITY = 100               # 기본 속도 (%)
ACC = 100                    # 기본 가속도 (%)

# Firebase 경로
SERVICE_ACCOUNT_KEY_PATH = "./your-firebase-key.json"
DATABASE_URL = "https://your-project.firebasedatabase.app"
```

### 폴링 주기 변경
```python
check_interval = 0.5  # 초 단위 (기본 0.5초)
```

## 🐛 문제 해결

### Firebase 연결 실패
```
❌ Firebase 초기화 실패: [Errno 2] No such file or directory
```
→ 서비스 계정 키 파일 경로 확인

### 로봇 통신 오류
```
❌ 로봇 작업 중 에러: ...
```
→ ROS2 환경 및 로봇 네트워크 연결 확인

### 명령이 실행되지 않음
- Firebase Database 규칙 확인 (읽기/쓰기 권한)
- 폴링 주기 확인 (콘솔 로그)
- `robot_command` 값 확인

## 📝 로그 해석

```
🔔🔔🔔... 로봇 시작 명령 수신!
```
→ Firebase에서 'start' 명령 감지

```
✅ 작업 완료! (총 5개)
```
→ 작업 완료 및 카운터 업데이트

## 🔐 보안 주의사항

- **절대 공개 저장소에 Firebase 키 파일 업로드 금지**
- `.gitignore`에 `*.json` 추가 권장
- Firebase Database 규칙 설정:
  ```json
  {
    "rules": {
      "robot": {
        ".read": "auth != null",
        ".write": "auth != null"
      }
    }
  }
  ```

## 📞 지원

문제 발생 시:
1. 터미널 출력 로그 확인
2. Firebase Console에서 데이터 상태 확인
3. 로봇 티치펜던트에서 에러 메시지 확인

## 📄 라이선스

이 프로젝트는 교육 및 연구 목적으로 작성되었습니다.

---

**작성일**: 2024  
**작성자**: Junseok  
**버전**: v2 (Firebase Polling)


이 README는:
- ✅ 실행 방법 명확히 제시
- ✅ 초보자도 따라할 수 있는 단계별 가이드
- ✅ 실제 출력 예시로 이해도 향상
- ✅ 문제 해결 섹션 포함
- ✅ 보안 주의사항 강조

