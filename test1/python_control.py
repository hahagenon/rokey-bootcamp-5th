import rclpy
import DR_init
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import threading
import time

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 이동 속도 및 가속도
VELOCITY = 100
ACC = 100

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# Firebase 설정
SERVICE_ACCOUNT_KEY_PATH = "./rokey-550f7-firebase-adminsdk-fbsvc-eba1fa0ef4.json"
DATABASE_URL = "https://rokey-550f7-default-rtdb.asia-southeast1.firebasedatabase.app"

print("=" * 60)
print("Firebase 초기화 시작...")
print("=" * 60)

# Firebase 초기화
try:
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {
        'databaseURL': DATABASE_URL
    })
    print("✅ Firebase 초기화 완료!")
except ValueError:
    print("⚠️  Firebase 앱이 이미 초기화되었습니다.")
except Exception as e:
    print(f"❌ Firebase 초기화 실패: {e}")
    exit(1)

# Firebase 참조
robot_ref = db.reference('/robot')
print(f"✅ Firebase 경로 설정: /robot")


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp

    print("\n" + "#" * 60)
    print("로봇 초기화 시작:")
    print(f"  ROBOT_ID: {ROBOT_ID}")
    print(f"  ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"  ROBOT_TCP: {ROBOT_TCP}")
    print(f"  ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"  VELOCITY: {VELOCITY}")
    print(f"  ACC: {ACC}")
    print("#" * 60)

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    print("✅ 로봇 초기화 완료!\n")


def perform_task():
    """로봇 작업 수행 - 1회만 실행"""
    print('\n' + '🤖' * 20)
    print('로봇 작업 시작...')
    print('🤖' * 20 + '\n')
    
    from DSR_ROBOT2 import movej, posj, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_digital_output, ON, OFF, set_digital_output, wait
    from DSR_ROBOT2 import set_singular_handling, DR_AVOID
    from DSR_ROBOT2 import set_velj, set_accj, set_velx, set_accx
    from DSR_ROBOT2 import movej, movel, posx, posj, movec, movesj
    from DSR_ROBOT2 import DR_MV_RA_DUPLICATE, DR_MV_MOD_ABS, DR_MV_APP_NONE, DR_MV_ORI_TEACH

    def opengripper():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)

    def closegripper():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    set_singular_handling(DR_AVOID)

    set_velj(60.0)
    set_accj(100.0)
    set_velx(250.0, 80.625)
    set_accx(1000.0, 322.5)

    # 1회만 실행 (기존 while 루프 제거)
    opengripper()
    movej(posj(-6.95, 3.12, 43.02, -0.07, 133.85, -4.94), radius=0.00, ra=DR_MV_RA_DUPLICATE)
    wait(1.00)
    movel(posx(451.46, 197.99, 252.51, 96.19, -179.73, 97.75), radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    closegripper()
    wait(1.00)
    movec(posx(453.46, 171.18, 310.12, 154.51, -179.76, 156.86), posx(451.17, 98.68, 346.55, 174.33, -179.14, 178.03), vel=[200.00, 76.50], acc=[800.00, 306.00], radius=0.00, ref=0, angle=[110.00,0.00], ra=DR_MV_RA_DUPLICATE)
    wait(1.00)
    opengripper()
    wait(1.00)
    movel(posx(453.75, -213.74, 252.23, 154.68, -179.81, 153.08), radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    wait(1.00)
    closegripper()
    wait(1.00)
    movec(posx(453.28, -174.30, 293.52, 95.73, -172.71, 86.05), posx(451.29, -122.55, 321.28, 96.47, -172.14, 86.51), vel=[200.00, 76.50], acc=[800.00, 306.00], radius=0.00, ref=0, angle=[70.00,0.00], ra=DR_MV_RA_DUPLICATE)
    wait(1.00)
    opengripper()
    movej(posj(0.53, 29.50, 84.70, -8.45, 33.41, -83.26), radius=0.00, ra=DR_MV_RA_DUPLICATE)
    closegripper()
    wait(1.00)
    movesj([posj(0.53, 29.50, 84.70, -8.44, 33.41, -83.26), posj(0.49, 19.79, 89.04, -8.45, 33.22, -83.26), posj(0.49, 5.57, 98.44, -8.45, 32.31, -83.26), posj(0.07, -15.90, 118.84, -8.45, 22.59, -83.26), posj(0.07, -27.08, 126.55, -8.45, 22.83, -83.26), posj(0.07, -38.78, 133.33, -8.45, 26.27, -83.26)], vel=141.17, acc=510.00)
    wait(1.00)
    opengripper()
    wait(1.00)
    movesj([posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), posj(-0.18, -18.72, 106.63, 0.00, 92.02, 0.00), posj(-4.28, -17.36, 124.77, -0.10, 71.89, -4.00)])
    closegripper()
    wait(1.00)

    movej(posj(-4.27, -19.76, 100.12, -0.14, 98.94, -4.00), radius=0.00, ra=DR_MV_RA_DUPLICATE)
    movej(posj(-4.27, -2.42, 86.41, -0.14, 95.51, 86.00), radius=0.00, ra=DR_MV_RA_DUPLICATE)
    movej(posj(-50.30, 22.26, 61.59, -0.04, 95.88, 40.06), vel=20.00, acc=100.00, radius=0.00, ra=DR_MV_RA_DUPLICATE)
    movej(posj(-59.81, 54.62, 13.75, 14.83, 133.87, 35.03), vel=50.00, acc=100.00, radius=0.00, ra=DR_MV_RA_DUPLICATE)
    movej(posj(-45.06, 19.61, 54.33, 19.48, 124.70, 50.13), vel=50.00, acc=100.00, radius=0.00, ra=DR_MV_RA_DUPLICATE)
    movej(posj(-4.27, -2.42, 86.41, -0.14, 95.51, -4.00), radius=0.00, ra=DR_MV_RA_DUPLICATE)
    movej(posj(-4.28, -17.36, 124.77, -0.10, 71.89, -4.00), radius=0.00, ra=DR_MV_RA_DUPLICATE)
    opengripper()
    wait(1.00)
    movej(posj(-6.95, 3.12, 43.02, -0.07, 133.85, -4.94), radius=0.00, ra=DR_MV_RA_DUPLICATE)
    
    print('\n' + '✅' * 20)
    print('로봇 작업 완료!')
    print('✅' * 20 + '\n')


def check_firebase_command():
    """Firebase 명령 확인 (폴링 방식)"""
    try:
        # Firebase에서 데이터 읽기
        data = robot_ref.get()
        
        if not data:
            return False
        
        command = data.get('robot_command', 'idle')
        
        # 'start' 명령 감지
        if command == 'start':
            print("\n" + "🔔" * 30)
            print("로봇 시작 명령 수신!")
            print("🔔" * 30)
            
            # 상태 업데이트: 작동 중
            robot_ref.update({
                'robot_status': 'working',
                'robot_command': 'processing'
            })
            print("📤 Firebase 업데이트: 상태 = working")
            
            try:
                # 로봇 작업 수행
                perform_task()
                
                # 완료 카운트 증가
                current_count = data.get('completed_count', 0)
                new_count = current_count + 1
                
                robot_ref.update({
                    'completed_count': new_count,
                    'robot_status': 'waiting',
                    'robot_command': 'idle',
                    'last_completed_time': time.time()
                })
                
                print(f"\n✅ 작업 완료! (총 {new_count}개)")
                print(f"📤 Firebase 업데이트: completed_count = {new_count}")
                
                return True
                
            except Exception as e:
                print(f"\n❌ 로봇 작업 중 에러: {e}")
                robot_ref.update({
                    'robot_status': 'error',
                    'robot_command': 'idle',
                    'error_message': str(e)
                })
                print("📤 Firebase 업데이트: 상태 = error")
                return False
        
        return False
        
    except Exception as e:
        print(f"⚠️  Firebase 확인 중 에러: {e}")
        return False


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 Firebase 폴링"""
    rclpy.init(args=args)
    node = rclpy.create_node("robot_firebase_controller", namespace=ROBOT_ID)
    
    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 로봇 초기화
        initialize_robot()
        
        # Firebase 초기 상태 설정
        current_data = robot_ref.get() or {}
        initial_sales = current_data.get('sales_count', 0)
        initial_completed = current_data.get('completed_count', 0)
        
        robot_ref.update({
            'robot_status': 'waiting',
            'robot_command': 'idle',
            'sales_count': initial_sales,
            'completed_count': initial_completed
        })
        
        print("\n" + "=" * 60)
        print("🚀 로봇 Firebase 폴링 시작!")
        print(f"📊 현재 상태:")
        print(f"   - 판매: {initial_sales}개")
        print(f"   - 완료: {initial_completed}개")
        print(f"   - 상태: 대기 중")
        print(f"\n💡 0.5초마다 Firebase 확인 중...")
        print("   HTML에서 [로봇 작동] 버튼을 눌러보세요!")
        print("=" * 60 + "\n")
        
        # 폴링 루프
        last_check_time = time.time()
        check_interval = 0.5  # 0.5초마다 확인
        
        while rclpy.ok():
            # ROS2 spin_once (논블로킹)
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Firebase 확인 주기
            current_time = time.time()
            if current_time - last_check_time >= check_interval:
                check_firebase_command()
                last_check_time = current_time

    except KeyboardInterrupt:
        print("\n\n" + "🛑" * 20)
        print("프로그램 종료 중...")
        print("🛑" * 20)
    except Exception as e:
        print(f"\n❌ 예상치 못한 에러: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()
        print("\n👋 프로그램 종료 완료")


if __name__ == "__main__":
    main()