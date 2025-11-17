import rclpy
import DR_init

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


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)

    # Tool과 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)



def perform_task():
    print('hi')
    from DSR_ROBOT2 import movej, posj, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_digital_output, ON, OFF, set_digital_output, wait
    from DSR_ROBOT2 import set_singular_handling, DR_AVOID
    from DSR_ROBOT2 import set_velj, set_accj, set_velx, set_accx
    from DSR_ROBOT2 import movej, movel, posx, posj, movec, movesj
    from DSR_ROBOT2 import DR_MV_RA_DUPLICATE, DR_MV_MOD_ABS, DR_MV_APP_NONE, DR_MV_ORI_TEACH

    def opengripper():
        # SetNode
        set_digital_output(1, OFF)
        set_digital_output(2, ON)

    _alias_sub_name_opengripper = opengripper

    def closegripper():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    _alias_sub_name_closegripper = closegripper

    set_singular_handling(DR_AVOID)

    set_velj(60.0)
    set_accj(100.0)
    set_velx(250.0, 80.625)
    set_accx(1000.0, 322.5)

    Gloop111926233 = 0
    while Gloop111926233 < 3:
        # CallNode
        _alias_sub_name_opengripper()
        # Move

        _alias_sub_name_opengripper()
        # MoveJNode
        movej(posj(-6.95, 3.12, 43.02, -0.07, 133.85, -4.94), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        # WaitNode
        wait(1.00)
        # MoveLNode
        movel(posx(451.46, 197.99, 252.51, 96.19, -179.73, 97.75), radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        # CallNode
        _alias_sub_name_closegripper()
        # WaitNode
        wait(1.00)
        # MoveCNode
        movec(posx(453.46, 171.18, 310.12, 154.51, -179.76, 156.86), posx(451.17, 98.68, 346.55, 174.33, -179.14, 178.03), vel=[200.00, 76.50], acc=[800.00, 306.00], radius=0.00, ref=0, angle=[110.00,0.00], ra=DR_MV_RA_DUPLICATE)
        # WaitNode
        wait(1.00)
        # CallNode
        _alias_sub_name_opengripper()
        # WaitNode
        wait(1.00)
        # MoveLNode
        movel(posx(453.75, -213.74, 252.23, 154.68, -179.81, 153.08), radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        # WaitNode
        wait(1.00)
        # CallNode
        _alias_sub_name_closegripper()
        # WaitNode
        wait(1.00)
        # MoveCNode
        movec(posx(453.28, -174.30, 293.52, 95.73, -172.71, 86.05), posx(451.29, -122.55, 321.28, 96.47, -172.14, 86.51), vel=[200.00, 76.50], acc=[800.00, 306.00], radius=0.00, ref=0, angle=[70.00,0.00], ra=DR_MV_RA_DUPLICATE)
        # WaitNode
        wait(1.00)
        # CallNode
        _alias_sub_name_opengripper()
        # MoveJNode
        movej(posj(0.53, 29.50, 84.70, -8.45, 33.41, -83.26), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        # CallNode
        _alias_sub_name_closegripper()
        # WaitNode
        wait(1.00)
        # MoveSJNode
        movesj([posj(0.53, 29.50, 84.70, -8.44, 33.41, -83.26), posj(0.49, 19.79, 89.04, -8.45, 33.22, -83.26), posj(0.49, 5.57, 98.44, -8.45, 32.31, -83.26), posj(0.07, -15.90, 118.84, -8.45, 22.59, -83.26), posj(0.07, -27.08, 126.55, -8.45, 22.83, -83.26), posj(0.07, -38.78, 133.33, -8.45, 26.27, -83.26)], vel=141.17, acc=510.00)
        # WaitNode
        wait(1.00)
        # CallNode
        _alias_sub_name_opengripper()
        # WaitNode
        wait(1.00)
        # MoveSJNode
        movesj([posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), posj(-0.18, -18.72, 106.63, 0.00, 92.02, 0.00), posj(-4.28, -17.36, 124.77, -0.10, 71.89, -4.00)])
        # CallNode
        _alias_sub_name_closegripper()
        # WaitNode
        wait(1.00)
        # MoveSJNode
        movesj([posj(-4.21, -18.37, 100.73, -0.10, 97.33, -4.00), posj(-4.18, 13.96, 70.12, -0.11, 95.46, -4.00)])
        # MoveLNode
        movel(posx(631.20, -40.10, 444.48, 175.29, -179.34, 175.51), vel=[50.00, 64.13], acc=[1000.00, 256.50], radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        # WaitNode
        wait(5.00)
        # MoveSJNode
        movesj([posj(-4.21, -18.37, 100.73, -0.10, 97.33, -4.00), posj(-4.28, -17.36, 124.77, -0.10, 71.89, -4.00)])
        # WaitNode
        wait(1.00)
        # CallNode
        _alias_sub_name_opengripper()
        # MoveJNode
        movej(posj(-6.95, 3.12, 43.02, -0.07, 133.85, -4.94), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        # WaitNode
        wait(10.00)
        Gloop111926233 = Gloop111926233 + 1


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_periodic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()
        print('hello')
        # 작업 수행 (한 번만 호출)
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()