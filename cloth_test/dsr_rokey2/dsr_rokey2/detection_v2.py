import cv2
import numpy as np
import os
import time
from pathlib import Path
from glob import glob

class PaperBagDetector:
    def __init__(self, 
                 center_box_size=50,
                 box_offset_x=0,
                 box_offset_y=0,
                 brightness_min=180,
                 saturation_max=60,
                 color_std_max=30,
                 rgb_min_threshold=160):
        """
        종이백 검출기
        
        Args:
            center_box_size: 중앙 검사 영역 크기 (픽셀)
            box_offset_x: 박스 x축 오프셋 (양수=오른쪽)
            box_offset_y: 박스 y축 오프셋 (양수=아래)
            brightness_min: 밝기 최소 임계값
            saturation_max: 채도 최대 임계값
            color_std_max: 색상 표준편차 최대값
            rgb_min_threshold: RGB 최소값
        """
        self.box_size = center_box_size
        self.offset_x = box_offset_x
        self.offset_y = box_offset_y
        self.brightness_min = brightness_min
        self.saturation_max = saturation_max
        self.color_std_max = color_std_max
        self.rgb_min = rgb_min_threshold
        
        print(f"🎯 종이백 검출 파라미터:")
        print(f"   - 검사 영역: {center_box_size}x{center_box_size}")
        print(f"   - 박스 오프셋: X={box_offset_x}, Y={box_offset_y}")
        print(f"   - 밝기 최소: {brightness_min}")
        print(f"   - 채도 최대: {saturation_max}")
        print(f"   - 색상 균일도: {color_std_max}")
        print(f"   - RGB 최소: {rgb_min_threshold}\n")
        
    def analyze_center_region(self, img):
        """중앙 영역 색상 분석"""
        h, w = img.shape[:2]
        
        # 중앙 좌표 + 오프셋
        cx = w // 2 + self.offset_x
        cy = h // 2 + self.offset_y
        half_box = self.box_size // 2
        
        # 중앙 영역 추출
        x1, y1 = cx - half_box, cy - half_box
        x2, y2 = cx + half_box, cy + half_box
        
        # 범위 체크
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        
        center_region = img[y1:y2, x1:x2]
        
        # RGB 분석
        b_mean = np.mean(center_region[:, :, 0])
        g_mean = np.mean(center_region[:, :, 1])
        r_mean = np.mean(center_region[:, :, 2])
        
        rgb_std = np.std([b_mean, g_mean, r_mean])
        
        # HSV 변환
        hsv = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        h_mean = np.mean(hsv[:, :, 0])
        s_mean = np.mean(hsv[:, :, 1])
        v_mean = np.mean(hsv[:, :, 2])
        
        return {
            'bbox': (x1, y1, x2, y2),
            'rgb_mean': (r_mean, g_mean, b_mean),
            'rgb_std': rgb_std,
            'hsv_mean': (h_mean, s_mean, v_mean),
            'brightness': v_mean,
            'saturation': s_mean
        }
    
    def is_paper_bag(self, analysis):
        """종이백 여부 판단"""
        r, g, b = analysis['rgb_mean']
        brightness = analysis['brightness']
        saturation = analysis['saturation']
        rgb_std = analysis['rgb_std']
        
        # 조건들
        conditions = {
            '밝기': brightness >= self.brightness_min,
            '채도': saturation <= self.saturation_max,
            '색균일': rgb_std <= self.color_std_max,
            'RGB최소': min(r, g, b) >= self.rgb_min
        }
        
        # 모든 조건 만족해야 종이백
        is_bag = all(conditions.values())
        
        return is_bag, conditions
    
    def draw_result(self, img, analysis, is_bag, conditions):
        """결과 시각화 (웹캠용)"""
        result_img = img.copy()
        x1, y1, x2, y2 = analysis['bbox']
        
        # 박스 색상 (True=초록, False=빨강)
        box_color = (0, 255, 0) if is_bag else (0, 0, 255)
        
        # 중앙 박스 그리기 (두껍게)
        cv2.rectangle(result_img, (x1, y1), (x2, y2), box_color, 3)
        
        # TRUE/FALSE 큰 텍스트
        result_text = "TRUE" if is_bag else "FALSE"
        cv2.putText(result_img, result_text, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, box_color, 3)
        
        # 세부 정보
        r, g, b = analysis['rgb_mean']
        info_lines = [
            f"RGB: ({r:.0f}, {g:.0f}, {b:.0f})",
            f"Bright: {analysis['brightness']:.0f}",
            f"Satur: {analysis['saturation']:.0f}",
            f"Std: {analysis['rgb_std']:.1f}"
        ]
        
        y_offset = 90
        for line in info_lines:
            cv2.putText(result_img, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 25
        
        # 조건 표시
        y_offset = 190
        for cond_name, passed in conditions.items():
            status = "O" if passed else "X"
            color = (0, 255, 0) if passed else (0, 0, 255)
            text = f"{status} {cond_name}"
            cv2.putText(result_img, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            y_offset += 25
        
        return result_img
    
    def run_webcam(self, camera_index=0, detection_interval=0.3):
        """
        웹캠 실시간 검출
        
        Args:
            camera_index: 카메라 인덱스 (기본 0)
            detection_interval: 검출 주기 (초 단위, 기본 0.3초)
        """
        print("=" * 60)
        print("🎥 웹캠 종이백 검출 시작")
        print("=" * 60)
        print(f"📹 카메라: {camera_index}")
        print(f"⏱️  검출 주기: {detection_interval}초 ({1/detection_interval:.1f} fps)")
        print(f"🚪 종료: 'q' 키 누르기\n")
        
        # 웹캠 열기
        cap = cv2.VideoCapture(camera_index)
        
        if not cap.isOpened():
            print("❌ 웹캠을 열 수 없습니다!")
            return
        
        # 해상도 설정 부분 삭제! (카메라 기본 해상도 사용)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 256)
        
        last_detection_time = 0
        current_result = None
        current_analysis = None
        current_is_bag = False
        current_conditions = {}
        
        print("✅ 웹캠 시작! 화면을 확인하세요...")
        
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("❌ 프레임을 읽을 수 없습니다!")
                break
            
            # ⭐ 여기서 256x256으로 리사이즈!
            frame = cv2.resize(frame, (256, 256))
            
            current_time = time.time()
            
            # 0.3초마다 검출
            if current_time - last_detection_time >= detection_interval:
                # 분석 실행
                current_analysis = self.analyze_center_region(frame)
                current_is_bag, current_conditions = self.is_paper_bag(current_analysis)
                
                last_detection_time = current_time
                
                # 터미널 출력 (선택사항)
                status = "✓ TRUE " if current_is_bag else "✗ FALSE"
                print(f"{status} | RGB: {current_analysis['rgb_mean'][0]:.0f},{current_analysis['rgb_mean'][1]:.0f},{current_analysis['rgb_mean'][2]:.0f}")
            
            # 화면 표시 (매 프레임)
            if current_analysis is not None:
                display_frame = self.draw_result(frame, current_analysis, 
                                                current_is_bag, current_conditions)
            else:
                display_frame = frame
            
            cv2.imshow('Paper Bag Detection', display_frame)
            
            # 'q' 키로 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n🛑 종료 중...")
                break
        
        # 정리
        cap.release()
        cv2.destroyAllWindows()
        print("✅ 웹캠 종료 완료!")
    
    def process_single_image(self, image_path, true_folder, false_folder):
        """단일 이미지 처리 (기존 코드)"""
        img = cv2.imread(image_path)
        if img is None:
            print(f"⚠️  이미지 로드 실패: {image_path}")
            return None
        
        # 256x256 리사이즈
        img_resized = cv2.resize(img, (256, 256))
        
        # 중앙 영역 분석
        analysis = self.analyze_center_region(img_resized)
        is_bag, conditions = self.is_paper_bag(analysis)
        
        # 시각화
        result_img = self.draw_result(img_resized, analysis, is_bag, conditions)
        
        # 저장
        filename = Path(image_path).name
        output_folder = true_folder if is_bag else false_folder
        output_path = os.path.join(output_folder, filename)
        cv2.imwrite(output_path, result_img)
        
        return {
            'path': image_path,
            'filename': filename,
            'is_paper_bag': is_bag,
            'analysis': analysis,
            'conditions': conditions,
            'output_path': output_path
        }
    
    def process_directory(self, input_dir, output_base='paper_bag_results'):
        """디렉토리 전체 처리"""
        
        # 출력 폴더 생성
        true_folder = os.path.join(output_base, 'True')
        false_folder = os.path.join(output_base, 'False')
        os.makedirs(true_folder, exist_ok=True)
        os.makedirs(false_folder, exist_ok=True)
        
        # 이미지 파일 찾기
        image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', 
                          '*.JPG', '*.JPEG', '*.PNG', '*.BMP']
        image_files = []
        for ext in image_extensions:
            image_files.extend(glob(os.path.join(input_dir, ext)))
            image_files.extend(glob(os.path.join(input_dir, '**', ext), recursive=True))
        
        image_files = list(set(image_files))
        
        if not image_files:
            print(f"❌ '{input_dir}' 디렉토리에서 이미지를 찾을 수 없습니다.")
            return
        
        print(f"\n{'='*60}")
        print(f"📁 총 {len(image_files)}개의 이미지 발견")
        print(f"🔍 종이백 검출 시작...\n")
        
        results = []
        true_count = 0
        false_count = 0
        
        for idx, image_path in enumerate(image_files, 1):
            print(f"[{idx}/{len(image_files)}] 처리 중: {Path(image_path).name}")
            
            result = self.process_single_image(image_path, true_folder, false_folder)
            
            if result:
                results.append(result)
                if result['is_paper_bag']:
                    true_count += 1
                    print(f"  ✓ 종이백 있음\n")
                else:
                    false_count += 1
                    print(f"  ✗ 종이백 없음\n")
        
        # 요약 저장
        self.save_summary(results, output_base, true_count, false_count)
        
        # 최종 출력
        print("=" * 60)
        print("🎉 검출 완료!")
        print("=" * 60)
        print(f"✅ 총 처리: {len(results)}개")
        print(f"📄 종이백 있음 (True): {true_count}개")
        print(f"❌ 종이백 없음 (False): {false_count}개")
        print(f"📊 비율: {true_count/len(results)*100:.1f}% / {false_count/len(results)*100:.1f}%")
        print(f"📂 결과 위치:")
        print(f"   - True:  {true_folder}")
        print(f"   - False: {false_folder}")
        print(f"   - Summary: {os.path.join(output_base, 'summary.txt')}")
        print("=" * 60)
        
        return results
    
    def save_summary(self, results, output_base, true_count, false_count):
        """요약 파일 저장"""
        summary_path = os.path.join(output_base, 'summary.txt')
        
        with open(summary_path, 'w', encoding='utf-8') as f:
            f.write("=" * 60 + "\n")
            f.write("종이백 검출 결과 요약\n")
            f.write("=" * 60 + "\n\n")
            
            f.write(f"총 이미지: {len(results)}개\n")
            f.write(f"종이백 있음 (True): {true_count}개\n")
            f.write(f"종이백 없음 (False): {false_count}개\n\n")
            
            f.write("=" * 60 + "\n")
            f.write("상세 결과\n")
            f.write("=" * 60 + "\n\n")
            
            for r in results:
                status = "TRUE " if r['is_paper_bag'] else "FALSE"
                f.write(f"[{status}] {r['filename']}\n")
                
                a = r['analysis']
                f.write(f"  RGB: ({a['rgb_mean'][0]:.0f}, {a['rgb_mean'][1]:.0f}, {a['rgb_mean'][2]:.0f})\n")
                f.write(f"  밝기: {a['brightness']:.0f}, 채도: {a['saturation']:.0f}\n")
                f.write(f"  RGB 표준편차: {a['rgb_std']:.1f}\n")
                
                f.write(f"  조건 충족:\n")
                for cond_name, passed in r['conditions'].items():
                    symbol = "✓" if passed else "✗"
                    f.write(f"    {symbol} {cond_name}\n")
                f.write("\n")


# 웹캠 실행 예시
if __name__ == "__main__":
    detector = PaperBagDetector(
        center_box_size=30,
        box_offset_x=30,
        box_offset_y=0,
        brightness_min=60,
        saturation_max=50,
        color_std_max=40,
        rgb_min_threshold=70
    )
    
    # 웹캠 실행 (0.3초마다 검출)
    detector.run_webcam(camera_index=4, detection_interval=0.3)
    
    # 또는 이미지 디렉토리 처리
    # input_directory = "/home/rokey/with_box"
    # results = detector.process_directory(input_directory, "paper_bag_results")