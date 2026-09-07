import numpy as np
from scipy.signal import butter, lfilter_zi, lfilter

class RealtimeButterworthFilter:
    def __init__(self, order, cutoff, fs):
        """
        order: 필터 차수 (보통 1~2 사용, 높을수록 딜레이 심함)
        cutoff: 차단 주파수 (Hz) - 이보다 빠른 진동은 제거됨
        fs: 샘플링 주파수 (Hz) - 루프가 1초에 몇 번 도는지
        """
        # 정규화된 주파수 (Nyquist Frequency 기준)
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        
        # 필터 계수 계산 (b: 분자, a: 분모)
        self.b, self.a = butter(order, cutoff / (0.5 * fs), btype='low', analog=False)
        self.zi = lfilter_zi(self.b, self.a)
        self.initialized = False  # 초기화 여부 플래그
        # 유효성 검증: 정규화된 주파수는 0 < Wn < 1 범위에 있어야 함
        if normal_cutoff >= 1.0:
            print(f"Warning: Cutoff frequency {cutoff}Hz is too high for sampling frequency {fs}Hz")
            print(f"Normalized frequency {normal_cutoff:.3f} >= 1.0, clamping to 0.95")
            normal_cutoff = 0.95
        elif normal_cutoff <= 0.0:
            print(f"Warning: Cutoff frequency {cutoff}Hz is too low, setting to 0.01")
            normal_cutoff = 0.01
        # 필터 초기 상태값 (State)
        # 데이터가 3개(x,y,z) 들어온다면 len(zi)도 맞춰줘야 함. 
        # 여기서는 간단히 1차원(값 1개) 기준으로 예시 작성 후 아래에서 확장 설명
        self.zi = lfilter_zi(self.b, self.a)
        
    def update(self, data):
        """
        새로운 데이터가 들어올 때마다 호출
        data: 숫자 1개 또는 리스트
        """
        # [핵심] 첫 데이터가 들어오면 필터 상태를 그 값으로 세팅!
        if not self.initialized:
            self.zi = self.zi * data 
            self.initialized = True
            return data
        
        filtered_data, self.zi = lfilter(self.b, self.a, [data], zi=self.zi)
        return filtered_data[0]

# --- 사용 예시 (로봇 루프 안이라고 가정) ---

# 설정: 샘플링 30Hz(카메라 FPS), 차단주파수 2Hz(아주 부드럽게), 1차 필터
# lpf_x = RealtimeButterworthFilter(order=1, cutoff=2.0, fs=30.0)
# lpf_y = RealtimeButterworthFilter(order=1, cutoff=2.0, fs=30.0)

# # 가상의 노이즈 데이터 (튀는 값들)
# noisy_inputs = [10, 10.2, 9.8, 10.1, 15.0(노이즈), 10.1, 9.9] 

# print("원본 -> 필터링 결과")
# for raw_val in noisy_inputs:
#     filtered_val = lpf_x.update(raw_val)
#     print(f"{raw_val} -> {filtered_val:.2f}")