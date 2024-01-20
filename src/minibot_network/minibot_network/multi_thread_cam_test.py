import threading
import time
import cv2

def camera_thread_function(thread_id, run_flag):
    cap = cv2.VideoCapture(0)  # 첫 번째 카메라 사용

    print(f"카메라 {thread_id}: 시작")
    while run_flag.is_set():
        ret, frame = cap.read()
        
        if not ret:
            print(f"카메라 {thread_id}: 영상 캡처 실패")
            break

        cv2.imshow(f"Camera {thread_id}", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"카메라 {thread_id}: 종료")

# 스레드 실행 플래그 및 스레드 리스트 초기화
run_flags = [threading.Event() for _ in range(3)]
threads = [threading.Thread(target=camera_thread_function, args=(i, run_flags[i])) for i in range(3)]

active_thread = None

while True:
    thread_number = int(input("실행할 스레드 번호를 입력하세요 (0-2), 종료하려면 -1 입력: "))
    if thread_number == -1:
        break

    if active_thread is not None:
        run_flags[active_thread].clear()
        threads[active_thread].join()

    run_flags[thread_number].set()
    threads[thread_number].start()
    active_thread = thread_number

# 종료 시 모든 스레드 정리
for flag in run_flags:
    flag.clear()
for t in threads:
    t.join()

print("모든 카메라 스레드가 종료되었습니다.")
