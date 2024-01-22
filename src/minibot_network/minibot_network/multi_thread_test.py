import threading
import time

# 스레드에서 실행할 함수
def thread_function(name, run_flag):
    print(f"Thread {name}: 시작")
    while run_flag.is_set():
        print(f"Thread {name}: 실행 중")
        time.sleep(1)
    print(f"Thread {name}: 종료")

# 스레드 실행 플래그 생성
run_flags = [threading.Event() for _ in range(3)]
# 각 플래그를 설정하여 스레드가 실행되도록 함
for flag in run_flags:
    flag.set()

# 스레드 생성 및 시작
threads = [threading.Thread(target=thread_function, args=(i+1, run_flags[i])) for i in range(3)]
for t in threads:
    t.start()

# 예제: 5초 후에 첫 번째 스레드를 중지
time.sleep(5)
run_flags[0].clear()

# 나머지 스레드도 중지 (실제 사용 시에는 조건에 따라 결정)
for flag in run_flags[1:]:
    flag.clear()

# 모든 스레드가 종료될 때까지 기다림
for t in threads:
    t.join()

print("모든 스레드가 종료되었습니다.")