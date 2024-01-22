import cv2
import threading
from main_server import MyServer

def print_test():
    frame = {}
    while True:
        try:
            frame = server_node.images

            print(frame)

            cv2.imshow('test', frame["192.168.1.7"])
            cv2.imshow('test2', frame["192.168.1.14"])

            if cv2.waitKey(1) == ord('q'):
                break
        except Exception as e:
            print(f"Error: {e}")

# MyServer instance creation
server_node = MyServer()

# Server running in a separate thread
server_thread = threading.Thread(target=server_node.run_server)
server_thread.start()

# Creating and starting the chat thread
chat = threading.Thread(target=print_test)
chat.start()

# Optional: Join the threads if you want the main script to wait for these threads
try:
    server_thread.join()
    chat.join()
except KeyboardInterrupt:
    print("Program interrupted")

if __name__ == "__main__":
    # Place the thread starting code here if you only want it to execute when the script is run directly
    pass
