import cv2

cap = cv2.VideoCapture('/dev/YourWebCam')

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
else:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('Webcam Test', frame)
        cv2.waitKey(0)

cap.release()
cv2.destroyAllWindows()