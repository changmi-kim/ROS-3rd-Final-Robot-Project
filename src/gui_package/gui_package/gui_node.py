import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import sys
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal

class PiCamSubscriber(Node):
    def __init__(self, image_callback):
        super().__init__('pi_cam_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            image_callback,
            10)
        self.bridge = CvBridge()

class CameraThread(QThread):
    change_pixmap_signal = pyqtSignal(QPixmap)

    def __init__(self):
        super().__init__()
        # self.image_callback 메서드를 인자로 전달
        self.pi_cam_subscriber = PiCamSubscriber(self.image_callback)

    def run(self):
        # 여기서 PiCamSubscriber를 다시 초기화하지 않아야 함
        rp.spin(self.pi_cam_subscriber)
        rp.shutdown()

    def image_callback(self, msg):
        cv_image = self.pi_cam_subscriber.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        qt_pixmap = self.convert_cv_qt(cv_image)
        self.change_pixmap_signal.emit(qt_pixmap)

    def convert_cv_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(640, 480)
        return QPixmap.fromImage(p)

class App(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Camera Viewer")
        self.display_width = 640
        self.display_height = 480
        self.image_label = QLabel(self)
        self.image_label.resize(self.display_width, self.display_height)

        vbox = QVBoxLayout()
        vbox.addWidget(self.image_label)
        self.setLayout(vbox)

        self.thread = CameraThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

    def closeEvent(self, event):
        self.thread.quit()
        self.thread.wait()

    def update_image(self, qt_pixmap):
        self.image_label.setPixmap(qt_pixmap)


# def main(args=None):
#     rp.init(args=args)
#     pi_cam_subscriber = PiCamSubscriber()
#     rp.spin(pi_cam_subscriber)
#     pi_cam_subscriber.destroy_node()
#     rp.shutdown()

# if __name__ == '__main__':
#     main()
def main():
    rp.init()
    # QApplication 인스턴스 생성
    app = QApplication(sys.argv)
    # App 클래스의 인스턴스를 생성하여 메인 윈도우를 만듭니다
    main_window = App()
    # 윈도우를 보이게 합니다
    main_window.show()
    # 애플리케이션의 메인 이벤트 루프를 시작합니다
    sys.exit(app.exec_())
    rp.shutdown()

if __name__ == "__main__":
    main()