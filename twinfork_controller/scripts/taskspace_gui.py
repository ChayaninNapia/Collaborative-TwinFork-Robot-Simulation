from PyQt5.QtWidgets import (
    QWidget, QLabel, QLineEdit, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox, QGroupBox, QTextEdit, QApplication
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont
import sys

class StatusBridge(QObject):
    update_status_signal = pyqtSignal(str, str)  # tag, status

class TaskspaceGUI(QWidget):
    def __init__(self, pose_callback=None):
        super().__init__()
        self.setWindowTitle("Taskspace GUI - Pallet Pose Sender")
        self.setGeometry(100, 100, 900, 600)
        self.pose_callback = pose_callback  # function(tag, x, y, theta)
        self.status_bridge = StatusBridge()
        self.status_bridge.update_status_signal.connect(self.set_status_text)
        self.init_ui()

    def init_ui(self):
        outer_layout = QVBoxLayout()
        main_layout = QHBoxLayout()

        # ==== Command Panel ====
        command_panel = QVBoxLayout()

        lift_box = self.create_command_box("Lift Command", "lift")
        drop_box = self.create_command_box("Drop Command", "drop")

        command_panel.addWidget(lift_box)
        command_panel.addWidget(drop_box)

        # ==== Status Panel ====
        status_panel = QVBoxLayout()
        status_title = QLabel("Status")
        status_title.setFont(QFont("Arial", 16, QFont.Bold))
        status_title.setAlignment(Qt.AlignCenter)

        self.ai_daeng_label = QLabel("Ai_Daeng")
        self.ai_daeng_label.setAlignment(Qt.AlignCenter)
        self.ai_daeng_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.ai_daeng_label.setStyleSheet("background-color: #ff4d4d; padding: 8px; border-radius: 5px;")

        self.ai_daeng_status = QLabel("Approaching")
        self.ai_daeng_status.setAlignment(Qt.AlignCenter)
        self.ai_daeng_status.setStyleSheet("border: 1px solid black; padding: 5px;")

        self.ai_khieow_label = QLabel("Ai_Khieow")
        self.ai_khieow_label.setAlignment(Qt.AlignCenter)
        self.ai_khieow_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.ai_khieow_label.setStyleSheet("background-color: #66ff66; padding: 8px; border-radius: 5px;")

        self.ai_khieow_status = QLabel("Standby..")
        self.ai_khieow_status.setAlignment(Qt.AlignCenter)
        self.ai_khieow_status.setStyleSheet("border: 1px solid black; padding: 5px;")

        status_panel.addWidget(status_title)
        status_panel.addWidget(self.ai_daeng_label)
        status_panel.addWidget(self.ai_daeng_status)
        status_panel.addWidget(self.ai_khieow_label)
        status_panel.addWidget(self.ai_khieow_status)

        # Add panels to main layout
        main_layout.addLayout(command_panel, 2)
        main_layout.addLayout(status_panel, 3)

        # ==== Log Panel ====
        log_title = QLabel("Console")
        log_title.setFont(QFont("Arial", 14, QFont.Bold))
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setStyleSheet("background-color: black; color: white; font-family: Courier;")
        self.log_display.setMinimumHeight(120)

        outer_layout.addLayout(main_layout)
        outer_layout.addWidget(log_title)
        outer_layout.addWidget(self.log_display)
        self.setLayout(outer_layout)

    def create_command_box(self, title, tag):
        box = QGroupBox(title)
        box.setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid black; margin-top: 10px; }")
        layout = QGridLayout()

        x_input = QLineEdit(); y_input = QLineEdit(); theta_input = QLineEdit()
        setattr(self, f"{tag}_x_input", x_input)
        setattr(self, f"{tag}_y_input", y_input)
        setattr(self, f"{tag}_theta_input", theta_input)

        layout.addWidget(QLabel("pallet pose"), 0, 0, 1, 2)
        layout.addWidget(QLabel("x:"), 1, 0); layout.addWidget(x_input, 1, 1)
        layout.addWidget(QLabel("y:"), 2, 0); layout.addWidget(y_input, 2, 1)
        layout.addWidget(QLabel("yaw:"), 3, 0); layout.addWidget(theta_input, 3, 1)

        send_btn = QPushButton("send")
        send_btn.setFixedSize(50, 50)
        send_btn.setStyleSheet("""
            QPushButton {
                border-radius: 25px;
                background-color: #90ee90;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #7edc7e;
            }
            QPushButton:pressed {
                background-color: #66cc66;
            }
        """)
        send_btn.clicked.connect(lambda: self.send_pose(tag))
        layout.addWidget(send_btn, 1, 2, 3, 1)

        box.setLayout(layout)
        return box

    def send_pose(self, tag):
        try:
            x = float(getattr(self, f"{tag}_x_input").text())
            y = float(getattr(self, f"{tag}_y_input").text())
            theta = float(getattr(self, f"{tag}_theta_input").text())

            msg = f"[{tag.upper()}] 📤 ส่งพาเลตที่ตำแหน่ง: x={x}, y={y}, θ={theta}"
            print(msg)
            self.log_display.append(msg)
            if self.pose_callback:
                self.pose_callback(tag, x, y, theta)
        except ValueError:
            self.log_display.append(f"[{tag.upper()}] ❌ Invalid input values!")
            QMessageBox.warning(self, "Invalid Input", "กรุณาใส่ตัวเลขให้ถูกต้องในทุกช่อง")

    def set_status_text(self, tag, text):
        if tag == 'ai_daeng':
            self.ai_daeng_status.setText(text)
        elif tag == 'ai_khieow':
            self.ai_khieow_status.setText(text)
        self.log_display.append(f"[{tag.upper()}] 🔁 Status updated: {text}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = TaskspaceGUI()
    gui.show()
    sys.exit(app.exec_())