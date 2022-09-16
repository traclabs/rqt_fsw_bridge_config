from python_qt_binding.QtWidgets import QDialog, QDialogButtonBox
from python_qt_binding.QtWidgets import QVBoxLayout, QLabel


class ConfirmDialog(QDialog):
    def __init__(self, confirm_msg, parent=None):
        super().__init__(parent)

        self.setWindowTitle("ROS2-FSW Bridge Confirmation")

        QBtn = QDialogButtonBox.Ok | QDialogButtonBox.Cancel

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.layout = QVBoxLayout()
        message = QLabel(confirm_msg)
        self.layout.addWidget(message)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)
