import sys
import numpy as np
from PyQt6 import QtCore, QtWidgets, QtGui
from scipy.integrate import solve_ivp
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import configparser
import os

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        deriv = (error - self.prev_error) / dt if dt > 0 else 0
        u = self.kp * error + self.ki * self.integral + self.kd * deriv
        self.prev_error = error
        return u

class KalmanFilter:
    def __init__(self, a=0.1, g=10.0, dt=0.01, Q=0.01, R=4.0, x0=0.0, P0=100.0):
        self.a = a
        self.g = g
        self.dt = dt
        self.Q = Q
        self.R = R
        self.x = x0
        self.P = P0
        self.F = np.exp(-self.a * self.dt)
        self.G = self.g * (1 - self.F) / self.a if self.a != 0 else self.g * self.dt

    def predict(self, u):
        self.x = self.F * self.x + self.G * u
        self.P = self.F * self.P * self.F + self.Q

    def update(self, z):
        K = self.P / (self.P + self.R)
        self.x += K * (z - self.x)
        self.P = (1 - K) * self.P

class WheelWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.angle = 0.0
        self.setMinimumSize(200, 200)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        center = rect.center()
        radius = min(rect.width(), rect.height()) / 2 - 10
        painter.drawEllipse(int(center.x() - radius), int(center.y() - radius), int(2 * radius), int(2 * radius))
        painter.save()
        painter.translate(center)
        painter.rotate(self.angle)
        painter.drawLine(0, 0, int(radius), 0)
        painter.restore()

    def rotate(self, delta_angle):
        self.angle = (self.angle + delta_angle) % 360
        self.update()

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Motor Simulator")
        self.central = QtWidgets.QWidget()
        self.setCentralWidget(self.central)
        layout = QtWidgets.QGridLayout(self.central)

        # Target RPM
        self.target_label = QtWidgets.QLabel("Target RPM:")
        self.target_label.setToolTip("Desired RPM for the motor.")
        layout.addWidget(self.target_label, 0, 0)
        self.target_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.target_slider.setRange(0, 5000)
        self.target_slider.setValue(100)
        layout.addWidget(self.target_slider, 0, 1)
        self.target_spin = QtWidgets.QSpinBox()
        self.target_spin.setRange(0, 5000)
        self.target_spin.setValue(100)
        layout.addWidget(self.target_spin, 0, 2)
        self.target_slider.valueChanged.connect(self.target_spin.setValue)
        self.target_spin.valueChanged.connect(self.target_slider.setValue)

        # Allow negative RPM
        self.allow_neg_checkbox = QtWidgets.QCheckBox("Allow Negative RPM")
        self.allow_neg_checkbox.setChecked(False)
        self.allow_neg_checkbox.setToolTip("Allow RPM to go negative (reverse direction).")
        layout.addWidget(self.allow_neg_checkbox, 0, 3)
        self.allow_neg_checkbox.stateChanged.connect(self.update_ranges)

        # PID gains
        self.kp_label = QtWidgets.QLabel("Kp:")
        self.kp_label.setToolTip("Proportional gain for PID controller.")
        layout.addWidget(self.kp_label, 1, 0)
        self.kp_spin = QtWidgets.QDoubleSpinBox()
        self.kp_spin.setRange(0, 10)
        self.kp_spin.setSingleStep(0.00001)
        self.kp_spin.setDecimals(5)
        self.kp_spin.setValue(0.2)
        layout.addWidget(self.kp_spin, 1, 1)
        self.ki_label = QtWidgets.QLabel("Ki:")
        self.ki_label.setToolTip("Integral gain for PID controller.")
        layout.addWidget(self.ki_label, 2, 0)
        self.ki_spin = QtWidgets.QDoubleSpinBox()
        self.ki_spin.setRange(0, 10)
        self.ki_spin.setSingleStep(0.00001)
        self.ki_spin.setDecimals(5)
        self.ki_spin.setValue(0.1)
        layout.addWidget(self.ki_spin, 2, 1)
        self.kd_label = QtWidgets.QLabel("Kd:")
        self.kd_label.setToolTip("Derivative gain for PID controller.")
        layout.addWidget(self.kd_label, 3, 0)
        self.kd_spin = QtWidgets.QDoubleSpinBox()
        self.kd_spin.setRange(0, 10)
        self.kd_spin.setSingleStep(0.00001)
        self.kd_spin.setDecimals(5)
        self.kd_spin.setValue(0.0)
        layout.addWidget(self.kd_spin, 3, 1)

        # Friction
        self.friction_label = QtWidgets.QLabel("Friction (a):")
        self.friction_label.setToolTip("Friction coefficient in the motor model.")
        layout.addWidget(self.friction_label, 4, 0)
        self.friction_spin = QtWidgets.QDoubleSpinBox()
        self.friction_spin.setRange(0, 1)
        self.friction_spin.setSingleStep(0.01)
        self.friction_spin.setValue(0.1)
        layout.addWidget(self.friction_spin, 4, 1)

        # Noise std
        self.noise_label = QtWidgets.QLabel("Noise Std:")
        self.noise_label.setToolTip("Standard deviation of the sensor noise.")
        layout.addWidget(self.noise_label, 5, 0)
        self.noise_spin = QtWidgets.QDoubleSpinBox()
        self.noise_spin.setRange(0, 10)
        self.noise_spin.setValue(2.0)
        layout.addWidget(self.noise_spin, 5, 1)

        # Sensor freq
        self.sensor_label = QtWidgets.QLabel("Sensor Freq (Hz):")
        self.sensor_label.setToolTip("Frequency of sensor measurements in Hz.")
        layout.addWidget(self.sensor_label, 4, 2)
        self.sensor_spin = QtWidgets.QDoubleSpinBox()
        self.sensor_spin.setRange(1, 1000)
        self.sensor_spin.setValue(20.0)
        layout.addWidget(self.sensor_spin, 5, 2)

        # Disturbance
        self.dist_group = QtWidgets.QGroupBox("Disturbance")
        dist_layout = QtWidgets.QVBoxLayout()
        self.dist_on = QtWidgets.QCheckBox("On")
        dist_layout.addWidget(self.dist_on)
        self.static_radio = QtWidgets.QRadioButton("Static")
        self.dynamic_radio = QtWidgets.QRadioButton("Dynamic")
        self.static_radio.setChecked(True)
        dist_layout.addWidget(self.static_radio)
        dist_layout.addWidget(self.dynamic_radio)
        self.dist_value_label = QtWidgets.QLabel("Value/Amp:")
        self.dist_value_label.setToolTip("Disturbance value for static mode or amplitude for dynamic mode.")
        dist_layout.addWidget(self.dist_value_label)
        self.dist_value_spin = QtWidgets.QDoubleSpinBox()
        self.dist_value_spin.setRange(-10, 10)
        self.dist_value_spin.setValue(0.5)
        dist_layout.addWidget(self.dist_value_spin)
        self.freq_label = QtWidgets.QLabel("Freq (Hz):")
        self.freq_label.setToolTip("Frequency of the sinusoidal disturbance in dynamic mode (Hz).")
        dist_layout.addWidget(self.freq_label)
        self.freq_spin = QtWidgets.QDoubleSpinBox()
        self.freq_spin.setRange(0, 1)
        self.freq_spin.setSingleStep(0.01)
        self.freq_spin.setValue(0.1)
        dist_layout.addWidget(self.freq_spin)
        self.dist_group.setLayout(dist_layout)
        layout.addWidget(self.dist_group, 1, 3, 5, 1)

        # Wheel
        self.wheel = WheelWidget()
        layout.addWidget(self.wheel, 6, 0, 1, 2)

        # Actual RPM display
        self.rpm_label = QtWidgets.QLabel("0.0 RPM")
        font = QtGui.QFont("Arial", 24)
        self.rpm_label.setFont(font)
        self.rpm_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.rpm_label, 7, 0, 1, 2)

        # Error Plot
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas, 6, 2, 2, 2)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Error vs Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Error (RPM)")
        self.times = []
        self.errors = []

        # RPM Plot
        self.rpm_figure = Figure()
        self.rpm_canvas = FigureCanvas(self.rpm_figure)
        layout.addWidget(self.rpm_canvas, 8, 0, 1, 4)
        self.ax_rpm = self.rpm_figure.add_subplot(111)
        self.ax_rpm.set_title("RPM vs Time")
        self.ax_rpm.set_xlabel("Time (s)")
        self.ax_rpm.set_ylabel("RPM")
        self.rpms_des = []
        self.rpms_act = []
        self.rpms_true = []

        # Buttons
        self.start_stop_btn = QtWidgets.QPushButton("Stop")
        layout.addWidget(self.start_stop_btn, 9, 0)
        self.start_stop_btn.clicked.connect(self.toggle_start_stop)
        self.is_running = True

        self.reset_btn = QtWidgets.QPushButton("Reset")
        layout.addWidget(self.reset_btn, 9, 1)
        self.reset_btn.clicked.connect(self.reset_sim)

        # Config save
        self.filename_label = QtWidgets.QLabel("Config File:")
        layout.addWidget(self.filename_label, 9, 2)
        self.filename_edit = QtWidgets.QLineEdit("config.cfg")
        layout.addWidget(self.filename_edit, 9, 3)
        self.save_btn = QtWidgets.QPushButton("Save Config")
        layout.addWidget(self.save_btn, 9, 4)
        self.save_btn.clicked.connect(self.save_config)

        # Sim params
        self.dt = 0.01
        self.t = 0.0
        self.rpm_true = 0.0
        self.u = 0.0
        self.pid = PID(self.kp_spin.value(), self.ki_spin.value(), self.kd_spin.value())
        self.kf = KalmanFilter(a=self.friction_spin.value(), g=10.0, dt=self.dt, Q=0.01, R=self.noise_spin.value()**2)
        self.sensor_freq = self.sensor_spin.value()
        self.last_measure = 0.0

        # Load config
        self.load_config()

        # Connect changes
        self.kp_spin.valueChanged.connect(self.update_pid)
        self.ki_spin.valueChanged.connect(self.update_pid)
        self.kd_spin.valueChanged.connect(self.update_pid)
        self.friction_spin.valueChanged.connect(self.update_kf)
        self.noise_spin.valueChanged.connect(self.update_kf)
        self.sensor_spin.valueChanged.connect(self.update_sensor)

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_sim)
        self.timer.start(int(self.dt * 1000))

    def update_ranges(self):
        allow_neg = self.allow_neg_checkbox.isChecked()
        min_rpm = -5000 if allow_neg else 0
        self.target_slider.setMinimum(min_rpm)
        self.target_spin.setMinimum(min_rpm)
        current = self.target_spin.value()
        if current < min_rpm:
            self.target_spin.setValue(min_rpm)

    def load_config(self):
        if not os.path.exists('config.cfg'):
            return
        config = configparser.ConfigParser()
        config.read('config.cfg')
        if 'PID' in config:
            self.kp_spin.setValue(config.getfloat('PID', 'kp'))
            self.ki_spin.setValue(config.getfloat('PID', 'ki'))
            self.kd_spin.setValue(config.getfloat('PID', 'kd'))
        if 'Model' in config:
            self.friction_spin.setValue(config.getfloat('Model', 'friction'))
            self.noise_spin.setValue(config.getfloat('Model', 'noise_std'))
            self.sensor_spin.setValue(config.getfloat('Model', 'sensor_freq'))
            if 'allow_negative' in config['Model']:
                self.allow_neg_checkbox.setChecked(config.getboolean('Model', 'allow_negative'))
        if 'Disturbance' in config:
            self.dist_on.setChecked(config.getboolean('Disturbance', 'on'))
            is_static = config.getboolean('Disturbance', 'static')
            self.static_radio.setChecked(is_static)
            self.dynamic_radio.setChecked(not is_static)
            self.dist_value_spin.setValue(config.getfloat('Disturbance', 'value'))
            self.freq_spin.setValue(config.getfloat('Disturbance', 'freq'))
        self.update_pid()
        self.update_kf()
        self.update_sensor()
        self.update_ranges()

    def save_config(self):
        filename = self.filename_edit.text()
        config = configparser.ConfigParser()
        config['PID'] = {
            'kp': str(self.kp_spin.value()),
            'ki': str(self.ki_spin.value()),
            'kd': str(self.kd_spin.value())
        }
        config['Model'] = {
            'friction': str(self.friction_spin.value()),
            'noise_std': str(self.noise_spin.value()),
            'sensor_freq': str(self.sensor_spin.value()),
            'allow_negative': str(self.allow_neg_checkbox.isChecked())
        }
        config['Disturbance'] = {
            'on': str(self.dist_on.isChecked()),
            'static': str(self.static_radio.isChecked()),
            'value': str(self.dist_value_spin.value()),
            'freq': str(self.freq_spin.value())
        }
        with open(filename, 'w') as configfile:
            config.write(configfile)

    def update_pid(self):
        self.pid.kp = self.kp_spin.value()
        self.pid.ki = self.ki_spin.value()
        self.pid.kd = self.kd_spin.value()

    def update_kf(self):
        self.kf.a = self.friction_spin.value()
        self.kf.F = np.exp(-self.kf.a * self.kf.dt)
        self.kf.G = self.kf.g * (1 - self.kf.F) / self.kf.a if self.kf.a != 0 else self.kf.g * self.kf.dt
        self.kf.R = self.noise_spin.value() ** 2

    def update_sensor(self):
        self.sensor_freq = self.sensor_spin.value()

    def toggle_start_stop(self):
        if self.is_running:
            self.timer.stop()
            self.start_stop_btn.setText("Start")
            self.is_running = False
        else:
            self.timer.start(int(self.dt * 1000))
            self.start_stop_btn.setText("Stop")
            self.is_running = True

    def reset_sim(self):
        self.rpm_true = 0.0
        self.u = 0.0
        self.kf = KalmanFilter(a=self.friction_spin.value(), g=10.0, dt=self.dt, Q=0.01, R=self.noise_spin.value()**2)
        self.pid = PID(self.kp_spin.value(), self.ki_spin.value(), self.kd_spin.value())
        self.t = 0.0
        self.last_measure = 0.0
        self.times = []
        self.errors = []
        self.rpms_des = []
        self.rpms_act = []
        self.rpms_true = []
        self.wheel.angle = 0.0
        self.wheel.update()
        self.rpm_label.setText("0.0 RPM")
        self.ax.cla()
        self.ax.set_title("Error vs Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Error (RPM)")
        self.canvas.draw()
        self.ax_rpm.cla()
        self.ax_rpm.set_title("RPM vs Time")
        self.ax_rpm.set_xlabel("Time (s)")
        self.ax_rpm.set_ylabel("RPM")
        self.rpm_canvas.draw()

    def get_dist(self, t):
        if not self.dist_on.isChecked():
            return 0.0
        value = self.dist_value_spin.value()
        if self.static_radio.isChecked():
            return value
        else:
            return value * np.sin(2 * np.pi * self.freq_spin.value() * t)

    def plant_fun(self, t_rel, y, u, a, g, dist_func, t_start):
        t_abs = t_start + t_rel
        dist = dist_func(t_abs)
        return [-a * y[0] + g * u - dist]

    def update_sim(self):
        target = self.target_spin.value()
        error = target - self.kf.x
        self.u = self.pid.compute(error, self.dt)
        self.kf.predict(self.u)
        sol = solve_ivp(self.plant_fun, (0, self.dt), [self.rpm_true], method='RK45',
                        args=(self.u, self.kf.a, self.kf.g, self.get_dist, self.t))
        self.rpm_true = sol.y[0, -1]
        if not self.allow_neg_checkbox.isChecked():
            self.rpm_true = max(0, self.rpm_true)
        period = 1.0 / self.sensor_freq if self.sensor_freq > 0 else self.dt
        if self.t + self.dt - self.last_measure >= period - 1e-6:
            noise = np.random.normal(0, self.noise_spin.value())
            rpm_meas = self.rpm_true + noise
            self.kf.update(rpm_meas)
            if not self.allow_neg_checkbox.isChecked():
                self.kf.x = max(0, self.kf.x)
            self.last_measure = self.t + self.dt
        delta_angle = self.rpm_true * self.dt / 60 * 360
        self.wheel.rotate(delta_angle)
        self.times.append(self.t)
        self.errors.append(error)
        self.rpms_des.append(target)
        self.rpms_act.append(self.kf.x)
        self.rpms_true.append(self.rpm_true)
        if len(self.times) > 1000:
            self.times.pop(0)
            self.errors.pop(0)
            self.rpms_des.pop(0)
            self.rpms_act.pop(0)
            self.rpms_true.pop(0)
        self.ax.cla()
        self.ax.plot(self.times, self.errors)
        self.ax.set_title("Error vs Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Error (RPM)")
        self.canvas.draw()
        self.ax_rpm.cla()
        self.ax_rpm.plot(self.times, self.rpms_des, label="Desired")
        self.ax_rpm.plot(self.times, self.rpms_act, label="Estimated Actual")
        self.ax_rpm.plot(self.times, self.rpms_true, label="True Actual")
        self.ax_rpm.legend()
        self.ax_rpm.set_title("RPM vs Time")
        self.ax_rpm.set_xlabel("Time (s)")
        self.ax_rpm.set_ylabel("RPM")
        self.rpm_canvas.draw()
        self.rpm_label.setText(f"{self.kf.x:.1f} RPM")
        self.t += self.dt

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())