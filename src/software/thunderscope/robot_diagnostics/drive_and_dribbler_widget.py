from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
import time
import software.python_bindings as tbots_cpp
from software.thunderscope.common import common_widgets
from proto.import_all_protos import *
from software.thunderscope.proto_unix_io import ProtoUnixIO


class DriveAndDribblerWidget(QWidget):
    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        """Initialize the widget to control the robot's motors

        :param proto_unix_io: the proto_unix_io object
        """
        self.input_a = time.time()
        self.constants = tbots_cpp.create2021RobotConstants()
        QWidget.__init__(self)
        layout = QVBoxLayout()

        self.proto_unix_io = proto_unix_io

        # Add widgets to layout
        layout.addWidget(self.setup_switch_drive_mode("Drive Mode Switch"))
        layout.addWidget(self.setup_direct_velocity("Drive - Direct Velocity"))
        layout.addWidget(self.setup_per_motor("Drive - Per Motor"))
        layout.addWidget(self.setup_dribbler("Dribbler"))

        self.enabled = True
        self.perMotorControl = False

        self.setLayout(layout)

    def refresh(self) -> None:
        """Refresh the widget and send the a MotorControl message with the current values"""
        motor_control = MotorControl()
        motor_control.dribbler_speed_rpm = int(self.dribbler_speed_rpm_slider.value())

        motor_control.direct_velocity_control.velocity.x_component_meters = (
            self.x_velocity_slider.value()
        )
        motor_control.direct_velocity_control.velocity.y_component_meters = (
            self.y_velocity_slider.value()
        )
        motor_control.direct_velocity_control.angular_velocity.radians_per_second = (
            self.angular_velocity_slider.value()
        )

        self.proto_unix_io.send_proto(MotorControl, motor_control)

    def value_change(self, value: float) -> str:
        """Converts the given float value to a string label

        :param value: float value to be converted
        """
        value = float(value)
        value_str = "%.2f" % value
        return value_str

    def setup_switch_drive_mode(self, title: str) -> QGroupBox:
        group_box = QGroupBox(title)
        dbox = QVBoxLayout()
        self.switch_drive_mode = QPushButton("Switch Drive Modes")
        self.switch_drive_mode.clicked.connect(self.switch_drive_mode())
        dbox.addWidget(
            self.switch_drive_mode, alignment=Qt.AlignmentFlag.AlignCenter
        )

        group_box.setLayout(dbox)
        return group_box

    def setup_direct_velocity(self, title: str) -> QGroupBox:
        """Create a widget to control the direct velocity of the robot's motors

        :param title: the name of the slider
        """
        group_box = QGroupBox(title)+
        dbox = QVBoxLayout()

        (
            x_layout,
            self.x_velocity_slider,
            self.x_velocity_label,
        ) = common_widgets.create_float_slider(
            "X (m/s)",
            2,
            -self.constants.robot_max_speed_m_per_s,
            self.constants.robot_max_speed_m_per_s,
            1,
        )
        (
            y_layout,
            self.y_velocity_slider,
            self.y_velocity_label,
        ) = common_widgets.create_float_slider(
            "Y (m/s)",
            2,
            -self.constants.robot_max_speed_m_per_s,
            self.constants.robot_max_speed_m_per_s,
            1,
        )
        (
            dps_layout,
            self.angular_velocity_slider,
            self.angular_velocity_label,
        ) = common_widgets.create_float_slider(
            "θ (rad/s)",
            2,
            -self.constants.robot_max_ang_speed_rad_per_s,
            self.constants.robot_max_ang_speed_rad_per_s,
            1,
        )

        # add listener functions for sliders to update label with slider value
        common_widgets.enable_slider(
            self.x_velocity_slider, self.x_velocity_label, self.value_change
        )
        common_widgets.enable_slider(
            self.x_velocity_slider, self.x_velocity_label, self.value_change
        )
        common_widgets.enable_slider(
            self.y_velocity_slider, self.y_velocity_label, self.value_change
        )
        common_widgets.enable_slider(
            self.angular_velocity_slider, self.angular_velocity_label, self.value_change
        )

        self.stop_and_reset_direct_velocity = QPushButton("Stop and Reset Direct Velocity")
        self.stop_and_reset_direct_velocity.clicked.connect(self.reset_direct_velocity_sliders)

        dbox.addLayout(x_layout)
        dbox.addLayout(y_layout)
        dbox.addLayout(dps_layout)
        dbox.addWidget(
            self.stop_and_reset_direct_velocity, alignment=Qt.AlignmentFlag.AlignCenter
        )

        group_box.setLayout(dbox)

        return group_box

    def setup_per_motor(self, title: str) -> QGroupBox:
        """Create a widget to control the direct velocity of the robot's motors

        :param title: the name of the slider
        """
        group_box = QGroupBox(title)
        dbox = QVBoxLayout()
        # create a power slider for each motor
        motors = ["front right", "front left", "back right", "back left"]
        self.motor_power_sliders = []
        power_layout = []
        motor_max_power = 1
        for i, motor in motors:
            (
                power_layout[i],
                self.motor_power_sliders[i],
                self.motor_power_labels[i],
            ) = common_widgets.create_float_slider(
                f"{motors[i]}%",
                2,
                -motor_max_power,  # maybe there should be a max motor power?
                motor_max_power,
                3,
            )

        common_widgets.enable_slider(
            self.motor_power_sliders[i], self.motor_power_labels[i], self.value_change
        )
        dbox.addLayout(power_layout[i])

        self.stop_and_reset_per_motor = QPushButton("Stop and Reset Per Motor")
        self.stop_and_reset_per_motor.clicked.connect(self.reset_per_motor_sliders())

        dbox.addWidget(
            self.stop_and_reset_per_motor, alignment=Qt.AlignmentFlag.AlignCenter
        )
        group_box.setLayout(dbox)

        return group_box

    def setup_dribbler(self, title: str) -> QGroupBox:
        """Create a widget to control the dribbler RPM

        :param title: the name of the slider
        """
        group_box = QGroupBox(title)
        dbox = QVBoxLayout()

        (
            dribbler_layout,
            self.dribbler_speed_rpm_slider,
            self.dribbler_speed_rpm_label,
        ) = common_widgets.create_float_slider(
            "RPM",
            1,
            self.constants.indefinite_dribbler_speed_rpm,
            -self.constants.indefinite_dribbler_speed_rpm,
            1,
        )

        # add listener function to update label with slider value
        common_widgets.enable_slider(
            self.dribbler_speed_rpm_slider,
            self.dribbler_speed_rpm_label,
            self.value_change,
        )

        self.stop_and_reset_dribbler = QPushButton("Stop and Reset Dribbler")
        self.stop_and_reset_dribbler.clicked.connect(self.reset_dribbler_slider)

        dbox.addLayout(dribbler_layout)
        dbox.addWidget(
            self.stop_and_reset_dribbler, alignment=Qt.AlignmentFlag.AlignCenter
        )
        group_box.setLayout(dbox)

        return group_box

    def switch_drive_mode(self):
        # if using per motor, switch to direct velocity and vice versa
        if self.perMotorControl:
            # disconnect and reset per motor sliders
            self.disconnect_per_motor_sliders()
            self.reset_per_motor_sliders()

            # enable direct velocity
            self.enable_direct_velocity()

            self.perMotorControl = False
        else:
            # disconnect and reset direct velocity sliders
            self.disconnect_direct_velocity_sliders()
            self.reset_direct_velocity_sliders()

            # enable per motor
            self.enable_per_motor()

            self.perMotorControl = True

    def disable_direct_velocity(self):
        # reset velocity slider values and disconnect
        self.reset_velocity_sliders()
        self.disconnect_velocity_sliders()

        # disable sliders by adding listener to keep slider value the same
        common_widgets.disable_slider(self.x_velocity_slider)
        common_widgets.disable_slider(self.y_velocity_slider)
        common_widgets.disable_slider(self.angular_velocity_slider)

        common_widgets.change_button_state(self.stop_and_reset_direct, False)

    def disable_dribbler(self):
        # reset dribbler slider values and disconnect
        self.reset_dribbler_sliders()
        self.disconnect_dribbler_sliders()

        # disable dribbler slider by adding listener to keep slider value the same
        common_widgets.disable_slider(self.dribbler_speed_rpm_slider)

        # disable button
        common_widgets.change_button_state(self.stop_and_reset_dribbler, False)

    def disable_per_motor(self):
        # reset per motor slider values and disconnect
        self.reset_per_motor_sliders()
        self.disconnect_per_motor_sliders()

        # disable motor slider by adding listener to keep slider value the same
        [common_widgets.disable_slider(per_motor_slider) for per_motor_slider in self.motor_power_sliders]

        # disable per motor stop/reset button
        common_widgets.change_button_state(self.stop_and_reset_per_motor, False)

    def enable_dribbler(self):
        common_widgets.enable_slider(
            self.dribbler_speed_rpm_slider,
            self.dribbler_speed_rpm_label,
            self.value_change,
        )

        # enable dribbler button+
        common_widgets.change_button_state(self.stop_and_reset_dribbler, True)

    def enable_direct_velocity(self):
        # reset motor control
        self.reset_direct_velocity_sliders()
        # enable all sliders by adding listener to update label with slider value
        common_widgets.enable_slider(
            self.x_velocity_slider, self.x_velocity_label, self.value_change
        )
        common_widgets.enable_slider(
            self.y_velocity_slider, self.y_velocity_label, self.value_change
        )
        common_widgets.enable_slider(
            self.angular_velocity_slider,
            self.angular_velocity_label,
            self.value_change,
        )

        common_widgets.change_button_state(self.stop_and_reset_direct_velocity, True)

    def enable_per_motor(self):
        # reset motor control
        self.reset_per_motor_sliders()
        # enable all sliders by adding listener to update label with slider value
        for i, per_motor_slider in self.motor_power_sliders:
            common_widgets.enable_slider(
                per_motor_slider, self.motor_power_labels[i], self.value_change
            )

        common_widgets.change_button_state(self.stop_and_reset_per_motor, True)

    def disconnect_direct_velocity_sliders(self) -> None:
        """Disconnect listener for changing values for velocity sliders"""
        self.x_velocity_slider.valueChanged.disconnect()
        self.y_velocity_slider.valueChanged.disconnect()
        self.angular_velocity_slider.valueChanged.disconnect()

    def disconnect_dribbler_sliders(self) -> None:
        """Disconnect listener for changing values for dribbler sliders"""
        self.dribbler_speed_rpm_slider.valueChanged.disconnect()

    def disconnect_per_motor_sliders(self) -> None:
        """Disconnect listener for changing values for motor sliders"""
        [motor_power_slider.valueChanged.disconnect() for motor_power_slider in self.motor_power_sliders]

    def disconnect_all_sliders(self) -> None:
        """Reset all sliders back to 0"""
        self.disconnect_dribbler_sliders()
        self.disconnect_per_motor_sliders()
        self.disconnect_direct_velocity_sliders()

    def reset_direct_velocity_sliders(self) -> None:
        """Reset direct sliders back to 0"""
        self.x_velocity_slider.setValue(0)
        self.y_velocity_slider.setValue(0)
        self.angular_velocity_slider.setValue(0)

    def reset_per_motor_sliders(self) -> None:
        """Reset motor sliders back to 0"""
        [motor_power_slider.setValue(0) for motor_power_slider in self.motor_power_sliders]

    def reset_dribbler_sliders(self) -> None:
        """Reset the dribbler slider back to 0"""
        self.dribbler_speed_rpm_slider.setValue(0)

    def reset_all_sliders(self) -> None:
        """Reset all sliders back to 0"""
        self.reset_direct_velocity_sliders()
        self.reset_dribbler_slider()
        self.reset_per_motor_sliders()
