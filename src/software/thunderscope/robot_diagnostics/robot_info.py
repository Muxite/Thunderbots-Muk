from __future__ import annotations
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.py_constants import *
from proto.import_all_protos import *
import software.thunderscope.common.common_widgets as common_widgets
from software.thunderscope.constants import *
from software.thunderscope.robot_diagnostics.motor_fault_view import MotorFaultView
import time as time
from typing import Type
from collections import deque


class BreakbeamLabel(QLabel):
    """Displays the current breakbeam status
    Extension of a QLabel which displays a tooltip and updates the UI with the current status
    """

    BREAKBEAM_BORDER = "border: 1px solid black"

    def __init__(self) -> None:
        """Constructs a breakbeam indicator and sets the UI to the default uninitialized state"""
        super().__init__()

    def update_breakbeam_status(self, new_breakbeam_status: bool) -> None:
        """Updates the current breakbeam status and refreshes the UI accordingly

        :param new_breakbeam_status: the new breakbeam status
        """
        self.breakbeam_status = new_breakbeam_status

        if self.breakbeam_status is None:
            self.setStyleSheet(
                f"background-color: transparent; {self.BREAKBEAM_BORDER}"
            )
        elif self.breakbeam_status:
            self.setStyleSheet(
                f"background-color: red; {self.BREAKBEAM_BORDER};" "border-color: red"
            )
        else:
            self.setStyleSheet(
                f"background-color: green; {self.BREAKBEAM_BORDER};"
                "border-color: green"
            )

    def event(self, event: QtCore.QEvent) -> bool:
        """Overridden event function which intercepts all events
        On hover, displays a tooltip with the current breakbeam status

        :param event: event to check
        """
        common_widgets.display_tooltip(
            event,
            (
                "No Signal Yet"
                if self.breakbeam_status is None
                else "In Beam"
                if self.breakbeam_status
                else "Not In Beam"
            ),
        )

        return super().event(event)


class RobotInfo(QWidget):
    # Offsets the minimum of the battery bar from the minimum ideal voltage
    # Allows battery % to go below the minimum ideal level
    BATTERY_MIN_OFFSET = 3

    toggle_one_connection_signal = QtCore.pyqtSignal(int, int)

    def __init__(
        self,
        robot_id: int,
        available_control_modes: list[IndividualRobotMode],
        individual_robot_control_mode_signal: Type[QtCore.pyqtSignal],
    ) -> None:
        """Initialize a single robot's info widget

        :param robot_id: id of robot whose info is being displayed
        :param available_control_modes: the currently available input modes for the robots
                                        according to what mode thunderscope is run in
        :param individual_robot_control_mode_signal: signal that should be emitted when a robot changes control mode
        """
        super().__init__()

        self.robot_id = robot_id
        self.individual_robot_control_mode_signal = individual_robot_control_mode_signal

        self.time_of_last_robot_status = time.time()

        self.layout = QHBoxLayout()

        self.status_layout = QVBoxLayout()

        # Battery Bar
        self.stats_layout = QHBoxLayout()
        self.battery_progress_bar = common_widgets.ColorProgressBar(
            MIN_BATTERY_VOLTAGE - self.BATTERY_MIN_OFFSET, MAX_BATTERY_VOLTAGE
        )

        # Battery Voltage Label
        self.battery_label = QLabel()

        # Label changes when voltage bar level changes
        self.battery_progress_bar.floatValueChanged.connect(
            lambda float_val: self.battery_label.setText("%.2fV" % float_val)
        )

        # Stop primitive received indicator
        self.stop_primitive_label = QLabel()
        self.stats_layout.addWidget(self.stop_primitive_label)

        # Primitive loss rate label
        self.primitive_loss_rate_label = common_widgets.ColorQLabel(
            label_text="P%",
            initial_value=0,
            max_val=MAX_ACCEPTABLE_PACKET_LOSS_PERCENT,
        )

        # Primitive round-trip time label and queue
        self.primitive_rtt_label = common_widgets.ColorQLabel(
            label_text="RTT:",
            initial_value=0,
            max_val=MAX_ACCEPTABLE_MILLISECOND_ROUND_TRIP_TIME,
            min_val=MIN_ACCEPTABLE_MILLISECOND_ROUND_TRIP_TIME,
        )
        self.previous_primitive_rtt_values = deque(
            maxlen=MAX_LENGTH_PRIMITIVE_SET_STORE
        )

        self.stats_layout.addWidget(self.primitive_loss_rate_label)
        self.stats_layout.addWidget(self.primitive_rtt_label)

        self.stats_layout.addWidget(self.battery_progress_bar)
        self.stats_layout.addWidget(self.battery_label)

        self.status_layout.addLayout(self.stats_layout)

        # Control mode dropdown
        self.control_mode_layout = QHBoxLayout()
        self.control_mode_menu = self.__create_control_mode_menu(
            available_control_modes
        )

        # Button to expand/collapse the robot status view
        self.expand_robot_status_button = QPushButton()
        self.expand_robot_status_button.setCheckable(True)
        self.expand_robot_status_button.setText("INFO")

        # motor fault visualisation for the 4 wheel motors
        self.motor_fault_view = MotorFaultView()

        self.control_mode_layout.addWidget(self.motor_fault_view)
        self.control_mode_layout.addWidget(self.control_mode_menu)
        self.control_mode_layout.addWidget(self.expand_robot_status_button)

        self.status_layout.addLayout(self.control_mode_layout)

        # Layout containing the Vision Pattern and breakbeam indicator
        self.robot_model_layout = QVBoxLayout()
        self.robot_model_layout.setContentsMargins(0, 5, 5, 0)

        # Vision Pattern
        self.color_vision_pattern = self.__create_vision_pattern(
            Colors.ROBOT_MIDDLE_BLUE, ROBOT_RADIUS, True
        )
        self.bw_vision_pattern = self.__create_vision_pattern(
            Colors.BW_ROBOT_MIDDLE_BLUE, ROBOT_RADIUS, False
        )

        self.robot_model = QLabel()

        # breakbeam indicator above robot
        self.breakbeam_label = BreakbeamLabel()
        self.breakbeam_label.setFixedWidth(self.color_vision_pattern.width())
        self.breakbeam_label.setFixedHeight(
            int(self.color_vision_pattern.width() * 0.25)
        )

        self.robot_model_layout.addWidget(self.breakbeam_label)
        self.robot_model_layout.addWidget(self.robot_model)
        self.layout.addLayout(self.robot_model_layout)

        self.__reset_ui()

        self.layout.addLayout(self.status_layout)
        self.setLayout(self.layout)

        # Last robot state
        self.last_robot_status = None
        self.last_robot_statistic = None

    def update_robot_status(self, robot_status: RobotStatus):
        """Receives a RobotStatus message and updates the UI to reflect the new data

        :param robot_status: The latest RobotStatus message for this robot
        """
        self.time_of_last_robot_status = time.time()
        self.last_robot_status = robot_status

        self.robot_model.setPixmap(self.color_vision_pattern)
        self.__update_ui()

        # We should check in after DISCONNECT_DURATION_MS and see whether we've
        # received any new RobotStatus messages within the time that passed.
        # If not, then the robot probably disconnected.
        QtCore.QTimer.singleShot(
            int(DISCONNECT_DURATION_MS), self.__check_for_disconnection
        )

    def update_robot_statistic(self, robot_statistic: RobotStatistic):
        """Receives a RobotStatistic message and updates the UI to reflect the new data

        :param robot_statistic: The latest RobotStatistic message for this robot
        """
        self.last_robot_statistic = robot_statistic
        self.__update_ui()

    def __create_control_mode_menu(
        self, available_control_modes: list[IndividualRobotMode]
    ) -> QComboBox:
        """Creates the drop down menu to select the input for each robot

        :param robot_id: the id of the robot this menu belongs to
        :param available_control_modes: the currently available input modes for the robots
                                        according to what mode thunderscope is run in
        :return: QComboBox object
        """
        control_mode_menu = QComboBox()

        control_mode_menu.addItems(
            [control_mode.name for control_mode in available_control_modes]
        )

        if IndividualRobotMode.AI in available_control_modes:
            control_mode_menu.setCurrentIndex(
                control_mode_menu.findText(IndividualRobotMode.AI.name)
            )
        else:
            control_mode_menu.setCurrentIndex(
                control_mode_menu.findText(IndividualRobotMode.NONE.name)
            )

        control_mode_menu.currentIndexChanged.connect(
            lambda mode: self.individual_robot_control_mode_signal.emit(
                self.robot_id, IndividualRobotMode(mode)
            )
        )

        return control_mode_menu

    def __create_vision_pattern(
        self, team_colour: QtGui.QColor, radius: int, connected: bool
    ) -> QtGui.QPixmap:
        """Given a robot id, team color and radius, draw the vision
        pattern on a pixmap and return it.

        :param team_colour: The team colour
        :param radius: The radius of the robot
        :param connected: True if vision pattern should have color, False if black and white
        """
        pixmap = QtGui.QPixmap(radius * 2, radius * 2)
        pixmap.fill(QtCore.Qt.GlobalColor.transparent)

        painter = QtGui.QPainter(pixmap)
        painter.setPen(pg.mkPen("black"))
        painter.setBrush(pg.mkBrush("black"))

        common_widgets.draw_robot(
            painter,
            QtCore.QRectF(
                0,
                0,
                int(radius * 2),
                int(radius * 2),
            ),
            -45,
            270,
        )

        # Draw the vision pattern
        # Draw the centre team color
        painter.setBrush(pg.mkBrush(team_colour))
        painter.drawEllipse(QtCore.QPointF(radius, radius), radius / 3, radius / 3)
        painter.setPen(pg.mkPen("white"))
        painter.drawText(
            QtCore.QPointF(radius - radius / 8, radius + radius / 4), str(self.robot_id)
        )
        painter.setPen(pg.mkPen("black"))

        # Grab the colors for the vision pattern and setup the locations
        # for the four circles in the four corners
        top_circle_locations = [
            QtCore.QPointF(radius + radius / 2 + 5, radius - radius / 2),
            QtCore.QPointF(radius - radius / 2 - 5, radius - radius / 2),
            QtCore.QPointF(radius - radius / 2, radius + radius / 2 + 5),
            QtCore.QPointF(radius + radius / 2, radius + radius / 2 + 5),
        ]

        for color, location in zip(
            (
                Colors.VISION_PATTERN_LOOKUP
                if connected
                else Colors.BW_VISION_PATTERN_LOOKUP
            )[self.robot_id],
            top_circle_locations,
        ):
            painter.setBrush(pg.mkBrush(color))
            painter.drawEllipse(location, radius / 5, radius / 5)

        painter.end()

        return pixmap

    def __check_for_disconnection(self) -> None:
        """Calculates the time between the last robot status received and now.
        If more than our threshold, assume the robot disconnected and reset the UI.
        """
        if (
            time.time() - self.time_of_last_robot_status
            >= DISCONNECT_DURATION_MS * SECONDS_PER_MILLISECOND
        ):
            self.__reset_ui()

    def __reset_ui(self) -> None:
        """Resets the UI to the default, uninitialized values"""
        self.__update_stop_primitive(None)

        self.robot_model.setPixmap(self.bw_vision_pattern)

        self.primitive_loss_rate_label.set_float_val(0)
        self.primitive_rtt_label.set_float_val(0)
        self.breakbeam_label.update_breakbeam_status(None)
        self.battery_progress_bar.setValue(self.battery_progress_bar.minimum())
        self.motor_fault_view.reset_ui()

    def __update_stop_primitive(self, is_running: bool | None) -> None:
        """Updates the stop primitive label based on the current running state

        :param is_running: true if the robot is running currently, false if the
                           robot is stopped, or None if the robot is disconnected
                           and there is no primitive executor status
        """
        if is_running is None:
            self.stop_primitive_label.setVisible(False)
        else:
            self.stop_primitive_label.setVisible(True)
            self.stop_primitive_label.setText("RUN" if is_running else "STOP")
            self.stop_primitive_label.setStyleSheet(
                f"background-color: {'green' if is_running else 'red'}; border: 1px solid black;"
            )

    def __update_ui(self) -> None:
        """Receives important sections of RobotStatus proto for this robot and updates widget with alerts
        Checks for
            - Whether breakbeam is tripped
            - If there are any motor faults
            - Battery voltage, and warns if it's too low
            - If this robot has errors
            - If the robot is stopped or running
        """
        if not self.last_robot_status or not self.last_robot_statistic:
            return

        motor_status = self.last_robot_status.motor_status
        power_status = self.last_robot_status.power_status
        network_status = self.last_robot_status.network_status
        primitive_executor_status = self.last_robot_status.primitive_executor_status
        rtt_time_seconds = self.last_robot_statistic.round_trip_time_seconds

        self.__update_stop_primitive(primitive_executor_status.running_primitive)

        self.primitive_loss_rate_label.set_float_val(
            network_status.primitive_packet_loss_percentage
        )

        self.primitive_rtt_label.set_float_val(
            self.__calculate_average_round_trip_time(
                rtt_time_seconds * MILLISECONDS_PER_SECOND
            )
        )

        self.breakbeam_label.update_breakbeam_status(power_status.breakbeam_tripped)

        self.motor_fault_view.refresh(
            motor_status,
            # we access the front left field just to get the enum descriptor
            # so that we can translate from enum indexes to fault names
            motor_status.front_left.DESCRIPTOR.fields_by_name["motor_faults"],
        )

        self.battery_progress_bar.setValue(power_status.battery_voltage)

    def __calculate_average_round_trip_time(self, new_time: float) -> int:
        """Logs the given round-trip time and calculates the average

        :param new_time: The new round-trip time to add
        :return: The mean integer value of the previous round-trip times in milliseconds
        """
        self.previous_primitive_rtt_values.append(new_time)
        sum_rtt_time_milliseconds = sum(self.previous_primitive_rtt_values)
        return int(sum_rtt_time_milliseconds / len(self.previous_primitive_rtt_values))
