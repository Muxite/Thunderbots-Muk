import pytest

import software.python_bindings as tbots
from proto.game_state_pb2 import GameState
from proto.import_all_protos import Command
from proto.message_translation.tbots_protobuf import create_world_state
from proto.play_pb2 import PlayName
from proto.ssl_gc_common_pb2 import Team
from proto.validation_pb2 import ValidationStatus
from software.gameplay_tests.util import pytest_main
from software.gameplay_tests.validation.ball_enters_region import (
    BallEventuallyExitsRegion,
)
from software.gameplay_tests.validation.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class SinglePenaltyShooter(Validation):
    """Only the kicker may remain ahead of the support line after NORMAL_START."""

    def __init__(self, support_line_x, require_ready=False, attacker_is_yellow=False):
        self.support_line_x = support_line_x
        self.require_ready = require_ready
        self.attacker_is_yellow = attacker_is_yellow
        self.shooter_id = None

    def get_validation_status(self, world):
        ready = (
            world.game_state.play_state == GameState.PLAY_STATE_READY
            and world.game_state.restart_reason == GameState.RESTART_REASON_PENALTY
        )
        if not ready:
            return (
                ValidationStatus.FAILING
                if self.require_ready
                else ValidationStatus.PASSING
            )
        # The simulated test world is seen from the blue team's side. Yellow attacks
        # towards -x, so "ahead of the line" flips with the attacker.
        team = world.enemy_team if self.attacker_is_yellow else world.friendly_team
        direction = -1 if self.attacker_is_yellow else 1
        shooters = [
            robot.id
            for robot in team.team_robots
            if robot.id != 0
            and direction * robot.current_state.global_position.x_meters
            > direction * self.support_line_x
        ]
        if len(shooters) != 1:
            return ValidationStatus.FAILING
        # The kick must not be handed over between robots
        if self.shooter_id is None:
            self.shooter_id = shooters[0]
        return (
            ValidationStatus.PASSING
            if shooters[0] == self.shooter_id
            else ValidationStatus.FAILING
        )

    def get_validation_geometry(self, world):
        return create_validation_geometry([])

    def __repr__(self):
        return "Exactly one field robot ahead of the penalty support line during READY"


SingleShooterEventually, _, SingleShooterAlways, _ = create_validation_types(
    SinglePenaltyShooter
)


@pytest.mark.parametrize(
    "attacker, layout",
    [
        ("blue", "tidy"),
        ("blue", "crowded"),
        ("blue", "scattered"),
        ("blue", "contested"),
        ("yellow", "tidy"),
        ("yellow", "scattered"),
        ("yellow", "contested"),
    ],
)
def test_only_one_robot_takes_penalty(gameplay_test_runner, attacker, layout):
    field = tbots.Field.createSSLDivisionBField()
    attacker_is_yellow = attacker == "yellow"
    # Direction (in the shared simulation frame) in which the attacker shoots
    s = -1.0 if attacker_is_yellow else 1.0
    ball = (
        field.enemyPenaltyMark() if attacker_is_yellow else field.friendlyPenaltyMark()
    )
    attacking_goalie = (
        field.enemyGoalCenter() if attacker_is_yellow else field.friendlyGoalCenter()
    ) + tbots.Vector(0.2 * s, 0)
    defending_goalie = (
        field.friendlyGoalCenter() if attacker_is_yellow else field.enemyGoalCenter()
    )

    def setup(*args):
        supporters = [
            tbots.Point(ball.x() - s * 1.3, y) for y in [-1.08, -0.36, 0.36, 1.08]
        ]
        if layout == "crowded":
            supporters[0] = ball + tbots.Vector(-0.3 * s, 0.35)
        elif layout == "scattered":
            supporters = [
                tbots.Point(-3.0 * s, 2.5),
                tbots.Point(0.5 * s, -2.0),
                tbots.Point(-3.5 * s, -1.0),
                tbots.Point(1.0 * s, 1.5),
            ]
        elif layout == "contested":
            # Ticket #3642: two robots start next to the ball (on either side of the
            # line to the goal), the rest are spread out. Only one may take the kick.
            supporters = [
                ball + tbots.Vector(-0.8 * s, -0.4),
                tbots.Point(-3.0 * s, 2.5),
                tbots.Point(0.5 * s, -2.0),
                tbots.Point(-3.5 * s, -1.0),
            ]
        if layout == "contested":
            first_shooter_candidate = ball + tbots.Vector(-0.6 * s, 0.5)
        elif layout == "scattered":
            first_shooter_candidate = tbots.Point(-2.5 * s, 0.0)
        else:
            first_shooter_candidate = ball + tbots.Vector(-0.22 * s, 0)
        attacking_robots = [
            attacking_goalie,
            first_shooter_candidate,
            *supporters,
        ]
        gameplay_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=(
                    [defending_goalie] if attacker_is_yellow else attacking_robots
                ),
                yellow_robot_locations=(
                    attacking_robots if attacker_is_yellow else [defending_goalie]
                ),
                ball_location=ball,
                ball_velocity=tbots.Vector(0, 0),
            )
        )
        gameplay_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        gameplay_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.PENALTY,
            team=Team.YELLOW if attacker_is_yellow else Team.BLUE,
        )
        gameplay_test_runner.set_plays(
            blue_play=(
                PlayName.HaltPlay if attacker_is_yellow else PlayName.PenaltyKickPlay
            ),
            yellow_play=(
                PlayName.PenaltyKickPlay if attacker_is_yellow else PlayName.HaltPlay
            ),
        )

    delay_s = 9.0 if layout in ("scattered", "contested") else 4.0
    gameplay_test_runner.run_test(
        setup=setup,
        always_validation_sequence_set=[
            [
                SingleShooterAlways(
                    ball.x() - s * 1.0, attacker_is_yellow=attacker_is_yellow
                )
            ]
        ],
        eventually_validation_sequence_set=[
            [
                SingleShooterEventually(
                    ball.x() - s * 1.0,
                    require_ready=True,
                    attacker_is_yellow=attacker_is_yellow,
                )
            ],
            [BallEventuallyExitsRegion([tbots.Circle(ball, 0.15)])],
        ],
        gc_cmd_with_delay=[(delay_s, Command.Type.NORMAL_START, Team.UNKNOWN)],
        test_timeout_s=delay_s + 8,
    )


if __name__ == "__main__":
    pytest_main(__file__)
