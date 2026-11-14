import pytest

import software.python_bindings as tbots
from proto.game_state_pb2 import GameState
from proto.import_all_protos import Command
from proto.message_translation.tbots_protobuf import create_world_state
from proto.play_pb2 import PlayName
from proto.ssl_gc_common_pb2 import Team
from proto.validation_pb2 import ValidationStatus
from software.gameplay_tests.simulated_test_fixture import pytest_main
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

    def __init__(self, support_line_x, require_ready=False):
        self.support_line_x = support_line_x
        self.require_ready = require_ready

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
        shooters = [
            robot.id
            for robot in world.friendly_team.team_robots
            if robot.id != 0
            and robot.current_state.global_position.x_meters > self.support_line_x
        ]
        return (
            ValidationStatus.PASSING if len(shooters) == 1 else ValidationStatus.FAILING
        )

    def get_validation_geometry(self, world):
        return create_validation_geometry([])

    def __repr__(self):
        return "Exactly one field robot ahead of the penalty support line during READY"


SingleShooterEventually, _, SingleShooterAlways, _ = create_validation_types(
    SinglePenaltyShooter
)


@pytest.mark.parametrize("crowded_setup", [False, True])
def test_only_one_robot_takes_penalty(simulated_test_runner, crowded_setup):
    field = tbots.Field.createSSLDivisionBField()
    ball = field.friendlyPenaltyMark()

    def setup(*args):
        supporters = [
            tbots.Point(ball.x() - 1.3, y) for y in [-1.08, -0.36, 0.36, 1.08]
        ]
        if crowded_setup:
            supporters[0] = ball + tbots.Vector(-0.3, 0.35)
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    field.friendlyGoalCenter() + tbots.Vector(0.2, 0),
                    ball + tbots.Vector(-0.22, 0),
                    *supporters,
                ],
                yellow_robot_locations=[field.enemyGoalCenter()],
                ball_location=ball,
                ball_velocity=tbots.Vector(0, 0),
            )
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.PENALTY, team=Team.BLUE
        )
        simulated_test_runner.set_plays(
            blue_play=PlayName.PenaltyKickPlay, yellow_play=PlayName.HaltPlay
        )

    simulated_test_runner.run_test(
        setup=setup,
        inv_always_validation_sequence_set=[[SingleShooterAlways(ball.x() - 1.0)]],
        inv_eventually_validation_sequence_set=[
            [SingleShooterEventually(ball.x() - 1.0, require_ready=True)],
            [BallEventuallyExitsRegion([tbots.Circle(ball, 0.15)])],
        ],
        ci_cmd_with_delay=[(4.0, Command.Type.NORMAL_START, Team.UNKNOWN)],
        test_timeout_s=12,
    )


if __name__ == "__main__":
    pytest_main(__file__)
