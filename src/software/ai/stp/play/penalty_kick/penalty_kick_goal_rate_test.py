"""Experiment harness measuring the penalty goal rate in simulation.

Not a regression test (tagged manual). Each seed perturbs the start conditions with a
seeded PRNG; one CSV row per seed is appended to $P2_OUT_DIR/goal_rate_$P2_LABEL.csv
and seeds already present are skipped, so interrupted runs can be resumed.

Environment: P2_LABEL (arm name), P2_SEED_START / P2_SEED_COUNT (seed range).
"""

import os
import random

import pytest

import software.python_bindings as tbots
from proto.game_state_pb2 import GameState
from proto.import_all_protos import Command
from proto.message_translation.tbots_protobuf import create_world_state
from proto.play_pb2 import PlayName
from proto.ssl_gc_common_pb2 import Team
from proto.validation_pb2 import ValidationStatus
from software.gameplay_tests.util import pytest_main
from software.gameplay_tests.validation.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)

LABEL = os.environ.get("P2_LABEL", "dev")
OUT_DIR = os.environ.get("P2_OUT_DIR", "/tmp/p2runs")
SEED_START = int(os.environ.get("P2_SEED_START", "0"))
SEED_COUNT = int(os.environ.get("P2_SEED_COUNT", "3"))
RESULT_FILE = os.path.join(OUT_DIR, f"goal_rate_{LABEL}.csv")
RESOLVE_TIMEOUT_S = 20.0


def seeds_done():
    try:
        with open(RESULT_FILE) as f:
            return {int(line.split(",")[0]) for line in f if line.strip()}
    except OSError:
        return set()


class PenaltyOutcome(Validation):
    """Records goal / miss / timeout plus legality indicators, then passes."""

    def __init__(self, seed, goal_x, start_x):
        self.seed = seed
        self.goal_x = goal_x
        self.start_x = start_x
        self.t_ready = None
        self.t_first_move = None
        self.max_speed = 0.0
        self.prev_speed = 0.0
        self.reaccel_events = 0
        self.min_x = start_x
        self.done = False

    def get_validation_status(self, world):
        if self.done:
            return ValidationStatus.PASSING
        t = world.time_sent.epoch_timestamp_seconds
        ready = (
            world.game_state.play_state == GameState.PLAY_STATE_READY
            and world.game_state.restart_reason == GameState.RESTART_REASON_PENALTY
        )
        ball = world.ball.current_state
        x = ball.global_position.x_meters
        y = ball.global_position.y_meters
        speed = (
            ball.global_velocity.x_component_meters**2
            + ball.global_velocity.y_component_meters**2
        ) ** 0.5
        if self.t_ready is None:
            if not ready:
                return ValidationStatus.FAILING
            self.t_ready = t
        if self.t_first_move is None and abs(x - self.start_x) > 0.3:
            self.t_first_move = t
        self.min_x = min(self.min_x, x)
        if (
            speed > 2.5
            and self.prev_speed < self.max_speed - 1.0
            and self.max_speed > 2.5
        ):
            self.reaccel_events += 1
        self.prev_speed = speed
        self.max_speed = max(self.max_speed, speed)

        outcome = None
        if x > self.goal_x and abs(y) < 0.5:
            outcome = "goal"
        elif x > self.goal_x or abs(y) > 3.1:
            outcome = "miss"
        elif self.max_speed > 2.0 and speed < 0.2:
            outcome = "miss"
        elif t - self.t_ready > RESOLVE_TIMEOUT_S:
            outcome = "timeout"
        if outcome is None:
            return ValidationStatus.FAILING
        self.done = True
        shot_t = (self.t_first_move - self.t_ready) if self.t_first_move else -1
        with open(RESULT_FILE, "a") as f:
            f.write(
                f"{self.seed},{outcome},{shot_t:.2f},{self.min_x - self.start_x:.2f},"
                f"{self.reaccel_events}\n"
            )
        return ValidationStatus.PASSING

    def get_validation_geometry(self, world):
        return create_validation_geometry([])

    def __repr__(self):
        return f"Penalty outcome recorded for seed {self.seed}"


OutcomeEventually, _, _, _ = create_validation_types(PenaltyOutcome)


@pytest.mark.parametrize("seed", range(SEED_START, SEED_START + SEED_COUNT))
def test_penalty_goal_rate(gameplay_test_runner, seed):
    if seed in seeds_done():
        pytest.skip("seed already recorded")
    os.makedirs(OUT_DIR, exist_ok=True)
    rng = random.Random(seed * 7919 + 13)
    field = tbots.Field.createSSLDivisionBField()
    ball = field.friendlyPenaltyMark()
    goalie_y = rng.uniform(-0.4, 0.4)
    shooter = ball + tbots.Vector(
        -0.22 + rng.uniform(-0.08, 0.08), rng.uniform(-0.1, 0.1)
    )
    supporters = [
        tbots.Point(ball.x() - 1.3 + rng.uniform(-0.1, 0.1), y + rng.uniform(-0.1, 0.1))
        for y in [-1.08, -0.36, 0.36, 1.08]
    ]

    def setup(*args):
        gameplay_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    field.friendlyGoalCenter() + tbots.Vector(0.2, 0),
                    shooter,
                    *supporters,
                ],
                yellow_robot_locations=[
                    field.enemyGoalCenter() + tbots.Vector(-0.2, goalie_y)
                ],
                ball_location=ball,
                ball_velocity=tbots.Vector(0, 0),
            )
        )
        gameplay_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        gameplay_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.PENALTY, team=Team.BLUE
        )
        gameplay_test_runner.set_plays(
            blue_play=PlayName.PenaltyKickPlay,
            yellow_play=PlayName.PenaltyKickEnemyPlay,
        )

    gameplay_test_runner.run_test(
        setup=setup,
        eventually_validation_sequence_set=[
            [OutcomeEventually(seed, field.enemyGoalCenter().x(), ball.x())]
        ],
        gc_cmd_with_delay=[(4.0, Command.Type.NORMAL_START, Team.UNKNOWN)],
        test_timeout_s=4.0 + RESOLVE_TIMEOUT_S + 6,
    )


if __name__ == "__main__":
    pytest_main(__file__)
