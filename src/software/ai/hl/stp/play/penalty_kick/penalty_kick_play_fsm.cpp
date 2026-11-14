#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play_fsm.h"

PenaltyKickPlayFSM::PenaltyKickPlayFSM(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayFSM<PenaltyKickPlayFSM>(ai_config_ptr),
      penalty_kick_tactic(std::make_shared<PenaltyKickTactic>(ai_config_ptr)),
      shooter_setup_tactic(std::make_shared<PenaltySetupTactic>(ai_config_ptr)),
      penalty_setup_tactics(std::vector<std::shared_ptr<PenaltySetupTactic>>())
{
    // Choose a setup shooter that can also dribble and kick after NORMAL_START.
    shooter_setup_tactic->mutableRobotCapabilityRequirements() =
        penalty_kick_tactic->robotCapabilityRequirements();
}

void PenaltyKickPlayFSM::performKick(const Update& event)
{
    updateSupportPositions(event);
    PriorityTacticVector tactics_to_run = {{}, {}};
    if (!kick_started)
    {
        // The robot parked behind the ball during setup takes the kick
        penalty_kick_tactic->setLastExecutionRobot(
            shooter_setup_tactic->getLastExecutionRobot());
        kick_started = true;
    }
    if (event.common.num_tactics > 0)
    {
        tactics_to_run[0].emplace_back(penalty_kick_tactic);
    }
    // Keep commanding the other robots away from the ball. In particular, a robot
    // replaced as shooter must retreat rather than receive the play's halt fallback.
    tactics_to_run[1].insert(tactics_to_run[1].end(), penalty_setup_tactics.begin(),
                             penalty_setup_tactics.end());
    event.common.set_tactics(tactics_to_run);
}

void PenaltyKickPlayFSM::setupPosition(const Update& event)
{
    kick_started = false;
    updateSupportPositions(event);
    PriorityTacticVector tactics_to_run = {{}, {}};
    if (event.common.num_tactics > 0)
    {
        const auto ball = event.common.world_ptr->ball().position();
        const auto goal = event.common.world_ptr->field().enemyGoalpostPos();
        const Vector behind_ball_direction = (ball - goal).normalize();
        const Point behind_ball =
            ball + behind_ball_direction *
                       (DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS + 0.1);
        shooter_setup_tactic->updateControlParams(behind_ball,
                                                  (goal - ball).orientation());
        // Assign the shooter first, before optimizing the supporting formation.
        tactics_to_run[0].emplace_back(shooter_setup_tactic);
    }
    tactics_to_run[1].insert(tactics_to_run[1].end(), penalty_setup_tactics.begin(),
                             penalty_setup_tactics.end());
    event.common.set_tactics(tactics_to_run);
}

void PenaltyKickPlayFSM::updateSupportPositions(const Update& event)
{
    const unsigned int num_supporters =
        event.common.num_tactics > 0 ? event.common.num_tactics - 1 : 0;
    if (num_supporters != penalty_setup_tactics.size())
    {
        penalty_setup_tactics.resize(num_supporters);
        for (auto& tactic : penalty_setup_tactics)
        {
            if (!tactic)
                tactic = std::make_shared<PenaltySetupTactic>(ai_config_ptr);
        }
    }
    const double support_x =
        event.common.world_ptr->field().friendlyPenaltyMark().x() - 1.25;
    for (size_t i = 0; i < penalty_setup_tactics.size(); ++i)
    {
        const double y_offset =
            8 *
            (static_cast<double>(i) -
             (static_cast<double>(penalty_setup_tactics.size()) - 1) / 2) *
            ROBOT_MAX_RADIUS_METERS;
        penalty_setup_tactics[i]->updateControlParams(Point(support_x, y_offset),
                                                      Angle::zero());
    }
}

bool PenaltyKickPlayFSM::setupPositionDone(const Update& event)
{
    return !event.common.world_ptr->gameState().isSetupState();
}

bool PenaltyKickPlayFSM::kickDone(const Update& event)
{
    return event.common.world_ptr->gameState().isStopped();
}
