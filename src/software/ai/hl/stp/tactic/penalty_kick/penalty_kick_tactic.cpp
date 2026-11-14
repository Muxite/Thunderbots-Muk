#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic.h"

PenaltyKickTactic::PenaltyKickTactic(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticBase<PenaltyKickFSM, DribbleFSM, KickFSM, GetBehindBallFSM>(
          {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick},
          ai_config_ptr)
{
    // The kick must not be handed over between robots mid-execution: every robot that
    // is not the current kicker has its FSM reset each tick, so its estimated cost
    // (reaching the ball) would always undercut the kicker's (dribbling to the shot).
    setStickyAssignment(true);
}

void PenaltyKickTactic::updateControlParams() {}

void PenaltyKickTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}
