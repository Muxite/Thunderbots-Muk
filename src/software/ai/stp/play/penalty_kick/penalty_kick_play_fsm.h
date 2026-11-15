#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/stp/play/play_fsm.hpp"
#include "software/ai/stp/tactic/halt/halt_tactic.h"
#include "software/ai/stp/tactic/move/move_tactic.h"
#include "software/ai/stp/tactic/penalty_kick/penalty_kick_tactic.h"
#include "software/logger/logger.h"

struct PenaltyKickPlayFSM : PlayFSM<PenaltyKickPlayFSM>
{
    /**
     * Control Parameters for PenaltyKickPlay
     */
    struct ControlParams
    {
    };

    class SetupPositionState;
    class PerformKickState;

    /**
     * Creates a penalty kick play FSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit PenaltyKickPlayFSM(
        std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Action to set up the robots in position to start the penalty kick
     *
     * @param event the PenaltyKickPlayFSM Update event
     */
    void setupPosition(const Update& event);

    /**
     * Action to make the penalty kick taking robot perform the PenaltyKickTactic
     *
     * @param event the PenaltyKickPlayFSM Update event
     */
    void performKick(const Update& event);

    /**
     * Guard to check if robots are in position to start the penalty kick
     *
     * @param event the PenaltyKickPlayFSM Update event
     *
     * @return whether the robots have gotten into position to start the penalty kick
     */
    bool setupPositionDone(const Update& event);

    /**
     * Guard to check if the robot has performed the kick
     *
     * @param event the PenaltyKickPlayFSM Update event
     *
     * @return whether the robot has finished performing a kick
     */
    bool kickDone(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(SetupPositionState)
        DEFINE_SML_STATE(PerformKickState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setupPosition)
        DEFINE_SML_ACTION(performKick)

        DEFINE_SML_GUARD(setupPositionDone)
        DEFINE_SML_GUARD(kickDone)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *SetupPositionState_S + Update_E[!setupPositionDone_G] / setupPosition_A =
                SetupPositionState_S,
            SetupPositionState_S + Update_E[setupPositionDone_G] / performKick_A =
                PerformKickState_S,
            PerformKickState_S + Update_E[!kickDone_G] / performKick_A,
            PerformKickState_S + Update_E[kickDone_G] = X, X + Update_E = X);
    }

   private:
    // Refresh the non-shooting formation in both setup and kick phases.
    void updateSupportPositions(const Update& event);

    // Whether the kick phase has started, i.e. the kicker has been handed over from the
    // setup shooter
    bool kick_started = false;

    // Latched once the ball has left the penalty mark, so a kicker that is lost or
    // replaced after the shot can never be followed by a second kick
    bool shot_taken = false;
    Point kick_start_ball_position;

    // How far the ball must travel from where the kick started to count as shot
    static constexpr double SHOT_TAKEN_DISTANCE_METERS = 0.5;

    // The ball must also be moving at least this fast (m/s), which a dribble never is
    static constexpr double SHOT_TAKEN_MIN_SPEED = 2.0;

    std::shared_ptr<PenaltyKickTactic> penalty_kick_tactic;
    std::shared_ptr<PenaltySetupTactic> shooter_setup_tactic;
    std::vector<std::shared_ptr<PenaltySetupTactic>> penalty_setup_tactics;
};
