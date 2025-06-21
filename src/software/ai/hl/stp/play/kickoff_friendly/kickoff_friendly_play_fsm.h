#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"
#include "software/logger/logger.h"

struct KickoffFriendlyPlayFSM
{
    class KickState;

    class SetupState;

    struct ControlParams
    {
        // The origin point of the enemy threat
        Point enemy_threat_origin;
        // The maximum allowed speed mode
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates a kickoff friendly play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit KickoffFriendlyPlayFSM(TbotsProto::AiConfig ai_config);

    /**
     * Action to move robots to starting positions
     *
     * @param event the FSM event
     */
    void setupKickoff(const Update& event);

    /**
     * Action to kick the ball.
     *
     * @param event the FSM event
     */
    void kickoff(const Update& event);

    /**
     * Guard that checks if the ball can be kicked.
     *
     * @param event
     */
    void canKick(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(SetupState)

        DEFINE_SML_STATE(KickState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setupKickoff)

        DEFINE_SML_ACTION(kickoff)

        DEFINE_SML_GUARD(canKick)

        return make_transition_table(
                // src_state + event [guard] / action = dest_state
                *SetupState_S + Update_E / setupKickoff_A = SetupState_S,
                SetupState_S + Update_E[canKick_G] = KickState_S,
                X + Update_E                                     = X);
    }

private:
    std::vector<Point> kickoff_setup_positions;
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<MoveTactic> move_tactic;
    std::shared_ptr<PrepareKickoffMoveTactic> prepare_kickoff_move_tactic;
    std::vector<std::shared_ptr<MoveTactic>> move_tactics;
    std::shared_ptr<KickoffChipTactic> kickoff_chip_tactic;
};