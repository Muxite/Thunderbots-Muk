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
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates a kickoff friendly play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit KickoffFriendlyPlayFSM(const TbotsProto::AiConfig& ai_config);

    /**
    * create a vector of setup positions if not already existing.
    *
    * @param world_ptr the world pointer
    */
    void createKickoffSetupPositions(const WorldPtr &world_ptr);

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
    * Guard that checks if positions are set up.
    *
    * @param event
    */
    bool isSetupDone(const Update& event);

    /**
     * Guard that checks if the ball can be kicked.
     *
     * @param event
     */
    bool canKick(const Update& event);

    /**
    * Guard that checks if game has started (ball kicked).
    *
    * @param event
    */
    bool isPlaying(const Update& event);


    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(SetupState)
        DEFINE_SML_STATE(KickState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setupKickoff)
        DEFINE_SML_ACTION(kickoff)

        DEFINE_SML_GUARD(isSetupDone)
        //DEFINE_SML_GUARD(canKick)
        DEFINE_SML_GUARD(isPlaying)

        return make_transition_table(
                // src_state + event [guard] / action = dest_state
                // PlaySelectionFSM will transition to OffensePlay after the kick.
                *SetupState_S + Update_E[!isSetupDone_G] / setupKickoff_A = SetupState_S,
                SetupState_S  + Update_E[isSetupDone_G]                   = KickState_S,
                KickState_S   + Update_E[!isPlaying_G] / kickoff_A        = KickState_S,
                KickState_S   + Update_E[isPlaying_G]                     = X,
                X + Update_E                                              = X);
    }

private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<KickoffChipTactic> kickoff_chip_tactic;
    std::vector<std::shared_ptr<MoveTactic>> move_tactics;
    std::vector<Point> kickoff_setup_positions;
};