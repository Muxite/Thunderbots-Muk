#pragma once

#include "software/ai/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/stp/tactic/kick_or_chip/kick_or_chip_fsm.h"
#include "software/ai/stp/tactic/move/move_fsm.h"
#include "software/ai/stp/tactic/tactic_base.hpp"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/intersection.h"

/**
 * Finite State Machine class for Penalty Kicks
 */
struct PenaltyKickFSM : TacticFSM<PenaltyKickFSM>
{
    using Update = TacticFSM<PenaltyKickFSM>::Update;

    struct ControlParams
    {
    };

    /**
     * Constructor for PenaltyKickFSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit PenaltyKickFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Helper function that determines whether the shooter robot has a viable shot on net.
     *
     * @param enemy_goalie  the enemy goalie that we are scoring against
     * @param field         the current field being played on
     * @param ball_position the coordinate location of the ball
     * @param robot         the shooter robot
     *
     * @return true if the robot has a viable shot and false if the enemy goalkeeper will
     * likely save the shot.
     */
    static bool evaluatePenaltyShot(std::optional<Robot> enemy_goalie, Field field,
                                    Point ball_position, Robot robot);

    /**
     * Helper function that returns the point on the enemy goal line where the shooter
     * should aim at.
     *
     * @param enemy_goalie  the goalie that the shooter is scoring against
     * @param field         the field being played on
     *
     * @return the Point on the enemy goal-line where the shooter robot should aim
     */
    static const Point evaluateNextShotPosition(std::optional<Robot> enemy_goalie,
                                                Field field);

    /**
     * Action that causes the shooter to shoot the ball.
     *
     * @param event          PenaltyKickFSM::Update event
     * @param processEvent   processes the KickOrChipFSM::Update
     */
    void shoot(const Update& event,
               boost::sml::back::process<KickOrChipFSM::Update> processEvent);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DribbleFSM)
        DEFINE_SML_STATE(KickOrChipFSM)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_SUB_FSM_UPDATE_ACTION(shoot, KickOrChipFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *DribbleFSM_S + Update_E / shoot_A = KickOrChipFSM_S,
            KickOrChipFSM_S + Update_E / shoot_A, KickOrChipFSM_S = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    };

   private:
    static constexpr double PENALTY_KICK_POST_OFFSET = 0.12;
    static constexpr double PENALTY_KICK_SHOT_SPEED  = 5.0;
    // the ball must at least reach this speed for the kick to be considered taken
    static constexpr double PENALTY_KICK_MIN_KICKED_BALL_SPEED = 3.0;

    // expected maximum acceleration of the opposition goalie robot
    static constexpr double PENALTY_KICK_GOALIE_MAX_ACC = 1.5;
    static constexpr double SSL_VISION_DELAY            = 0.30;  // seconds

    // the fraction of the enemy side of the field that we define to be the lower limit of
    // where we choose to shoot in other words, it helps define the minimum distance at
    // which we decide to potentially shoot
    static constexpr double PENALTY_KICK_MIN_SHOT_X_DISTANCE_FACTOR = 1.0 / 3.0;

    Angle shot_angle;
    bool shot_angle_chosen = false;
};
