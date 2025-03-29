#include "software/ai/hl/stp/play/kickoff_friendly/kickoff_friendly_play_fsm.h"

KickoffFriendlyPlayFSM ::KickoffFriendlyPlayFSM(TbotsProto::AiConfig ai_config)
        : ai_config(ai_config), // add things
{
}

void KickoffFriendlyPlayFSM::setupKickoff(const Update& event)
{
    std::vector<Point> kickoff_setup_positions = {
            // Robot 1
            Point(world_ptr->field().centerPoint() +
                  Vector(-world_ptr->field().centerCircleRadius(), 0)),
            // Robot 2
            // Goalie positions will be handled by the goalie tactic
            // Robot 3
            Point(
                    world_ptr->field().centerPoint() +
                    Vector(-world_ptr->field().centerCircleRadius() - 4 * ROBOT_MAX_RADIUS_METERS,
                           -1.0 / 3.0 * world_ptr->field().yLength())),
            // Robot 4
            Point(
                    world_ptr->field().centerPoint() +
                    Vector(-world_ptr->field().centerCircleRadius() - 4 * ROBOT_MAX_RADIUS_METERS,
                           1.0 / 3.0 * world_ptr->field().yLength())),
            // Robot 5
            Point(world_ptr->field().friendlyGoalpostPos().x() +
                  world_ptr->field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
                  world_ptr->field().friendlyGoalpostPos().y()),
            // Robot 6
            Point(world_ptr->field().friendlyGoalpostNeg().x() +
                  world_ptr->field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
                  world_ptr->field().friendlyGoalpostNeg().y()),
    };



    PriorityTacticVector tactics_to_return = {{}};

    event.common.set_tactics(tactics_to_return);
}

void KickoffFriendlyPlayFSM::kickoff(const Update& event)
{

    PriorityTacticVector tactics_to_return = {{}};

    event.common.set_tactics(tactics_to_return);
}
