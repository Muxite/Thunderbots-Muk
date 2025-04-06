#include "software/ai/hl/stp/play/kickoff_friendly/kickoff_friendly_play_fsm.h"

KickoffFriendlyPlayFSM ::KickoffFriendlyPlayFSM(TbotsProto::AiConfig ai_config)
        : ai_config(ai_config), // add things
{
}

void KickoffFriendlyPlayFSM::setupKickoff(const Update& event)
{
    // **** this could be done once instead of redone every event

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

    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<PrepareKickoffMoveTactic>(), std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>()};

    auto enemy_threats =
            getAllEnemyThreats(world_ptr->field(), world_ptr->friendlyTeam(),
                               world_ptr->enemyTeam(), world_ptr->ball(), false);

    // first priority is the kicker.
    move_tactics.at(0)->mutableRobotCapabilityRequirements() = {
            RobotCapability::Kick, RobotCapability::Chip};

    // set each tactic to its movement location.
    for (unsigned i = 0; i < kickoff_setup_positions.size(); i++)
    {
        move_tactics.at(i)->updateControlParams(kickoff_setup_positions.at(i),
                                                Angle::zero());
        result[0].emplace_back(move_tactics.at(i));
    }

    PriorityTacticVector tactics_to_return = {{}};

    event.common.set_tactics(tactics_to_return);
}

void KickoffFriendlyPlayFSM::kickoff(const Update& event)
{

    auto enemy_threats =
            getAllEnemyThreats(world_ptr->field(), world_ptr->friendlyTeam(),
                               world_ptr->enemyTeam(), world_ptr->ball(), false);

    PriorityTacticVector tactics_to_return = {{}};

    // TODO (#2612): This needs to be adjusted post field testing, ball needs to land
    // exactly in the middle of the enemy field
    kickoff_chip_tactic->updateControlParams(
            world_ptr->ball().position(),
            world_ptr->field().centerPoint() +
            Vector(world_ptr->field().xLength() / 6, 0));

    tactics_to_return[0].emplace_back(kickoff_chip_tactic);

    // the robot at position 0 will be closest to the ball, so positions starting from
    // 1 will be assigned to the rest of the robots
    for (unsigned i = 1; i < kickoff_setup_positions.size(); i++)
    {
        move_tactics.at(i)->updateControlParams(kickoff_setup_positions.at(i),
                                                Angle::zero());
        tactics_to_return[0].emplace_back(move_tactics.at(i));
    }
    event.common.set_tactics(tactics_to_return);
}

bool KickoffFriendlyPlayFSM::started(const Update &event)
{
    return world_ptr->gameState().isPlaying();
}
