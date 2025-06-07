#include "software/ai/hl/stp/play/kickoff_friendly/kickoff_friendly_play_fsm.h"

KickoffFriendlyPlayFSM::KickoffFriendlyPlayFSM(TbotsProto::AiConfig& ai_config)
    : ai_config(ai_config),
        move_tactic(std::make_shared<MoveTactic>()),
        prepare_kickoff_move_tactic(std::make_shared<PrepareKickoffMoveTactic>()),
        kickoff_chip_tactic(std::makeshared<KickoffChipTactic>())
{
}

void KickoffFriendlyPlayFSM::setupKickoff(const KickoffFriendlyPlayFSM::Update &event)
{
    // Since we only have 6 robots at the maximum, the number one priority
    // is the robot doing the kickoff up front. The goalie is the second most
    // important, followed by 3 and 4 setup for offense. 5 and 6 will stay
    // back near the goalie just in case the ball quickly returns to the friendly
    // side of the field.
    //
    // 		+--------------------+--------------------+
    // 		|                    |                    |
    // 		|               3    |                    |
    // 		|                    |                    |
    // 		+--+ 5               |                 +--+
    // 		|  |                 |                 |  |
    // 		|  |               +-+-+               |  |
    // 		|2 |               |1  |               |  |
    // 		|  |               +-+-+               |  |
    // 		|  |                 |                 |  |
    // 		+--+ 6               |                 +--+
    // 		|                    |                    |
    // 		|               4    |                    |
    // 		|                    |                    |
    // 		+--------------------+--------------------+
    //
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

    PriorityTacticVector tactics_to_run = {{}};

    move_tactics = {
        prepare_kickoff_move_tactic(), move_tactic(),
        move_tactic(), move_tactic(),
        move_tactic()};

    auto enemy_threats =
            getAllEnemyThreats(world_ptr->field(), world_ptr->friendlyTeam(),
                               world_ptr->enemyTeam(), world_ptr->ball(), false);

    // first priority requires the ability to kick and chip.
    move_tactics.at(0)->mutableRobotCapabilityRequirements() = {
            RobotCapability::Kick, RobotCapability::Chip};

    // set each tactic to its movement location.
    for (unsigned i = 0; i < kickoff_setup_positions.size(); i++)
    {
        move_tactics.at(i)->updateControlParams(kickoff_setup_positions.at(i),
                                                Angle::zero());
        tactics_to_run[0].emplace_back(move_tactics.at(i));
    }

    event.common.set_tactics(tactics_to_run);
}

void KickoffFriendlyPlayFSM::kickoff(const KickoffFriendlyPlayFSM::Update &event)
{

    auto enemy_threats =
            getAllEnemyThreats(world_ptr->field(), world_ptr->friendlyTeam(),
                               world_ptr->enemyTeam(), world_ptr->ball(), false);

    PriorityTacticVector tactics_to_run = {{}};

    // TODO (#2612): This needs to be adjusted post field testing, ball needs to land
    // exactly in the middle of the enemy field
    kickoff_chip_tactic->updateControlParams(
            world_ptr->ball().position(),
            world_ptr->field().centerPoint() +
            Vector(world_ptr->field().xLength() / 6, 0));

    tactics_to_run[0].emplace_back(kickoff_chip_tactic);

    // the robot at position 0 will be closest to the ball, so positions starting from
    // 1 will be assigned to the rest of the robots
    for (unsigned i = 1; i < kickoff_setup_positions.size(); i++)
    {
        move_tactics.at(i)->updateControlParams(kickoff_setup_positions.at(i),
                                                Angle::zero());
        tactics_to_run[0].emplace_back(move_tactics.at(i));
    }
    event.common.set_tactics(tactics_to_run);
}

bool KickoffFriendlyPlayFSM::canKick(const KickoffFriendlyPlayFSM::Update &event)
{
    return world_ptr->gameState().canKick();
}
