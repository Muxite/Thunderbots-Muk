#include "software/ai/stp/play/penalty_kick/penalty_kick_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/ai/stp/play/penalty_kick/penalty_kick_play.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(PenaltyKickPlayFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::HALT);
    world->updateGameState(game_state);
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    world->updateGameState(game_state);
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    world->updateGameState(game_state);

    int num_tactics = 5;

    FSM<PenaltyKickPlayFSM> fsm(
        PenaltyKickPlayFSM{std::make_shared<TbotsProto::AiConfig>()});

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickPlayFSM::SetupPositionState>));

    fsm.process_event(PenaltyKickPlayFSM::Update(
        PenaltyKickPlayFSM::ControlParams{},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickPlayFSM::SetupPositionState>));

    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world->updateGameState(game_state);

    fsm.process_event(PenaltyKickPlayFSM::Update(
        PenaltyKickPlayFSM::ControlParams{},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickPlayFSM::PerformKickState>));

    fsm.process_event(PenaltyKickPlayFSM::Update(
        PenaltyKickPlayFSM::ControlParams{},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickPlayFSM::PerformKickState>));
}

TEST(PenaltyKickPlayFSMTest, normal_start_immediately_publishes_one_kicker_and_supporters)
{
    auto world = TestUtil::createBlankTestingWorld();
    GameState state;
    state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    world->updateGameState(state);
    FSM<PenaltyKickPlayFSM> fsm(
        PenaltyKickPlayFSM{std::make_shared<TbotsProto::AiConfig>()});
    PriorityTacticVector published;
    bool called = false;
    PenaltyKickPlayFSM::Update event(
        {}, PlayUpdate(
                world, 5,
                [&](PriorityTacticVector tactics)
                {
                    published = tactics;
                    called    = true;
                },
                InterPlayCommunication{}, [](InterPlayCommunication) {}));
    fsm.process_event(event);
    state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world->updateGameState(state);
    called = false;
    fsm.process_event(event);
    EXPECT_TRUE(called);
    for (int tick = 0; tick < 3; ++tick)
    {
        fsm.process_event(event);
        size_t kickers = 0, supporters = 0;
        for (const auto& priority : published)
            for (const auto& tactic : priority)
            {
                if (std::dynamic_pointer_cast<PenaltyKickTactic>(tactic))
                    ++kickers;
                if (std::dynamic_pointer_cast<PenaltySetupTactic>(tactic))
                    ++supporters;
            }
        EXPECT_EQ(kickers, 1);
        EXPECT_EQ(supporters, 4);
    }
}

TEST(PenaltyKickPlayAssignmentTest,
     replaced_setup_shooter_retreats_instead_of_halting_near_ball)
{
    auto world      = TestUtil::createBlankTestingWorld();
    const auto ball = world->field().friendlyPenaltyMark();
    TestUtil::setBallPosition(world, ball, Timestamp());
    auto set_team = [&](bool disable_first_shooter)
    {
        Team team(Duration::fromSeconds(1));
        team.updateRobots({Robot(0, Point(-4, 0), Vector(), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp()),
                           Robot(1, ball + Vector(-0.21, 0), Vector(), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp(), false,
                                 disable_first_shooter
                                     ? std::set<RobotCapability>{RobotCapability::Dribble}
                                     : std::set<RobotCapability>{}),
                           Robot(2, ball + Vector(-1, 1), Vector(), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp())});
        team.assignGoalie(0);
        auto updated = std::make_shared<World>(world->field(), world->ball(), team,
                                               world->enemyTeam());
        updated->updateGameState(world->gameState());
        world = updated;
    };
    set_team(false);
    GameState state;
    state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    world->updateGameState(state);
    PenaltyKickPlay play(std::make_shared<TbotsProto::AiConfig>());
    auto tick = [&]()
    { return play.get(world, InterPlayCommunication{}, [](InterPlayCommunication) {}); };
    tick();
    // Simulate a capability change requiring the other robot to take the kick.
    set_team(true);
    state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world->updateGameState(state);
    tick();
    auto primitives      = tick();
    unsigned int kickers = 0;
    for (const auto& [tactic, id] : play.getTacticRobotIdAssignment())
    {
        if (std::dynamic_pointer_cast<const PenaltyKickTactic>(tactic))
        {
            ++kickers;
            EXPECT_EQ(id, 2);
        }
        if (id == 1)
        {
            EXPECT_NE(std::dynamic_pointer_cast<const PenaltySetupTactic>(tactic),
                      nullptr);
        }
    }
    EXPECT_EQ(kickers, 1);
    ASSERT_TRUE(primitives->robot_primitives().contains(1));
    EXPECT_FALSE(primitives->robot_primitives().at(1).has_stop());
}

TEST(PenaltyKickPlayAssignmentTest, kicker_is_the_robot_parked_behind_the_ball)
{
    auto world      = TestUtil::createBlankTestingWorld();
    const auto ball = world->field().friendlyPenaltyMark();
    TestUtil::setBallPosition(world, ball, Timestamp());
    Team team(Duration::fromSeconds(1));
    team.updateRobots({Robot(0, Point(-4, 0), Vector(), Angle::zero(),
                             AngularVelocity::zero(), Timestamp()),
                       Robot(1, ball + Vector(-0.21, 0), Vector(), Angle::zero(),
                             AngularVelocity::zero(), Timestamp()),
                       Robot(2, ball + Vector(-1.3, 0.4), Vector(), Angle::zero(),
                             AngularVelocity::zero(), Timestamp()),
                       Robot(3, ball + Vector(-1.3, -0.4), Vector(), Angle::zero(),
                             AngularVelocity::zero(), Timestamp())});
    team.assignGoalie(0);
    world =
        std::make_shared<World>(world->field(), world->ball(), team, world->enemyTeam());
    GameState state;
    state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    world->updateGameState(state);
    PenaltyKickPlay play(std::make_shared<TbotsProto::AiConfig>());
    play.get(world, InterPlayCommunication{}, [](InterPlayCommunication) {});
    state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world->updateGameState(state);
    auto primitives =
        play.get(world, InterPlayCommunication{}, [](InterPlayCommunication) {});
    for (const auto& [tactic, id] : play.getTacticRobotIdAssignment())
    {
        if (std::dynamic_pointer_cast<const PenaltyKickTactic>(tactic))
        {
            EXPECT_EQ(id, 1);
        }
    }
    // No field robot may receive a stop primitive on the NORMAL_START tick.
    for (RobotId id : {1, 2, 3})
    {
        ASSERT_TRUE(primitives->robot_primitives().contains(id));
        EXPECT_FALSE(primitives->robot_primitives().at(id).has_stop());
    }
}

TEST(PenaltyKickPlayFSMTest, setup_shooter_is_prioritized_and_can_execute_the_kick)
{
    auto world = TestUtil::createBlankTestingWorld();
    PenaltyKickPlayFSM play(std::make_shared<TbotsProto::AiConfig>());
    PriorityTacticVector published;
    play.setupPosition(PenaltyKickPlayFSM::Update(
        {}, PlayUpdate(
                world, 5, [&](PriorityTacticVector tactics) { published = tactics; },
                InterPlayCommunication{}, [](InterPlayCommunication) {})));
    ASSERT_FALSE(published.empty());
    ASSERT_EQ(published.front().size(), 1);
    const auto& capabilities = published.front().front()->robotCapabilityRequirements();
    EXPECT_TRUE(capabilities.contains(RobotCapability::Move));
    EXPECT_TRUE(capabilities.contains(RobotCapability::Dribble));
    EXPECT_TRUE(capabilities.contains(RobotCapability::Kick));
}

TEST(PenaltyKickPlayFSMTest,
     changing_robot_budgets_preserves_one_shooter_without_underflow)
{
    auto world = TestUtil::createBlankTestingWorld();
    PenaltyKickPlayFSM play(std::make_shared<TbotsProto::AiConfig>());
    for (unsigned int budget : {0u, 1u, 5u, 2u, 0u, 6u})
        for (bool kicking : {false, true})
        {
            PriorityTacticVector published;
            bool called = false;
            PenaltyKickPlayFSM::Update event(
                {}, PlayUpdate(
                        world, budget,
                        [&](PriorityTacticVector tactics)
                        {
                            published = tactics;
                            called    = true;
                        },
                        InterPlayCommunication{}, [](InterPlayCommunication) {}));
            if (kicking)
                play.performKick(event);
            else
                play.setupPosition(event);
            ASSERT_TRUE(called);
            size_t count = 0;
            for (const auto& priority : published)
                count += priority.size();
            EXPECT_EQ(count, budget);
            if (budget > 0)
            {
                ASSERT_EQ(published.front().size(), 1);
                EXPECT_EQ(published.back().size(), budget - 1);
            }
        }
}

namespace
{
// Builds a world with the given robots (index 0 is the goalie) and the ball on the
// friendly penalty mark
std::shared_ptr<World> makePenaltyWorld(
    const std::vector<std::pair<RobotId, Point>>& robot_positions,
    const GameState& game_state, RobotId goalie_id = 0)
{
    auto base       = TestUtil::createBlankTestingWorld();
    const auto ball = base->field().friendlyPenaltyMark();
    TestUtil::setBallPosition(base, ball, Timestamp());
    std::vector<Robot> robots;
    for (const auto& [id, position] : robot_positions)
    {
        robots.emplace_back(id, position, Vector(), Angle::zero(),
                            AngularVelocity::zero(), Timestamp());
    }
    Team team(Duration::fromSeconds(1));
    team.updateRobots(robots);
    team.assignGoalie(goalie_id);
    auto world =
        std::make_shared<World>(base->field(), base->ball(), team, base->enemyTeam());
    world->updateGameState(game_state);
    return world;
}

std::optional<RobotId> kickerId(const PenaltyKickPlay& play)
{
    std::optional<RobotId> kicker;
    unsigned int count = 0;
    for (const auto& [tactic, id] : play.getTacticRobotIdAssignment())
    {
        if (std::dynamic_pointer_cast<const PenaltyKickTactic>(tactic))
        {
            kicker = id;
            ++count;
        }
    }
    EXPECT_LE(count, 1u);
    return kicker;
}
}  // namespace

TEST(PenaltyKickPlayAssignmentTest, kicker_is_not_handed_over_to_a_cheaper_robot)
{
    GameState state;
    state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    const auto ball = TestUtil::createBlankTestingWorld()->field().friendlyPenaltyMark();
    auto positions =
        std::vector<std::pair<RobotId, Point>>{{0, Point(-4, 0)},
                                               {1, ball + Vector(-0.21, 0)},
                                               {2, ball + Vector(-1.3, 0.4)},
                                               {3, ball + Vector(-1.3, -0.4)}};
    PenaltyKickPlay play(std::make_shared<TbotsProto::AiConfig>());
    auto tick = [&]()
    {
        auto world = makePenaltyWorld(positions, state);
        play.get(world, InterPlayCommunication{}, [](InterPlayCommunication) {});
    };
    tick();
    state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    tick();
    ASSERT_EQ(kickerId(play), std::optional<RobotId>(1));

    // The kicker falls behind while another robot gets close to the ball: the kick
    // must not be handed over
    positions[1].second = ball + Vector(-2.0, 0.0);
    positions[2].second = ball + Vector(-0.3, 0.0);
    for (int i = 0; i < 5; ++i)
    {
        tick();
        EXPECT_EQ(kickerId(play), std::optional<RobotId>(1));
    }
}

TEST(PenaltyKickPlayAssignmentTest, kick_is_reassigned_once_if_the_kicker_disappears)
{
    GameState state;
    state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    const auto ball = TestUtil::createBlankTestingWorld()->field().friendlyPenaltyMark();
    auto positions =
        std::vector<std::pair<RobotId, Point>>{{0, Point(-4, 0)},
                                               {1, ball + Vector(-0.21, 0)},
                                               {2, ball + Vector(-1.3, 0.4)},
                                               {3, ball + Vector(-1.3, -0.4)}};
    PenaltyKickPlay play(std::make_shared<TbotsProto::AiConfig>());
    auto tick = [&]()
    {
        auto world = makePenaltyWorld(positions, state);
        return play.get(world, InterPlayCommunication{}, [](InterPlayCommunication) {});
    };
    tick();
    state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    tick();
    ASSERT_EQ(kickerId(play), std::optional<RobotId>(1));

    // The kicker is removed from the field mid-kick
    positions.erase(positions.begin() + 1);
    auto primitives = tick();
    auto new_kicker = kickerId(play);
    ASSERT_TRUE(new_kicker.has_value());
    EXPECT_NE(new_kicker.value(), 0u);  // never the goalie
    EXPECT_NE(new_kicker.value(), 1u);
    for (int i = 0; i < 5; ++i)
    {
        primitives = tick();
        EXPECT_EQ(kickerId(play), new_kicker);
    }
    // Everyone but the goalie keeps moving rather than stopping
    for (RobotId id : {2, 3})
    {
        ASSERT_TRUE(primitives->robot_primitives().contains(id));
        EXPECT_FALSE(primitives->robot_primitives().at(id).has_stop());
    }
}

TEST(PenaltyKickPlayAssignmentTest, goalie_next_to_the_ball_never_takes_the_kick)
{
    GameState state;
    state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    const auto ball = TestUtil::createBlankTestingWorld()->field().friendlyPenaltyMark();
    const std::vector<std::pair<RobotId, Point>> positions = {
        {0, ball + Vector(-0.2, 0)},
        {1, ball + Vector(-1.3, 0.4)},
        {2, ball + Vector(-1.3, -0.4)}};
    PenaltyKickPlay play(std::make_shared<TbotsProto::AiConfig>());
    for (int i = 0; i < 4; ++i)
    {
        if (i == 2)
        {
            state.updateRefereeCommand(RefereeCommand::NORMAL_START);
        }
        auto world = makePenaltyWorld(positions, state);
        play.get(world, InterPlayCommunication{}, [](InterPlayCommunication) {});
        for (const auto& [tactic, id] : play.getTacticRobotIdAssignment())
        {
            if (std::dynamic_pointer_cast<const PenaltyKickTactic>(tactic))
            {
                EXPECT_NE(id, 0u);
            }
        }
    }
}

TEST(PenaltyKickPlayFSMTest, referee_sequence_finishes_the_play_on_stop)
{
    for (auto end_command : {RefereeCommand::STOP, RefereeCommand::HALT})
    {
        auto world = TestUtil::createBlankTestingWorld();
        GameState state;
        state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
        world->updateGameState(state);
        FSM<PenaltyKickPlayFSM> fsm(
            PenaltyKickPlayFSM{std::make_shared<TbotsProto::AiConfig>()});
        auto update = [&]()
        {
            fsm.process_event(PenaltyKickPlayFSM::Update(
                {}, PlayUpdate(
                        world, 5, [](PriorityTacticVector) {}, InterPlayCommunication{},
                        [](InterPlayCommunication) {})));
        };
        update();
        EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickPlayFSM::SetupPositionState>));
        state.updateRefereeCommand(RefereeCommand::NORMAL_START);
        world->updateGameState(state);
        update();
        EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickPlayFSM::PerformKickState>));
        state.updateRefereeCommand(end_command);
        world->updateGameState(state);
        update();
        // HALT replaces the play through play selection, so only STOP ends this FSM
        if (end_command == RefereeCommand::STOP)
        {
            EXPECT_TRUE(fsm.is(boost::sml::X));
            update();
            EXPECT_TRUE(fsm.is(boost::sml::X));
        }
    }
}

TEST(PenaltyKickPlayAssignmentTest,
     play_rebuilt_in_ready_state_gives_the_kick_to_the_robot_behind_the_ball)
{
    // An AI config change resets the play selection, so a brand new PenaltyKickPlay can
    // be created after NORMAL_START with no memory of who was parked behind the ball.
    // It must still pick exactly one kicker, and it must be the robot behind the ball
    // rather than another robot that merely happens to be close.
    GameState state;
    state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_US);
    state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    const auto ball = TestUtil::createBlankTestingWorld()->field().friendlyPenaltyMark();
    auto positions =
        std::vector<std::pair<RobotId, Point>>{{0, Point(-4, 0)},
                                               {1, ball + Vector(-0.21, 0)},
                                               {2, ball + Vector(-0.45, 0.3)},
                                               {3, ball + Vector(-1.3, -0.4)}};
    PenaltyKickPlay rebuilt_play(std::make_shared<TbotsProto::AiConfig>());
    auto world = makePenaltyWorld(positions, state);
    auto primitives =
        rebuilt_play.get(world, InterPlayCommunication{}, [](InterPlayCommunication) {});
    EXPECT_EQ(kickerId(rebuilt_play), std::optional<RobotId>(1));
    for (RobotId id : {1, 2, 3})
    {
        ASSERT_TRUE(primitives->robot_primitives().contains(id));
        EXPECT_FALSE(primitives->robot_primitives().at(id).has_stop());
    }
}
