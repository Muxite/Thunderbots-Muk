#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play.h"
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
