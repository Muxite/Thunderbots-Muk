#include "software/ai/stp/tactic/kick_or_chip/kick_or_chip_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

class KickOrChipFSMTest : public ::testing::TestWithParam<AutoChipOrKick>
{
};

TEST_P(KickOrChipFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot robot                  = ::TestUtil::createRobotAtPos(Point(-2, -3));
    KickOrChipFSM::ControlParams control_params{
        .kick_or_chip_origin    = Point(-2, 1.5),
        .kick_or_chip_direction = Angle::threeQuarter(),
        .auto_chip_or_kick      = GetParam()};

    FSM<KickOrChipFSM> fsm{KickOrChipFSM(std::make_shared<TbotsProto::AiConfig>()),
                           GetBehindBallFSM(std::make_shared<TbotsProto::AiConfig>())};

    // Start in GetBehindBallFSM state's GetBehindBallState
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::GetBehindBallState>));

    // Transition to GetBehindBallFSM state's GetBehindBallState
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::GetBehindBallState>));

    // Robot is now behind ball
    robot = Robot(0,
                  RobotState(Point(-2, 1.7), Vector(), Angle::threeQuarter(),
                             AngularVelocity::zero()),
                  Timestamp::fromSeconds(123));
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    // Transition to KickOrChipState
    EXPECT_TRUE(fsm.is(boost::sml::state<KickOrChipFSM::KickOrChipState>));

    // Change the kick or chip direction and expect the FSM to realign
    control_params.kick_or_chip_direction = Angle::quarter();
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::GetBehindBallState>));

    // Robot is now behind ball in the new direction
    robot = Robot(
        0,
        RobotState(Point(-2, 1.3), Vector(), Angle::quarter(), AngularVelocity::zero()),
        Timestamp::fromSeconds(124));
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    // Transition to KickOrChipState again
    EXPECT_TRUE(fsm.is(boost::sml::state<KickOrChipFSM::KickOrChipState>));

    // Ball is now kicked or chipped
    ::TestUtil::setBallVelocity(world, Vector(0, 2.1), Timestamp::fromSeconds(123));

    EXPECT_TRUE(world->ball().hasBallBeenKicked(Angle::quarter()));

    // Tactic is done
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}

TEST_P(KickOrChipFSMTest, minimum_kicked_ball_speed_is_configurable)
{
    auto world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-0.2, 0));
    KickOrChipFSM::ControlParams control_params{
        .kick_or_chip_origin    = Point(0, 0),
        .kick_or_chip_direction = Angle::zero(),
        .auto_chip_or_kick      = GetParam()};
    KickOrChipFSM guards(std::make_shared<TbotsProto::AiConfig>());
    auto ball_chicked = [&]()
    {
        return guards.ballChicked(KickOrChipFSM::Update(
            control_params,
            TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    };

    ::TestUtil::setBallVelocity(world, Vector(0.49, 0), Timestamp::fromSeconds(1));
    EXPECT_FALSE(ball_chicked());
    ::TestUtil::setBallVelocity(world, Vector(0.5, 0), Timestamp::fromSeconds(2));
    EXPECT_TRUE(ball_chicked());

    control_params.min_kicked_ball_speed_meters_per_second = 3.0;
    ::TestUtil::setBallVelocity(world, Vector(2.9, 0), Timestamp::fromSeconds(3));
    EXPECT_FALSE(ball_chicked());
    ::TestUtil::setBallVelocity(world, Vector(3, 0), Timestamp::fromSeconds(4));
    EXPECT_TRUE(ball_chicked());
    ::TestUtil::setBallVelocity(world, Vector(-5, 0), Timestamp::fromSeconds(5));
    EXPECT_FALSE(ball_chicked());
}

TEST_P(KickOrChipFSMTest, realignment_takes_priority_over_completion)
{
    auto world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp());
    Robot robot = ::TestUtil::createRobotAtPos(Point(-0.2, 0));
    KickOrChipFSM::ControlParams control_params{
        .kick_or_chip_origin    = Point(0, 0),
        .kick_or_chip_direction = Angle::zero(),
        .auto_chip_or_kick      = GetParam()};
    auto config = std::make_shared<TbotsProto::AiConfig>();
    FSM<KickOrChipFSM> fsm{KickOrChipFSM(config), GetBehindBallFSM(config)};
    auto update = [&]()
    {
        fsm.process_event(KickOrChipFSM::Update(
            control_params,
            TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    };
    update();
    ASSERT_TRUE(fsm.is(boost::sml::state<KickOrChipFSM::KickOrChipState>));

    // Both guards are true: the robot should realign rather than report success.
    control_params.kick_or_chip_direction = Angle::quarter();
    ::TestUtil::setBallVelocity(world, Vector(0, 5), Timestamp::fromSeconds(1));
    update();
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
}

INSTANTIATE_TEST_CASE_P(
    AutoChipOrKickModes, KickOrChipFSMTest,
    ::testing::Values(AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, 1.2},
                      AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, 1.2}));
