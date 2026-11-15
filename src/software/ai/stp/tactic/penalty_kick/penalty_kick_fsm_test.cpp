#include "software/ai/stp/tactic/penalty_kick/penalty_kick_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(PenaltyKickFSM, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().friendlyPenaltyMark(),
                                Timestamp::fromSeconds(0));
    Robot robot = ::TestUtil::createRobotAtPos(world->field().friendlyPenaltyMark());

    FSM<PenaltyKickFSM> fsm{PenaltyKickFSM(std::make_shared<TbotsProto::AiConfig>()),
                            DribbleFSM(std::make_shared<TbotsProto::AiConfig>()),
                            KickOrChipFSM(std::make_shared<TbotsProto::AiConfig>()),
                            GetBehindBallFSM(std::make_shared<TbotsProto::AiConfig>())};

    PenaltyKickFSM::ControlParams control_params{};

    fsm.process_event(PenaltyKickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    // Shoot on the first update, without approaching the keeper or waiting.
    EXPECT_TRUE(fsm.is(boost::sml::state<KickOrChipFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<KickOrChipFSM>)>(
        boost::sml::state<GetBehindBallFSM>));

    robot = ::TestUtil::createRobotAtPos(world->ball().position() - Vector(0.2, 0));
    fsm.process_event(PenaltyKickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<KickOrChipFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<KickOrChipFSM>)>(
        boost::sml::state<KickOrChipFSM::KickOrChipState>));

    // A ball pushed forward slowly must not finish the penalty.
    ::TestUtil::setBallVelocity(world, Vector(1, 0), Timestamp::fromSeconds(1));
    fsm.process_event(PenaltyKickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<KickOrChipFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<KickOrChipFSM>)>(
        boost::sml::state<KickOrChipFSM::KickOrChipState>));

    ::TestUtil::setBallVelocity(world, Vector(5, 0), Timestamp::fromSeconds(2));
    fsm.process_event(PenaltyKickFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));

    EXPECT_TRUE(fsm.is(boost::sml::X));
}

TEST(PenaltyKickFSM, shot_direction_is_committed_when_goalie_moves)
{
    auto world      = ::TestUtil::createBlankTestingWorld();
    const auto ball = world->field().friendlyPenaltyMark();
    ::TestUtil::setBallPosition(world, ball, Timestamp());
    auto set_goalie = [&](double y)
    {
        Team enemy({Robot(0, Point(world->field().enemyGoalCenter().x(), y), Vector(),
                          Angle::half(), AngularVelocity::zero(), Timestamp())});
        enemy.assignGoalie(0);
        world->updateEnemyTeamState(enemy);
    };
    set_goalie(0.2);
    const Angle direction =
        (PenaltyKickFSM::evaluateNextShotPosition(world->enemyTeam().goalie(),
                                                world->field()) -
         ball)
            .orientation();
    Robot robot(0, ball - Vector::createFromAngle(direction).normalize(0.2), Vector(),
                direction, AngularVelocity::zero(), Timestamp());
    auto config = std::make_shared<TbotsProto::AiConfig>();
    FSM<PenaltyKickFSM> fsm{PenaltyKickFSM(config), DribbleFSM(config),
                            KickOrChipFSM(config), GetBehindBallFSM(config)};
    auto update = [&]()
    {
        fsm.process_event(PenaltyKickFSM::Update(
            {}, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    };
    update();
    update();
    ASSERT_TRUE(fsm.is<decltype(boost::sml::state<KickOrChipFSM>)>(
        boost::sml::state<KickOrChipFSM::KickOrChipState>));

    set_goalie(-0.2);
    ::TestUtil::setBallVelocity(world, Vector::createFromAngle(direction).normalize(5),
                               Timestamp::fromSeconds(1));
    update();
    EXPECT_TRUE(fsm.is(boost::sml::X));
}

TEST(PenaltyKickFSMTest, no_enemy_goalie)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, Point(4, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Vector behind_ball_direction =
        -(world->ball().position() - world->field().enemyGoalCenter()).normalize();
    Point behind_ball =
        Point(4, 0) - behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                      BALL_MAX_RADIUS_METERS);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));

    EXPECT_TRUE(PenaltyKickFSM::evaluatePenaltyShot(std::nullopt, world->field(),
                                                    world->ball().position(), shooter));
}

TEST(PenaltyKickFSMTest, enemy_goalie_offset_left_no_viable_shot)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().friendlyPenaltyMark(),
                                Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), 0.2);
    Vector behind_ball_direction =
        -(world->ball().position() - world->field().enemyGoalpostNeg()).normalize();
    Point behind_ball = world->ball().position() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world->updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world->updateFriendlyTeamState(friendly);

    EXPECT_FALSE(PenaltyKickFSM::evaluatePenaltyShot(std::optional<Robot>(enemy_goalie),
                                                     world->field(),
                                                     world->ball().position(), shooter));
}

TEST(PenaltyKickFSMTest, enemy_goalie_offset_right_no_viable_shot)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().enemyGoalCenter() + Vector(-3, 0),
                                Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), -0.2);
    Vector behind_ball_direction =
        (world->ball().position() - enemy_goalie_pos).normalize();
    Point behind_ball = world->ball().position() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world->updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world->updateFriendlyTeamState(friendly);

    EXPECT_FALSE(PenaltyKickFSM::evaluatePenaltyShot(std::optional<Robot>(enemy_goalie),
                                                     world->field(),
                                                     world->ball().position(), shooter));
}

TEST(PenaltyKickFSMTest, enemy_goalie_right_viable_shot_left)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, Point(4, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), +0.2);
    Vector behind_ball_direction =
        -(world->ball().position() - world->field().enemyGoalpostNeg()).normalize();
    Point behind_ball = world->ball().position() -
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world->updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world->updateFriendlyTeamState(friendly);

    EXPECT_TRUE(PenaltyKickFSM::evaluatePenaltyShot(std::optional<Robot>(enemy_goalie),
                                                    world->field(),
                                                    world->ball().position(), shooter));
}

TEST(PenaltyKickFSMTest, enemy_goalie_left_viable_shot_right)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, Point(4, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), -0.2);
    Vector behind_ball_direction =
        -(world->ball().position() - world->field().enemyGoalpostPos()).normalize();
    Point behind_ball = world->ball().position() -
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS);

    Robot enemy_goalie = Robot(0, enemy_goalie_pos, Vector(0, +0.2), Angle::half(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team enemy({enemy_goalie});
    enemy.assignGoalie(0);
    world->updateEnemyTeamState(enemy);

    Robot shooter =
        Robot(0, behind_ball, Vector(0, 0), behind_ball_direction.orientation(),
              AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly({shooter});
    world->updateFriendlyTeamState(friendly);

    EXPECT_TRUE(PenaltyKickFSM::evaluatePenaltyShot(std::optional<Robot>(enemy_goalie),
                                                    world->field(),
                                                    world->ball().position(), shooter));
}

TEST(PenaltyKickFSMTest, no_enemy_goalie_shot_position)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().friendlyPenaltyMark(),
                                Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    EXPECT_EQ(PenaltyKickFSM::evaluateNextShotPosition(std::nullopt, world->field()),
              world->field().enemyGoalCenter());
}

TEST(PenaltyKickFSMTest, enemy_goalie_left_shot_right)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().friendlyPenaltyMark(),
                                Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), 0.2);
    Robot enemy_goalie     = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Point shot_position = PenaltyKickFSM::evaluateNextShotPosition(
        std::optional<Robot>(enemy_goalie), world->field());
    EXPECT_LE(shot_position.y(), 0);
    EXPECT_EQ(shot_position.x(), world->field().enemyGoalCenter().x());
}

TEST(PenaltyKickFSMTest, enemy_goalie_right_shot_left)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    ::TestUtil::setBallPosition(world, world->field().friendlyPenaltyMark(),
                                Timestamp::fromSeconds(0));
    ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(0));

    Point enemy_goalie_pos = Point(world->field().enemyGoalCenter().x(), -0.2);
    Robot enemy_goalie     = Robot(0, enemy_goalie_pos, Vector(0, 0), Angle::half(),
                                   AngularVelocity::zero(), Timestamp::fromSeconds(0));

    Point shot_position = PenaltyKickFSM::evaluateNextShotPosition(
        std::optional<Robot>(enemy_goalie), world->field());
    EXPECT_GE(shot_position.y(), 0);
    EXPECT_EQ(shot_position.x(), world->field().enemyGoalCenter().x());
}
