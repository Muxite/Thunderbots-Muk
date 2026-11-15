#include "software/ai/stp/play/penalty_kick_enemy/penalty_kick_enemy_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(PenaltyKickEnemyPlayFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::HALT);
    world->updateGameState(game_state);
    game_state.updateRefereeCommand(RefereeCommand::STOP);
    world->updateGameState(game_state);
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
    world->updateGameState(game_state);

    int num_tactics = 5;
    std::shared_ptr<GoalieTactic> goalie_tactic =
        std::make_shared<GoalieTactic>(std::make_shared<TbotsProto::AiConfig>());

    FSM<PenaltyKickEnemyPlayFSM> fsm(
        PenaltyKickEnemyPlayFSM{std::make_shared<TbotsProto::AiConfig>()});

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickEnemyPlayFSM::SetupPositionState>));

    fsm.process_event(PenaltyKickEnemyPlayFSM::Update(
        PenaltyKickEnemyPlayFSM::ControlParams{goalie_tactic},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickEnemyPlayFSM::SetupPositionState>));

    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world->updateGameState(game_state);

    fsm.process_event(PenaltyKickEnemyPlayFSM::Update(
        PenaltyKickEnemyPlayFSM::ControlParams{goalie_tactic},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickEnemyPlayFSM::DefendKickState>));

    fsm.process_event(PenaltyKickEnemyPlayFSM::Update(
        PenaltyKickEnemyPlayFSM::ControlParams{goalie_tactic},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickEnemyPlayFSM::DefendKickState>));
}

TEST(PenaltyKickEnemyPlayFSMTest, other_robots_keep_their_positions_during_the_kick)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    GameState game_state;
    game_state.updateRefereeCommand(RefereeCommand::PREPARE_PENALTY_THEM);
    world->updateGameState(game_state);
    std::shared_ptr<GoalieTactic> goalie_tactic =
        std::make_shared<GoalieTactic>(std::make_shared<TbotsProto::AiConfig>());
    FSM<PenaltyKickEnemyPlayFSM> fsm(
        PenaltyKickEnemyPlayFSM{std::make_shared<TbotsProto::AiConfig>()});

    size_t published = 0;
    auto update      = [&]()
    {
        fsm.process_event(PenaltyKickEnemyPlayFSM::Update(
            PenaltyKickEnemyPlayFSM::ControlParams{goalie_tactic},
            PlayUpdate(
                world, 5,
                [&](PriorityTacticVector tactics)
                {
                    published = 0;
                    for (const auto& priority : tactics)
                    {
                        published += priority.size();
                    }
                },
                InterPlayCommunication{}, [](InterPlayCommunication comm) {})));
    };
    update();
    game_state.updateRefereeCommand(RefereeCommand::NORMAL_START);
    world->updateGameState(game_state);
    published = 0;
    update();
    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickEnemyPlayFSM::DefendKickState>));
    EXPECT_EQ(published, 5u);
    published = 0;
    update();
    EXPECT_EQ(published, 5u);
}
