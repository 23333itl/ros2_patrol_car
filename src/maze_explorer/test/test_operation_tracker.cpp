#include <gtest/gtest.h>
#include "maze_explorer/operation_tracker.hpp"

using maze_explorer::OperationTracker;
using maze_explorer::Phase;
using maze_explorer::ProgressWatchdog;

TEST(GoalExclusions, NavigationFailureIsStillCoolingAfterRecoveryWait)
{
  maze_explorer::GoalExclusions exclusions;
  exclusions.remember(1.0, 2.0, 30.0, 0.15, false, 0);
  EXPECT_DOUBLE_EQ(exclusions.blockedUntil(1.0, 2.0, 12.0, 0), 30.0);
  EXPECT_DOUBLE_EQ(exclusions.blockedUntil(1.0, 2.0, 30.0, 0), 30.0);
  EXPECT_DOUBLE_EQ(exclusions.blockedUntil(1.0, 2.0, 32.0, 0), 32.0);
}

TEST(GoalExclusions, OverlapsWaitForAllAndProgressUnlocksShortGoal)
{
  maze_explorer::GoalExclusions exclusions;
  exclusions.remember(1.0, 2.0, 20.0, 0.15, false, 0);
  exclusions.remember(1.05, 2.0, 30.0, 0.15, false, 0);
  EXPECT_DOUBLE_EQ(exclusions.blockedUntil(1.0, 2.0, 10.0, 0), 30.0);
  exclusions.remember(1.0, 2.0, 15.0, 0.1, true, 0);
  EXPECT_TRUE(std::isinf(exclusions.blockedUntil(1.0, 2.0, 40.0, 0)));
  EXPECT_DOUBLE_EQ(exclusions.blockedUntil(1.0, 2.0, 40.0, 1), 40.0);
}

TEST(OperationTracker, CancelBeforeAcknowledgementKeepsExclusiveOwnership)
{
  OperationTracker tracker;
  const auto first = tracker.begin(Phase::Sending);
  ASSERT_TRUE(tracker.cancel(first));
  ASSERT_TRUE(tracker.acknowledge(first));
  EXPECT_EQ(tracker.phase(), Phase::Canceling);
  EXPECT_THROW(tracker.begin(Phase::BackingUp), std::logic_error);
  ASSERT_TRUE(tracker.terminal(first));
  EXPECT_NO_THROW(tracker.begin(Phase::BackingUp));
}

TEST(OperationTracker, LateResultCannotCompleteAnotherGoalOrRestartFinishedExplorer)
{
  OperationTracker tracker;
  const auto first = tracker.begin(Phase::Planning);
  ASSERT_TRUE(tracker.terminal(first));
  const auto second = tracker.begin(Phase::Sending);
  EXPECT_FALSE(tracker.terminal(first));
  EXPECT_TRUE(tracker.accepts(second));
  tracker.finish();
  EXPECT_FALSE(tracker.acknowledge(second));
  EXPECT_FALSE(tracker.terminal(second));
  EXPECT_EQ(tracker.phase(), Phase::Finished);
}

TEST(OperationTracker, WaitingResumesButCannotReleaseAnActiveCancellation)
{
  OperationTracker tracker;
  const auto first = tracker.begin(Phase::Sending);
  ASSERT_TRUE(tracker.cancel(first));
  EXPECT_FALSE(tracker.wait());
  EXPECT_FALSE(tracker.resume());
  EXPECT_EQ(tracker.phase(), Phase::Canceling);
  ASSERT_TRUE(tracker.terminal(first));
  ASSERT_TRUE(tracker.wait());
  EXPECT_EQ(tracker.phase(), Phase::Waiting);
  EXPECT_FALSE(tracker.accepts(first));
  EXPECT_FALSE(tracker.terminal(first));
  EXPECT_THROW(tracker.begin(Phase::BackingUp), std::logic_error);
  ASSERT_TRUE(tracker.resume());
  EXPECT_NO_THROW(tracker.begin(Phase::Planning));
  tracker.finish();
  EXPECT_FALSE(tracker.resume());
}

TEST(ProgressWatchdog, RotationAndPausedSimulationDoNotResetTranslationProgress)
{
  ProgressWatchdog watchdog;
  watchdog.start(5.0, 0.0, 0.0, 1.0);
  for (int i = 0; i < 100; ++i) {watchdog.observe(5.0, 0.0, 0.0);}
  EXPECT_FALSE(watchdog.noProgress(5.0));
  watchdog.observe(34.0, 0.09, 0.0);
  EXPECT_TRUE(watchdog.noProgress(35.0));
}

TEST(ProgressWatchdog, TotalLimitBoundsRepeatedBackAndForthMovement)
{
  ProgressWatchdog watchdog;
  watchdog.start(0.0, 0.0, 0.0, 1.0);
  for (int time = 10; time <= 120; time += 10) {
    watchdog.observe(time, (time / 10) % 2 ? 0.2 : 0.0, 0.0);
  }
  EXPECT_FALSE(watchdog.noProgress(120.0));
  EXPECT_TRUE(watchdog.totalExpired(120.0));
  watchdog.start(0.0, 0.0, 0.0, 12.0);
  EXPECT_FALSE(watchdog.totalExpired(329.0));
  EXPECT_TRUE(watchdog.totalExpired(330.0));
}

TEST(ExplorationProgress, ReturningToSameObservationPointDoesNotEarnMoreRecovery)
{
  maze_explorer::VisitedGoals visited;
  EXPECT_TRUE(visited.mark(1.0, 1.0));
  EXPECT_FALSE(visited.mark(1.0, 1.0));
  EXPECT_FALSE(visited.mark(1.04, 1.02));
  EXPECT_TRUE(visited.mark(1.4, 1.0));
  EXPECT_FALSE(visited.mark(1.0, 1.0));
}
