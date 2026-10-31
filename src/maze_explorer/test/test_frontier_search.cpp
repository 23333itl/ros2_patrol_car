#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <utility>

#include "gtest/gtest.h"
#include "maze_explorer/frontier_search.hpp"

namespace
{

using maze_explorer::findCandidates;
using maze_explorer::footprintFree;
using maze_explorer::goalFootprintFree;
using maze_explorer::knownFreeCells;
using maze_explorer::validMap;
using maze_explorer::worldToCell;

nav_msgs::msg::OccupancyGrid makeMap(
  unsigned int width, unsigned int height, float resolution, std::int8_t fill)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = width;
  map.info.height = height;
  map.info.resolution = resolution;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(static_cast<std::size_t>(width) * height, fill);
  return map;
}

void fillRectangle(
  nav_msgs::msg::OccupancyGrid & map, int min_x, int min_y,
  int max_x, int max_y, std::int8_t value)
{
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      map.data[static_cast<std::size_t>(y) * map.info.width + x] = value;
    }
  }
}

std::pair<double, double> cellCenter(
  const nav_msgs::msg::OccupancyGrid & map, int x, int y)
{
  const double yaw = maze_explorer::detail::originYaw(map);
  const double mx = (x + 0.5) * map.info.resolution;
  const double my = (y + 0.5) * map.info.resolution;
  return {
    map.info.origin.position.x + std::cos(yaw) * mx - std::sin(yaw) * my,
    map.info.origin.position.y + std::sin(yaw) * mx + std::cos(yaw) * my};
}

nav_msgs::msg::OccupancyGrid roomMap()
{
  auto map = makeMap(30, 30, 0.05F, -1);
  fillRectangle(map, 3, 3, 26, 26, 0);
  return map;
}

nav_msgs::msg::OccupancyGrid shortApproachMap()
{
  auto map = makeMap(30, 15, 0.05F, 100);
  fillRectangle(map, 3, 4, 14, 10, 0);
  fillRectangle(map, 15, 4, 26, 10, -1);
  return map;
}

TEST(FrontierSearch, KeepsFarStandpointsInSameClusterWhenNearestIsTooClose)
{
  const auto map = roomMap();
  const auto robot = cellCenter(map, 7, 15);
  const auto result = findCandidates(map, robot.first, robot.second, 0.14, {});

  EXPECT_TRUE(result.robot_in_known_free);
  EXPECT_GT(result.frontier_cells, 5U);
  ASSERT_GT(result.candidates.size(), 1U);
  std::set<std::pair<int, int>> distinct;
  double previous_distance = 0.0;
  for (const auto & candidate : result.candidates) {
    EXPECT_FALSE(candidate.short_goal);
    EXPECT_GE(candidate.path_distance + 1e-9, 0.35);
    EXPECT_GE(std::hypot(candidate.x - robot.first, candidate.y - robot.second) + 1e-9, 0.35);
    EXPECT_TRUE(footprintFree(map, candidate.x, candidate.y, 0.14));
    EXPECT_GE(candidate.path_distance, previous_distance);
    previous_distance = candidate.path_distance;
    int x = -1;
    int y = -1;
    ASSERT_TRUE(worldToCell(map, candidate.x, candidate.y, x, y));
    EXPECT_TRUE(distinct.emplace(x, y).second);
  }

  // Excluding near standpoints must not eliminate the entire connected frontier.
  const auto farther = findCandidates(map, robot.first, robot.second, 0.14,
    [&](double x, double y) {
      return std::hypot(x - robot.first, y - robot.second) < 0.70;
    });
  ASSERT_FALSE(farther.candidates.empty());
  for (const auto & candidate : farther.candidates) {
    EXPECT_GE(std::hypot(candidate.x - robot.first, candidate.y - robot.second), 0.70);
  }
}

TEST(FrontierSearch, SafeActualPoseCanConnectWhenItsRoundedCenterTouchesWall)
{
  auto map = makeMap(40, 20, 0.05F, 0);
  fillRectangle(map, 0, 0, 39, 3, 100);
  fillRectangle(map, 30, 4, 39, 19, -1);
  // Wall upper edge y=0.20; rounded center y=0.325 overlaps the 0.14 body,
  // while actual y=0.345 has full clearance and can move toward y=0.375.
  EXPECT_TRUE(footprintFree(map, .4, .345, .14));
  EXPECT_FALSE(footprintFree(map, .425, .325, .14));
  const auto result = findCandidates(map, .4, .345, .14, {});
  EXPECT_TRUE(result.robot_footprint_free);
  EXPECT_GT(result.reachable_cells, 0U);
  EXPECT_FALSE(result.candidates.empty());
}

TEST(FrontierSearch, FullPathCheckRejectsWallBetweenClearEndpoints)
{
  auto map = makeMap(40, 20, .05F, 0);
  fillRectangle(map, 19, 0, 20, 19, 100);
  EXPECT_TRUE(footprintFree(map, .5, .5, .14));
  EXPECT_TRUE(footprintFree(map, 1.5, .5, .14));
  EXPECT_FALSE(maze_explorer::segmentFootprintFree(map, .5, .5, 1.5, .5, .14));
  EXPECT_TRUE(maze_explorer::segmentFootprintFree(map, .4, .4, .6, .6, .14));
}

TEST(FrontierSearch, UsesShortApproachAfterRetreatFromOccludedFrontier)
{
  const auto map = shortApproachMap();
  const auto robot = cellCenter(map, 8, 7);
  const auto result = findCandidates(map, robot.first, robot.second, 0.14, {});

  EXPECT_TRUE(result.robot_in_known_free);
  EXPECT_EQ(result.frontier_cells, 7U);
  ASSERT_FALSE(result.candidates.empty());
  for (const auto & candidate : result.candidates) {
    EXPECT_TRUE(candidate.short_goal);
    EXPECT_GE(candidate.path_distance + 1e-9, 0.15);
    EXPECT_LT(candidate.path_distance, 0.35);
    EXPECT_TRUE(footprintFree(map, candidate.x, candidate.y, 0.14));
  }
  int x = -1;
  int y = -1;
  ASSERT_TRUE(worldToCell(map, result.candidates.front().x, result.candidates.front().y, x, y));
  EXPECT_EQ(x, 11);
  EXPECT_EQ(y, 7);
}

TEST(FrontierSearch, OnlyUsesShortCandidatesWhenAllOrdinaryCandidatesAreExcluded)
{
  const auto map = roomMap();
  const auto robot = cellCenter(map, 9, 15);
  const auto result = findCandidates(map, robot.first, robot.second, 0.14,
    [&](double x, double y) {
      return std::hypot(x - robot.first, y - robot.second) + 1e-9 >= 0.35;
    });

  ASSERT_FALSE(result.candidates.empty());
  for (const auto & candidate : result.candidates) {
    EXPECT_TRUE(candidate.short_goal);
    EXPECT_GE(std::hypot(candidate.x - robot.first, candidate.y - robot.second) + 1e-9, 0.15);
    EXPECT_LT(std::hypot(candidate.x - robot.first, candidate.y - robot.second), 0.35);
  }
}

TEST(FrontierSearch, KeepsShortAlternativesWhenOrdinaryCandidatesExist)
{
  const auto map = roomMap();
  const auto robot = cellCenter(map, 9, 15);
  const auto result = findCandidates(map, robot.first, robot.second, 0.14, {});

  ASSERT_FALSE(result.candidates.empty());
  ASSERT_FALSE(result.short_candidates.empty());
  for (const auto & candidate : result.candidates) {
    EXPECT_FALSE(candidate.short_goal);
  }
  double previous_distance = 0.0;
  for (const auto & candidate : result.short_candidates) {
    EXPECT_TRUE(candidate.short_goal);
    EXPECT_GE(candidate.path_distance + 1e-9, 0.15);
    EXPECT_GE(std::hypot(candidate.x - robot.first, candidate.y - robot.second) + 1e-9, 0.15);
    EXPECT_TRUE(candidate.path_distance < 0.35 ||
      std::hypot(candidate.x - robot.first, candidate.y - robot.second) < 0.35);
    EXPECT_GE(candidate.path_distance, previous_distance);
    previous_distance = candidate.path_distance;
  }
}

TEST(FrontierSearch, CountsSmallFrontiersEvenWhenNoClusterCanBeSelected)
{
  auto map = makeMap(10, 10, 0.05F, -1);
  fillRectangle(map, 5, 5, 5, 5, 0);
  const auto robot = cellCenter(map, 5, 5);
  const auto result = findCandidates(map, robot.first, robot.second, 0.01, {});

  EXPECT_TRUE(result.robot_in_known_free);
  EXPECT_EQ(result.frontier_cells, 1U);
  EXPECT_EQ(result.small_frontier_cells, 1U);
  EXPECT_EQ(result.frontier_clusters, 1U);
  EXPECT_EQ(result.significant_frontier_clusters, 0U);
  EXPECT_TRUE(result.candidates.empty());
}

TEST(FrontierSearch, ExcludingEveryGoalDoesNotEraseSignificantFrontiers)
{
  const auto map = roomMap();
  const auto robot = cellCenter(map, 15, 15);
  const auto result = findCandidates(map, robot.first, robot.second, 0.13,
    [](double, double) {return true;});
  ASSERT_TRUE(result.robot_footprint_free);
  EXPECT_TRUE(result.candidates.empty());
  EXPECT_TRUE(result.short_candidates.empty());
  EXPECT_GT(result.significant_frontier_clusters, 0U);
  EXPECT_GT(result.excluded_standpoints, 0U);
}

TEST(FrontierSearch, SeparatedSmallClustersDoNotAddUpToASignificantFrontier)
{
  auto map = makeMap(30, 30, 0.05F, 100);
  fillRectangle(map, 3, 3, 26, 26, 0);
  fillRectangle(map, 27, 8, 27, 8, -1);
  fillRectangle(map, 27, 16, 27, 16, -1);
  const auto robot = cellCenter(map, 15, 15);
  const auto result = findCandidates(map, robot.first, robot.second, 0.13, {});
  ASSERT_TRUE(result.robot_footprint_free);
  EXPECT_EQ(result.frontier_cells, 6U);
  EXPECT_EQ(result.frontier_clusters, 2U);
  EXPECT_EQ(result.small_clusters, 2U);
  EXPECT_EQ(result.small_frontier_cells, 6U);
  EXPECT_EQ(result.significant_frontier_clusters, 0U);
  EXPECT_TRUE(result.candidates.empty());
}

TEST(FrontierSearch, RejectsFootprintEdgeIntersectionWithObstacleOrUnknownCell)
{
  auto map = makeMap(30, 30, 0.05F, 0);
  const auto robot = cellCenter(map, 10, 10);
  ASSERT_TRUE(footprintFree(map, robot.first, robot.second, 0.12));

  const auto obstacle_center = cellCenter(map, 12, 12);
  // The cell center is clear of the circle, but its near corner intersects it.
  ASSERT_GT(std::hypot(
      obstacle_center.first - robot.first, obstacle_center.second - robot.second), 0.12);
  fillRectangle(map, 12, 12, 12, 12, 100);
  EXPECT_FALSE(footprintFree(map, robot.first, robot.second, 0.12));
  fillRectangle(map, 12, 12, 12, 12, -1);
  EXPECT_FALSE(footprintFree(map, robot.first, robot.second, 0.12));
}

TEST(FrontierSearch, ChecksCellsTouchedAtTheLowSideCircleBoundary)
{
  auto map = makeMap(10, 10, 1.0F, 0);
  fillRectangle(map, 1, 3, 1, 3, 100);
  // The circle is tangent to the right edge of the occupied cell at x=2.
  EXPECT_FALSE(footprintFree(map, 3.0, 3.5, 1.0));
}

TEST(FrontierSearch, AllowsUnknownOnlyInTheExtraObstacleClearanceRing)
{
  auto map = makeMap(30, 30, 0.05F, 0);
  const auto robot = cellCenter(map, 10, 10);
  fillRectangle(map, 13, 10, 13, 10, -1);

  ASSERT_TRUE(footprintFree(map, robot.first, robot.second, 0.10));
  ASSERT_FALSE(footprintFree(map, robot.first, robot.second, 0.15));
  EXPECT_TRUE(goalFootprintFree(map, robot.first, robot.second, 0.10, 0.05));
}

TEST(FrontierSearch, RejectsActualObstacleInTheExtraClearanceRing)
{
  auto map = makeMap(30, 30, 0.05F, 0);
  const auto robot = cellCenter(map, 10, 10);
  fillRectangle(map, 13, 10, 13, 10, 50);

  ASSERT_TRUE(footprintFree(map, robot.first, robot.second, 0.10));
  EXPECT_FALSE(goalFootprintFree(map, robot.first, robot.second, 0.10, 0.05));
}

TEST(FrontierSearch, StillRejectsUnknownInsideTheBodyAndMapOutsideClearanceRing)
{
  auto map = makeMap(30, 30, 0.05F, 0);
  const auto robot = cellCenter(map, 10, 10);
  fillRectangle(map, 12, 10, 12, 10, -1);
  EXPECT_FALSE(goalFootprintFree(map, robot.first, robot.second, 0.10, 0.05));

  map = makeMap(30, 30, 0.05F, 0);
  const auto near_edge = cellCenter(map, 2, 10);
  ASSERT_TRUE(footprintFree(map, near_edge.first, near_edge.second, 0.10));
  EXPECT_FALSE(goalFootprintFree(map, near_edge.first, near_edge.second, 0.10, 0.05));
}

TEST(FrontierSearch, ObstacleMarginPreservesShortStepAtUnknownBlindCorner)
{
  auto map = makeMap(30, 18, 0.05F, 100);
  fillRectangle(map, 3, 4, 14, 12, 0);
  fillRectangle(map, 15, 4, 26, 12, -1);
  const auto robot = cellCenter(map, 8, 8);
  const double body_radius = 0.13 + std::sqrt(2.0) * 0.01;

  // Applying the obstacle margin to unknown space as well erases the 0.15m step.
  EXPECT_TRUE(findCandidates(map, robot.first, robot.second,
      body_radius + 0.05, {}).candidates.empty());
  const auto result = findCandidates(map, robot.first, robot.second, body_radius, {}, 0.05);
  ASSERT_FALSE(result.candidates.empty());
  for (const auto & candidate : result.candidates) {
    EXPECT_TRUE(candidate.short_goal);
    EXPECT_TRUE(goalFootprintFree(map, candidate.x, candidate.y, body_radius, 0.05));
    EXPECT_GE(candidate.path_distance + 1e-9, 0.15);
  }
  int x = -1;
  int y = -1;
  ASSERT_TRUE(worldToCell(map, result.candidates.front().x, result.candidates.front().y, x, y));
  EXPECT_EQ(x, 11);
  EXPECT_EQ(y, 8);
}

TEST(FrontierSearch, RecordedCornerFindsForwardViewpointsBeyondWallHuggingParentChain)
{
  // The former raw-parent retreat returned zero goals for these 23 frontiers:
  // every parent chain collapsed onto cell (42,41), only 0.068m from the robot.
  // The recorded SLAM map is test input; no true world map enters the runtime.
  const auto fixture = std::filesystem::path(__FILE__).parent_path() /
    "fixtures" / "recorded_corner.pgm";
  std::ifstream stream(fixture, std::ios::binary);
  ASSERT_TRUE(stream.is_open()) << fixture;
  std::string magic;
  unsigned int width = 0;
  unsigned int height = 0;
  int maximum = 0;
  stream >> magic >> width >> height >> maximum;
  ASSERT_EQ(magic, "P5");
  ASSERT_EQ(maximum, 255);
  stream.get();
  auto map = makeMap(width, height, 0.05F, -1);
  map.info.origin.position.x = -0.399;
  map.info.origin.position.y = -0.390;
  for (int y = static_cast<int>(height) - 1; y >= 0; --y) {
    for (unsigned int x = 0; x < width; ++x) {
      const int pixel = stream.get();
      ASSERT_NE(pixel, std::char_traits<char>::eof());
      map.data[static_cast<std::size_t>(y) * width + x] =
        pixel == 205 ? -1 : pixel < 100 ? 100 : 0;
    }
  }
  const auto result = findCandidates(map, 1.660536, 1.665278, 0.144142, {}, 0.05);
  EXPECT_TRUE(result.robot_in_known_free);
  EXPECT_EQ(result.frontier_cells, 23U);
  ASSERT_GT(result.candidates.size(), 1U);
  bool observes_forward_frontier = false;
  for (const auto & candidate : result.candidates) {
    EXPECT_FALSE(candidate.short_goal);
    EXPECT_GT(candidate.path_distance, 0.35);
    EXPECT_TRUE(goalFootprintFree(map, candidate.x, candidate.y, 0.144142, 0.05));
    // These standpoints must advance around the corner rather than retreat
    // into the previously explored corridor to meet a distance threshold.
    if (candidate.x > 2.0 && candidate.y > 2.2) {
      observes_forward_frontier = true;
    }
  }
  EXPECT_TRUE(observes_forward_frontier);
}

TEST(FrontierSearch, DoesNotCrossUnknownSpaceToDisconnectedFreeRegion)
{
  auto map = makeMap(40, 30, 0.05F, -1);
  fillRectangle(map, 3, 3, 14, 26, 0);
  fillRectangle(map, 18, 3, 36, 26, 0);
  const auto robot = cellCenter(map, 8, 15);
  const auto result = findCandidates(map, robot.first, robot.second, 0.14, {});

  ASSERT_FALSE(result.candidates.empty());
  for (const auto & candidate : result.candidates) {
    EXPECT_LT(candidate.x, 0.75);
    EXPECT_TRUE(footprintFree(map, candidate.x, candidate.y, 0.14));
  }
}

TEST(FrontierSearch, HandlesNegativeCoordinatesAndOutOfBoundsWithoutTruncation)
{
  auto map = makeMap(20, 20, 0.05F, 0);
  map.info.origin.position.x = -2.0;
  map.info.origin.position.y = -3.0;
  int x = -1;
  int y = -1;
  const auto inside = cellCenter(map, 2, 3);
  ASSERT_TRUE(worldToCell(map, inside.first, inside.second, x, y));
  EXPECT_EQ(x, 2);
  EXPECT_EQ(y, 3);
  EXPECT_FALSE(worldToCell(map, -2.001, -2.8, x, y));
  EXPECT_FALSE(worldToCell(map, -1.8, -3.001, x, y));
  EXPECT_FALSE(worldToCell(map, 1e300, 1e300, x, y));
  EXPECT_FALSE(worldToCell(map, std::numeric_limits<double>::quiet_NaN(), -2.8, x, y));
  EXPECT_FALSE(footprintFree(map, -1.98, -2.5, 0.14));
  const auto result = findCandidates(map, -2.001, -2.8, 0.14, {});
  EXPECT_FALSE(result.robot_in_known_free);
  EXPECT_TRUE(result.candidates.empty());
}

TEST(FrontierSearch, TransformsRotatedMapOriginForCellsFootprintsAndGoals)
{
  auto map = shortApproachMap();
  const double half_yaw = std::acos(-1.0) / 4.0;
  map.info.origin.position.x = -2.0;
  map.info.origin.position.y = 3.0;
  // Also exercise quaternion normalization before extracting yaw.
  map.info.origin.orientation.z = 2.0 * std::sin(half_yaw);
  map.info.origin.orientation.w = 2.0 * std::cos(half_yaw);
  const auto robot = cellCenter(map, 8, 7);
  int x = -1;
  int y = -1;
  ASSERT_TRUE(worldToCell(map, robot.first, robot.second, x, y));
  EXPECT_EQ(x, 8);
  EXPECT_EQ(y, 7);
  EXPECT_TRUE(footprintFree(map, robot.first, robot.second, 0.14));
  const auto result = findCandidates(map, robot.first, robot.second, 0.14, {});
  ASSERT_FALSE(result.candidates.empty());
  const auto expected = cellCenter(map, 11, 7);
  EXPECT_NEAR(result.candidates.front().x, expected.first, 1e-9);
  EXPECT_NEAR(result.candidates.front().y, expected.second, 1e-9);
  EXPECT_TRUE(result.candidates.front().short_goal);
}

TEST(FrontierSearch, RejectsMalformedMapAndInvalidRadius)
{
  auto map = makeMap(10, 10, 0.05F, 0);
  EXPECT_TRUE(validMap(map));
  EXPECT_EQ(knownFreeCells(map), 100U);
  EXPECT_FALSE(footprintFree(map, 0.25, 0.25, -0.1));
  EXPECT_FALSE(footprintFree(map, 0.25, 0.25, std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(goalFootprintFree(map, 0.25, 0.25, 0.10, -0.01));
  EXPECT_FALSE(goalFootprintFree(map, 0.25, 0.25, 0.10,
      std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(findCandidates(map, 0.25, 0.25, 0.10, {}, -0.01).candidates.empty());
  map.data.pop_back();
  EXPECT_FALSE(validMap(map));
  EXPECT_EQ(knownFreeCells(map), 0U);
  EXPECT_TRUE(findCandidates(map, 0.25, 0.25, 0.14, {}).candidates.empty());
  map = makeMap(10, 10, 0.05F, 0);
  map.info.resolution = 0.0F;
  EXPECT_FALSE(validMap(map));
  map = makeMap(10, 10, 0.05F, 0);
  map.info.origin.orientation.w = 0.0;
  EXPECT_FALSE(validMap(map));
}

}  // namespace
