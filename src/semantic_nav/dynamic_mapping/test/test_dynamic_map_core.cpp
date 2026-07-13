#include "dynamic_mapping/dynamic_map_core.hpp"

#include <gtest/gtest.h>

namespace dynamic_mapping {
namespace {

TEST(DynamicMapCore, ConvertsWorldAndGridCoordinates) {
  const GridGeometry geometry{10, 20, 0.5, -1.0, 2.0};

  Cell cell;
  ASSERT_TRUE(worldToGrid(geometry, -0.75, 2.25, cell));
  EXPECT_EQ(cell.x, 0);
  EXPECT_EQ(cell.y, 0);

  ASSERT_TRUE(worldToGrid(geometry, 3.99, 11.99, cell));
  EXPECT_EQ(cell.x, 9);
  EXPECT_EQ(cell.y, 19);

  EXPECT_FALSE(worldToGrid(geometry, 4.1, 2.0, cell));

  const auto center = gridToWorldCenter(geometry, Cell{2, 3});
  EXPECT_DOUBLE_EQ(center.first, 0.25);
  EXPECT_DOUBLE_EQ(center.second, 3.75);

  const auto index = cellIndex(geometry, Cell{2, 3});
  ASSERT_TRUE(index.has_value());
  EXPECT_EQ(*index, 32U);
}

TEST(DynamicMapCore, RaytraceUsesBresenhamCells) {
  const auto cells = raytraceCells(Cell{0, 0}, Cell{5, 3});
  const std::vector<Cell> expected{
      Cell{0, 0}, Cell{1, 1}, Cell{2, 1},
      Cell{3, 2}, Cell{4, 2}, Cell{5, 3}};

  ASSERT_EQ(cells.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(cells[i].x, expected[i].x);
    EXPECT_EQ(cells[i].y, expected[i].y);
  }
}

TEST(DynamicMapCore, EvidenceRequiresHitsAndDistinctPoses) {
  EvidenceThresholds thresholds;
  thresholds.min_absence_hits = 5;
  thresholds.min_distinct_poses = 3;
  thresholds.min_pose_separation_m = 0.25;
  EvidenceGrid evidence(thresholds);
  evidence.reset(4);

  evidence.addFreeSpaceEvidence(2, Pose2D{0.0, 0.0});
  evidence.addFreeSpaceEvidence(2, Pose2D{0.1, 0.0});
  evidence.addFreeSpaceEvidence(2, Pose2D{0.2, 0.0});
  evidence.addFreeSpaceEvidence(2, Pose2D{0.3, 0.0});
  EXPECT_FALSE(evidence.cleared(2));

  evidence.addFreeSpaceEvidence(2, Pose2D{0.35, 0.0});
  EXPECT_FALSE(evidence.cleared(2));

  evidence.addFreeSpaceEvidence(2, Pose2D{0.6, 0.0});
  EXPECT_TRUE(evidence.cleared(2));
  EXPECT_EQ(evidence.evidence(2).absence_hits, 6);
  EXPECT_EQ(evidence.evidence(2).distinct_poses.size(), 3U);
  EXPECT_EQ(evidence.clearedCount(), 1U);

  evidence.resetEvidence(2);
  EXPECT_FALSE(evidence.cleared(2));
  EXPECT_EQ(evidence.evidence(2).absence_hits, 0);
}

TEST(DynamicMapCore, FiltersLabelsCaseInsensitively) {
  const auto labels = normalizedLabelSet({"Person", "chair", "BOX"});
  EXPECT_TRUE(labels.find("person") != labels.end());
  EXPECT_TRUE(labels.find("chair") != labels.end());
  EXPECT_TRUE(labels.find("box") != labels.end());
  EXPECT_TRUE(labels.find("cart") == labels.end());
}

TEST(DynamicMapCore, PointInPolygon) {
  const std::vector<std::pair<double, double>> polygon{
      {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
  EXPECT_TRUE(pointInPolygon(1.0, 1.0, polygon));
  EXPECT_FALSE(pointInPolygon(3.0, 1.0, polygon));
}

}  // namespace
}  // namespace dynamic_mapping
