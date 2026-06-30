#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace dynamic_mapping {

struct GridGeometry {
  unsigned int width{0};
  unsigned int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
};

struct Cell {
  int x{0};
  int y{0};
};

struct Pose2D {
  double x{0.0};
  double y{0.0};
};

struct EvidenceThresholds {
  int min_absence_hits{5};
  int min_distinct_poses{3};
  double min_pose_separation_m{0.25};
};

struct CellEvidence {
  int absence_hits{0};
  std::vector<Pose2D> distinct_poses;
  bool cleared{false};
};

bool sameGeometry(const GridGeometry& a, const GridGeometry& b);
bool worldToGrid(const GridGeometry& geometry, double wx, double wy, Cell& cell);
std::pair<double, double> gridToWorldCenter(const GridGeometry& geometry,
                                            const Cell& cell);
std::optional<std::size_t> cellIndex(const GridGeometry& geometry,
                                     const Cell& cell);
std::vector<Cell> raytraceCells(const Cell& start, const Cell& end);
bool pointInPolygon(double x, double y,
                    const std::vector<std::pair<double, double>>& polygon);
std::string normalizeLabel(const std::string& label);

class EvidenceGrid {
public:
  explicit EvidenceGrid(EvidenceThresholds thresholds = {});

  void reset(std::size_t size);
  void setThresholds(EvidenceThresholds thresholds);
  void addFreeSpaceEvidence(std::size_t index, const Pose2D& pose);
  void resetEvidence(std::size_t index);

  const CellEvidence& evidence(std::size_t index) const;
  bool cleared(std::size_t index) const;
  std::size_t clearedCount() const;

private:
  bool isDistinctPose(const CellEvidence& evidence, const Pose2D& pose) const;
  void updateCleared(CellEvidence& evidence) const;

  EvidenceThresholds thresholds_;
  std::vector<CellEvidence> cells_;
  CellEvidence empty_;
};

std::unordered_set<std::string>
normalizedLabelSet(const std::vector<std::string>& labels);

}  // namespace dynamic_mapping
