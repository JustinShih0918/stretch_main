#include "dynamic_mapping/dynamic_map_core.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>

namespace dynamic_mapping {

bool sameGeometry(const GridGeometry& a, const GridGeometry& b) {
  return a.width == b.width && a.height == b.height &&
         std::abs(a.resolution - b.resolution) < 1e-9 &&
         std::abs(a.origin_x - b.origin_x) < 1e-9 &&
         std::abs(a.origin_y - b.origin_y) < 1e-9;
}

bool worldToGrid(const GridGeometry& geometry, double wx, double wy, Cell& cell) {
  if (geometry.resolution <= 0.0) {
    return false;
  }
  const int mx = static_cast<int>(
      std::floor((wx - geometry.origin_x) / geometry.resolution));
  const int my = static_cast<int>(
      std::floor((wy - geometry.origin_y) / geometry.resolution));
  if (mx < 0 || my < 0 || mx >= static_cast<int>(geometry.width) ||
      my >= static_cast<int>(geometry.height)) {
    return false;
  }
  cell = Cell{mx, my};
  return true;
}

std::pair<double, double> gridToWorldCenter(const GridGeometry& geometry,
                                            const Cell& cell) {
  return {geometry.origin_x + (static_cast<double>(cell.x) + 0.5) *
                                  geometry.resolution,
          geometry.origin_y + (static_cast<double>(cell.y) + 0.5) *
                                  geometry.resolution};
}

std::optional<std::size_t> cellIndex(const GridGeometry& geometry,
                                     const Cell& cell) {
  if (cell.x < 0 || cell.y < 0 || cell.x >= static_cast<int>(geometry.width) ||
      cell.y >= static_cast<int>(geometry.height)) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(cell.y) * geometry.width +
         static_cast<std::size_t>(cell.x);
}

std::vector<Cell> raytraceCells(const Cell& start, const Cell& end) {
  std::vector<Cell> cells;
  int x0 = start.x;
  int y0 = start.y;
  const int x1 = end.x;
  const int y1 = end.y;
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  while (true) {
    cells.push_back(Cell{x0, y0});
    if (x0 == x1 && y0 == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
  return cells;
}

bool pointInPolygon(double x, double y,
                    const std::vector<std::pair<double, double>>& polygon) {
  if (polygon.size() < 3) {
    return false;
  }
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto& pi = polygon[i];
    const auto& pj = polygon[j];
    const bool intersects =
        ((pi.second > y) != (pj.second > y)) &&
        (x < (pj.first - pi.first) * (y - pi.second) /
                     (pj.second - pi.second) +
                 pi.first);
    if (intersects) {
      inside = !inside;
    }
  }
  return inside;
}

std::string normalizeLabel(const std::string& label) {
  std::string out;
  out.reserve(label.size());
  for (const char c : label) {
    out.push_back(static_cast<char>(
        std::tolower(static_cast<unsigned char>(c))));
  }
  return out;
}

EvidenceGrid::EvidenceGrid(EvidenceThresholds thresholds)
    : thresholds_(thresholds) {}

void EvidenceGrid::reset(std::size_t size) {
  cells_.clear();
  cells_.resize(size);
}

void EvidenceGrid::setThresholds(EvidenceThresholds thresholds) {
  thresholds_ = thresholds;
  for (auto& evidence : cells_) {
    updateCleared(evidence);
  }
}

void EvidenceGrid::addFreeSpaceEvidence(std::size_t index, const Pose2D& pose) {
  if (index >= cells_.size()) {
    return;
  }
  auto& evidence = cells_[index];
  evidence.absence_hits++;
  if (isDistinctPose(evidence, pose)) {
    evidence.distinct_poses.push_back(pose);
  }
  updateCleared(evidence);
}

void EvidenceGrid::resetEvidence(std::size_t index) {
  if (index >= cells_.size()) {
    return;
  }
  cells_[index] = CellEvidence{};
}

const CellEvidence& EvidenceGrid::evidence(std::size_t index) const {
  if (index >= cells_.size()) {
    return empty_;
  }
  return cells_[index];
}

bool EvidenceGrid::cleared(std::size_t index) const {
  return index < cells_.size() && cells_[index].cleared;
}

std::size_t EvidenceGrid::clearedCount() const {
  return static_cast<std::size_t>(std::count_if(
      cells_.begin(), cells_.end(),
      [](const CellEvidence& evidence) { return evidence.cleared; }));
}

bool EvidenceGrid::isDistinctPose(const CellEvidence& evidence,
                                  const Pose2D& pose) const {
  const double min_sq = thresholds_.min_pose_separation_m *
                        thresholds_.min_pose_separation_m;
  for (const auto& prior : evidence.distinct_poses) {
    const double dx = prior.x - pose.x;
    const double dy = prior.y - pose.y;
    if (dx * dx + dy * dy < min_sq) {
      return false;
    }
  }
  return true;
}

void EvidenceGrid::updateCleared(CellEvidence& evidence) const {
  evidence.cleared =
      evidence.absence_hits >= thresholds_.min_absence_hits &&
      static_cast<int>(evidence.distinct_poses.size()) >=
          thresholds_.min_distinct_poses;
}

std::unordered_set<std::string>
normalizedLabelSet(const std::vector<std::string>& labels) {
  std::unordered_set<std::string> out;
  for (const auto& label : labels) {
    out.insert(normalizeLabel(label));
  }
  return out;
}

}  // namespace dynamic_mapping
