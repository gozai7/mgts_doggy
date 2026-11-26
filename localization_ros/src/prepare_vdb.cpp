#include <openvdb/openvdb.h>
#include <openvdb/tools/TopologyToLevelSet.h>
#include <fstream>
#include <cmath>
#include <iostream>
#include <stdexcept>

// Input: little-endian native float32 XYZ triples from the validated PCD reader.
int main(int argc, char** argv) {
  try {
    if (argc != 4) throw std::invalid_argument("prepare_vdb points.f32 map.vdb resolution_m");
    const double resolution = std::stod(argv[3]);
    if (!std::isfinite(resolution) || resolution <= 0) throw std::invalid_argument("Invalid resolution");
    openvdb::initialize();
    auto grid = openvdb::FloatGrid::create();
    grid->setTransform(openvdb::math::Transform::createLinearTransform(resolution));
    auto accessor = grid->getAccessor();
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) throw std::runtime_error("Cannot open XYZ input");
    float p[3]; std::size_t count = 0;
    while (input.read(reinterpret_cast<char*>(p), sizeof(p))) {
      for (float value : p) if (!std::isfinite(value)) throw std::runtime_error("Nonfinite XYZ");
      accessor.setValue(grid->transform().worldToIndexCellCentered(openvdb::Vec3d(p[0], p[1], p[2])), 1.f);
      ++count;
    }
    if (input.gcount() || !count) throw std::runtime_error("Empty or truncated XYZ input");
    auto level = openvdb::tools::topologyToLevelSet(*grid, 4, 0, 0, 0);
    level->setName("map");
    openvdb::io::File(argv[2]).write({level});
    std::cout << "points=" << count << " active_voxels=" << level->activeVoxelCount() << '\n';
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
