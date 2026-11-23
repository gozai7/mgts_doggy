#include <iomanip>
#include <fstream>
#include <karto_sdk/Mapper.h>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  karto::Mapper graph;
  graph.LoadFromFile(argv[1]);
  std::ofstream csv(argv[2]);
  csv.exceptions(std::ios::badbit | std::ios::failbit);
  csv << "t,x,y,yaw\n" << std::setprecision(17);
  for (const auto *scan : graph.GetAllProcessedScans()) {
    const auto &pose = scan->GetCorrectedPose();
    csv << scan->GetTime() << ',' << pose.GetX() << ',' << pose.GetY()
              << ',' << pose.GetHeading() << '\n';
  }
}
