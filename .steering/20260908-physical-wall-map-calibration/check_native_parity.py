"""Compile the actual production Map declaration in isolation and inspect cells."""
from pathlib import Path
import subprocess

package = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
source = (package / 'src/mpc_controller_cpp.cpp').read_text()
start = source.index('struct Obstacle\n{')
end = source.index('\nstd::vector<std::pair<int, int>> line_cells', start)
root = Path('/output/20260908-physical-wall-map-calibration')
root.mkdir(exist_ok=True)
code = """#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
""" + source[start:end] + """
int main(int argc, char ** argv) {
  if (argc != 3) return 2;
  const Map map(argv[1]);
  std::ofstream out(argv[2], std::ios::binary);
  for (int r = 0; r < map.height; ++r) {
    out.write(reinterpret_cast<const char *>(map.data.ptr(r)), map.width);
  }
  return out.good() ? 0 : 1;
}
"""
(root / 'native-map-parity.cpp').write_text(code)
flags = subprocess.check_output(['pkg-config', '--cflags', '--libs', 'opencv4', 'yaml-cpp'], text=True).split()
subprocess.run(['g++', '-std=c++17', '-O0', str(root/'native-map-parity.cpp'), '-o', str(root/'native-map-parity'), *flags], check=True)
subprocess.run([str(root/'native-map-parity'), str(package/'env/final_ver3/occupancy_grid_map.yaml'), str(root/'native-calibrated-grid.bin')], check=True)
import sys
sys.path.insert(0, str(package/'tools/kaleidoscope'))
from kaleidoscope.trajectory_clearance import load_occupancy_grid, CellState
grid = load_occupancy_grid(package/'env/final_ver3/occupancy_grid_map.yaml')
expected = bytes(0 if c is CellState.OCCUPIED else 1 for c in grid.cells)
actual = (root/'native-calibrated-grid.bin').read_bytes()
assert actual == expected, 'native/editor raster mismatch'
print(f'native/editor parity: {len(actual)}cells, {actual.count(0)}occupied')
