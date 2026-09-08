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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
""" + source[start:end] + """
int main(int argc, char ** argv) {
  if (argc != 2) return 2;
  const Map map(argv[1]);
  std::cout << "center=" << int(map.data.at<signed char>(2, 2)) << "\\n";
  return map.data.at<signed char>(2, 2) == 0 ? 0 : 1;
}
"""
(root / 'native-map.cpp').write_text(code)
flags = subprocess.check_output(['pkg-config', '--cflags', '--libs', 'opencv4', 'yaml-cpp'], text=True).split()
subprocess.run(['g++', '-std=c++17', '-O0', str(root/'native-map.cpp'), '-o', str(root/'native-map'), *flags], check=True)
(root / 'map.pgm').write_bytes(b'P5\n5 5\n255\n' + bytes([0 if i == 12 else 255 for i in range(25)]))
(root / 'map.yaml').write_text('image: map.pgm\nresolution: 1\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\npreserve_occupied_cells: true\n')
subprocess.run([str(root/'native-map'), str(root/'map.yaml')], check=True)
