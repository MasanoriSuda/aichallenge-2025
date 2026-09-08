"""Observe the first invalid-input boundary in the changed normal comparison."""
from pathlib import Path
import shlex
import subprocess

package=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
build=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out=Path('/output/20260909-side-peer-normal-boundary')
out.mkdir(exist_ok=False)
source=next(Path('/output/20260909-peer-envelope-dev2-r1/world-target-comparison-r2/396/candidate').glob('*/snapshot.yaml'))
code=(package/'src/mpcc_rate_resolved_dynamic_obstacle.cpp').read_text()
code='#include <iostream>\n'+code
code=code.replace('const auto invalid = []() {',r'''const auto invalid = [&request]() {
    std::cerr << "Cartesian request rejected\n";
    if (request.cartesian_prediction && request.cartesian_prediction->course_frame.knots) {
      const auto & f=request.cartesian_prediction->course_frame;
      std::cerr << "window=" << f.knots->front().progress_m << "," << f.knots->back().progress_m << " origin=" << f.progress_origin_m << '\n';
      for (int k=1;k<=request.wall_only_problem.horizon_steps;++k) {
        const double p=f.progress_origin_m+request.wall_only_primal[k*7+4];
        if (!mpc_stage_geometry::sample_course_frame(*f.knots,p)) std::cerr.precision(17),std::cerr << "stage=" << k << " query=" << p << " relative=" << request.wall_only_primal[k*7+4] << '\n';
      }
    }
''')
code=code.replace('auto result = refine(local);','auto result = refine(local);\n  if (result.reason==Reason::InvalidInput) std::cerr << "legacy classification rejected\\n";')
(out/'dynamic.cpp').write_text(code)
(out/'main.cpp').write_text(r'''
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include <iostream>
int main(int argc,char **argv) {
  std::string detail;
  auto s=multi_purpose_mpc_ros::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(argv[1],&detail);
  if (!s) {std::cout<<detail;return 2;}
  multi_purpose_mpc_ros::mpcc_rate_resolved_shadow::SolverContext solver;
  std::cout<<solver.evaluate(s->source).detail<<'\n';
}
''')
link=shlex.split((build/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
command=[link[0],'-std=c++17','-O2','-I'+str(package/'include'),'-I/usr/include/eigen3',str(out/'main.cpp'),str(out/'dynamic.cpp'),'-o',str(out/'probe')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:
    subprocess.run(command,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=90)
with (out/'probe.log').open('w') as log:
    subprocess.run([str(out/'probe'),str(source)],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
print((out/'probe.log').read_text(),flush=True)
