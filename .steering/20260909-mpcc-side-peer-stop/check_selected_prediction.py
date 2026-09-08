"""Compile the actual controller selection block against front/side observations.

The shim supplies data only; the tested lambda and call are read verbatim from
the controller. This catches the missing side dispatch that text checks missed.
"""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

out=Path(sys.argv[1])
out.mkdir(parents=True,exist_ok=False)
path=Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp')
source=path.read_text()
start=source.index('    const auto set_target_execution_prediction =')
end=source.index('    output.overtake_entry_target_speed =',start)
body=source[start:end]
prefix=r'''
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
struct Case { bool front,side,jump,velocity_valid,expected; double expected_longitudinal; };
struct Controller {
  struct { struct {double prediction_time=0.3;} v2x_gap; } cfg;
  double current_speed_mps_=2.0;
  bool run(const Case & test) {
    struct {
      bool target_execution_prediction_valid{},target_execution_position_jump{},target_execution_course_progress_rejected{};
      double target_execution_longitudinal{},target_execution_lateral{},target_execution_predicted_longitudinal{},target_execution_predicted_lateral{};
    } output;
    const bool has_front_vehicle=test.front,has_side_vehicle=test.side;
    const bool nearest_front_position_jump=false,nearest_front_course_progress_rejected=false,nearest_front_lateral_velocity_valid=true;
    const double nearest_front_distance=8.0,nearest_front_course_lateral=0.5,nearest_front_speed=3.0,nearest_front_lateral_velocity=0.2;
    const bool nearest_side_position_jump=test.jump,nearest_side_course_progress_rejected=false,nearest_side_lateral_velocity_valid=test.velocity_valid;
    const double nearest_side_course_longitudinal=1.25,nearest_side_course_lateral=-2.6,nearest_side_speed=1.5,nearest_side_lateral_velocity=-0.1;
'''
suffix=r'''
    return output.target_execution_prediction_valid==test.expected &&
      (!test.expected || std::abs(output.target_execution_predicted_longitudinal-test.expected_longitudinal)<1e-12);
  }
};
int main() {
  const Case cases[]={{true,false,false,true,true,8.3},{false,true,false,true,true,1.1},
    {false,false,false,true,false,0},{false,true,true,true,false,0},
    {false,true,false,false,false,0},{true,true,true,false,true,8.3}};
  int failed=0,index=0;
  for (const auto & test : cases) {
    const bool ok=Controller{}.run(test);
    std::cout<<"case="<<index++<<" accepted="<<ok<<'\n';failed+=!ok;
  }
  return failed ? 1 : 0;
}
'''
(out/'probe.cpp').write_text(prefix+body+suffix)
with (out/'build.log').open('w') as log:
    subprocess.run(['g++','-std=c++17','-O2',str(out/'probe.cpp'),'-o',str(out/'probe')],
                   stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
with (out/'test.log').open('w') as log:
    result=subprocess.run([str(out/'probe')],stdout=log,stderr=subprocess.STDOUT,timeout=5)
report=dict(controller_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            extracted_block_sha256=hashlib.sha256(body.encode()).hexdigest(),return_code=result.returncode)
(out/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
print((out/'test.log').read_text())
sys.exit(result.returncode)
