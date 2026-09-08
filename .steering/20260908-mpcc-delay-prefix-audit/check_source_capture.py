"""Exercise the actual pipeline source-capture branch for every normal intent."""
from pathlib import Path
import subprocess
import sys

package=Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros').resolve()
source=(package/'src/mpc_controller_cpp.cpp').read_text()
begin=source.index('RateResolvedPipelineEvaluation evaluate_rate_resolved_pipeline(')
begin=source.index('  RateResolvedPipelineEvaluation evaluation;',begin)+len('  RateResolvedPipelineEvaluation evaluation;')
end=source.index('  evaluation.dynamic_sqp_depth =',begin)
prolog=source[begin:end]
out=Path(sys.argv[1]).resolve()
out.mkdir(parents=True,exist_ok=False)
cpp='''#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"
#include <iostream>
#include <memory>
namespace rate_resolved_shadow = multi_purpose_mpc_ros::mpcc_rate_resolved_shadow;
namespace mpcc_contract = multi_purpose_mpc_ros::mpcc_execution_contract;
std::shared_ptr<const rate_resolved_shadow::Snapshot> capture(const rate_resolved_shadow::Snapshot &snapshot) {
  struct {std::shared_ptr<const rate_resolved_shadow::Snapshot> solver_source_snapshot;} evaluation;
'''+prolog+'''
  return evaluation.solver_source_snapshot;
}
int main() {
  using Intent=mpcc_contract::ControlIntent;
  int missing=0;
  for (auto intent:{Intent::Track,Intent::Cruise,Intent::Follow,Intent::ShiftOut,Intent::Pass,Intent::Return,Intent::Rejoin}) {
    rate_resolved_shadow::Snapshot input;
    input.identity.sequence=42;
    input.identity.source_context.intent=intent;
    const auto owned=capture(input);
    input.identity.sequence=99;
    const bool ok=owned && owned.get()!=&input && owned->identity.sequence==42 && owned->identity.source_context.intent==intent;
    std::cout << "intent=" << static_cast<int>(intent) << ", immutable_source=" << ok << '\\n';
    missing+=!ok;
  }
  return missing ? 1 : 0;
}
'''
(out/'capture.cpp').write_text(cpp)
subprocess.run(['c++','-std=c++17','-I'+str(package/'include'),'-I/usr/include/eigen3',str(out/'capture.cpp'),'-o',str(out/'capture')],check=True)
result=subprocess.run([str(out/'capture')])
raise SystemExit(result.returncode)
