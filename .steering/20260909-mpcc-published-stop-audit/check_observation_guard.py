"""Execute the actual C++ recorder guard, with only data supplied by the shim."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

source = Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp')
code = source.read_text()
start = code.index('    if (',code.index('  void record_rate_resolved_normal_authority_failure_snapshot('))
end = code.index('    const auto reject =',start)
guard = code[start:end]
out = Path(sys.argv[1])
out.mkdir(exist_ok=False)
prefix = r'''
#include <iostream>
#include <optional>
namespace mpcc_contract { enum class ControlIntent { Unknown, Stop, Track, Cruise }; }
using Intent = mpcc_contract::ControlIntent;
bool captured = false;
void observe(bool has_authority, Intent requested_intent, bool published_stop_retained) {
  struct {std::optional<int> production_authority;} retained;
  if (has_authority) retained.production_authority = 1;
'''
suffix = r'''
  captured = true;
}
int main() {
  struct Case { bool authority; Intent intent; bool previous_stop; bool expected; };
  const Case cases[] = {{false,Intent::Cruise,true,true}, {false,Intent::Cruise,false,true},
    {false,Intent::Track,true,true}, {true,Intent::Cruise,true,false},
    {false,Intent::Stop,true,false}, {false,Intent::Unknown,true,false}};
  int failed = 0, index = 0;
  for (const auto & c : cases) {
    captured = false; observe(c.authority,c.intent,c.previous_stop);
    const bool ok = captured == c.expected;
    std::cout << "case=" << index++ << " pass=" << ok << " captured=" << captured << '\n';
    failed += !ok;
  }
  return failed ? 1 : 0;
}
'''
(out/'probe.cpp').write_text(prefix+guard+suffix)
with (out/'build.log').open('w') as log:
    subprocess.run(['g++','-std=c++17','-O2',str(out/'probe.cpp'),'-o',str(out/'probe')],
                   stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
with (out/'test.log').open('w') as log:
    result = subprocess.run([str(out/'probe')],stdout=log,stderr=subprocess.STDOUT,timeout=5)
(out/'manifest.json').write_text(json.dumps(dict(controller_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    guard_sha256=hashlib.sha256(guard.encode()).hexdigest(),return_code=result.returncode),indent=2)+'\n')
print((out/'test.log').read_text())
sys.exit(result.returncode)
