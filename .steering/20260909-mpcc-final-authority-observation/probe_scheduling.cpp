// Diagnostic uses existing native fixture and actual recorder/worker classes.
// It does not execute the controller; caller suppression is separately observed
// in the live949/951record and the current source-contract test.
#include "test_mpcc_architecture_snapshot.cpp"
#include "multi_purpose_mpc_ros/latest_only_worker.hpp"
#include <future>
#include <iostream>
#include <stdexcept>
int main(int argc,char **argv) {
  if(argc!=2)return 2;
  namespace a=multi_purpose_mpc_ros::mpcc_architecture_snapshot;
  namespace m=multi_purpose_mpc_ros;
  auto observed=a::make_publication_observation();
  auto world=*observed.source;
  YAML::Node report;
  report["fixture"]= "synthetic native fixture, not a substitute for live949/951";
  for(std::uint64_t id:{1001U,1002U}) {
    world.identity.sequence=id;world.identity.source_context.decision_id=id;
    world.identity.source_context=m::mpcc_execution_contract::seal_problem_context(world.identity.source_context);
    a::bind_failure_world(observed,world);
    const auto recorded=a::record_authority_failure(world,"terminal-contingency-unavailable","bounded bucket observation",observed,"bucket");
    report["terminal"][std::to_string(id)]=a::to_string(recorded.status);
  }
  const auto final=a::record_authority_failure(world,"normal-authority-unavailable","separate final boundary if actually invoked",observed,"bucket");
  report["explicit_final_record"]=a::to_string(final.status);

  m::LatestOnlyWorker worker;
  std::promise<void> entered,release,completed;
  auto releasing=release.get_future().share();
  auto ready=entered.get_future();auto finished=completed.get_future();
  std::atomic<bool> first_final_ran{false},later_final_ran{false};
  worker.submit_latest([&]{entered.set_value();releasing.wait();});
  ready.wait();
  const auto first=worker.submit_latest([&]{first_final_ran=true;});
  const auto later=worker.submit_latest([&]{later_final_ran=true;completed.set_value();});
  release.set_value();finished.wait();worker.stop();
  report["first_final_accepted"]=first.accepted;
  report["later_replaced_pending"]=later.replaced_pending;
  report["first_final_executed"]=first_final_ran.load();
  report["later_final_executed"]=later_final_ran.load();
  report["authority"]=false;
  std::ofstream(argv[1])<<report<<'\n';std::cout<<report<<'\n';
  if(final.status!=a::RecordStatus::Written || !later.replaced_pending || first_final_ran)return 3;
}
