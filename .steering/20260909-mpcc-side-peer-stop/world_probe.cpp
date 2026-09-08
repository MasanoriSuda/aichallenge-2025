// Offline architecture comparison of common Cartesian support linearization.
#include "multi_purpose_mpc_ros/mpcc_architecture_comparison.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"
// Reuse the exact native row/artifact and wall-snapshot builders in this
// standalone TU. No production file or library is edited by this probe.
#include "mpcc_architecture_comparison.cpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace m = multi_purpose_mpc_ros;
namespace arch = m::mpcc_architecture_snapshot;
namespace cmp = m::mpcc_architecture_comparison;
namespace osqp = m::persistent_osqp;
using State = Eigen::Matrix<double,7,1>;
YAML::Node vec(const Eigen::VectorXd & x) {
  YAML::Node n; for (int i=0;i<x.size();++i) n.push_back(x[i]); return n;
}
struct Pose {Eigen::Vector2d xy; double heading;};
Pose pose(const m::mpcc_rate_resolved_shadow::Snapshot & s,const State & x) {
  const auto f = m::mpc_stage_geometry::sample_course_frame(s.wall_course_frame_knots,s.course_progress_origin_m+x[4]);
  if (!f) throw std::runtime_error("course frame unavailable");
  const double c=std::cos(f->heading_rad),sn=std::sin(f->heading_rad);
  return {{f->x_m+c*x[1]-sn*x[0],f->y_m+sn*x[1]+c*x[0]},f->heading_rad+x[2]};
}
double support(const m::mpcc_rate_resolved_shadow::Snapshot & s,double heading,const Eigen::Vector2d & n) {
  const auto & b=s.replay_world->physical_footprint;
  const double f=std::cos(heading)*n[0]+std::sin(heading)*n[1];
  const double l=-std::sin(heading)*n[0]+std::cos(heading)*n[1];
  return std::abs(f)*((f>=0?b.front_extent_m:b.rear_extent_m)+b.margin_m)+
    std::abs(l)*((l>=0?b.left_extent_m:b.right_extent_m)+b.margin_m);
}
int main(int argc,char **argv) {
  if (argc!=3 && argc!=4) return 2;
  const bool equality_objective=argc==4;
  const bool cartesian=argc!=4 || std::string(argv[3])=="world";
  std::string detail;
  const auto record=arch::load_recorded_interaction_snapshot(argv[1],&detail);
  if (!record || !record->recorded_qp || !record->assembly_request || !record->recorded_qp->warm_start)
    throw std::runtime_error("complete failed QP needed/"+detail);
  const auto & source=record->source;
  if (source.replay_world->obstacles.size()!=1) throw std::runtime_error("one sealed peer required");
  const auto & peer=source.replay_world->obstacles.front();
  const auto & b=source.replay_world->physical_footprint;
  YAML::Node report;
  report["scope"]="offline-only common Cartesian plane linearization; unchanged hard constraints/model/objective";
  report["source_fingerprint"]=record->interaction_fingerprint;
  report["cartesian_rows"]=cartesian;
  report["equality_residual_objective"]=equality_objective;
  for (const bool equil : {false,true}) for (const bool warm : {false,true}) {
    auto assembly=*record->assembly_request;
    Eigen::VectorXd primal=record->recorded_qp->warm_start->primal;
    const std::string name=std::string(equil?"equilibrated":"normalized")+(warm?"-warm":"-cold");
    for (int iteration=0;iteration<8;++iteration) {
      YAML::Node step; step["iteration"]=iteration;
      if (iteration>0) {
        const auto rel=m::mpcc_rate_resolved_adapter::relinearize_around_primal(source.request,primal,assembly);
        if (!rel.applied) { step["detail"]=m::mpcc_rate_resolved_adapter::to_string(rel.reason);report[name].push_back(step);break; }
      }
      auto qp=m::mpcc_rate_resolved_problem::assemble(assembly);
      if (!qp) throw std::runtime_error("assembly rejected");
      const int n=source.request.horizon_steps;
      const int first=qp->constraints.rows()-assembly.dynamic_obstacle_constraints.size();
      Eigen::MatrixXd A(qp->constraints);
      if (static_cast<int>(assembly.dynamic_obstacle_constraints.size())!=n) throw std::runtime_error("one dynamic row per stage needed");
      double elapsed=source.control_prediction_origin_sec-source.replay_world->observed_sec;
      for (int k=0;cartesian && k<n;++k) {
        elapsed+=source.request.inputs[k].stage_dt_sec;
        State x=primal.segment<7>((k+1)*7);
        const auto p=pose(source,x);
        Eigen::Vector2d target{peer.x_m+elapsed*peer.velocity_x_mps,peer.y_m+elapsed*peer.velocity_y_mps};
        const auto relative=(target-p.xy).eval();
        const double c=std::cos(p.heading),s=std::sin(p.heading);
        Eigen::Vector2d body{c*relative[0]+s*relative[1],-s*relative[0]+c*relative[1]};
        Eigen::Vector2d gap{body[0]-std::clamp(body[0],-b.rear_extent_m-b.margin_m,b.front_extent_m+b.margin_m),
          body[1]-std::clamp(body[1],-b.right_extent_m-b.margin_m,b.left_extent_m+b.margin_m)};
        if (gap.norm()==0) throw std::runtime_error("peer center inside body");
        gap.normalize(); Eigen::Vector2d normal{c*gap[0]-s*gap[1],s*gap[0]+c*gap[1]};
        const auto residual=[&](const State & v) {auto q=pose(source,v);return normal.dot(q.xy-target)+support(source,q.heading,normal)+peer.radius_m;};
        const double f=residual(x);
        State gradient=State::Zero();
        for (int j : {0,1,2,4}) {
          const double h=1e-5; State plus=x,minus=x;plus[j]+=h;minus[j]-=h;
          gradient[j]=(residual(plus)-residual(minus))/(2*h);
        }
        A.row(first+k).setZero(); A.block<1,7>(first+k,(k+1)*7)=gradient.transpose();
        qp->lower_bound[first+k]=-std::numeric_limits<double>::infinity();
        qp->upper_bound[first+k]=gradient.dot(x)-f;
        if (iteration==0) {
          step["witness_residual_m"].push_back(f);
          step["state_gradient"].push_back(vec(gradient));
        }
      }
      qp->constraints=A.sparseView();
      // Exact-feasible-set preserving objective: every added residual is an
      // existing hard equality. Its value is identically zero on the original
      // feasible set. No constraint relaxation or racing tradeoff is added.
      if (equality_objective) {
        Eigen::MatrixXd residual=Eigen::MatrixXd::Zero(A.rows(),A.cols());
        Eigen::VectorXd target=Eigen::VectorXd::Zero(A.rows());
        for (int row=0;row<A.rows();++row) {
          if (!std::isfinite(qp->lower_bound[row]) || qp->lower_bound[row]!=qp->upper_bound[row]) continue;
          const double scale=1.0/(1.0+std::abs(qp->lower_bound[row]));
          residual.row(row)=scale*A.row(row);target[row]=scale*qp->lower_bound[row];
        }
        Eigen::MatrixXd cost=2*residual.transpose()*residual;
        cost.triangularView<Eigen::StrictlyLower>().setZero();
        qp->quadratic_cost+=cost.sparseView();
        qp->linear_cost-=2*residual.transpose()*target;
      }
      osqp::PersistentOsqpSolver solver(equil ? osqp::ConstraintPreconditioningPolicy::RowToleranceNormalizedWithInternalEquilibration : osqp::ConstraintPreconditioningPolicy::RowToleranceNormalized);
      const std::optional<osqp::WarmStart> seed=warm ? std::optional<osqp::WarmStart>{{primal,Eigen::VectorXd::Zero(qp->constraints.rows())}} : std::nullopt;
      const auto solved=solver.solve(qp->quadratic_cost,qp->constraints,qp->linear_cost,qp->lower_bound,qp->upper_bound,seed,qp->variable_scaling);
      step["solved"]=solved.result.has_value();step["detail"]=solved.failure_detail;step["iterations"]=solved.telemetry.iterations;
      step["status"]=solved.telemetry.status;step["solve_ms"]=solved.telemetry.total_ms;
      if (!solved.result) { report[name].push_back(step);break; }
      primal=solved.result->primal;step["primal"]=vec(primal);
      auto current=*record;current.assembly_request=assembly;current.recorded_qp->problem=*qp;
      const auto proof=cmp::verify_external_primal(current,primal,cmp::ExternalPrimalConstraintPolicy::ExactRecorded);
      bool accepted=false;
      if (!proof.arms.empty()) {
        step["proof_stage"]=cmp::to_string(proof.arms[0].stage);step["proof_detail"]=proof.arms[0].detail;
        accepted=proof.arms[0].bundle.has_value();
      }
      // A full-horizon solved Stop is its own contingency. The generic
      // external-primal arm above instead asks for a new racing-line Stop.
      // Apply the actual Stop-lattice native proof contract separately.
      const auto built=cmp::build_external_artifact(source,*qp,primal,cmp::ExternalPrimalConstraintPolicy::ExactRecorded);
      if (built.value) {
        const auto physical=m::mpcc_rate_resolved_physical_adapter::build(*built.value,source.identity.source_context.intent,source.identity.source_context.stage_geometry_id);
        if (physical.exact_trajectory) {
          const auto & exact=*physical.exact_trajectory;
          auto world_snapshot=cmp::wall_snapshot(source,*source.replay_world,exact);
          const auto wall=m::mpcc_rate_resolved_physical_wall::evaluate(world_snapshot);
          const auto dynamic=m::mpcc_rate_resolved_dynamic_proof::evaluate_current_world(source,world_snapshot);
          const bool rest=!exact.velocity_mps.empty() && exact.velocity_mps.back()<=std::max(1e-9,built.value->physical_global_tolerance);
          step["stop_proof"]["wall"]=m::mpcc_rate_resolved_physical_wall::to_string(wall.outcome);
          step["stop_proof"]["dynamic_clear"]=dynamic.valid && dynamic.clear;
          step["stop_proof"]["minimum_clearance_m"]=dynamic.minimum_clearance_m;
          step["stop_proof"]["terminal_velocity_mps"]=exact.velocity_mps.back();
          step["stop_proof"]["rest_tolerance_mps"]=std::max(1e-9,built.value->physical_global_tolerance);
          const auto certified=m::mpcc_rate_resolved_certified_plan::build(
            std::make_shared<const m::mpcc_rate_resolved_execution_artifact::ExecutionArtifact>(*built.value),world_snapshot,wall,
            std::make_shared<const m::mpcc_rate_resolved_shadow::Snapshot>(source));
          step["stop_proof"]["certified_reason"]=m::mpcc_rate_resolved_certified_plan::to_string(certified.reason);
          accepted=rest && dynamic.valid && dynamic.clear && wall.outcome==m::mpcc_rate_resolved_physical_wall::Outcome::Accepted && certified.plan!=nullptr;
        }
      }
      step["candidate_fingerprint"]=cmp::exact_problem_fingerprint(*qp,record->interaction_fingerprint);
      if (accepted) {
        step["qp"]["lower_bound"]=vec(qp->lower_bound);
        step["qp"]["upper_bound"]=vec(qp->upper_bound);
        step["qp"]["linear_cost"]=vec(qp->linear_cost);
        step["qp"]["variable_scaling"]=vec(qp->variable_scaling.physical_units_per_solver_unit);
        for (const auto & item : {std::pair<const char *,const Eigen::SparseMatrix<double>*>{"constraints",&qp->constraints},{"quadratic_cost",&qp->quadratic_cost}}) {
          auto node=step["qp"][item.first];node["rows"]=item.second->rows();node["columns"]=item.second->cols();
          for (int c=0;c<item.second->outerSize();++c) for (Eigen::SparseMatrix<double>::InnerIterator e(*item.second,c);e;++e) {
            YAML::Node t;t.push_back(e.row());t.push_back(e.col());t.push_back(e.value());node["triplets"].push_back(t);
          }
        }
      }
      step["bundle"]=accepted;report[name].push_back(step);
      if (accepted) break;
    }
  }
  YAML::Emitter emitter; emitter.SetDoublePrecision(17);emitter<<report;
  std::ofstream out(argv[2]);out<<emitter.c_str()<<'\n';
  std::cout<<"saved "<<argv[2]<<std::endl;
}
