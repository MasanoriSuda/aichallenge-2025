#ifdef PARTICIPANT_EKF
#include <aichallenge_ekf_localizer/ekf_localizer.hpp>
#include <aichallenge_ekf_localizer/state_index.hpp>
#else
#include <ekf_localizer/ekf_localizer.hpp>
#include <ekf_localizer/state_index.hpp>
#endif
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <dlfcn.h>
#include <fstream>
#include <iostream>
namespace {
thread_local rclcpp::Clock * frozen_clock = nullptr;
thread_local std::int64_t frozen_ns = 0;
}
rclcpp::Time rclcpp::Clock::now() {
 if(this==frozen_clock)return rclcpp::Time(frozen_ns,RCL_ROS_TIME);
 using Fn=rclcpp::Time (*)(rclcpp::Clock *);
 static auto fn=reinterpret_cast<Fn>(dlsym(RTLD_NEXT,"_ZN6rclcpp5Clock3nowEv"));
 if(!fn)throw std::runtime_error("Clock::now unavailable");
 return fn(this);
}
void header(const YAML::Node & n,std_msgs::msg::Header & h) {
 h.frame_id=n["frame_id"].as<std::string>();h.stamp.sec=n["stamp"]["sec"].as<std::int32_t>();h.stamp.nanosec=n["stamp"]["nanosec"].as<std::uint32_t>();
}
void pose(const YAML::Node & n,geometry_msgs::msg::PoseWithCovariance & p) {
 const auto a=n["pose"];p.pose.position.x=a["position"]["x"].as<double>();p.pose.position.y=a["position"]["y"].as<double>();p.pose.position.z=a["position"]["z"].as<double>();
 const auto q=a["orientation"];p.pose.orientation.x=q["x"].as<double>();p.pose.orientation.y=q["y"].as<double>();p.pose.orientation.z=q["z"].as<double>();p.pose.orientation.w=q["w"].as<double>();
 for(std::size_t i=0;i<36;++i)p.covariance[i]=n["covariance"][i].as<double>();
}
void twist(const YAML::Node & n,geometry_msgs::msg::TwistWithCovariance & p) {
 const auto a=n["twist"];p.twist.linear.x=a["linear"]["x"].as<double>();p.twist.linear.y=a["linear"]["y"].as<double>();p.twist.linear.z=a["linear"]["z"].as<double>();
 p.twist.angular.x=a["angular"]["x"].as<double>();p.twist.angular.y=a["angular"]["y"].as<double>();p.twist.angular.z=a["angular"]["z"].as<double>();
 for(std::size_t i=0;i<36;++i)p.covariance[i]=n["covariance"][i].as<double>();
}
class EKFLocalizerTestSuite {
public:
 static void run(EKFLocalizer & n,const YAML::Node & rows,const char * destination) {
  n.timer_control_->cancel();n.timer_tf_->cancel();bool initialised=false;YAML::Node result(YAML::NodeType::Sequence);
  for(const auto & row:rows) {
   const auto kind=row["kind"].as<std::string>();const auto m=row["message"];
   if(kind=="odom") {
    const auto sec=m["header"]["stamp"]["sec"].as<std::int64_t>();const auto nano=m["header"]["stamp"]["nanosec"].as<std::int64_t>();
    const auto ns=sec*1000000000LL+nano;
    if(!initialised) {
     geometry_msgs::msg::PoseWithCovariance p;pose(m["pose"],p);geometry_msgs::msg::TwistWithCovariance t;twist(m["twist"],t);
     Eigen::MatrixXd x=Eigen::MatrixXd::Zero(6,1);x(IDX::X)=p.pose.position.x;x(IDX::Y)=p.pose.position.y;
     const auto q=p.pose.orientation;x(IDX::YAW)=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));x(IDX::VX)=t.twist.linear.x;x(IDX::WZ)=t.twist.angular.z;
     n.ekf_.init(x,Eigen::MatrixXd::Identity(6,6)*0.01,n.params_.extend_state_step);
     const rclcpp::Time epoch(ns,RCL_ROS_TIME);n.last_predict_time_=std::make_shared<const rclcpp::Time>(epoch);
     n.z_filter_.init(p.pose.position.z,0.01,epoch);n.roll_filter_.init(0,0.01,epoch);n.pitch_filter_.init(0,0.01,epoch);n.is_activated_=true;initialised=true;continue;
    }
    if(ns<n.last_predict_time_->nanoseconds())throw std::runtime_error("unexpected input clock regression");
    frozen_ns=ns;frozen_clock=n.get_clock().get();n.timerCallback();frozen_clock=nullptr;
    YAML::Node item;item["stamp_ns"]=ns;auto state=n.ekf_.getLatestX();auto covariance=n.ekf_.getLatestP();
    for(int i=0;i<6;++i)item["state"].push_back(state(i));
    for(int i=0;i<6;++i)for(int j=0;j<6;++j)item["covariance"].push_back(covariance(i,j));
    result.push_back(item);
   } else if(initialised && kind=="pose") {
    auto p=std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();header(m["header"],p->header);pose(m["pose"],p->pose);n.callbackPoseWithCovariance(p);
   } else if(initialised && kind=="twist") {
    auto t=std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();header(m["header"],t->header);twist(m["twist"],t->twist);n.callbackTwistWithCovariance(t);
   }
  }
  YAML::Emitter emitter;emitter.SetDoublePrecision(17);emitter<<result;std::ofstream(destination)<<emitter.c_str()<<'\n';
  std::cout<<"Replayed "<<result.size()<<" observed-clock ticks with shared initial state\n";
 }
};
int main(int argc,char **argv) {
 if(argc!=3)return 2;const auto input=YAML::LoadFile(argv[1]);rclcpp::init(argc,argv);
 rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("enable_yaw_bias_estimation",false),rclcpp::Parameter("pose_smoothing_steps",1),rclcpp::Parameter("twist_smoothing_steps",1),rclcpp::Parameter("tf_rate",30.0)});
 auto node=std::make_shared<EKFLocalizer>("ekf_parity_replay",options);EKFLocalizerTestSuite::run(*node,input,argv[2]);node.reset();rclcpp::shutdown();
}
