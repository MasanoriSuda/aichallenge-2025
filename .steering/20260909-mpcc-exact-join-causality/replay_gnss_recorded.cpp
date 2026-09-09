#include "gnss_poser/gnss_poser_core.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
int main(int argc,char **argv) {
  if(argc!=3)return 2;
  const auto inputs=YAML::LoadFile(argv[1]);
  rclcpp::init(0,nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("base_frame","replay_base"),
    rclcpp::Parameter("gnss_frame","replay_antenna"),rclcpp::Parameter("gnss_base_frame","replay_output"),
    rclcpp::Parameter("coordinate_system",1),rclcpp::Parameter("buff_epoch",1),
    rclcpp::Parameter("use_gnss_ins_orientation",false),rclcpp::Parameter("gnss_change_threshold",.2),
    rclcpp::Parameter("unknown_position_covariance",.1)});
  auto poser=std::make_shared<gnss_poser::GNSSPoser>(options);
  auto peer=std::make_shared<rclcpp::Node>("recorded_gnss_peer");
  geometry_msgs::msg::PoseStamped::SharedPtr received;
  auto sub=peer->create_subscription<geometry_msgs::msg::PoseStamped>("gnss_pose",10,[&](geometry_msgs::msg::PoseStamped::SharedPtr p){received=p;});
  auto pub=peer->create_publisher<sensor_msgs::msg::NavSatFix>("fix",10);
  tf2_ros::StaticTransformBroadcaster broadcaster(peer);
  geometry_msgs::msg::TransformStamped transform;transform.header.frame_id="replay_base";transform.child_frame_id="replay_antenna";
  transform.transform.translation.x=-.26;transform.transform.rotation.w=1.;broadcaster.sendTransform(transform);
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(poser);executor.add_node(peer);
  YAML::Node report;report["input"]=argv[1];report["authority"]=false;
  report["limits"]="Actual repaired node, original NavSatFix source stamps/coordinates and fixed local antenna extrinsics; independent offline replay, no EKF or dynamic race acceptance. Initial heading before sufficient displacement remains unobserved.";
  for(const auto & row:inputs) {
    const auto n=row["fix"]["message"];sensor_msgs::msg::NavSatFix fix;
    fix.header.frame_id="gnss_link";fix.header.stamp.sec=n["header"]["stamp"]["sec"].as<int>();fix.header.stamp.nanosec=n["header"]["stamp"]["nanosec"].as<unsigned>();
    fix.status.status=n["status"]["status"].as<int>();fix.status.service=n["status"]["service"].as<unsigned>();
    fix.latitude=n["latitude"].as<double>();fix.longitude=n["longitude"].as<double>();fix.altitude=n["altitude"].as<double>();
    fix.position_covariance_type=n["position_covariance_type"].as<unsigned>();
    for(std::size_t i=0;i<9;++i)fix.position_covariance[i]=n["position_covariance"][i].as<double>();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    do {pub->publish(fix);executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    while((!received || received->header.stamp!=fix.header.stamp)&&std::chrono::steady_clock::now()<deadline);
    if(!received || received->header.stamp!=fix.header.stamp)throw std::runtime_error("recorded GNSS replay missing output");
    YAML::Node value;value["stamp"]=n["header"]["stamp"];value["original"]=row["original"];
    const auto & p=received->pose;value["x"]=p.position.x;value["y"]=p.position.y;
    value["yaw"]=std::atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),1-2*(p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z));
    report["outputs"].push_back(value);
  }
  YAML::Emitter e;e.SetDoublePrecision(std::numeric_limits<double>::max_digits10);e<<report;
  std::ofstream(argv[2])<<e.c_str()<<'\n';std::cout<<report["outputs"].size()<<" fixes replayed\n";
  rclcpp::shutdown();
}
