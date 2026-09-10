#pragma once
#include "interval_model.hpp"
#include "multi_purpose_mpc_ros/recovery_footprint.hpp"

namespace enclosure {
namespace recovery=multi_purpose_mpc_ros::recovery_footprint;
struct FootprintBox {recovery::Pose2D pose;recovery::FootprintExtents extents;};
inline FootprintBox footprint(const Box& states,const recovery::FootprintExtents& original){
 if(!original.valid())throw std::runtime_error("invalid original footprint");
 FootprintBox out;
 out.pose={states[0].lo+(states[0].hi-states[0].lo)/2,
   states[1].lo+(states[1].hi-states[1].lo)/2,
   states[2].lo+(states[2].hi-states[2].lo)/2};
 const I c=cosine(I(out.pose.yaw_rad)),sn=sine(I(out.pose.yaw_rad));
 const I dx=states[0]-I(out.pose.x_m),dy=states[1]-I(out.pose.y_m);
 const I forward=c*dx+sn*dy,left=-sn*dx+c*dy;
 const I angle=states[2]-I(out.pose.yaw_rad),ca=cosine(angle),sa=sine(angle);
 // Include the original margin before rotating each corner. The returned
 // zero margin represents that already-expanded rectangle, not a relaxation.
 const I front=I(original.front_extent_m)+I(original.margin_m);
 const I rear=-I(original.rear_extent_m)-I(original.margin_m);
 const I lhs=I(original.left_extent_m)+I(original.margin_m);
 const I rhs=-I(original.right_extent_m)-I(original.margin_m);
 for(const I x:{front,rear})for(const I y:{lhs,rhs}){
  const I f=forward+ca*x-sa*y,l=left+sa*x+ca*y;
  out.extents.front_extent_m=std::max(out.extents.front_extent_m,f.hi);
  out.extents.rear_extent_m=std::max(out.extents.rear_extent_m,-f.lo);
  out.extents.left_extent_m=std::max(out.extents.left_extent_m,l.hi);
  out.extents.right_extent_m=std::max(out.extents.right_extent_m,-l.lo);
 }
 if(!out.extents.valid())throw std::runtime_error("invalid enclosed footprint");
 return out;
}
inline Box segment_box(Box first,const Box&last){for(size_t i=0;i<first.size();++i)first[i]=hull(first[i],last[i]);return first;}
}
