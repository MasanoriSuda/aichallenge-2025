#pragma once
// Diagnostic numerical enclosure of the native midpoint body map. No runtime
// authority, receiver bound, or contact assumption is introduced by this file.
#include "multi_purpose_mpc_ros/mpcc_vehicle_model.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>
namespace enclosure {
namespace model=multi_purpose_mpc_ros::mpcc_vehicle_model;
struct I { double lo{},hi{}; I()=default; I(double v):lo(v),hi(v){} I(double l,double h):lo(l),hi(h){} };
inline double adjacent(double x,bool upward){
 static_assert(std::numeric_limits<double>::is_iec559 && sizeof(double)==sizeof(std::uint64_t));
 if(std::isnan(x) || x==(upward?INFINITY:-INFINITY))return x;
 if(x==0)return upward?std::numeric_limits<double>::denorm_min():-std::numeric_limits<double>::denorm_min();
 std::uint64_t bits;std::memcpy(&bits,&x,sizeof x);
 if((x>0)==upward)++bits;else --bits;
 std::memcpy(&x,&bits,sizeof x);return x;
}
inline double down(double x){return adjacent(x,false);}
inline double up(double x){return adjacent(x,true);}
inline I operator+(I a,I b){if(a.lo==0 && a.hi==0)return b;if(b.lo==0 && b.hi==0)return a;return {down(a.lo+b.lo),up(a.hi+b.hi)};}
inline I operator-(I a,I b){return {down(a.lo-b.hi),up(a.hi-b.lo)};}
inline I operator-(I a){return {-a.hi,-a.lo};}
inline I operator*(I a,I b){if((a.lo==0 && a.hi==0)||(b.lo==0 && b.hi==0))return I(0);const std::array<double,4> p{a.lo*b.lo,a.lo*b.hi,a.hi*b.lo,a.hi*b.hi};return {down(*std::min_element(p.begin(),p.end())),up(*std::max_element(p.begin(),p.end()))};}
inline I operator/(I a,double b){if(b==0)throw std::runtime_error("zero divisor");return a*I(down(1/b),up(1/b));}
inline I hull(I a,I b){return {std::min(a.lo,b.lo),std::max(a.hi,b.hi)};}
inline I sine(I a){if(a.hi-a.lo>=2*M_PI)return {-1,1};double l=std::min(std::sin(a.lo),std::sin(a.hi)),h=std::max(std::sin(a.lo),std::sin(a.hi));if(std::ceil((a.lo-M_PI/2)/(2*M_PI))<=std::floor((a.hi-M_PI/2)/(2*M_PI)))h=1;if(std::ceil((a.lo+M_PI/2)/(2*M_PI))<=std::floor((a.hi+M_PI/2)/(2*M_PI)))l=-1;return {std::max(-1.,down(down(l))),std::min(1.,up(up(h)))};}
inline I cosine(I a){if(a.hi-a.lo>=2*M_PI)return {-1,1};double l=std::min(std::cos(a.lo),std::cos(a.hi)),h=std::max(std::cos(a.lo),std::cos(a.hi));if(std::ceil(a.lo/(2*M_PI))<=std::floor(a.hi/(2*M_PI)))h=1;if(std::ceil((a.lo-M_PI)/(2*M_PI))<=std::floor((a.hi-M_PI)/(2*M_PI)))l=-1;return {std::max(-1.,down(down(l))),std::min(1.,up(up(h)))};}
constexpr size_t N=7; // yaw/body/steering plus input; Cartesian translation is exact
struct J {I v;std::array<I,N> d{};J()=default;J(double x):v(x){}J(I x):v(x){};};
inline J operator+(const J&a,const J&b){J r(a.v+b.v);for(size_t i=0;i<N;++i)r.d[i]=a.d[i]+b.d[i];return r;}
inline J operator-(const J&a,const J&b){J r(a.v-b.v);for(size_t i=0;i<N;++i)r.d[i]=a.d[i]-b.d[i];return r;}
inline J operator-(const J&a){J r(-a.v);for(size_t i=0;i<N;++i)r.d[i]=-a.d[i];return r;}
inline J operator*(const J&a,const J&b){J r(a.v*b.v);for(size_t i=0;i<N;++i)r.d[i]=a.d[i]*b.v+a.v*b.d[i];return r;}
inline J operator/(const J&a,double b){J r(a.v/b);for(size_t i=0;i<N;++i)r.d[i]=a.d[i]/b;return r;}
inline J sin(const J&a){J r(sine(a.v));for(size_t i=0;i<N;++i)r.d[i]=cosine(a.v)*a.d[i];return r;}
inline J cos(const J&a){J r(cosine(a.v));for(size_t i=0;i<N;++i)r.d[i]=-sine(a.v)*a.d[i];return r;}
inline J clamp(J a,double lo,double hi){if(a.v.hi<=lo)return J(lo);if(a.v.lo>=hi)return J(hi);if(a.v.lo<lo || a.v.hi>hi)for(auto& d:a.d)d=hull(d,I(0));a.v={std::max(a.v.lo,lo),std::min(a.v.hi,hi)};return a;}
inline J max(const J&a,const J&b){if(a.v.lo>=b.v.hi)return a;if(b.v.lo>=a.v.hi)return b;J r(I(std::max(a.v.lo,b.v.lo),std::max(a.v.hi,b.v.hi)));for(size_t i=0;i<N;++i)r.d[i]=hull(a.d[i],b.d[i]);return r;}
inline I bounds(I a){return a;}
inline I bounds(const J&a){return a.v;}
inline I sin(I a){return sine(a);}
inline I cos(I a){return cosine(a);}
inline I clamp(I a,double lo,double hi){return {std::clamp(a.lo,lo,hi),std::clamp(a.hi,lo,hi)};}
inline I max(I a,I b){return {std::max(a.lo,b.lo),std::max(a.hi,b.hi)};}
inline I unite(I a,I b){return hull(a,b);}
inline J unite(const J&a,const J&b){J r(hull(a.v,b.v));for(size_t i=0;i<N;++i)r.d[i]=hull(a.d[i],b.d[i]);return r;}
using JS=std::array<J,8>;using Box=std::array<I,8>;
template<class T> inline std::array<T,8> increment(std::array<T,8> s,const std::array<T,6>&d,double dt){for(size_t i=0;i<6;++i)s[i]=s[i]+d[i]*T(dt);return s;}
template<class T> inline std::array<T,6> derivative(const std::array<T,8>&s,T wire,const model::Parameters&p,double dt,bool * discontinuous){
 const T u=s[3],v=s[4],r=s[5];const double interval=std::max(dt,p.minimum_force_interval_sec);
 T a=clamp(wire,-p.maximum_wire_deceleration_mps2,p.maximum_wire_acceleration_mps2);
 if(bounds(u).hi<0)a=max(a,-u/interval);
 else if(bounds(u).lo<0){
  const T united=unite(a,max(a,-u/interval));
  if(discontinuous && bounds(wire).lo<0)*discontinuous=true;
  a=united;
 }
 a=clamp(a,-p.maximum_wire_deceleration_mps2,p.maximum_wire_acceleration_mps2);
 const T rolling=clamp(-u/interval,-p.rolling_mps2,p.rolling_mps2);
 T fx,fy,moment;
 for(const auto&w:p.wheels){const T angle=w.steerable?s[7]:T(0),c=cos(angle),sn=sin(angle);
  const T slip=-sn*(u-r*T(w.com_left_m))+c*(v+r*T(w.com_forward_m));
  const T lateral=T(-w.cornering_per_sec)*slip,drive=T(w.traction_fraction)*(a+rolling);
  const T ax=c*drive-sn*lateral,ay=sn*drive+c*lateral;fx=fx+ax;fy=fy+ay;moment=moment+T(w.com_forward_m)*ay-T(w.com_left_m)*ax;
 }
 const T forward=u+r*T(p.com_left_m),left=v-r*T(p.com_forward_m),c=cos(s[2]),sn=sin(s[2]);
 return {c*forward-sn*left,sn*forward+c*left,r,fx-T(p.drag_per_sec)*u+r*v,fy-T(p.drag_per_sec)*v-r*u,moment*T(p.mass_kg)/p.yaw_inertia_kgm2-T(p.angular_drag_per_sec)*r};
}
template<class T> inline std::array<T,8> map(std::array<T,8> s,T a,const model::Parameters&p,double dt,bool rest,bool * discontinuous=nullptr){
 const T demand=T(p.tire_grip)*clamp(s[6]*T(p.steering_wire_gain),-p.maximum_wire_steering_rad,p.maximum_wire_steering_rad);
 s[7]=s[7]+clamp(T(dt/(p.tire_lag_sec+dt))*(demand-s[7]),-p.tire_slew_radps*dt,p.tire_slew_radps*dt);
 if(rest){s[3]=T(0);s[4]=T(0);s[5]=T(0);return s;}
 const auto d1=derivative(s,a,p,dt,discontinuous);const auto mid=increment(s,d1,.5*dt);
 return increment(s,derivative(mid,a,p,dt,discontinuous),dt);
}
inline Box centered_step(const Box&b,I acceleration,const model::Parameters&p,double dt,bool rest){
 JS inputs;Box center;std::array<I,N> offsets{};
 for(size_t i=2;i<8;++i){const double c=b[i].lo+(b[i].hi-b[i].lo)/2;inputs[i]=J(b[i]);inputs[i].d[i-2]=I(1);center[i]=I(c);offsets[i-2]=b[i]-I(c);}
 const double ac=acceleration.lo+(acceleration.hi-acceleration.lo)/2;J a(acceleration);a.d[6]=I(1);offsets[6]=acceleration-I(ac);
 bool discontinuous=false;
 const auto range=map(inputs,a,p,dt,rest,&discontinuous);const auto point=map(center,I(ac),p,dt,rest);Box result;
 for(size_t i=0;i<8;++i){I delta;for(size_t j=0;j<N;++j)delta=delta+range[i].d[j]*offsets[j];result[i]=discontinuous?range[i].v:point[i]+delta;
  if(i<2)result[i]=b[i]+result[i];
  if(rest && i>=3 && i<=5)result[i]=I(0);
  if(!std::isfinite(result[i].lo)||!std::isfinite(result[i].hi))throw std::runtime_error("unbounded numerical enclosure");}
 return result;
}
inline std::vector<Box> step_parts(const Box&b,I a,const model::Parameters&p,double dt){
 if(!model::valid(p)||!p.nominal_settled_contact||dt<=0||model::integration_steps(dt,p.maximum_step_sec)!=1)throw std::runtime_error("unsupported enclosure context");
 std::vector<Box> parts;
 const auto add=[&](double low,double high,I wire,bool rest){Box q=b;q[3]={std::max(b[3].lo,low),std::min(b[3].hi,high)};if(q[3].lo>q[3].hi||wire.lo>wire.hi)return;parts.push_back(centered_step(q,wire,p,dt,rest));};
 if(a.lo<=0){const I wire(a.lo,std::min(0.,a.hi));add(-p.sleep_speed_mps,p.sleep_speed_mps,wire,true);add(-INFINITY,-p.sleep_speed_mps,wire,false);add(p.sleep_speed_mps,INFINITY,wire,false);}
 if(a.hi>0){const I wire(std::max(0.,a.lo),a.hi);add(-INFINITY,0,wire,false);add(0,INFINITY,wire,false);}
 if(parts.empty())throw std::runtime_error("empty branch enclosure");return parts;
}
inline Box joined(const std::vector<Box>&parts){
 if(parts.empty())throw std::runtime_error("empty body population");
 Box out=parts.front();for(size_t k=1;k<parts.size();++k)for(size_t i=0;i<8;++i)out[i]=hull(out[i],parts[k][i]);return out;
}
inline bool at_rest(const Box&s){return s[3].lo==0 && s[3].hi==0 && s[4].lo==0 && s[4].hi==0 && s[5].lo==0 && s[5].hi==0;}
inline Box step(const Box&b,I a,const model::Parameters&p,double dt){return joined(step_parts(b,a,p,dt));}
inline std::vector<Box> advance_partitioned(std::vector<Box> states,I a,const model::Parameters&p,double duration,Box* swept){
 const auto count=model::integration_steps(duration,p.maximum_step_sec);
 if(count==0)throw std::runtime_error("invalid partitioned duration");
 if(swept)*swept=joined(states);
 const auto merge=[](std::optional<Box>&to,const Box&from){if(!to)to=from;else for(size_t i=0;i<8;++i)(*to)[i]=hull((*to)[i],from[i]);};
 for(size_t k=0;k<count;++k){
   // Rest is a separate hybrid mode. Merging it with moving states before
   // the next map invents combinations of zero u and moving vy/r, which can
   // make a valid enclosure useless. Splitting/merging changes no trajectory.
   std::array<std::optional<Box>,5> bins;
   const std::array<double,5> boundaries{-INFINITY,-p.sleep_speed_mps,0,p.sleep_speed_mps,INFINITY};
   for(const auto&state:states)for(const auto&next:step_parts(state,a,p,duration/count)){
     if(at_rest(next)){merge(bins[0],next);continue;}
     for(size_t j=0;j<4;++j){Box clipped=next;clipped[3]={std::max(next[3].lo,boundaries[j]),std::min(next[3].hi,boundaries[j+1])};if(clipped[3].lo<=clipped[3].hi)merge(bins[j+1],clipped);}
   }
   states.clear();for(const auto&bin:bins)if(bin)states.push_back(*bin);
   if(swept){const auto all=joined(states);for(size_t i=0;i<8;++i)(*swept)[i]=hull((*swept)[i],all[i]);}
 }
 return states;
}
inline Box advance(Box state,I acceleration,const model::Parameters&p,double duration,Box* swept=nullptr){
 if(!std::isfinite(duration)||duration<0)throw std::runtime_error("invalid range duration");
 if(swept)*swept=state;
 if(duration==0)return state;
 const auto count=model::integration_steps(duration,p.maximum_step_sec);
 if(count==0)throw std::runtime_error("invalid range step count");
 const double dt=duration/static_cast<double>(count);
 for(size_t i=0;i<count;++i){state=step(state,acceleration,p,dt);if(swept)for(size_t j=0;j<state.size();++j)(*swept)[j]=hull((*swept)[j],state[j]);}
 return state;
}
inline std::array<double,8> values(const model::State&s){return {s.x_m,s.y_m,s.yaw_rad,s.forward_velocity_mps,s.lateral_velocity_mps,s.yaw_rate_radps,s.desired_steering_rad,s.tire_steering_rad};}
inline Box point(const model::State&s){Box b;auto v=values(s);for(size_t i=0;i<8;++i)b[i]=I(v[i]);return b;}
}
