#define main preserved_native_driver_main
#include "../native_vehicle_model.cpp"
#undef main
#include "footprint_enclosure.hpp"
#include <chrono>
#include <random>

int main()
{
  const auto p=read_parameters();std::mt19937 rng(149356);std::uniform_real_distribution<double> unit(0,1);
  std::mt19937_64 bit_rng(9284147);size_t rounding_checks=0;
  const auto check_rounding=[&](double v){for(const bool up:{false,true}){const double actual=enclosure::adjacent(v,up),expected=std::nextafter(v,up?INFINITY:-INFINITY);require((std::isnan(actual)&&std::isnan(expected))||std::memcmp(&actual,&expected,sizeof v)==0,"adjacent differs from nextafter");++rounding_checks;}};
  for(const double v:std::array<double,9>{0.,-0.,INFINITY,-INFINITY,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::denorm_min(),-std::numeric_limits<double>::denorm_min(),std::numeric_limits<double>::max(),-std::numeric_limits<double>::max()})check_rounding(v);
  for(size_t i=0;i<100000;++i){const auto bits=bit_rng();double v;std::memcpy(&v,&bits,sizeof v);check_rounding(v);}
  std::cout<<"rounding_checks="<<rounding_checks<<std::endl;
  size_t corner_checks=0;
  const enclosure::recovery::FootprintExtents original{1.42,.52,.62,.62,.08};
  for(size_t i=0;i<1000;++i){
    auto box=enclosure::point({0,0,0,1,0,0,0,0});
    for(size_t j=0;j<3;++j){double c=20*unit(rng)-10,r=unit(rng)*(j==2?3.2:1.);box[j]={c-r,c+r};}
    const auto f=enclosure::footprint(box,original);
    for(size_t sample=0;sample<16;++sample){
      std::array<double,3> pose{};for(size_t j=0;j<3;++j)pose[j]=box[j].lo+(box[j].hi-box[j].lo)*(sample<8?static_cast<double>((sample>>j)&1):unit(rng));
      for(const double x:{original.front_extent_m+original.margin_m,-original.rear_extent_m-original.margin_m})for(const double y:{original.left_extent_m+original.margin_m,-original.right_extent_m-original.margin_m}){
        const double dx=pose[0]+std::cos(pose[2])*x-std::sin(pose[2])*y-f.pose.x_m,dy=pose[1]+std::sin(pose[2])*x+std::cos(pose[2])*y-f.pose.y_m;
        const double forward=std::cos(f.pose.yaw_rad)*dx+std::sin(f.pose.yaw_rad)*dy,left=-std::sin(f.pose.yaw_rad)*dx+std::cos(f.pose.yaw_rad)*dy;
        require(forward>=-f.extents.rear_extent_m && forward<=f.extents.front_extent_m && left>=-f.extents.right_extent_m && left<=f.extents.left_extent_m,"corner outside footprint enclosure");++corner_checks;
      }
    }
  }
  std::cout<<"footprint_corner_checks="<<corner_checks<<std::endl;
  size_t duration_checks=0,split_cases=0;
  for(size_t k=1;k<=2000;++k){
    const double start=k*.005,duration=(start+.005)-start;
    split_cases+=model::integration_steps(duration,p.maximum_step_sec)>1;
    for(const double u:{0.,.020,1.5,4.5})for(const double input:{-3.,1.37}){
      const model::State state{0,0,.2,u,.03,.1,.05,.05};
      const auto expected=model::advance(state,{input,0},p,duration);require(expected.has_value(),"native duration failed");
      const auto box=enclosure::advance(enclosure::point(state),enclosure::I(input),p,duration);const auto values=enclosure::values(expected->state);
      for(size_t i=0;i<8;++i){require(values[i]>=box[i].lo && values[i]<=box[i].hi,"duration native result outside enclosure");++duration_checks;}
    }
  }
  require(split_cases>0,"duration regression must cover native subdivision");
  std::cout<<"duration_checks="<<duration_checks<<" native_split_cases="<<split_cases<<std::endl;
  size_t point_checks=0,enclosed_samples=0;double max_point_width=0;
  for(size_t j=0;j<400;++j){
    model::State s{0,0,unit(rng)-.5,10*unit(rng),.2*(unit(rng)-.5),unit(rng)-.5,.4*(unit(rng)-.5),.4*(unit(rng)-.5)};
    const double a=-3+4.3296*unit(rng);const auto exact=model::advance(s,{a,0},p,.005);require(exact.has_value(),"point native failed");
    const auto box=enclosure::step(enclosure::point(s),enclosure::I(a),p,.005);const auto values=enclosure::values(exact->state);
    for(size_t i=0;i<8;++i){if(values[i]<box[i].lo || values[i]>box[i].hi){std::cerr<<"point outside j="<<j<<" coordinate="<<i<<" value="<<std::setprecision(17)<<values[i]<<" box="<<box[i].lo<<","<<box[i].hi<<"\n";return 1;}max_point_width=std::max(max_point_width,box[i].hi-box[i].lo);++point_checks;}
  }
  for(const double u:{-.021,-.020,-.019,0.,.019,.020,.021,2.})for(const double a:{-8.,-3.,0.,1e-9,1.37,2.}){
    const model::State state{0,0,.2,u,.03,.1,.05,.05};const auto exact=model::advance(state,{a,0},p,.005);require(exact.has_value(),"boundary native failed");
    const auto box=enclosure::step(enclosure::point(state),enclosure::I(a),p,.005);const auto values=enclosure::values(exact->state);
    for(size_t j=0;j<8;++j){if(values[j]<box[j].lo||values[j]>box[j].hi){std::cerr<<"boundary outside u="<<u<<" input="<<a<<" coordinate="<<j<<" value="<<std::setprecision(17)<<values[j]<<" box="<<box[j].lo<<","<<box[j].hi<<"\n";return 1;}++point_checks;}
  }
  std::cout<<"point_checks="<<point_checks<<" max_width="<<std::setprecision(17)<<max_point_width<<std::endl;
  for(const double speed:{1.5,4.,8.}){
    model::State initial{0,0,.2,speed,.03,.1,.05,.05};auto box=enclosure::point(initial);std::vector<model::State> samples(200,initial);
    const auto begin=std::chrono::steady_clock::now();int rest_tick=-1;double enclosure_ms=0,oracle_ms=0;
    for(size_t k=0;k<1200;++k){
      // All input values may vary independently inside the declared interval
      // for200ms; thereafter one common held Stop command. This is a numerical
      // overapproximation experiment, not a receipt/Update bound or authority.
      const enclosure::I wire=k<40?enclosure::I(-3,1.3296):enclosure::I(-3);
      const auto range_begin=std::chrono::steady_clock::now();
      box[6]=enclosure::I(.05);box=enclosure::step(box,wire,p,.005);
      enclosure_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-range_begin).count();
      const auto oracle_begin=std::chrono::steady_clock::now();
      for(size_t sample_index=0;sample_index<samples.size();++sample_index){auto & s=samples[sample_index];s.desired_steering_rad=.05;double a=k<40?-3+4.3296*unit(rng):-3;
        // Include extrema and abrupt transitions in addition to random interiors.
        if(k<40){if(sample_index==0)a=-3;if(sample_index==1)a=1.3296;if(sample_index==2)a=k%2?-3:1.3296;if(sample_index==3)a=k<20?-3:1.3296;if(sample_index==4)a=k<20?1.3296:-3;}
        const auto exact=model::advance(s,{a,0},p,.005);require(exact.has_value(),"sample native failed");s=exact->state;auto values=enclosure::values(s);
        for(size_t i=0;i<8;++i){if(values[i]<box[i].lo || values[i]>box[i].hi){std::cerr<<"sample outside speed="<<speed<<" tick="<<k<<" coordinate="<<i<<" value="<<std::setprecision(17)<<values[i]<<" box="<<box[i].lo<<","<<box[i].hi<<"\n";return 1;}++enclosed_samples;}
      }
      oracle_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-oracle_begin).count();
      if(k==39 || k==99 || k==199 || k==399 || k==799){std::cout<<"speed="<<speed<<" t="<<(k+1)*.005<<" x=["<<box[0].lo<<","<<box[0].hi<<"] y=["<<box[1].lo<<","<<box[1].hi<<"] u=["<<box[3].lo<<","<<box[3].hi<<"] r=["<<box[5].lo<<","<<box[5].hi<<"]"<<std::endl;}
      if(box[3].lo==0 && box[3].hi==0 && box[4].lo==0 && box[4].hi==0 && box[5].lo==0 && box[5].hi==0){rest_tick=k;break;}
      if(box[3].hi-box[3].lo>100 || box[0].hi-box[0].lo>100){std::cout<<"speed="<<speed<<" inconclusive=wrapping-exceeds-100-observation-only-width tick="<<k<<std::endl;break;}
    }
    std::cout<<"speed="<<speed<<" rest_tick="<<rest_tick<<" elapsed_ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()<<" enclosure_ms="<<enclosure_ms<<" oracle_ms="<<oracle_ms<<" includes_200_sample_oracles=1"<<std::endl;
  }
  std::cout<<"contained_scalar_samples="<<enclosed_samples<<" authority=false continuous_delivery_bound=false"<<std::endl;
  size_t hybrid_checks=0;
  for(const double speed:{0.,.01,.28,4.,8.}){
    const model::State initial{0,0,0,speed,.017,.064,.3492,.3505};
    std::vector<enclosure::Box> boxes{enclosure::point(initial)};std::vector<model::State> samples(100,initial);
    bool rest=false;size_t maximum_boxes=1,steps=0;double range_ms=0;
    for(size_t k=0;k<1200;++k){
      const enclosure::I wire=k<50?enclosure::I(-3,1.37):enclosure::I(-3);
      const auto start=std::chrono::steady_clock::now();enclosure::Box swept;
      boxes=enclosure::advance_partitioned(std::move(boxes),wire,p,.005,&swept);
      range_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      const auto box=enclosure::joined(boxes);maximum_boxes=std::max(maximum_boxes,boxes.size());
      for(size_t j=0;j<samples.size();++j){
        double a=k<50?-3+4.37*unit(rng):-3;
        if(k<50){if(j==0)a=-3;if(j==1)a=1.37;if(j==2)a=k%2?-3:1.37;}
        const auto next=model::advance(samples[j],{a,0},p,.005);require(next.has_value(),"hybrid native failed");samples[j]=next->state;
        const auto values=enclosure::values(next->state);for(size_t i=0;i<8;++i){require(values[i]>=box[i].lo && values[i]<=box[i].hi,"native outside partitioned range");++hybrid_checks;}
      }
      ++steps;if(k>=50 && enclosure::at_rest(box)){rest=true;break;}
    }
    require(rest,"partitioned Stop did not reach full model rest");
    std::cout<<"hybrid speed="<<speed<<" steps="<<steps<<" maximum_boxes="<<maximum_boxes<<" range_ms="<<range_ms<<std::endl;
  }
  std::cout<<"hybrid_scalar_checks="<<hybrid_checks<<std::endl;
}
