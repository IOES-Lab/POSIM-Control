// SPDX-License-Identifier: Apache-2.0
// Native simulation-time propulsion controller, not a pose animator.
#include <chrono>
#include <cmath>
#include <mutex>
#include <vector>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/LinearVelocityReset.hh>
#include <gz/sim/components/AngularVelocityReset.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#include <algorithm>
#include "VoyageLimits.hh"
#include "AttitudeSafety.hh"
namespace wwos {
class PromotionPilot final: public gz::sim::System,
 public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 gz::transport::Node node;
 gz::transport::Node::Publisher left,right,status,odometry;
 std::mutex mutex;
 std::vector<gz::math::Vector3d> path;
 std::size_t point=0;
 gz::sim::Entity model=gz::sim::kNullEntity;
 bool initialized=false,restore=false;
 int controlMode=1;
 gz::math::Pose3d initial;
 gz::math::Vector3d initialV,initialW;
 double speed=0.,commandSpeed=0.,integral=0.,next=0.,heartbeat=0.,nextStatus=0.;
 double restoreTime=-1.,lastControl=-1.;gz::math::Vector3d restoreAnchor;
 double nextOdometry=0.,rebaseTime=0.;unsigned long rebaseEpoch=0,appliedEpoch=0;
 gz::math::Vector3d originShift;
 static double Wall(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
 public:
 void Configure(const gz::sim::Entity&,const std::shared_ptr<const sdf::Element>&cfg,
                gz::sim::EntityComponentManager&,gz::sim::EventManager&) override {
  left=node.Advertise<gz::msgs::Double>("/model/wamv/joint/left_propeller_joint/cmd_thrust");
  right=node.Advertise<gz::msgs::Double>("/model/wamv/joint/right_propeller_joint/cmd_thrust");
  status=node.Advertise<gz::msgs::Double_V>("/wwos/promotional/pilot");
  odometry=node.Advertise<gz::msgs::Odometry>("/model/wamv/odometry");
  if(cfg->HasElement("restore_pose")){
   restore=true;initial=cfg->Get<gz::math::Pose3d>("restore_pose");
   initialV=cfg->Get<gz::math::Vector3d>("restore_linear");
   initialW=cfg->Get<gz::math::Vector3d>("restore_angular");
  }
  node.Subscribe("/wwos/promotional/path",&PromotionPilot::Receive,this);
  node.Subscribe("/wwos/promotional/rebase",&PromotionPilot::Rebase,this);
 }
 void Rebase(const gz::msgs::Double_V &m){
  if(m.data_size()!=5 || m.data(0)!=1)return;
  for(auto x:m.data())if(!std::isfinite(x))return;
  if(m.data(1)<1 || m.data(1)!=std::floor(m.data(1)) || m.data(4)<0)return;
  if(std::remainder(m.data(2),256)!=0 || std::remainder(m.data(3),256)!=0)return;
  std::lock_guard<std::mutex> g(mutex);
  if(m.data(1)<=appliedEpoch)return;
  rebaseEpoch=m.data(1);originShift={m.data(2),m.data(3),0};rebaseTime=m.data(4);
 }
 void Receive(const gz::msgs::Double_V&m){
  const auto &v=m.data();
  if(v.size()<3 || (v[0]!=1 && v[0]!=2 && v[0]!=3) || v[1]<0 || v[1]>VoyageSpeedMetresPerSecond ||
     v[2]<0 || v[2]>256 || v[2]!=std::floor(v[2]) ||
     (v.size()!=3+2*int(v[2]) && v.size()!=4+2*int(v[2])))return;
  for(double x:v)if(!std::isfinite(x))return;
  std::lock_guard<std::mutex> g(mutex);
  unsigned long epoch=v.size()==4+2*int(v[2])?static_cast<unsigned long>(v[v.size()-1]):0;
  if(epoch!=appliedEpoch)return; // Delayed old-frame heartbeats cannot steer a new origin.
  heartbeat=Wall();
  if(v[2]==0){path.clear();speed=0;point=0;return;}
  // Heartbeat with identical plan does not reset its progress.
  std::vector<gz::math::Vector3d> nextPath;
  for(int i=0;i<int(v[2]);i++)nextPath.emplace_back(v[3+2*i],v[4+2*i],0);
  if(nextPath!=path){path=std::move(nextPath);point=0;}
  speed=v[1];controlMode=int(v[0]);
 }
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager&ecm) override {
  if(model==gz::sim::kNullEntity)model=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("wamv"));
  if(model==gz::sim::kNullEntity || info.paused)return;
  gz::sim::Model body(model);auto canonical=body.CanonicalLink(ecm);
  if(canonical==gz::sim::kNullEntity)return;
  gz::sim::Link link(canonical);link.EnableVelocityChecks(ecm);
  if(!initialized){
   initialized=true;
   if(restore){
    body.SetWorldPoseCmd(ecm,initial);
    for(auto entity:body.Links(ecm)){
     ecm.SetComponentData<gz::sim::components::WorldLinearVelocityReset>(entity,initial.Rot().RotateVector(initialV));
     ecm.SetComponentData<gz::sim::components::WorldAngularVelocityReset>(entity,initial.Rot().RotateVector(initialW));
    }
   }
   return;
  }
  double t=std::chrono::duration<double>(info.simTime).count();
  {
   std::lock_guard<std::mutex> g(mutex);
   if(rebaseEpoch>appliedEpoch && t>=rebaseTime){
    auto world=ecm.EntityByComponents(gz::sim::components::World());
    ecm.Each<gz::sim::components::Model,gz::sim::components::ParentEntity>(
     [&](const gz::sim::Entity &entity,const gz::sim::components::Model*,const gz::sim::components::ParentEntity *parent){
      if(parent->Data()!=world)return true;
      gz::sim::Model item(entity);auto pose=gz::sim::worldPose(entity,ecm);pose.Pos()-=originShift;
      if(entity==model){
       // Coordinates change; every link retains its measured world velocity.
       for(auto e:item.Links(ecm)){
        gz::sim::Link l(e);l.EnableVelocityChecks(ecm);
        if(auto v=l.WorldLinearVelocity(ecm))ecm.SetComponentData<gz::sim::components::WorldLinearVelocityReset>(e,*v);
        if(auto v=l.WorldAngularVelocity(ecm))ecm.SetComponentData<gz::sim::components::WorldAngularVelocityReset>(e,*v);
       }
      }
      item.SetWorldPoseCmd(ecm,pose);return true;
     });
    for(auto &point:path)point-=originShift;
    restoreAnchor-=originShift;appliedEpoch=rebaseEpoch;
   }
  }
  auto pose=gz::sim::worldPose(model,ecm);
  // Observe the actual pose after the one-time restore command was applied.
  // ROS odometry may arrive later while this boat legitimately keeps moving.
  if(restoreTime<0){restoreTime=t;restoreAnchor=pose.Pos();}
  auto worldV=link.WorldLinearVelocity(ecm);auto worldW=link.WorldAngularVelocity(ecm);
  if(!worldV || !worldW)return;
  auto localV=pose.Rot().RotateVectorReverse(*worldV);auto localW=pose.Rot().RotateVectorReverse(*worldW);
  const double upZ=pose.Rot().RotateVector(gz::math::Vector3d::UnitZ).Z();
  const double upright=UprightThrustScale(upZ);
  if(t>=nextOdometry){
   nextOdometry=t+.05;gz::msgs::Odometry m;
   auto stamp=m.mutable_header()->mutable_stamp();auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
   stamp->set_sec(ns/1000000000);stamp->set_nsec(ns%1000000000);
   auto frame=m.mutable_header()->add_data();frame->set_key("frame_id");frame->add_value("map");
   frame=m.mutable_header()->add_data();frame->set_key("child_frame_id");frame->add_value("wamv/base_link");
   auto p=m.mutable_pose()->mutable_position();p->set_x(pose.Pos().X());p->set_y(pose.Pos().Y());p->set_z(pose.Pos().Z());
   auto q=m.mutable_pose()->mutable_orientation();q->set_x(pose.Rot().X());q->set_y(pose.Rot().Y());q->set_z(pose.Rot().Z());q->set_w(pose.Rot().W());
   auto v=m.mutable_twist()->mutable_linear();v->set_x(localV.X());v->set_y(localV.Y());v->set_z(localV.Z());
   v=m.mutable_twist()->mutable_angular();v->set_x(localW.X());v->set_y(localW.Y());v->set_z(localW.Z());
   odometry.Publish(m);
  }
  // A labelled, bounded physical tow can free a pre-existing grounding.
  // Forces are applied at EVERY physics step. Collision, mass and drag stay
  // enabled; no pose/velocity command is used to move the ship.
  {
   std::lock_guard<std::mutex> g(mutex);
   if(controlMode==3 && upright>0. && !path.empty() && Wall()-heartbeat<3){
    auto delta=path.back()-gz::math::Vector3d(pose.Pos().X(),pose.Pos().Y(),0);
    if(delta.Length()>3){
     auto desired=delta.Normalized()*std::min(speed,.8);
     gz::math::Vector3d force=(desired-gz::math::Vector3d(worldV->X(),worldV->Y(),0))*500.;
     if(force.Length()>900.)force=force.Normalized()*900.;
     link.AddWorldForce(ecm,force);
    }
   }
  }
  if(t<next)return;next=t+.05;
  const double dt=lastControl<0?.05:std::clamp(t-lastControl,.001,.2);lastControl=t;
  double a=0,b=0,error=0,distance=0;bool finished=true;
  std::lock_guard<std::mutex> g(mutex);
  if(!path.empty() && Wall()-heartbeat<3){
   while(point+1<path.size() && (path[point]-gz::math::Vector3d(pose.Pos().X(),pose.Pos().Y(),0)).Length()<5)++point;
   auto d=path[point]-gz::math::Vector3d(pose.Pos().X(),pose.Pos().Y(),0);distance=d.Length();
   finished=point+1==path.size() && distance<5;
   error=std::remainder(std::atan2(d.Y(),d.X())-pose.Rot().Yaw(),2*M_PI);
   // Brake before a waypoint, especially before bends. Turning at cruise
   // speed would cut through a carefully checked coastal corridor.
   double arrivalSpeed=0.;
   if(point+1<path.size()){
    auto outgoing=path[point+1]-path[point];
    double bend=std::abs(std::remainder(std::atan2(outgoing.Y(),outgoing.X())-
                                      std::atan2(d.Y(),d.X()),2*M_PI));
    arrivalSpeed=bend<.2?speed:bend<.6?2.:.8;
   }
   double braking=std::sqrt(arrivalSpeed*arrivalSpeed+2*.65*std::max(0.,distance-5.));
   double desired=finished?0:std::min(speed,braking)*std::pow(std::max(0.,std::cos(error)),2)*upright;
   // A finite acceleration ramp and anti-windup PI act through the real
   // upstream propellers. Their existing 2000 N limits remain unchanged.
   commandSpeed+=std::clamp(desired-commandSpeed,-.65*dt,.45*dt);
   double velocityError=commandSpeed-localV.X();
   integral=std::clamp(integral+velocityError*dt,-4.,4.);
   double yawLimit=SafeTurnRate(localV.X(),upZ);
   double yawRate=finished?0:std::clamp(error*.8,-yawLimit,yawLimit);
   double thrust=700*velocityError+100*integral,turn=800*(yawRate-localW.Z());
   a=std::clamp(thrust-turn,-2000.,2000.)*upright;b=std::clamp(thrust+turn,-2000.,2000.)*upright;
   if(upright==0.){integral=0.;commandSpeed=0.;}
  }else{
   commandSpeed=std::max(0.,localV.X());integral=0.;
  }
  gz::msgs::Double m;m.set_data(a);left.Publish(m);m.set_data(b);right.Publish(m);
  if(Wall()>nextStatus){
   nextStatus=Wall()+.2;gz::msgs::Double_V report;
   for(double v:{1.,double(point),finished?1.:0.,distance,a,b,t,error,speed,commandSpeed,localV.X(),restoreTime,restoreAnchor.X(),restoreAnchor.Y(),double(controlMode)})report.add_data(v);
   status.Publish(report);
  }
 }
};
}
GZ_ADD_PLUGIN(wwos::PromotionPilot,gz::sim::System,wwos::PromotionPilot::ISystemConfigure,wwos::PromotionPilot::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(wwos::PromotionPilot,"wwos::PromotionPilot")
