// SPDX-License-Identifier: Apache-2.0
// A second physical WAM-V. No pose animation or periodic velocity forcing.
#include "Following.hh"
#include "AttitudeSafety.hh"
#include <chrono>
#include <deque>
#include <mutex>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/LinearVelocityReset.hh>
#include <gz/sim/components/AngularVelocityReset.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/transport/Node.hh>

namespace wwos {
class PromotionFollower final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  gz::transport::Node node;
  gz::transport::Node::Publisher left, right, status;
  gz::sim::Entity model, leader = gz::sim::kNullEntity;
  std::deque<gz::math::Vector3d> trail;
  std::mutex mutex;
  gz::math::Vector3d shift, initialV;
  double rebaseTime = 0., next = 0., integral = 0., commandSpeed = 0.;
  unsigned long epoch = 0, pendingEpoch = 0;
  bool initialized = false;
public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &cfg,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    model = entity;
    epoch = cfg->Get<unsigned int>("origin_epoch", 0).first;
    initialV = cfg->Get<gz::math::Vector3d>("initial_linear", {}).first;
    left = node.Advertise<gz::msgs::Double>("/model/wamv_follower/joint/left_propeller_joint/cmd_thrust");
    right = node.Advertise<gz::msgs::Double>("/model/wamv_follower/joint/right_propeller_joint/cmd_thrust");
    status = node.Advertise<gz::msgs::Double_V>("/wwos/promotional/follower");
    node.Subscribe("/wwos/promotional/rebase", &PromotionFollower::Rebase, this);
  }
  void Rebase(const gz::msgs::Double_V &m) {
    if (m.data_size() != 5 || m.data(0) != 1) return;
    for (double x : m.data()) if (!std::isfinite(x)) return;
    if (m.data(1) < 1 || m.data(1) != std::floor(m.data(1)) || m.data(4) < 0) return;
    if (std::remainder(m.data(2), 256) != 0 || std::remainder(m.data(3), 256) != 0) return;
    std::lock_guard<std::mutex> guard(mutex);
    if (m.data(1) <= epoch) return;
    pendingEpoch = m.data(1); shift = {m.data(2), m.data(3), 0}; rebaseTime = m.data(4);
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    if (leader != gz::sim::kNullEntity && !ecm.HasEntity(leader)) leader = gz::sim::kNullEntity;
    if (leader == gz::sim::kNullEntity)
      leader = ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name("wamv"));
    if (leader == gz::sim::kNullEntity) {
      gz::msgs::Double neutral; neutral.set_data(0.); left.Publish(neutral); right.Publish(neutral);
      integral = 0.; commandSpeed = 0.; trail.clear(); return;
    }
    gz::sim::Model body(model), lead(leader);
    gz::sim::Link link(body.CanonicalLink(ecm)), leadLink(lead.CanonicalLink(ecm));
    link.EnableVelocityChecks(ecm); leadLink.EnableVelocityChecks(ecm);
    auto v = link.WorldLinearVelocity(ecm), w = link.WorldAngularVelocity(ecm);
    auto lv = leadLink.WorldLinearVelocity(ecm);
    const double t = std::chrono::duration<double>(info.simTime).count();
    auto pose = gz::sim::worldPose(model, ecm), target = gz::sim::worldPose(leader, ecm);
    if (!initialized) {
      // Exactly once at spawn: inherit the leader's measured world velocity.
      // Starting a stationary hull behind a 12.5 m/s boat loses the formation.
      for (auto e : body.Links(ecm))
        ecm.SetComponentData<gz::sim::components::WorldLinearVelocityReset>(e, initialV);
      initialized = true; commandSpeed = initialV.Length();
      auto delta = target.Pos() - pose.Pos(); delta.Z(0);
      for (double f = 0; f <= 1.; f += .05) trail.push_back(pose.Pos() + f * delta);
      return;
    }
    {
      std::lock_guard<std::mutex> guard(mutex);
      if (pendingEpoch > epoch && t >= rebaseTime) {
        // The world pilot shifts every model. Preserve this hull's velocity
        // too, and translate breadcrumbs; never apply the shift a second time.
        for (auto e : body.Links(ecm)) {
          gz::sim::Link l(e); l.EnableVelocityChecks(ecm);
          if (auto velocity = l.WorldLinearVelocity(ecm))
            ecm.SetComponentData<gz::sim::components::WorldLinearVelocityReset>(e, *velocity);
          if (auto angular = l.WorldAngularVelocity(ecm))
            ecm.SetComponentData<gz::sim::components::WorldAngularVelocityReset>(e, *angular);
        }
        for (auto &point : trail) point -= shift;
        epoch = pendingEpoch;
        return;
      }
    }
    if (!v || !w || !lv || t < next) return;
    const double dt = next > 0 ? std::min(.2, t - (next - .05)) : .05; next = t + .05;
    if (trail.empty() || (target.Pos() - trail.back()).Length() > 1.) trail.push_back(target.Pos());
    while (trail.size() > 600) trail.pop_front();
    auto delta = target.Pos() - pose.Pos(); delta.Z(0); const double gap = delta.Length();
    auto direction = gap > .1 ? delta / gap : gz::math::Vector3d(1, 0, 0);
    double closing = ((*v) - (*lv)).Dot(direction);
    // Follow the path actually sailed rather than cutting across a shoreline.
    std::size_t closest = 0; double nearest = 1e12;
    for (std::size_t i = 0; i < trail.size(); ++i) {
      const double distance = (trail[i] - pose.Pos()).SquaredLength();
      if (distance < nearest) { nearest = distance; closest = i; }
    }
    auto aim = trail[closest]; double along = 0.;
    for (std::size_t i = closest + 1; i < trail.size(); ++i) {
      along += (trail[i] - trail[i - 1]).Length(); aim = trail[i];
      // Look far enough along the sailed bend to start turning while the
      // leader is still visible. An 8 m lookahead delayed 90-degree turns.
      if (along > std::clamp(gap - 4., 18., 32.)) break;
    }
    while (closest > 1) { trail.pop_front(); --closest; }
    auto localV = pose.Rot().RotateVectorReverse(*v), localW = pose.Rot().RotateVectorReverse(*w);
    const double yaw = pose.Rot().Yaw();
    double error = std::remainder(std::atan2(aim.Y() - pose.Pos().Y(), aim.X() - pose.Pos().X()) - yaw, 2 * M_PI);
    double bearing = std::remainder(std::atan2(delta.Y(), delta.X()) - yaw, 2 * M_PI);
    double leadSpeed = std::max(0., target.Rot().RotateVectorReverse(*lv).X());
    auto request = Following(gap, closing, leadSpeed, error, bearing);
    const double upZ = pose.Rot().RotateVector(gz::math::Vector3d::UnitZ).Z();
    const double upright = UprightThrustScale(upZ);
    request.speed *= upright;
    const double turnLimit = SafeTurnRate(localV.X(), upZ);
    request.yawRate = std::clamp(request.yawRate, -turnLimit, turnLimit);
    commandSpeed += std::clamp(request.speed - commandSpeed, -2.5 * dt, .9 * dt);
    double speedError = commandSpeed - localV.X();
    integral = std::clamp(integral + speedError * dt, -8., 8.);
    double force = std::clamp(750. * speedError + 100. * integral, -1900., 1900.);
    double differential = 1600. * (request.yawRate - localW.Z());
    double a = std::clamp(force - differential, -2000., 2000.) * upright;
    double b = std::clamp(force + differential, -2000., 2000.) * upright;
    if (upright == 0.) { integral = 0.; commandSpeed = 0.; }
    gz::msgs::Double m; m.set_data(a); left.Publish(m); m.set_data(b); right.Publish(m);
    gz::msgs::Double_V report;
    for (double x : {1., t, double(epoch), pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(),
                    pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W(), localV.X(),
                    gap, a, b, commandSpeed, bearing, double(request.braking)}) report.add_data(x);
    status.Publish(report);
  }
};
}
GZ_ADD_PLUGIN(wwos::PromotionFollower, gz::sim::System, wwos::PromotionFollower::ISystemConfigure,
              wwos::PromotionFollower::ISystemPreUpdate)
