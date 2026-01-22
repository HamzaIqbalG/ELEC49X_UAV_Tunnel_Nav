#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/transport/Node.hh>

#include <mutex>
#include <string>

namespace uav_tunnel_gz
{
class CmdVelPlugin : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override
  {
    this->model_ = gz::sim::Model(entity);
    if (!this->model_.Valid(ecm))
    {
      gzerr << "CmdVelPlugin must be attached to a model.\n";
      return;
    }

    this->link_name_ = "base_link";
    if (sdf && sdf->HasElement("link_name"))
    {
      this->link_name_ = sdf->Get<std::string>("link_name");
    }

    this->cmd_topic_ = "/cmd_vel";
    if (sdf && sdf->HasElement("topic"))
    {
      this->cmd_topic_ = sdf->Get<std::string>("topic");
    }

    this->node_.Subscribe(this->cmd_topic_, &CmdVelPlugin::OnCmdVel, this);
    gzmsg << "CmdVelPlugin listening on [" << this->cmd_topic_ << "]\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused)
    {
      return;
    }

    const auto link_entity = this->model_.LinkByName(ecm, this->link_name_);
    if (link_entity == gz::sim::kNullEntity)
    {
      return;
    }

    gz::msgs::Twist cmd;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      cmd = this->last_cmd_;
    }

    const gz::math::Vector3d linear(cmd.linear().x(), cmd.linear().y(),
                                    cmd.linear().z());
    const gz::math::Vector3d angular(cmd.angular().x(), cmd.angular().y(),
                                     cmd.angular().z());

    auto lin_comp =
        ecm.Component<gz::sim::components::LinearVelocityCmd>(link_entity);
    if (!lin_comp)
    {
      ecm.CreateComponent(link_entity,
                          gz::sim::components::LinearVelocityCmd(linear));
    }
    else
    {
      *lin_comp = gz::sim::components::LinearVelocityCmd(linear);
    }

    auto ang_comp =
        ecm.Component<gz::sim::components::AngularVelocityCmd>(link_entity);
    if (!ang_comp)
    {
      ecm.CreateComponent(link_entity,
                          gz::sim::components::AngularVelocityCmd(angular));
    }
    else
    {
      *ang_comp = gz::sim::components::AngularVelocityCmd(angular);
    }
  }

private:
  void OnCmdVel(const gz::msgs::Twist &msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->last_cmd_ = msg;
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string link_name_;
  std::string cmd_topic_;
  gz::transport::Node node_;
  std::mutex mutex_;
  gz::msgs::Twist last_cmd_;
};
}  // namespace uav_tunnel_gz

GZ_ADD_PLUGIN(uav_tunnel_gz::CmdVelPlugin, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(uav_tunnel_gz::CmdVelPlugin,
                    "uav_tunnel_gz::CmdVelPlugin")
