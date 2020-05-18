/*
 * Desc: Gazebo link attacher plugin.
 */

/** \author Sammy Pfeiffer  - sam.pfeiffer@pal-robotics.com (original implementation)
    \author Robert Penicka  - robert.penicka@fel.cvut.cz (rewrite and modify)
*/

#include <ros/ros.h>

#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/PhysicsTypes.hh"
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include <ignition/math/Pose3.hh>
#include <mrs_msgs/GazeboAttach.h>
#include <mrs_msgs/GazeboAttachTyped.h>
#include <mrs_msgs/GazeboDeleteModel.h>
#include <mrs_msgs/GazeboApplyForce.h>
#include <mutex>
#include <unordered_map>

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#define DEFAULT_JOINT_TYPE "revolute"
//#define SLEEP_TIME_AFTER_ATTACH 0.05
namespace gazebo
{

typedef boost::array<double, 3>              force_t;
typedef boost::array<double, 3>              torque_t;
typedef std::tuple<std::string, std::string> ModelLinkName;

struct ApplyForceStruct
{
  ApplyForceStruct(force_t force, torque_t torque) {
    this->force  = force;
    this->torque = torque;
  }

  force_t  force;
  torque_t torque;
};

typedef struct AttachDetachStruct
{
  AttachDetachStruct(std::string model1, std::string link1, std::string model2, std::string link2, std::string joint_type) {
    this->model1     = model1;
    this->link1      = link1;
    this->model2     = model2;
    this->link2      = link2;
    this->joint_type = joint_type;
  }

  std::string model1;
  std::string link1;
  std::string model2;
  std::string link2;
  std::string joint_type;
} AttachDetachStruct;

class GazeboRosLinkAttacher : public WorldPlugin {
public:
  /// \brief Constructor
  GazeboRosLinkAttacher();

  /// \brief Destructor
  virtual ~GazeboRosLinkAttacher();

  /// \brief Load the controller
  void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/);

  void OnUpdate();

  void OnPhysicsUpdate();

  /// \brief Attach with a revolute joint
  bool do_attach(std::string model1, std::string link1, std::string model2, std::string link2, std::string joint_type);

  /// \brief Detach
  bool do_detach(std::string model1, std::string link1, std::string model2, std::string link2);

  /// \brief Delete
  bool do_delete(std::string model_name);

  /// \brief Delete
  bool do_apply_force(std::string model_name, std::string link_name, force_t force, torque_t torque);

  /// \brief Internal representation of a fixed joint
  struct fixedJoint
  {
    std::string       model1;
    physics::ModelPtr m1;
    std::string       link1;
    physics::LinkPtr  l1;
    std::string       model2;
    physics::ModelPtr m2;
    std::string       link2;
    physics::LinkPtr  l2;
    physics::JointPtr joint;
  };

  bool getJoint(std::string model1, std::string link1, std::string model2, std::string link2, fixedJoint &joint);

private:
  ros::NodeHandle    nh_;
  ros::ServiceServer attach_service_;
  ros::ServiceServer detach_service_;
  ros::ServiceServer attach_typed_service_;
  ros::ServiceServer delete_service_;
  ros::ServiceServer apply_force_service_;

  std::vector<event::ConnectionPtr> connections;

  std::mutex                                mtx_attach;
  std::vector<AttachDetachStruct>           toAttachVector;
  std::mutex                                mtx_detach;
  std::vector<AttachDetachStruct>           toDetachVector;
  std::mutex                                mtx_delete;
  std::vector<std::string>                  toDeleteVector;
  std::mutex                                mtx_apply_force;
  std::map<ModelLinkName, ApplyForceStruct> toApplyForceMap;

  void HandleAttaches();
  void HandleDetaches();
  void HandleDeletes();
  void HandleForces();

  bool attach_callback(mrs_msgs::GazeboAttach::Request &req, mrs_msgs::GazeboAttach::Response &res);

  bool detach_callback(mrs_msgs::GazeboAttach::Request &req, mrs_msgs::GazeboAttach::Response &res);

  bool attach_typed_callback(mrs_msgs::GazeboAttachTyped::Request &req, mrs_msgs::GazeboAttachTyped::Response &res);

  bool delete_callback(mrs_msgs::GazeboDeleteModel::Request &req, mrs_msgs::GazeboDeleteModel::Response &res);

  bool apply_force_callback(mrs_msgs::GazeboApplyForce::Request &req, mrs_msgs::GazeboApplyForce::Response &res);

  common::Time prevUpdateTime;
  common::Time updatePeriod;

  common::Time prevUpdatePhysicsTime;
  common::Time updatePhysicsPeriod;

  std::vector<fixedJoint> joints;

  /// \brief The physics engine.
  physics::PhysicsEnginePtr physics;

  /// \brief Pointer to the world.
  physics::WorldPtr world;

  std::vector<std::string> allowed_joint_types;
};

// Constructor
GazeboRosLinkAttacher::GazeboRosLinkAttacher() : nh_("link_attacher_node") {
}

// Destructor
GazeboRosLinkAttacher::~GazeboRosLinkAttacher() {
}

void GazeboRosLinkAttacher::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/) {
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->world   = _world;
  this->physics = this->world->Physics();

  this->attach_service_ = this->nh_.advertiseService("attach", &GazeboRosLinkAttacher::attach_callback, this);
  ROS_INFO_STREAM("Attach service at: " << this->nh_.resolveName("attach"));
  this->detach_service_ = this->nh_.advertiseService("detach", &GazeboRosLinkAttacher::detach_callback, this);
  ROS_INFO_STREAM("Detach service at: " << this->nh_.resolveName("detach"));
  this->attach_typed_service_ = this->nh_.advertiseService("attach_typed", &GazeboRosLinkAttacher::attach_typed_callback, this);
  ROS_INFO_STREAM("AttachTyped service at: " << this->nh_.resolveName("attach_typed"));
  this->delete_service_ = this->nh_.advertiseService("delete", &GazeboRosLinkAttacher::delete_callback, this);
  ROS_INFO_STREAM("Delete service at: " << this->nh_.resolveName("delete"));
  this->apply_force_service_ = this->nh_.advertiseService("apply_force", &GazeboRosLinkAttacher::apply_force_callback, this);
  ROS_INFO_STREAM("ApplyForce service at: " << this->nh_.resolveName("apply_force"));
  ROS_INFO("Link attacher node initialized.");

  this->prevUpdateTime = common::Time::GetWallTime();
  this->updatePeriod   = common::Time(0, common::Time::SecToNano(0.75));

  this->prevUpdatePhysicsTime = common::Time::GetWallTime();
  this->updatePhysicsPeriod   = common::Time(0, common::Time::SecToNano(0.05));

  allowed_joint_types.push_back("revolute");
  allowed_joint_types.push_back("ball");
  allowed_joint_types.push_back("gearbox");
  allowed_joint_types.push_back("prismatic");
  allowed_joint_types.push_back("revolute2");
  allowed_joint_types.push_back("universal");
  allowed_joint_types.push_back("piston");
  allowed_joint_types.push_back("fixed");
  ROS_INFO_STREAM("ConnectWorldUpdateEnd");

  this->connections.push_back(event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboRosLinkAttacher::OnUpdate, this)));
  this->connections.push_back(event::Events::ConnectBeforePhysicsUpdate(boost::bind(&GazeboRosLinkAttacher::OnPhysicsUpdate, this)));
}

void GazeboRosLinkAttacher::OnUpdate() {
  if (common::Time::GetWallTime() - this->prevUpdateTime < this->updatePeriod) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mtx_attach);
    this->HandleAttaches();
  }

  {
    std::lock_guard<std::mutex> lock(mtx_detach);
    this->HandleDetaches();
  }

  {
    std::lock_guard<std::mutex> lock(mtx_delete);
    this->HandleDeletes();
  }

  this->prevUpdateTime = common::Time::GetWallTime();
}

void GazeboRosLinkAttacher::OnPhysicsUpdate() {
  if (common::Time::GetWallTime() - this->prevUpdatePhysicsTime < this->updatePhysicsPeriod) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mtx_apply_force);
    this->HandleForces();
  }

  this->prevUpdatePhysicsTime = common::Time::GetWallTime();
}

void GazeboRosLinkAttacher::HandleAttaches() {
  for (const auto &toAttach : toAttachVector) {
    do_attach(toAttach.model1, toAttach.link1, toAttach.model2, toAttach.link2, toAttach.joint_type);
  }
  toAttachVector.clear();
}

void GazeboRosLinkAttacher::HandleDetaches() {
  for (const auto &toDetach : toDetachVector) {
    do_detach(toDetach.model1, toDetach.link1, toDetach.model2, toDetach.link2);
  }
  toDetachVector.clear();
}

void GazeboRosLinkAttacher::HandleDeletes() {
  for (const std::string &toDelete : toDeleteVector) {
    do_delete(toDelete);
  }
  toDeleteVector.clear();
}

void GazeboRosLinkAttacher::HandleForces() {
  for (const auto &toApplyForce : toApplyForceMap) {
    do_apply_force(std::get<0>(toApplyForce.first), std::get<1>(toApplyForce.first), toApplyForce.second.force, toApplyForce.second.torque);
  }
  // do not delete - the force needs to be applied continuously
}

bool GazeboRosLinkAttacher::do_attach(std::string model1, std::string link1, std::string model2, std::string link2, std::string joint_type) {

  // look for any previous instance of the joint first.
  // if we try to create a joint in between two links
  // more than once (even deleting any reference to the first one)
  // gazebo hangs/crashes
  fixedJoint j;
  if (this->getJoint(model1, link1, model2, link2, j)) {
    ROS_INFO_STREAM("Joint already existed, reusing it.");
    j.joint->Attach(j.l1, j.l2);
    // ros::Duration(SLEEP_TIME_AFTER_ATTACH).sleep();
    return true;
  }
  j.model1 = model1;
  j.link1  = link1;
  j.model2 = model2;
  j.link2  = link2;
  ROS_DEBUG_STREAM("Getting BasePtr of " << model1);
  physics::BasePtr b1 = this->world->BaseByName(model1);
  if (b1 == NULL) {
    ROS_ERROR_STREAM(model1 << " model was not found");
    return false;
  }
  ROS_DEBUG_STREAM("Getting BasePtr of " << model2);
  physics::BasePtr b2 = this->world->BaseByName(model2);
  if (b2 == NULL) {
    ROS_ERROR_STREAM(model2 << " model was not found");
    return false;
  }

  ROS_DEBUG_STREAM("Casting into ModelPtr");
  physics::ModelPtr m1(dynamic_cast<physics::Model *>(b1.get()));
  j.m1 = m1;
  physics::ModelPtr m2(dynamic_cast<physics::Model *>(b2.get()));
  j.m2 = m2;

  ROS_DEBUG_STREAM("Getting link: '" << link1 << "' from model: '" << model1 << "'");
  physics::LinkPtr l1 = m1->GetLink(link1);
  if (l1 == NULL) {
    ROS_ERROR_STREAM(link1 << " link was not found");
    return false;
  }
  if (l1->GetInertial() == NULL) {
    ROS_ERROR_STREAM("link1 inertia is NULL!");
  } else
    ROS_DEBUG_STREAM("link1 inertia is not NULL, for example, mass is: " << l1->GetInertial()->Mass());
  j.l1 = l1;
  ROS_DEBUG_STREAM("Getting link: '" << link2 << "' from model: '" << model2 << "'");
  physics::LinkPtr l2 = m2->GetLink(link2);
  if (l2 == NULL) {
    ROS_ERROR_STREAM(link2 << " link was not found");
    return false;
  }
  if (l2->GetInertial() == NULL) {
    ROS_ERROR_STREAM("link2 inertia is NULL!");
  } else
    ROS_DEBUG_STREAM("link2 inertia is not NULL, for example, mass is: " << l2->GetInertial()->Mass());
  j.l2 = l2;

  ROS_DEBUG_STREAM("Links are: " << l1->GetName() << " and " << l2->GetName());

  ROS_DEBUG_STREAM("Creating " << joint_type << " joint on model: '" << model1 << "'");
  j.joint = this->physics->CreateJoint(joint_type, m1);
  this->joints.push_back(j);

  ROS_DEBUG_STREAM("Attach");
  j.joint->Attach(l1, l2);
  ROS_DEBUG_STREAM("Loading links");
  j.joint->Load(l1, l2, ignition::math::Pose3d());
  ROS_DEBUG_STREAM("SetModel");
  j.joint->SetModel(m2);

  /*
   * If SetModel is not done we get:
   * ***** Internal Program Error - assertion (this->GetParentModel() != __null)
   failed in void gazebo::physics::Entity::PublishPose():
   /tmp/buildd/gazebo2-2.2.3/gazebo/physics/Entity.cc(225):
   An entity without a parent model should not happen

   * If SetModel is given the same model than CreateJoint given
   * Gazebo crashes with
   * ***** Internal Program Error - assertion (self->inertial != __null)
   failed in static void gazebo::physics::ODELink::MoveCallback(dBodyID):
   /tmp/buildd/gazebo2-2.2.3/gazebo/physics/ode/ODELink.cc(183): Inertial pointer is NULL
   */

  ROS_DEBUG_STREAM("SetHightstop");
  j.joint->SetUpperLimit(0, 0);
  ROS_DEBUG_STREAM("SetLowStop");
  j.joint->SetLowerLimit(0, 0);
  ROS_DEBUG_STREAM("Init");
  j.joint->Init();
  ROS_INFO_STREAM("Attach finished.");
  // ros::Duration(SLEEP_TIME_AFTER_ATTACH).sleep();
  return true;
}

bool GazeboRosLinkAttacher::do_detach(std::string model1, std::string link1, std::string model2, std::string link2) {
  // search for the instance of joint and do detach
  fixedJoint j;
  if (this->getJoint(model1, link1, model2, link2, j)) {
    j.joint->Detach();
    // ros::Duration(SLEEP_TIME_AFTER_ATTACH).sleep();
    return true;
  }

  return false;
}

bool GazeboRosLinkAttacher::do_delete(std::string model_name) {
  physics::ModelPtr model = world->ModelByName(model_name);
  world->RemoveModel(model);

  return true;
}

bool GazeboRosLinkAttacher::do_apply_force(std::string model_name, std::string link_name, force_t force, torque_t torque) {
  physics::ModelPtr model = world->ModelByName(model_name);
  if (!model) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Model '%s' not found", ros::this_node::getName().c_str(), model_name.c_str());
    return false;
  }
  physics::LinkPtr link = model->GetLink(link_name);
  if (!link) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Link '%s' not found in model", ros::this_node::getName().c_str(), link_name.c_str());
    return false;
  }

  link->SetLinearDamping(0.1);  // to avoid infinite movement
  link->SetForce(ignition::math::Vector3d(force[0], force[1], force[2]));
  link->SetTorque(ignition::math::Vector3d(torque[0], torque[1], torque[2]));

  return true;
}

bool GazeboRosLinkAttacher::getJoint(std::string model1, std::string link1, std::string model2, std::string link2, fixedJoint &joint) {
  fixedJoint j;
  for (std::vector<fixedJoint>::iterator it = this->joints.begin(); it != this->joints.end(); ++it) {
    j = *it;
    if ((j.model1.compare(model1) == 0) && (j.model2.compare(model2) == 0) && (j.link1.compare(link1) == 0) && (j.link2.compare(link2) == 0)) {
      joint = j;
      return true;
    }
  }
  return false;
}

bool GazeboRosLinkAttacher::attach_callback(mrs_msgs::GazeboAttach::Request &req, mrs_msgs::GazeboAttach::Response &res) {
  ROS_INFO_STREAM("Received request to attach model: '" << req.model_name_1 << "' using link: '" << req.link_name_1 << "' with model: '" << req.model_name_2
                                                        << "' using link: '" << req.link_name_2 << "'");

  {
    std::lock_guard<std::mutex> lock(mtx_attach);
    this->toAttachVector.push_back(AttachDetachStruct(req.model_name_1, req.link_name_1, req.model_name_2, req.link_name_2, DEFAULT_JOINT_TYPE));
  }

  res.ok = true;
  return true;
}

bool GazeboRosLinkAttacher::attach_typed_callback(mrs_msgs::GazeboAttachTyped::Request &req, mrs_msgs::GazeboAttachTyped::Response &res) {
  ROS_INFO_STREAM("Received request to attach model: '" << req.model_name_1 << "' using link: '" << req.link_name_1 << "' with model: '" << req.model_name_2
                                                        << "' using link: '" << req.link_name_2 << "' and joint '" << req.joint_type << "'");
  bool allowed_joint_type = false;
  for (size_t var = 0; var < allowed_joint_types.size(); ++var) {
    if (allowed_joint_types[var].compare(req.joint_type) == 0) {
      allowed_joint_type = true;
      break;
    }
  }
  if (!allowed_joint_type) {
    ROS_ERROR_STREAM("Unknown joint type " + req.joint_type);
    res.ok = false;
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(mtx_attach);
    this->toAttachVector.push_back(AttachDetachStruct(req.model_name_1, req.link_name_1, req.model_name_2, req.link_name_2, req.joint_type));
  }
  res.ok = true;
  return true;
}

bool GazeboRosLinkAttacher::detach_callback(mrs_msgs::GazeboAttach::Request &req, mrs_msgs::GazeboAttach::Response &res) {
  ROS_INFO_STREAM("Received request to detach model: '" << req.model_name_1 << "' using link: '" << req.link_name_1 << "' with model: '" << req.model_name_2
                                                        << "' using link: '" << req.link_name_2 << "'");

  {
    std::lock_guard<std::mutex> lock(mtx_detach);
    this->toDetachVector.push_back(AttachDetachStruct(req.model_name_1, req.link_name_1, req.model_name_2, req.link_name_2, ""));
  }

  res.ok = true;
  return true;
}

bool GazeboRosLinkAttacher::delete_callback(mrs_msgs::GazeboDeleteModel::Request &req, mrs_msgs::GazeboDeleteModel::Response &res) {
  ROS_INFO_STREAM("Received request to delete model: '" << req.model_name << "'");

  {
    std::lock_guard<std::mutex> lock(mtx_delete);
    this->toDeleteVector.push_back(req.model_name);
  }

  res.ok = true;
  return true;
}

bool GazeboRosLinkAttacher::apply_force_callback(mrs_msgs::GazeboApplyForce::Request &req, mrs_msgs::GazeboApplyForce::Response &res) {
  ROS_INFO_THROTTLE(1.0, "Received request to continuously apply force [%.2fN, %.2fN, %.2fN] to model: '%s'", req.force[0], req.force[1], req.force[2],
                    req.model_name.c_str());

  {
    std::lock_guard<std::mutex> lock(mtx_apply_force);
    ModelLinkName               key = std::make_tuple(req.model_name, req.link_name);
    ApplyForceStruct            val(req.force, req.torque);
    auto                        it = toApplyForceMap.find(key);
    if (it == toApplyForceMap.end()) {
      toApplyForceMap.insert(std::pair<ModelLinkName, ApplyForceStruct>(key, val));
    } else {
      it->second = val;
    }
  }

  res.ok = true;
  return true;
}


// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosLinkAttacher)

}  // namespace gazebo
