#include "TypedLog.h"
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include "gazebo/common/common.hh"
#include "gazebo_ros/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

namespace LogTopics {
nowtech::log::TopicInstance system;
nowtech::log::TopicInstance dumpModel;
}

namespace gazebo {

class LinkIntersectionPlugin : public ModelPlugin {
private:
  physics::ModelPtr              mPhysicsModel;
  gazebo_ros::Node::SharedPtr    mNode;
  event::ConnectionPtr           mUpdateEvent;

  std::vector<std::string>       mJointNames;
  std::vector<physics::JointPtr> mJoints;
  nowtech::log::LogFormatConfig  mLogConfig;

public:
  LinkIntersectionPlugin() {}

  ~LinkIntersectionPlugin() {
  //  mRosNode.shutdown();
    Log::unregisterCurrentTask();
    Log::done();
  }

  virtual void Load(physics::ModelPtr aModel, sdf::ElementPtr aSdf)  {
    RCLCPP_FATAL_STREAM(
      rclcpp::get_logger("LinkIntersectionPlugin"),
      "Loading LinkIntersectionPlugin");
    LogSender::init();
    Log::init(mLogConfig);
    Log::registerTopic(LogTopics::system, "system");
    Log::registerTopic(LogTopics::dumpModel, "dumpUrdf");
    Log::registerCurrentTask("main");
    mPhysicsModel = aModel;
    mNode = gazebo_ros::Node::Get(aSdf);

//    mRosNode.getParam(cLaunchParamPublishPeriod, mPublishPeriod);
//    gzdbg << "Publish period = " << mPublishPeriod << std::endl;

    mUpdateEvent = event::Events::ConnectWorldUpdateBegin(boost::bind(&LinkIntersectionPlugin::update, this));

 //   ros::spinOnce();
   // ROS_INFO("Started CDPR Plugin for %s.", aModel->GetName().c_str());
  }

private:
  void update() {
 //   ros::spinOnce();
  }
};

GZ_REGISTER_MODEL_PLUGIN(LinkIntersectionPlugin)

}