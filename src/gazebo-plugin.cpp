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
  gazebo::physics::ModelPtr      mModel;
  gazebo::physics::WorldPtr      mWorld;
  gazebo_ros::Node::SharedPtr    mNode;
  event::ConnectionPtr           mUpdateEvent;

  std::vector<std::string>       mJointNames;
  std::vector<physics::JointPtr> mJoints;
  nowtech::log::LogFormatConfig  mLogConfig;

  gazebo::common::Time           mLastSimTime;
  gazebo::common::Time           mLastUpdateTime;

public:
  LinkIntersectionPlugin() {}

  ~LinkIntersectionPlugin() {
  //  mRosNode.shutdown();
    Log::unregisterCurrentTask();
    Log::done();
  }

  virtual void Load(physics::ModelPtr aModel, sdf::ElementPtr aSdf)  {
    mModel = aModel;
    mWorld = aModel->GetWorld();
    mNode = gazebo_ros::Node::Get(aSdf);
//    auto physicsEngine = mWorld->Physics();
//    physicsEngine->SetParam("friction_model", "cone_model");
    LogSender::init();
    Log::init(mLogConfig);
    Log::registerTopic(LogTopics::system, "system");
    Log::registerTopic(LogTopics::dumpModel, "dumpUrdf");
    Log::registerCurrentTask("main");

    Log::i() << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX Log::i()" << Log::end;
    // TODO find out how to shut down log

    mUpdateEvent = event::Events::ConnectWorldUpdateBegin(boost::bind(&LinkIntersectionPlugin::update, this));
  }

private:
  void update() {
 //   ros::spinOnce();
  }
};

GZ_REGISTER_MODEL_PLUGIN(LinkIntersectionPlugin)

}