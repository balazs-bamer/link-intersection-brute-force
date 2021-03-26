#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>

namespace gazebo {

class LinkIntersectionPlugin : public ModelPlugin {
private:
  ros::NodeHandle                mRosNode;
  physics::ModelPtr              mPhysicsModel;
  event::ConnectionPtr           mUpdateEvent;

  std::vector<std::string>       mJointNames;
  std::vector<physics::JointPtr> mJoints;

public:
  LinkIntersectionPlugin() {}

  ~LinkIntersectionPlugin() {
    mRosNode.shutdown();
  }

  virtual void Load(physics::ModelPtr aModel, sdf::ElementPtr aSdf)  {
    mPhysicsModel = aModel;
    mRosNode = ros::NodeHandle();

//    mRosNode.getParam(cLaunchParamPublishPeriod, mPublishPeriod);
//    gzdbg << "Publish period = " << mPublishPeriod << std::endl;

    mUpdateEvent = event::Events::ConnectWorldUpdateBegin(boost::bind(&CdprGazeboPlugin::update, this));

    ros::spinOnce();
    ROS_INFO("Started CDPR Plugin for %s.", aModel->GetName().c_str());
  }

private:
  void update() {
    ros::spinOnce();
  }
};

GZ_REGISTER_MODEL_PLUGIN(LinkIntersectionPlugin)

}