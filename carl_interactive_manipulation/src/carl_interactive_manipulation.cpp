#include <carl_interactive_manipulation/carl_interactive_manipulation.h>

using namespace std;

CarlInteractiveManipulation::CarlInteractiveManipulation() :
    acGrasp("jaco_arm/manipulation/grasp", true), acPickup("jaco_arm/manipulation/pickup", true), acHome(
        "carl_moveit_wrapper/common_actions/ready_arm", true)
{
  joints.resize(6);

  //messages
  cartesianCmd = n.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd", 1);
  jointStateSubscriber = n.subscribe("jaco_arm/joint_states", 1, &CarlInteractiveManipulation::updateJoints, this);
  segmentedObjectsSubscriber = n.subscribe("rail_segmentation/segmented_objects_visualization", 1,
                                           &CarlInteractiveManipulation::segmentedObjectsCallback, this);

  //services
  armCartesianPositionClient = n.serviceClient<wpi_jaco_msgs::GetCartesianPosition>("jaco_arm/get_cartesian_position");
  armEStopClient = n.serviceClient<wpi_jaco_msgs::EStop>("jaco_arm/software_estop");
  eraseTrajectoriesClient = n.serviceClient<std_srvs::Empty>("jaco_arm/erase_trajectories");
  jacoFkClient = n.serviceClient<wpi_jaco_msgs::JacoFK>("jaco_arm/kinematics/fk");
  qeClient = n.serviceClient<wpi_jaco_msgs::QuaternionToEuler>("jaco_conversions/quaternion_to_euler");
  pickupSegmentedClient = n.serviceClient<rail_pick_and_place_msgs::PickupSegmentedObject>("rail_pick_and_place/pickup_segmented_object");
  removeObjectClient = n.serviceClient<rail_segmentation::RemoveObject>("rail_segmentation/remove_object");

  //actionlib
  ROS_INFO("Waiting for grasp, and pickup action servers...");
  acGrasp.waitForServer();
  acPickup.waitForServer();
  acHome.waitForServer();
  ROS_INFO("Finished waiting for action servers");

  markerPose.resize(6);
  lockPose = false;
  movingArm = false;
  disableArmMarkerCommands = false;

  imServer.reset(
      new interactive_markers::InteractiveMarkerServer("carl_interactive_manipulation", "carl_markers", false));

  ros::Duration(0.1).sleep();

  makeHandMarker();

  //setup object menu
  objectMenuHandler.insert("Pickup", boost::bind(&CarlInteractiveManipulation::processPickupMarkerFeedback, this, _1));
  objectMenuHandler.insert("Remove", boost::bind(&CarlInteractiveManipulation::processRemoveMarkerFeedback, this, _1));

  imServer->applyChanges();
}

void CarlInteractiveManipulation::updateJoints(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (unsigned int i = 0; i < 6; i++)
  {
    joints.at(i) = msg->position.at(i);
  }

  //perform safety check if arm is moving due to interactive marker Cartesian control
  if (movingArm)
  {
    if (fabs(msg->effort[0]) >= J1_THRESHOLD || fabs(msg->effort[1] >= J2_THRESHOLD) || fabs(msg->effort[2] >= J3_THRESHOLD)
        || fabs(msg->effort[3]) >= J4_THRESHOLD || fabs(msg->effort[4] >= J5_THRESHOLD) || fabs(msg->effort[5] >= J6_THRESHOLD)
        || fabs(msg->effort[6]) >= F1_THRESHOLD || fabs(msg->effort[7] >= F2_THRESHOLD) || fabs(msg->effort[8] >= F3_THRESHOLD))
    {
      ROS_INFO("Arm is moving dangerously, attempty recovery...");
      armCollisionRecovery();
    }
  }
}

void CarlInteractiveManipulation::segmentedObjectsCallback(
    const rail_segmentation::SegmentedObjectList::ConstPtr& objectList)
{
  ROS_INFO("Received new segmented point clouds");
  clearSegmentedObjects();
  recognizedMenuHandlers.clear();
  recognizedMenuHandlers.resize(objectList->objects.size());
  for (unsigned int i = 0; i < objectList->objects.size(); i++)
  {
    visualization_msgs::InteractiveMarker objectMarker;
    objectMarker.header = objectList->objects[i].objectCloud.header;

    objectMarker.pose.position.x = 0.0;
    objectMarker.pose.position.y = 0.0;
    objectMarker.pose.position.z = 0.0;
    objectMarker.pose.orientation.x = 0.0;
    objectMarker.pose.orientation.y = 0.0;
    objectMarker.pose.orientation.z = 0.0;
    objectMarker.pose.orientation.w = 1.0;

    stringstream ss;
    ss.str("");
    ss << "object" << i;
    objectMarker.name = ss.str();

    visualization_msgs::Marker cloudMarker;
    cloudMarker.header = objectList->objects[i].objectCloud.header;
    cloudMarker.type = visualization_msgs::Marker::CUBE_LIST;
    if (objectList->objects[i].recognized)
    {
      cloudMarker.color.r = ((float)(rand()) / (float)(RAND_MAX)) / 3.0 + .1;
      cloudMarker.color.g = ((float)(rand()) / (float)(RAND_MAX)) / 3.0 + .4;
      cloudMarker.color.b = ((float)(rand()) / (float)(RAND_MAX)) / 4.0 + .75;
    }
    else
    {
      cloudMarker.color.r = ((float)(rand()) / (float)(RAND_MAX)) / 3.0 + .66;
      cloudMarker.color.g = ((float)(rand()) / (float)(RAND_MAX)) / 4.0;
      cloudMarker.color.b = ((float)(rand()) / (float)(RAND_MAX)) / 5.0;
    }
    cloudMarker.color.a = 1.0;

    //add point cloud to cloud marker
    sensor_msgs::PointCloud cloudCopy;
    sensor_msgs::convertPointCloud2ToPointCloud(objectList->objects[i].objectCloud, cloudCopy);
    cloudMarker.scale.x = .01;
    cloudMarker.scale.y = .01;
    cloudMarker.scale.z = .01;
    cloudMarker.points.resize(cloudCopy.points.size());
    float xAvg = 0;
    float yAvg = 0;
    float zAvg = 0;
    for (unsigned int j = 0; j < cloudCopy.points.size(); j++)
    {
      cloudMarker.points[j].x = cloudCopy.points[j].x;
      cloudMarker.points[j].y = cloudCopy.points[j].y;
      cloudMarker.points[j].z = cloudCopy.points[j].z;
      xAvg += cloudCopy.points[j].x;
      yAvg += cloudCopy.points[j].y;
      zAvg += cloudCopy.points[j].z;
    }
    xAvg /= cloudCopy.points.size();
    yAvg /= cloudCopy.points.size();
    zAvg /= cloudCopy.points.size();

    visualization_msgs::InteractiveMarkerControl objectControl;
    ss << "control";
    objectControl.name = ss.str();
    objectControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    //objectControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    objectControl.always_visible = true;
    objectControl.markers.resize(1);
    objectControl.markers[0] = cloudMarker;
    objectMarker.controls.push_back(objectControl);

    //object label
    visualization_msgs::InteractiveMarkerControl objectLabelControl;
    stringstream namestream;
    namestream.str("");
    namestream << "object" << i << "_label";
    objectLabelControl.name = namestream.str();
    objectLabelControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    objectLabelControl.always_visible = true;
    visualization_msgs::Marker objectLabel;
    objectLabel.header = objectList->objects[i].objectCloud.header;
    objectLabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    objectLabel.pose.position.x = xAvg;
    objectLabel.pose.position.y = yAvg;
    objectLabel.pose.position.z = zAvg + .1;
    objectLabel.pose.orientation.x = 0;
    objectLabel.pose.orientation.y = 0;
    objectLabel.pose.orientation.z = 0;
    objectLabel.pose.orientation.w = 1;
    objectLabel.scale.x = .1;
    objectLabel.scale.y = .1;
    objectLabel.scale.z = .1;
    objectLabel.color.r = 1.0;
    objectLabel.color.g = 1.0;
    objectLabel.color.b = 1.0;
    objectLabel.color.a = 1.0;
    objectLabel.text = objectList->objects[i].name;
    objectLabelControl.markers.resize(1);
    objectLabelControl.markers[0] = objectLabel;
    objectMarker.controls.push_back(objectLabelControl);

    //pickup menu
    visualization_msgs::InteractiveMarkerControl objectMenuControl;
    objectMenuControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    ss << "_menu";
    objectMenuControl.name = ss.str();
    objectMarker.controls.push_back(objectMenuControl);

    imServer->insert(objectMarker);

    if (objectList->objects[i].recognized)
    {
      stringstream ss2;
      ss2.str("");
      ss2 << "Pickup " << objectList->objects[i].name;
      recognizedMenuHandlers[i].insert(ss2.str(), boost::bind(&CarlInteractiveManipulation::processPickupMarkerFeedback, this, _1));
      recognizedMenuHandlers[i].insert("Remove", boost::bind(&CarlInteractiveManipulation::processRemoveMarkerFeedback, this, _1));
      recognizedMenuHandlers[i].apply(*imServer, objectMarker.name);
    }
    else
    {  
      objectMenuHandler.apply(*imServer, objectMarker.name);
    }

    segmentedObjects.push_back(objectMarker);
  }

  imServer->applyChanges();

  ROS_INFO("Point cloud markers created");
}

void CarlInteractiveManipulation::clearSegmentedObjects()
{
  for (unsigned int i = 0; i < segmentedObjects.size(); i++)
  {
    stringstream ss;
    ss.str("");
    ss << "object" << i;
    imServer->erase(ss.str());
  }
  segmentedObjects.clear();

  imServer->applyChanges();
}

void CarlInteractiveManipulation::makeHandMarker()
{
  visualization_msgs::InteractiveMarker iMarker;
  iMarker.header.frame_id = "jaco_link_base";

  //initialize position to the jaco arm's current position
  wpi_jaco_msgs::JacoFK fkSrv;
  for (unsigned int i = 0; i < 6; i++)
  {
    fkSrv.request.joints.push_back(joints.at(i));
  }
  if (jacoFkClient.call(fkSrv))
  {
    iMarker.pose = fkSrv.response.handPose.pose;
  }
  else
  {
    iMarker.pose.position.x = 0.0;
    iMarker.pose.position.y = 0.0;
    iMarker.pose.position.z = 0.0;
    iMarker.pose.orientation.x = 0.0;
    iMarker.pose.orientation.y = 0.0;
    iMarker.pose.orientation.z = 0.0;
    iMarker.pose.orientation.w = 1.0;
  }
  iMarker.scale = .2;

  iMarker.name = "jaco_hand_marker";
  iMarker.description = "JACO Hand Control";

  //make a sphere control to represent the end effector position
  visualization_msgs::Marker sphereMarker;
  sphereMarker.type = visualization_msgs::Marker::SPHERE;
  sphereMarker.scale.x = iMarker.scale * 1;
  sphereMarker.scale.y = iMarker.scale * 1;
  sphereMarker.scale.z = iMarker.scale * 1;
  sphereMarker.color.r = .5;
  sphereMarker.color.g = .5;
  sphereMarker.color.b = .5;
  sphereMarker.color.a = 0.0;
  visualization_msgs::InteractiveMarkerControl sphereControl;
  sphereControl.markers.push_back(sphereMarker);
  sphereControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  sphereControl.name = "jaco_hand_origin_marker";
  iMarker.controls.push_back(sphereControl);

  //add 6-DOF controls
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  //menu
  interactive_markers::MenuHandler::EntryHandle fingersSubMenuHandle = menuHandler.insert("Fingers");
  menuHandler.insert(fingersSubMenuHandle, "Grasp",
                     boost::bind(&CarlInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert(fingersSubMenuHandle, "Release",
                     boost::bind(&CarlInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert("Pickup", boost::bind(&CarlInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert("Home", boost::bind(&CarlInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert("Retract", boost::bind(&CarlInteractiveManipulation::processHandMarkerFeedback, this, _1));

  visualization_msgs::InteractiveMarkerControl menuControl;
  menuControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  menuControl.name = "jaco_hand_menu";
  iMarker.controls.push_back(menuControl);

  imServer->insert(iMarker);
  imServer->setCallback(iMarker.name, boost::bind(&CarlInteractiveManipulation::processHandMarkerFeedback, this, _1));

  menuHandler.apply(*imServer, iMarker.name);
}

void CarlInteractiveManipulation::processPickupMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    rail_pick_and_place_msgs::PickupSegmentedObject::Request req;
    rail_pick_and_place_msgs::PickupSegmentedObject::Response res;
    req.objectIndex = atoi(feedback->marker_name.substr(6).c_str());
    if (!pickupSegmentedClient.call(req, res))
    {
      ROS_INFO("Could not call pickup service.");
      return;
    }
    if (res.success)
    {
      if (!removeObjectMarker(req.objectIndex))
        return;
    }

    imServer->applyChanges();
  }
}

void CarlInteractiveManipulation::processRemoveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    if (!removeObjectMarker(atoi(feedback->marker_name.substr(6).c_str())))
      return;

    imServer->applyChanges();
  }
}

bool CarlInteractiveManipulation::removeObjectMarker(int index)
{
  rail_segmentation::RemoveObject::Request req;
  rail_segmentation::RemoveObject::Response res;
  req.index = index;
  if (!removeObjectClient.call(req, res))
  {
    ROS_INFO("Could not call remove object service.");
    return false;
  }
  return true;
}

void CarlInteractiveManipulation::processHandMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  switch (feedback->event_type)
  {
    //Send a stop command so that when the marker is released the arm stops moving
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      if (feedback->marker_name.compare("jaco_hand_marker") == 0)
      {
        lockPose = true;
        movingArm = false;
        if (disableArmMarkerCommands)
          disableArmMarkerCommands = false;
        sendStopCommand();
      }
      break;

      //Menu actions
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      if (feedback->marker_name.compare("jaco_hand_marker") == 0)
      {
        if (feedback->menu_entry_id == 2)	//grasp requested
        {
          wpi_jaco_msgs::ExecuteGraspGoal graspGoal;
          graspGoal.closeGripper = true;
          graspGoal.limitFingerVelocity = false;
          acGrasp.sendGoal(graspGoal);
        }
        else if (feedback->menu_entry_id == 3)	//release requested
        {
          wpi_jaco_msgs::ExecuteGraspGoal graspGoal;
          graspGoal.closeGripper = false;
          graspGoal.limitFingerVelocity = false;
          acGrasp.sendGoal(graspGoal);
        }
        else if (feedback->menu_entry_id == 4)	//pickup requested
        {
          wpi_jaco_msgs::ExecutePickupGoal pickupGoal;
          pickupGoal.limitFingerVelocity = false;
          pickupGoal.setLiftVelocity = false;
          acPickup.sendGoal(pickupGoal);
        }
        else if (feedback->menu_entry_id == 5)  //home requested
        {
          acGrasp.cancelAllGoals();
          acPickup.cancelAllGoals();
          wpi_jaco_msgs::HomeArmGoal homeGoal;
          homeGoal.retract = false;
          homeGoal.numAttempts = 3;
          acHome.sendGoal(homeGoal);
          acHome.waitForResult(ros::Duration(10.0));
        }
        else if (feedback->menu_entry_id == 6)
        {
          acGrasp.cancelAllGoals();
          acPickup.cancelAllGoals();
          wpi_jaco_msgs::HomeArmGoal homeGoal;
          homeGoal.retract = true;
          homeGoal.retractPosition.position = true;
          homeGoal.retractPosition.armCommand = true;
          homeGoal.retractPosition.fingerCommand = false;
          homeGoal.retractPosition.repeat = false;
          homeGoal.retractPosition.joints.resize(6);
          homeGoal.retractPosition.joints[0] = -2.57;
          homeGoal.retractPosition.joints[1] = 1.39;
          homeGoal.retractPosition.joints[2] = .527;
          homeGoal.retractPosition.joints[3] = -.084;
          homeGoal.retractPosition.joints[4] = .515;
          homeGoal.retractPosition.joints[5] = -1.745;
          homeGoal.numAttempts = 3;
          acHome.sendGoal(homeGoal);
          acHome.waitForResult(ros::Duration(15.0));
        }
      }
      break;

      //Send movement commands to the arm to follow the pose marker
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      if (feedback->marker_name.compare("jaco_hand_marker") == 0
          && feedback->control_name.compare("jaco_hand_origin_marker") != 0)
      {
        if (!(lockPose || disableArmMarkerCommands))
        {
          movingArm = true;

          acGrasp.cancelAllGoals();
          acPickup.cancelAllGoals();

          //convert pose for compatibility with JACO API
          wpi_jaco_msgs::QuaternionToEuler qeSrv;
          qeSrv.request.orientation = feedback->pose.orientation;
          if (qeClient.call(qeSrv))
          {
            wpi_jaco_msgs::CartesianCommand cmd;
            cmd.position = true;
            cmd.armCommand = true;
            cmd.fingerCommand = false;
            cmd.repeat = false;
            cmd.arm.linear.x = feedback->pose.position.x;
            cmd.arm.linear.y = feedback->pose.position.y;
            cmd.arm.linear.z = feedback->pose.position.z;
            cmd.arm.angular.x = qeSrv.response.roll;
            cmd.arm.angular.y = qeSrv.response.pitch;
            cmd.arm.angular.z = qeSrv.response.yaw;

            cartesianCmd.publish(cmd);

            markerPose[0] = feedback->pose.position.x;
            markerPose[1] = feedback->pose.position.y;
            markerPose[2] = feedback->pose.position.z;
            markerPose[3] = qeSrv.response.roll;
            markerPose[4] = qeSrv.response.pitch;
            markerPose[5] = qeSrv.response.yaw;
          }
          else
            ROS_INFO("Quaternion to Euler conversion service failed, could not send pose update");
        }
      }
      break;

      //Mouse down events
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      lockPose = false;
      break;

      //As with mouse clicked, send a stop command when the mouse is released on the marker
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      if (feedback->marker_name.compare("jaco_hand_marker") == 0)
      {
        lockPose = true;
        movingArm = false;
        if (disableArmMarkerCommands)
          disableArmMarkerCommands = false;
        sendStopCommand();
      }
      break;
  }

  //Update interactive marker server
  imServer->applyChanges();
}

void CarlInteractiveManipulation::sendStopCommand()
{
  wpi_jaco_msgs::CartesianCommand cmd;
  cmd.position = false;
  cmd.armCommand = true;
  cmd.fingerCommand = false;
  cmd.repeat = true;
  cmd.arm.linear.x = 0.0;
  cmd.arm.linear.y = 0.0;
  cmd.arm.linear.z = 0.0;
  cmd.arm.angular.x = 0.0;
  cmd.arm.angular.y = 0.0;
  cmd.arm.angular.z = 0.0;
  cartesianCmd.publish(cmd);

  std_srvs::Empty srv;
  if (!eraseTrajectoriesClient.call(srv))
  {
    ROS_INFO("Could not call erase trajectories service...");
  }
}

void CarlInteractiveManipulation::updateMarkerPosition()
{
  wpi_jaco_msgs::JacoFK fkSrv;
  for (unsigned int i = 0; i < 6; i++)
  {
    fkSrv.request.joints.push_back(joints.at(i));
  }

  if (jacoFkClient.call(fkSrv))
  {
    imServer->setPose("jaco_hand_marker", fkSrv.response.handPose.pose);
    imServer->applyChanges();
  }
  else
  {
    ROS_INFO("Failed to call forward kinematics service");
  }
}

void CarlInteractiveManipulation::armCollisionRecovery()
{
  //check if recovery is already in progress
  if (disableArmMarkerCommands)
    return;

  wpi_jaco_msgs::GetCartesianPosition posSrv;
  if (!armCartesianPositionClient.call(posSrv))
  {
    ROS_INFO("Could not call arm Cartesian position service.");
    return;
  }
  wpi_jaco_msgs::CartesianCommand cmd;
  cmd.armCommand = true;
  cmd.fingerCommand = false;
  cmd.position = false;
  cmd.repeat = true;
  cmd.arm.linear.x = 0;
  cmd.arm.linear.y = 0;
  cmd.arm.linear.z = 0;
  cmd.arm.angular.x = 0;
  cmd.arm.angular.y = 0;
  cmd.arm.angular.z = 0;

  vector<float> error;
  error.resize(3);
  error[0] = posSrv.response.pos.linear.x - markerPose[0];
  error[1] = posSrv.response.pos.linear.y - markerPose[1];
  error[2] = posSrv.response.pos.linear.z - markerPose[2];
  /* Currently using translation only, as the rotations don't seem to move the arm into very dangerous positions
  error[3] = posSrv.response.pos.angular.x - markerPose[3];
  error[4] = posSrv.response.pos.angular.y - markerPose[4];
  error[5] = posSrv.response.pos.angular.z - markerPose[5];
  for (unsigned int i = 3; i <= 5; i ++)
  {
    if (error[i] > M_PI)
      error[i] -= M_PI;
    else if (error[i] < -M_PI)
      error[i] += M_PI;
  }
  */
  float maxError;
  if (fabs(error[0]) > fabs(error[1]) && fabs(error[0]) > fabs(error[2]))
  {
    if (error[0] < 0)
      cmd.arm.linear.x = -.175;
    else
      cmd.arm.linear.x = .175;

    maxError = error[0];
  }
  else if (fabs(error[1]) > fabs(error[0]) && fabs(error[1]) > fabs(error[2]))
  {
    if (error[1] < 0)
      cmd.arm.linear.y = -.175;
    else
      cmd.arm.linear.y = .175;
    maxError = error[1];
  }
  else
  {
    if (error[2] < 0)
      cmd.arm.linear.z = -.175;
    else
      cmd.arm.linear.z = .175;
    maxError = error[2];
  }

  //ignore if error is less than 1 cm to prevent accidental clicks from starting a recovery behavior
  if (fabs(maxError) < .01)
  {
    return;
  }
  else
    disableArmMarkerCommands = true;

  wpi_jaco_msgs::EStop eStopSrv;
  eStopSrv.request.enableEStop = true;
  if (!armEStopClient.call(eStopSrv))
  {
    ROS_INFO("Could not call arm EStop service.");
    return;
  }

  eStopSrv.request.enableEStop = false;
  if (!armEStopClient.call(eStopSrv))
  {
    ROS_INFO("Could not call arm EStop service.");
    return;
  }

  ros::Rate loopRate(60);
  for (unsigned int i = 0; i < 60; i ++)
  {
    cartesianCmd.publish(cmd);
    loopRate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jaco_interactive_manipulation");

  CarlInteractiveManipulation cim;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    cim.updateMarkerPosition();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

