#include <carl_interactive_manipulation/create_parking_spots.h>

/**
\brief runs an interacive marker server creates clickable navigation goals infront of furniture
*/
int main(int argc, char** argv)
{
  ros::init(argc,argv,"create_parking_spots");

  CreateParkingSpots parkingSpots;
 
  ros::spin();

  return EXIT_SUCCESS;
}

CreateParkingSpots::CreateParkingSpots() : client_("move_base",true),server_("parking_markers")
{
  ROS_INFO("waiting for server...");
  
  bool status = client_.waitForServer();
  if (!status){
    ROS_INFO("Couldn't connect in time...");
  }
  else
  {

  ROS_INFO("connected to server");

  urdf::Model ilab;

  //load the urdf with all the furniture off the param server
  if (!ilab.initParam("/ilab_description"))
  {
    ROS_INFO("couldn't find /ilab_description on param server.");
  }
  else 
  {

    std::map<std::string, boost::shared_ptr<urdf::Link> > links = ilab.links_;
    std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator itr;
    
    //go through all links and filter out the ones that end in "nav_goal_link"
    for(itr = links.begin(); itr != links.end(); itr++) 
    {
      std::string link_name = itr->first;
      if (isNavGoal(link_name))
      {
        visualization_msgs::InteractiveMarker marker = createParkingSpot(link_name);
        ROS_INFO("creating marker %s",link_name.c_str());
        server_.insert(marker,boost::bind(&CreateParkingSpots::onClick, this, _1));
      }
    }

    ROS_INFO("creating clickable nav goals...");

    //when these are called the markers will actually appear
    server_.applyChanges();  
    
    }
  }
}

void CreateParkingSpots::onClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f)
{
  if (f->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP){

    //copy header and pose from marker to new action goal
    //rotate 90 deg around z axis
    geometry_msgs::Pose new_pose = f->pose;
    new_pose.orientation.x = 0;
    new_pose.orientation.y = 0;
    new_pose.orientation.z = 1;
    new_pose.orientation.w = 1;

    //create pose
    geometry_msgs::PoseStamped target_pose;
    target_pose.header = f->header;
    target_pose.pose = new_pose;

    //create goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = target_pose;

    //send action goal
    client_.sendGoal(goal);

    bool finished_before_timeout = client_.waitForResult();

    if (finished_before_timeout)
    {
      ROS_INFO("finished successfully!");
    }
    else 
    {
      ROS_INFO("timed out! goal not reached in 30 seconds");
    }
  }   
}

bool CreateParkingSpots::isNavGoal(std::string link_name)
{
  std::string search_param = "nav_goal_link";
  return link_name.find(search_param,link_name.length()-search_param.length()) != std::string::npos;
}

visualization_msgs::InteractiveMarker CreateParkingSpots::createParkingSpot(std::string frame_id)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.scale = 1;
  int_marker.name = frame_id + "_parking_spot";
  
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.name = "button";
  
  visualization_msgs::Marker box;
  box.type = visualization_msgs::Marker::CUBE;
  box.scale.x = 0.15;
  box.scale.y = 0.15;
  box.scale.z = 0.05;
  box.color.r = 0;
  box.color.g = 0.5;
  box.color.b = 0.25;
  box.color.a = 1;
 
  control.markers.push_back(box);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  return int_marker;
}