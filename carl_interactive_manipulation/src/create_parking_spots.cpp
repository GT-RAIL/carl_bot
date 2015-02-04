//adds interactive (clickable) markers and set those locations as navigation goals
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <urdf/model.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* Client_ptr;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

Client_ptr client_ptr;

//when you release the mouse on a marker this gets called
//eventually this will set the location of that marker as a nav goal for Carl
void onClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f){
  if (f->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP){
    
    ROS_INFO("marker clicked!");
    

    //copy header and pose from marker to new action goal
    //rotate 90 deg around z axis
    geometry_msgs::Pose new_pose = f->pose;
    new_pose.orientation.x=0;
    new_pose.orientation.y=0;
    new_pose.orientation.z=1;
    new_pose.orientation.w=1;

    //create pose
    geometry_msgs::PoseStamped target_pose;
    target_pose.header = f->header;
    target_pose.pose = new_pose;

    //create goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = target_pose;

    //send action goal
    client_ptr->sendGoal(goal);

    bool finished_before_timeout = client_ptr->waitForResult(ros::Duration(20.0));

    if (finished_before_timeout){
      ROS_INFO("finished successfully!");
    }
    else {
      ROS_INFO("timed out! That's dissappointing... :( #TheStuggleIsReal");
    }
  }   
}

//returns true only if the string ends in "nav_goal_link"
bool isNavGoal(std::string link_name){
  std::string search_param = "nav_goal_link";
  return link_name.find(search_param,link_name.length()-search_param.length()) != std::string::npos;
}

//creates a clickable marker at the origin of the given frame id
//this frame id is a string of the name of a link
visualization_msgs::InteractiveMarker createParkingSpot(std::string frame_id){
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.scale = 1;
  int_marker.name = frame_id+"_parking_spot";
  
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.name = "button";
  
  visualization_msgs::Marker box;
  box.type = visualization_msgs::Marker::CUBE;
  box.scale.x=0.15;
  box.scale.y=0.15;
  box.scale.z=0.05;
  box.color.r=0;
  box.color.g=0.5;
  box.color.b=0.25;
  box.color.a=1;
 
  control.markers.push_back(box);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  return int_marker;
} 

int main(int argc, char** argv){

  ros::init(argc,argv,"create_parking_spots");
  ros::NodeHandle node;
  ros::Rate rate(10.0);

  interactive_markers::InteractiveMarkerServer server("parking_markers");

  
  client_ptr = new Client("move_base", true);

  ROS_INFO("waiting for server...");
  
  client_ptr->waitForServer();

  ROS_INFO("connected to server");

  urdf::Model ilab;

  //load the urdf with all the furniture off the param server
  if (!ilab.initParam("/ilab_description")){
    ROS_INFO("couldn't find /ilab_description on param server.");
    return 1;
  }

  std::map<std::string, boost::shared_ptr<urdf::Link> > links = ilab.links_;
  std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator itr;
  
  //go through all links and filter out the ones that end in "nav_goal_link"
  for(itr = links.begin(); itr != links.end(); itr++) {
    std::string link_name = itr->first;
    if (isNavGoal(link_name)){
      server.insert(createParkingSpot(link_name), &onClick);
    }
  }

  ROS_INFO("creating clickable nav goals...");

  //when these are called the markers will actually appear
  server.applyChanges();
  ros::spin();


  return 0;
}
