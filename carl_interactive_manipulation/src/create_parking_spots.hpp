#ifndef CREATE_PARKING_SPOTS_H_
#define CREATE_PARKING_SPOTS_H_

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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ParkingSpots{

public:

	ParkingSpots();

private:

	static MoveBaseClient client_;
	static void OnClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f);
	visualization_msgs::InteractiveMarker CreateParkingSpot(std::string frame_id);
	bool IsNavGoal(std::string link_name);	
};

#endif