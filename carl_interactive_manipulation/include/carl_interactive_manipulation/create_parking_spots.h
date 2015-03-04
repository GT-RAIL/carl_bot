
/*!
 * \create_parking_spots.h
 * \brief Creates clickable parking spots for web interface/rviz
 *
 * create_parking_spots runs a ROS interactive marker server. It creates clickable markers that send move_base goals to carl. 
 * The positions of the markers are defined in the rail_collada_models package, and the ilab_description must be loaded on the parameter
 * server for parking spots to be generated.
 *
 * \author Peter Mitrano, WPI - pdmitrano@wpi.edu
 * \date February 11, 2015
 */

#ifndef CREATE_PARKING_SPOTS_H_
#define CREATE_PARKING_SPOTS_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <urdf/model.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*!
 * \class ParkingSpots
 * \brief Creates clickable parking spots for carl
 */

class CreateParkingSpots{

public:

	/**
  	* \brief Constructor
	*	This starts the server, waits for param server and create clickable parking spots
  	*/
	CreateParkingSpots();

private:

	MoveBaseClient client_; //!< uses move base client to set nav goals

	interactive_markers::InteractiveMarkerServer server_; //!< creates marker server for clickable parking spots
	
	/**
   	* /brief Creates an interactive marker for the given link
   	* @param frame_id the frame id, or link name, to be used to create the marker
   	* @return the interactive marker to be used as a clickable parking spot
   	*/
	visualization_msgs::InteractiveMarker createParkingSpot(std::string frame_id);
	
	/**
	* /brief Callback function for when marker is clicked. Sets move_base goal to carl
   	* @param f contains information about where the marker was clicked, if it was a press or release, and the marker pose
   	*/
	void onClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f);
	
	/**
   	* /brief Check if a given link name ends with nav_goal, and should therefore have a parknig spot made at its location
   	* @param link_name the frame id or name of the link
   	* @return if the link ends in nav_goal and needs a marker
   	*/
	bool isNavGoal(std::string link_name);	
};

#endif