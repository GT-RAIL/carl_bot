#include "carl_dynamixel/front_joints_node.h"

/*
 * Constructor to inialize the node
 */
back_joints::back_joints() 
{
	std::string servo_name;
	int num, id;
	
	ros::NodeHandle private_node_handle_("~");
	node.getParam("front_servos/num_servos", num);
	for(int s = 1; s <= num; s++){
		std::stringstream ss;
		ss<<s;
		std::string servo_num = ss.str();
		std::string param_id ("front_servos/" + servo_num + "/id");
		std::string param_name ("front_servos/" + servo_num + "/link_name");
		node.getParam(param_id,id);
		node.getParam(param_name,servo_name);
		back_servos[id] = servo_name;
	}
	
	//setup the subscriber
    creative_sub = node.subscribe("motor_states/front_port", 1000, &back_joints::joints, this);
    
    //setup the publisher
	joint_states_pub = node.advertise<sensor_msgs::JointState>("dynamixel_front", 1);
    
    
}

void back_joints::joints(const dynamixel_msgs::MotorStateList::ConstPtr& state){
	sensor_msgs::JointState back_states;
	for(int s = 0; s < state->motor_states.size(); s++){
		dynamixel_msgs::MotorState servo = state->motor_states[s];
		std::string name = back_servos[servo.id];
		if(name.size() == 0){
			name = "UNDEFINED";
		}
		back_states.name.push_back(name);
		float servo_pos_deg = ((float)servo.position-512.0)*0.2929;
		back_states.position.push_back((servo_pos_deg*M_PI)/180.0);
	}
	back_states.header.stamp = ros::Time::now();
	joint_states_pub.publish(back_states);
}

/*
 * main function
 */
int main(int argc, char **argv)
{
	//initialize the node
	ros::init(argc, argv, "front_dynamixel");

	// initialize the joystick controller
	back_joints back;

	//main loop
	ros::Rate loop_rate(30);
	while (ros::ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return EXIT_SUCCESS;
}
