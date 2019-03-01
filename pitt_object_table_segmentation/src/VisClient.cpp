/**
 *  This is the source file of a service client VisCLient. It acts as a client to the ER server, and it is also a subscriber to the topic /scene_data.
 */

#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "armor_msgs/ArmorDirectiveReq.h"
#include "armor_msgs/ArmorDirectiveRes.h"
#include "armor_msgs/ArmorDirective.h"
#include "armor_msgs/ArmorDirectiveList.h"
#include <vector>
#include <iostream>
#include <map>
#include <cstdlib>
#include "pitt_msgs/Attribute.h" /**< For out message type which tells attributes of a single shape. */
#include "pitt_msgs/All_shapes_attributes.h" /**< For out message to Encoding reasoner. */

using namespace pitt_msgs;
using namespace armor_msgs;
using namespace ros;

/**
 *  This fuction interprets an int type variable into string type.
 *  @param n Input an integer.
 *  @return A string.
 */
std::string int_to_string(int n)
{
	std::string string;
	std::stringstream ss;
	ss << n;
	string = ss.str();

	return string;
}

/**
 *  This fuction is called when a property to be added to the ontology.
 *  @param string Property name.
 *  @param object Indevidual name.
 *  @param value  The value of property to be added.
 *  @see add_object
 *  @return The request to be sent to ARMOR.
 */
ArmorDirectiveReq add_prop(std::string string, std::string object, int value)
{
	ArmorDirectiveReq req;
	std::vector<std::string> args;
	req.client_name = "VisClient";
	req.reference_name = "ontology";
	req.command = "ADD";
	req.primary_command_spec = "DATAPROP";
	req.secondary_command_spec = "IND";
	args.push_back(string);
	args.push_back(object);
	if (string == "shape" || string == "color") {
		args.push_back("STRING");
		args.push_back(int_to_string(value));
	}
	else {
		args.push_back("INTEGER");
		args.push_back(int_to_string(value));	// args[] is an array of string
	}
	req.args = args;

	return req;
}

/**
 *  This fuction is called when an object to be added to the ontology. The function add_prop is called multipe times to add every properties of the object to the ontology.
 *  @param id Object id.
 *  @param shape The shape of the object.
 *  @param color The color of the object.
 *  @param x_centr X coordiante of the centroid.
 *  @param y_centr Y coordiante of the centroid.
 *  @param z_centr Z coordiante of the centroid.
 *  @param radius The radius of the objetc.
 *  @param x_axis The orientation error between the x axis of the Baxter and the x axis of the object.
 *  @param y_axis The orientation error between the y axis of the Baxter and the y axis of the object.
 *  @param z_axis The orientation error between the z axis of the Baxter and the z axis of the object.
 *  @param obj The name of the object(individual).
 *  @see add_prop
 */
void add_object(int id, std::string shape, std::string color, int x_centr, int y_centr, int z_centr, int radius, int x_axis, int y_axis, int z_axis, int obj, std::vector<ArmorDirectiveReq>* allRequests)
{
	std::string id_string = int_to_string(id);
	std::string obj_string = int_to_string(obj);
	std::string x_c_str = int_to_string(x_centr);
	std::string y_c_str = int_to_string(y_centr);
	std::string z_c_str = int_to_string(z_centr);
	std::string radius_str = int_to_string(radius);
	std::string x_axis_str = int_to_string(x_axis);
	std::string y_axis_str = int_to_string(y_axis);
	std::string z_axis_str = int_to_string(z_axis);

	//initialize the request
	allRequests->push_back(add_prop(id_string, obj_string, id));
	allRequests->push_back(add_prop(shape, obj_string, atoi(shape.c_str())));
	allRequests->push_back(add_prop(color, obj_string, atoi(color.c_str())));
	allRequests->push_back(add_prop(x_c_str, obj_string, x_centr));
	allRequests->push_back(add_prop(y_c_str, obj_string, y_centr));
	allRequests->push_back(add_prop(z_c_str, obj_string, z_centr));
	allRequests->push_back(add_prop(radius_str, obj_string, radius));
	allRequests->push_back(add_prop(x_axis_str, obj_string, x_axis));
	allRequests->push_back(add_prop(y_axis_str, obj_string, y_axis));
	allRequests->push_back(add_prop(z_axis_str, obj_string, z_axis));
}

/**
 * This function is the callback function of the subscriber.
 * @param msg The message get from the topic scene_data.
 */
void scene_data_callback(const All_shapes_attributes::Ptr& msg)
{
	NodeHandle srn;	// declare a node handle
	ServiceClient client = srn.serviceClient<ArmorDirectiveList>("/armor_interface_serialized_srv");	// client to ARMOR
	ArmorDirectiveList srv;

    std::vector<ArmorDirectiveReq> allRequests;

	// beginning identifier: LOAD_INIT_
	    ArmorDirectiveReq reqinit;
		std::vector<std::string> argsinit;
		reqinit.client_name = "VisClient";
		reqinit.reference_name = "ontology";
		reqinit.command = "LOAD";
		reqinit.primary_command_spec = "FILE";
		reqinit.secondary_command_spec = "";
		argsinit.push_back("/Downloads/");
		argsinit.push_back("http://www.semanticweb.org/emaroLab/luca-buoncompagni/sit");
		argsinit.push_back("true");
		argsinit.push_back("PELLET");
		argsinit.push_back("true");
		reqinit.args = argsinit;


    allRequests.push_back(reqinit);
    // declare the request to be sent to the server
	ROS_INFO("The number of objects is %d", msg->VisOutput.size());
	for (int j = 0; j < msg->VisOutput.size(); j++)	// add the objects from scene_data one by one, calling the fuction add_object
	{
		add_object(msg->VisOutput[j].object_id, msg->VisOutput[j].shape_tag, msg->VisOutput[j].color_tag, msg->VisOutput[j].Centroid[0], msg->VisOutput[j].Centroid[1], msg->VisOutput[j].Centroid[2], msg->VisOutput[j].radius, msg->VisOutput[j].Axis[0], msg->VisOutput[j].Axis[1], msg->VisOutput[j].Axis[2], j, &allRequests);
	}

	// ending identifier: SCENE_UPDATED_
	ArmorDirectiveReq reqend;
	std::vector<std::string> argsend;
	reqend.client_name = "VisClient";
	reqend.reference_name = "ontology";
	reqend.command = "SCENE";
	reqend.primary_command_spec = "UPDATED";
	reqend.secondary_command_spec = "NULL";
	argsend.push_back("true");
	reqend.args = argsend;
	allRequests.push_back(reqend);

	// call the service
	//ROS_INFO("Size of all req is %d", allRequests.size());
	//ROS_INFO("Size of all req is %s", allRequests[0].command.c_str());
	//ROS_INFO("Size of all req is %s", allRequests[allRequests.size()-1].command.c_str());
	srv.request.armor_requests = allRequests;
	if (client.call(srv))
	{
		ROS_INFO("Number of responses are %d", srv.response.armor_responses.size());
		ROS_INFO("Success: %d", srv.response.armor_responses[0].success);
	}
	else
	{
		ROS_ERROR("Failed to call service");
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");	// initialize
	ros::NodeHandle n;	// declare a nodehandle

	ros::Subscriber sub2 = n.subscribe("scene_data", 1, scene_data_callback);	// subscribe to the topic scene_data
	ros::spin();

	return 0;
}
