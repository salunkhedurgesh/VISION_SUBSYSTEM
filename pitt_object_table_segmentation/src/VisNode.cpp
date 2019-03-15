/**
 * This code is a publisher-subscriber node. It subscribes to /ransac_segmentation/trackedShapes and publishes on /scene_data
 */
#include "ros/ros.h"
#include "std_msgs/String.h"	//what do we need to include?
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "pitt_msgs/ClustersOutput.h" // for cluster input (msg(
#include "pitt_msgs/InliersCluster.h" // for cluster input (sub msg)
#include "pitt_msgs/PrimitiveSegmentation.h" // for ransac service output
#include "pitt_msgs/TrackedShapes.h" // for out message (an array of TrackedShape)
#include "pitt_msgs/TrackedShape.h" // for out message
#include "pitt_msgs/Attribute.h" // for out message type which tells attributes of a single shape
#include "pitt_msgs/All_shapes_attributes.h" // for out message t Encoding reasoner
#include "point_cloud_library/pc_manager.h" // for my static library
#include "point_cloud_library/srv_manager.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>

using namespace ros;
using namespace pcm;
using namespace srvm;
using namespace pitt_msgs;

/**
 * This function finds the highest voted shape
 * @param a : The number of times the object was detected as a plane
 * @param b : The number of times the object was detected as a sphere
 * @param c : The number of times the object was detected as a cone
 * @param d : The number of times the object was detected as a cylinder
 * @return The max voted shape as a string, for eg: "Cone"
 */
std::string findMax(int a, int b, int c, int d)
{
  struct shapecount{
    int data;
    string shape;
  };
  struct shapecount nums[4];

  nums[0].data = a; nums[0].shape = "Plane";
  nums[1].data = b; nums[1].shape = "Sphere";
  nums[2].data = c; nums[2].shape = "Cone";
  nums[3].data = d; nums[3].shape = "Cylinder";

  int max=0;
  int ind_max=0;

  // find the max
  for (int i = 0; i <4; i++)
  {
    if(nums[i].data>max)
    {
        ind_max=i;
        max=nums[i].data;
    }
  }
    //ROS_INFO("IND MAX is %d", ind_max);
    //ROS_INFO("A B C D are  %d %d %d %d", a,b,c,d);
  return(nums[ind_max].shape);
}

/**
 * This declares a struct that stores the x, y and z co-ordinates of the centroid as an entity
 */
struct centroid{

  float x_c;
  float y_c;
  float z_c;

};

/**
 * This declares a struct that stores the x, y and z axis versor lemma of the detected object as an entity
 */
struct axis{

  float x_axis;
  float y_axis;
  float z_axis;

};

/**
 * This declares a struct that stores the attributes of each object
 */
struct object{
  int id;
  string shape;
  string color;
  float radius;
  float height;
  struct centroid centr;
  struct axis axis;

};

struct object obj[10]; //we assume to have max 10 objects on the table

int k[10]={0};
int cylinder[10]={0};
int sphere[10]={0};
int cone[10]={0};
int plane[10]={0};

struct object obj_cyl[10];
struct object obj_sph[10];
struct object obj_cone[10];
struct object obj_plane[10];

struct object optimized_obj[10];

Publisher Scene_data_pub; // to publish out results

/**
 * This a callback function for the subscriber to /ransac_segmentation/trackedShapes topic
 * @param msg : The data published on the /ransac_segmentation/trackedShapes topic
 */

void ransacCallback(const TrackedShapes::Ptr& msg)
{
  int num= msg->tracked_shapes.size();

  ROS_INFO("NUM %d", num);
  /***
   * The following code reads the array of the attributes related to each object
   * Further, it checks for the shape and updates the relevant attributes
   * The output stores the average of all the attributes discretely
   */
  for(int count = 0;count<num;count++)
    {
       obj[count].id=msg->tracked_shapes[count].object_id;
       obj[count].shape=msg->tracked_shapes[count].shape_tag;
       obj[count].centr.x_c= msg->tracked_shapes[count].x_est_centroid;
       obj[count].centr.y_c= msg->tracked_shapes[count].y_est_centroid;
       obj[count].centr.z_c= msg->tracked_shapes[count].z_est_centroid;
       obj[count].color = msg->tracked_shapes[count].color.data;


        if (obj[count].shape== "Cylinder")
        {
            obj[count].axis.x_axis = msg->tracked_shapes[count].coefficients[3];
            obj[count].axis.y_axis = msg->tracked_shapes[count].coefficients[4];
            obj[count].axis.z_axis = msg->tracked_shapes[count].coefficients[5];
            obj[count].radius = msg->tracked_shapes[count].coefficients[6];
            obj[count].height = msg->tracked_shapes[count].coefficients[7];
        }

        if (obj[count].shape== "Cone")
        {
            obj[count].axis.x_axis = msg->tracked_shapes[count].coefficients[3];
            obj[count].axis.y_axis = msg->tracked_shapes[count].coefficients[4];
            obj[count].axis.z_axis = msg->tracked_shapes[count].coefficients[5];
            obj[count].height = msg->tracked_shapes[count].coefficients[7];
            obj[count].radius = msg->tracked_shapes[count].coefficients[8];
        }

        if(obj[count].shape== "Sphere")
        {
            obj[count].axis.x_axis = 0;
            obj[count].axis.y_axis = 0;
            obj[count].axis.z_axis = 0;

            obj[count].height =0;
            obj[count].radius = msg->tracked_shapes[count].coefficients[3];
        }

        if(obj[count].shape== "Plane")
        {
            obj[count].axis.x_axis = 0;
            obj[count].axis.y_axis = 0;
            obj[count].axis.z_axis = 0;

            obj[count].height =0.2; //default value
            obj[count].radius = 0;

        }


       /**
        * comparing id -->put the optimized values to zero if the scene has changed
        */

       /**
        * THE COMMENTED CODE -> TO BE UNCOMMENTED AFTER WE ARE SURE THAT THE PITT algorithm is robust enough and does not
        * give different scenes when the actual scene has not changed
        * PLEASE READ THE REPORT -> Recommandation -> Vision Module TO UNDERSTAND BETTER
        */

       /*if(k[count]=!obj[count].id) //scene changed
         {
             cylinder[count]=0;
             sphere[count]=0;
             cone[count]=0;
             plane[count]=0;

             obj_cyl[count].radius=0;
             obj_sph[count].radius=0;
             obj_cone[count].radius=0;
             obj_plane[count].radius=0;

             obj_cyl[count].height=0;
             obj_sph[count].height=0;
             obj_cone[count].height=0;
             obj_plane[count].height=0;

             obj_cyl[count].centr.x_c=0;
             obj_cyl[count].centr.y_c=0;
             obj_cyl[count].centr.z_c=0;

             obj_sph[count].centr.x_c=0;
             obj_sph[count].centr.y_c=0;
             obj_sph[count].centr.z_c=0;

             obj_cone[count].centr.x_c=0;
             obj_cone[count].centr.y_c=0;
             obj_cone[count].centr.z_c=0;

             obj_plane[count].centr.x_c=0;
             obj_plane[count].centr.y_c=0;
             obj_plane[count].centr.z_c=0;


             obj_cyl[count].axis.x_axis=0;
             obj_cyl[count].axis.y_axis=0;
             obj_cyl[count].axis.z_axis=0;

             obj_sph[count].axis.x_axis=0;
             obj_sph[count].axis.y_axis=0;
             obj_sph[count].axis.z_axis=0;

             obj_cone[count].axis.x_axis=0;
             obj_cone[count].axis.y_axis=0;
             obj_cone[count].axis.z_axis=0;

             obj_plane[count].axis.x_axis=0;
             obj_plane[count].axis.y_axis=0;
             obj_plane[count].axis.z_axis=0;

           }*/


       /**
        * The following code updates the relevant counts for the detected shape and calculates the new average
        * for the attributes
        */

       if(obj[count].shape=="Cylinder")
        {
             cylinder[count]++; //counting how many times a cylinder is detected

             //optimizing the parameters

             obj_cyl[count].radius= (obj_cyl[count].radius*(cylinder[count]-1)+obj[count].radius)/cylinder[count];

             obj_cyl[count].height= (obj_cyl[count].height*(cylinder[count]-1)+obj[count].height)/cylinder[count];

             obj_cyl[count].centr.x_c= (obj_cyl[count].centr.x_c*(cylinder[count]-1)+obj[count].centr.x_c)/cylinder[count];
             obj_cyl[count].centr.y_c= (obj_cyl[count].centr.y_c*(cylinder[count]-1)+obj[count].centr.y_c)/cylinder[count];
             obj_cyl[count].centr.z_c= (obj_cyl[count].centr.z_c*(cylinder[count]-1)+obj[count].centr.z_c)/cylinder[count];

             obj_cyl[count].axis.x_axis= (obj_cyl[count].axis.x_axis*(cylinder[count]-1)+obj[count].axis.x_axis)/cylinder[count];
             obj_cyl[count].axis.y_axis= (obj_cyl[count].axis.y_axis*(cylinder[count]-1)+obj[count].axis.y_axis)/cylinder[count];
             obj_cyl[count].axis.z_axis= (obj_cyl[count].axis.z_axis*(cylinder[count]-1)+obj[count].axis.z_axis)/cylinder[count];

          }

          if(obj[count].shape=="Sphere")
          {

            sphere[count]++;  //counting how many times a sphere is detected

            //optimizing the parameters

            obj_sph[count].radius= (obj_sph[count].radius*(sphere[count]-1)+obj[count].radius)/sphere[count];

            obj_sph[count].height= (obj_sph[count].height*(sphere[count]-1)+obj[count].height)/sphere[count];

            obj_sph[count].centr.x_c= (obj_sph[count].centr.x_c*(sphere[count]-1)+obj[count].centr.x_c)/sphere[count];
            obj_sph[count].centr.y_c= (obj_sph[count].centr.y_c*(sphere[count]-1)+obj[count].centr.y_c)/sphere[count];
            obj_sph[count].centr.z_c= (obj_sph[count].centr.z_c*(sphere[count]-1)+obj[count].centr.z_c)/sphere[count];

            obj_sph[count].axis.x_axis=0;
            obj_sph[count].axis.y_axis=0;
            obj_sph[count].axis.z_axis=0;

          }

          if(obj[count].shape=="Cone")
          {

            cone[count]++; //counting how many times a cone is detected

            //optimizing the parameters

            obj_cone[count].radius= (obj_cone[count].radius*(cone[count]-1)+obj[count].radius)/cone[count];

            obj_cone[count].height= (obj_cone[count].height*(cone[count]-1)+obj[count].height)/cone[count];

            obj_cone[count].centr.x_c= (obj_cone[count].centr.x_c*(cone[count]-1)+obj[count].centr.x_c)/cone[count];
            obj_cone[count].centr.y_c= (obj_cone[count].centr.y_c*(cone[count]-1)+obj[count].centr.y_c)/cone[count];
            obj_cone[count].centr.z_c= (obj_cone[count].centr.z_c*(cone[count]-1)+obj[count].centr.z_c)/cone[count];

            obj_cone[count].axis.x_axis= (obj_cone[count].axis.x_axis*(cone[count]-1)+obj[count].axis.x_axis)/cone[count];
            obj_cone[count].axis.y_axis= (obj_cone[count].axis.y_axis*(cone[count]-1)+obj[count].axis.y_axis)/cone[count];
            obj_cone[count].axis.z_axis= (obj_cone[count].axis.z_axis*(cone[count]-1)+obj[count].axis.z_axis)/cone[count];

          }

          if(obj[count].shape=="Plane")
          {
          	 plane[count]++; //counting how many times a plane is detected

             //optimizing the parameters

             obj_plane[count].radius= (obj_plane[count].radius*(plane[count]-1)+obj[count].radius)/plane[count];

             obj_plane[count].height= (obj_plane[count].height*(plane[count]-1)+obj[count].height)/plane[count];

             obj_plane[count].centr.x_c= (obj_plane[count].centr.x_c*(plane[count]-1)+obj[count].centr.x_c)/plane[count];
             obj_plane[count].centr.y_c= (obj_plane[count].centr.y_c*(plane[count]-1)+obj[count].centr.y_c)/plane[count];
             obj_plane[count].centr.z_c= (obj_plane[count].centr.z_c*(plane[count]-1)+obj[count].centr.z_c)/plane[count];

             obj_plane[count].axis.x_axis= (obj_plane[count].axis.x_axis*(plane[count]-1)+obj[count].axis.x_axis)/plane[count];
             obj_plane[count].axis.y_axis= (obj_plane[count].axis.y_axis*(plane[count]-1)+obj[count].axis.y_axis)/plane[count];
             obj_plane[count].axis.z_axis= (obj_plane[count].axis.z_axis*(plane[count]-1)+obj[count].axis.z_axis)/plane[count];

          }


        ROS_INFO("Values: cylinder %d sphere %d cone %d plane %d", cylinder[count], sphere[count], cone[count], plane[count]);


        /**
         * INITIALIZING THE OPTIMIZED STRUCTURE
         */


        //initializing id
         optimized_obj[count].id=obj[count].id;

        //initializing the official shape depending on the counter
        optimized_obj[count].shape=findMax(plane[count],sphere[count],cone[count],cylinder[count]);

        ROS_INFO("The official shape is %s",optimized_obj[count].shape.c_str());

        //initializing the official color
        optimized_obj[count].color= obj[count].color;

       //average

        if (optimized_obj[count].shape=="Cylinder")
         {
           optimized_obj[count].radius=obj_cyl[count].radius;
           optimized_obj[count].height=obj_cyl[count].height;
           optimized_obj[count].centr=obj_cyl[count].centr;
           optimized_obj[count].axis=obj_cyl[count].axis;
          }

         if (optimized_obj[count].shape=="Sphere")
         {
           optimized_obj[count].radius=obj_sph[count].radius;
           optimized_obj[count].height=obj_sph[count].height;
           optimized_obj[count].centr=obj_sph[count].centr;
           optimized_obj[count].axis=obj_sph[count].axis;
        }

        if (optimized_obj[count].shape=="Cone")
         {
           optimized_obj[count].radius=obj_cone[count].radius;
           optimized_obj[count].height=obj_cone[count].height;
           optimized_obj[count].centr=obj_cone[count].centr;
           optimized_obj[count].axis=obj_cone[count].axis;
        }

        if (optimized_obj[count].shape=="Plane")
         {
           optimized_obj[count].radius=obj_plane[count].radius;
           optimized_obj[count].height=obj_plane[count].height;
           optimized_obj[count].centr=obj_plane[count].centr;
           optimized_obj[count].axis=obj_plane[count].axis;
        }


    }

    /**
     * initializing the message to be sent on scene_data
     */

    All_shapes_attributes::Ptr all_shapes ( new All_shapes_attributes);
    int j =0;
    while(j<2)
    {
      Attribute::Ptr one_shape ( new Attribute);
      one_shape->object_id = optimized_obj[j].id;
      one_shape->shape_tag = optimized_obj[j].shape;
      one_shape->color_tag = optimized_obj[j].color;
      one_shape->Centroid[0] = optimized_obj[j].centr.x_c;
      one_shape->Centroid[1] = optimized_obj[j].centr.y_c;
      one_shape->Centroid[2]= optimized_obj[j].centr.z_c;
      one_shape->radius = optimized_obj[j].radius;
      one_shape->Axis[0] = optimized_obj[j].axis.x_axis;
      one_shape->Axis[1] = optimized_obj[j].axis.y_axis;
      one_shape->Axis[2] = optimized_obj[j].axis.z_axis;

      all_shapes->VisOutput.push_back( *one_shape);
      j++;
    }
  Scene_data_pub.publish(all_shapes); //publishing the message on the topic
  }

/**
 * This is the main function where we declare the nodehandles and relevant subscribers and publishers
 * @param argc : The standard inputs
 * @param argv : The standard inputs
 * @return : A return value
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "VisNode");
  ros::NodeHandle sn;
  ros::NodeHandle pn;

  Subscriber sub = sn.subscribe("ransac_segmentation/trackedShapes", 10, ransacCallback); //subscribe to the topic: we need a buffer bc a message is only for one object
  Scene_data_pub = pn.advertise<All_shapes_attributes>("scene_data", 1);

  ros::spin();

  return 0;
}
