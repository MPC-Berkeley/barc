#include "ros/ros.h"
#include "std_msgs/String.h"
#include "marvelmind_nav/hedge_pos.h"
#include <visualization_msgs/Marker.h>

#define ROS_NODE_NAME "subscriber_test"
#define HEDGE_POSITION_TOPIC_NAME "/hedge_pos"

#define SCALE_HEDGE 3.0

ros::Publisher rviz_marker_pub;
uint32_t rviz_shape;


void showRvizObject(float x, float y, float z) 
{
	if (rviz_marker_pub.getNumSubscribers() < 1) return;
	
	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type
    marker.type = rviz_shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.05*SCALE_HEDGE;
    marker.scale.y = 0.05*SCALE_HEDGE;
    marker.scale.z = 0.02*SCALE_HEDGE;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(5);
     
    rviz_marker_pub.publish(marker);
}

void hedgePosCallback(const marvelmind_nav::hedge_pos& hedge_pos_msg)
{
  ROS_INFO("Hedgehog data: %d, X=%.3f  Y= %.3f  Z=%.3f  flags=%d", 	
				(int) hedge_pos_msg.timestamp_ms, 
				(float) hedge_pos_msg.x_m, (float) hedge_pos_msg.y_m, (float) hedge_pos_msg.z_m,  
				(int) hedge_pos_msg.flags);
				
  if ((hedge_pos_msg.flags&(1<<0))==0)
    {				
      showRvizObject(hedge_pos_msg.x_m, hedge_pos_msg.y_m, hedge_pos_msg.z_m);
    }  
}

/**
 * Test subscriber node for getting data from Marvelmind publishers nodes
 */
int main(int argc, char **argv)
{
	
  // initialize ROS node
  ros::init(argc, argv, ROS_NODE_NAME);

  // ROS node reference 
  ros::NodeHandle n;

  // Declare need to subscribe data from topic
  ros::Subscriber sub = n.subscribe(HEDGE_POSITION_TOPIC_NAME, 1000, hedgePosCallback);
  
  // Declare publisher for rviz visualization
  rviz_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Set our initial shape type to be a cube
  rviz_shape = visualization_msgs::Marker::CUBE;

  ros::spin();

  return 0;
}
