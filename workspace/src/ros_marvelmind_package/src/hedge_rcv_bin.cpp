#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "marvelmind_nav/hedge_pos.h"
extern "C" 
{
#include "marvelmind_nav/marvelmind_hedge.h"
}

#include <sstream>

#define ROS_NODE_NAME "hedge_rcv_bin"
#define POSITION_TOPIC_NAME "/hedge_pos"

struct MarvelmindHedge * hedge= NULL;

static uint32_t hedge_timestamp_prev= 0;
marvelmind_nav::hedge_pos hedge_pos_msg;// message for publishing to ROS topic

////////////////////////////////////////////////////////////////////////

static int hedgeReceivePrepare(int argc, char **argv)
{
	 // get port name from command line arguments (if specified)
    const char * ttyFileName;
    if (argc==2) ttyFileName=argv[1];
    else ttyFileName=DEFAULT_TTY_FILENAME;
    
    // Init
    hedge=createMarvelmindHedge ();
    if (hedge==NULL)
    {
        ROS_INFO ("Error: Unable to create MarvelmindHedge");
        return -1;
    }
    hedge->ttyFileName=ttyFileName;
    hedge->verbose=true; // show errors and warnings
    startMarvelmindHedge (hedge);
}

static bool hedgeReceiveCheck(void)
{
  if (hedge->haveNewValues_)
    {
        struct PositionValue position;
        getPositionFromMarvelmindHedge (hedge, &position);
        
        hedge_pos_msg.flags= position.flags;
        if (hedge_pos_msg.flags&(1<<1))// flag of timestamp format 
          {
			hedge_pos_msg.timestamp_ms= position.timestamp;// msec
		  }	
	     else 
	      {
            hedge_pos_msg.timestamp_ms= position.timestamp*15.625;// alpha-cycles ==> msec
          } 
          
        hedge_pos_msg.x_m= position.x/100.0; 
        hedge_pos_msg.y_m= position.y/100.0; 
        hedge_pos_msg.z_m= position.z/100.0; 
        
        hedge->haveNewValues_=false;
    }
}

/**
 * Node for Marvelmind hedgehog binary streaming data processing
 */
int main(int argc, char **argv)
{
  // initialize ROS node
  ros::init(argc, argv, ROS_NODE_NAME);
  
  // prepare hedgehog data receiver module
  hedgeReceivePrepare(argc, argv);

  // ROS node reference 
  ros::NodeHandle n;

  // Register topic for puplishing messages
  ros::Publisher hedge_pos_publisher = n.advertise<marvelmind_nav::hedge_pos>(POSITION_TOPIC_NAME, 1000);

  // 50 Hz 
  ros::Rate loop_rate(50);

  // default values for position message
  hedge_pos_msg.timestamp_ms = 0;
  hedge_pos_msg.x_m = 0.0;
  hedge_pos_msg.y_m = 0.0;
  hedge_pos_msg.z_m = 0.0;
  hedge_pos_msg.flags = (1<<0);// 'data not available' flag

  
  while (ros::ok())
  {
    if (hedge->terminationRequired)
      {
		  break;
      }	  
	  
    if (hedgeReceiveCheck())
     {// hedgehog data received
		ROS_INFO("%d, %d, X=%.3f  Y= %.3f  Z=%.3f   flags=%d", 	
				(int) hedge_pos_msg.timestamp_ms, 
				(int) (hedge_pos_msg.timestamp_ms - hedge_timestamp_prev),
				(float) hedge_pos_msg.x_m, (float) hedge_pos_msg.y_m, (float) hedge_pos_msg.z_m,  
				(int) hedge_pos_msg.flags);
        hedge_pos_publisher.publish(hedge_pos_msg);
        
        hedge_timestamp_prev= hedge_pos_msg.timestamp_ms;
     }   

    ros::spinOnce();

    loop_rate.sleep();
  }

  // Exit
  if (hedge != NULL) 
    {
      stopMarvelmindHedge (hedge);
      destroyMarvelmindHedge (hedge);
    }

  return 0;
}
