#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "marvelmind_nav/hedge_pos.h"
#include "marvelmind_nav/hedge_pos_a.h"
#include "marvelmind_nav/beacon_pos_a.h"
extern "C" 
{
#include "marvelmind_nav/marvelmind_hedge.h"
}

#include <sstream>

#define ROS_NODE_NAME "hedge_rcv_bin"
#define HEDGE_POSITION_TOPIC_NAME "/hedge_pos"
#define HEDGE_POSITION_ADDRESSED_TOPIC_NAME "/hedge_pos_a"
#define BEACONS_POSITION_ADDRESSED_TOPIC_NAME "/beacons_pos_a"

struct MarvelmindHedge * hedge= NULL;

static uint32_t hedge_timestamp_prev= 0;
marvelmind_nav::hedge_pos hedge_pos_noaddress_msg;// hedge coordinates message (old version without address) for publishing to ROS topic
marvelmind_nav::hedge_pos_a hedge_pos_msg;// hedge coordinates message for publishing to ROS topic
marvelmind_nav::beacon_pos_a beacon_pos_msg;// stationary beacon coordinates message for publishing to ROS topic

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
        
        hedge_pos_msg.address= position.address;
        
        hedge_pos_msg.flags= position.flags;
        hedge_pos_noaddress_msg.flags= position.flags;
        if (hedge_pos_msg.flags&(1<<1))// flag of timestamp format 
          {
			hedge_pos_msg.timestamp_ms= position.timestamp;// msec
			hedge_pos_noaddress_msg.timestamp_ms= position.timestamp;
		  }	
	     else 
	      {
            hedge_pos_msg.timestamp_ms= position.timestamp*15.625;// alpha-cycles ==> msec
            hedge_pos_noaddress_msg.timestamp_ms= position.timestamp*15.625;
          } 
          
        hedge_pos_msg.x_m= position.x/1000.0; 
        hedge_pos_msg.y_m= position.y/1000.0; 
        hedge_pos_msg.z_m= position.z/1000.0; 
        
        hedge_pos_noaddress_msg.x_m= position.x/1000.0; 
        hedge_pos_noaddress_msg.y_m= position.y/1000.0; 
        hedge_pos_noaddress_msg.z_m= position.z/1000.0;
        
        hedge->haveNewValues_=false;
        
        return true;
    }
   return false;
}

static bool beaconReceiveCheck(void)
{
  uint8_t i;
  struct StationaryBeaconsPositions positions;
  struct StationaryBeaconPosition *bp= NULL;
  bool foundUpd= false;
  uint8_t n;
	
  getStationaryBeaconsPositionsFromMarvelmindHedge (hedge, &positions);
  n= positions.numBeacons;
  if (n == 0) 
	return false;
  
  for(i=0;i<n;i++)
  {
	  bp= &positions.beacons[i];
	  if (bp->updatedForMsg)
	  {
		  clearStationaryBeaconUpdatedFlag(hedge, bp->address);
		  foundUpd= true;
		  break;
	  } 
  }
  if (!foundUpd)
	return false;
  if (bp == NULL) 
	return false;
  	      
  beacon_pos_msg.address= bp->address;
  beacon_pos_msg.x_m= bp->x/1000.0; 
  beacon_pos_msg.y_m= bp->y/1000.0; 
  beacon_pos_msg.z_m= bp->z/1000.0; 
  
  return true;
}

/**
 * Node for Marvelmind hedgehog binary streaming data processing
 */
int main(int argc, char **argv)
{uint8_t beaconReadIterations;
  // initialize ROS node
  ros::init(argc, argv, ROS_NODE_NAME);
  
  // prepare hedgehog data receiver module
  hedgeReceivePrepare(argc, argv);

  // ROS node reference 
  ros::NodeHandle n;

  // Register topics for puplishing messages
  ros::Publisher hedge_pos_publisher = n.advertise<marvelmind_nav::hedge_pos_a>(HEDGE_POSITION_ADDRESSED_TOPIC_NAME, 1000);
  ros::Publisher hedge_pos_noaddress_publisher = n.advertise<marvelmind_nav::hedge_pos>(HEDGE_POSITION_TOPIC_NAME, 1000);
  ros::Publisher beacons_pos_publisher = n.advertise<marvelmind_nav::beacon_pos_a>(BEACONS_POSITION_ADDRESSED_TOPIC_NAME, 1000);

  // 50 Hz 
  ros::Rate loop_rate(50);

  // default values for position message
  hedge_pos_msg.address= 0;
  hedge_pos_msg.timestamp_ms = 0;
  hedge_pos_msg.x_m = 0.0;
  hedge_pos_msg.y_m = 0.0;
  hedge_pos_msg.z_m = 0.0;
  hedge_pos_msg.flags = (1<<0);// 'data not available' flag
  
  hedge_pos_noaddress_msg.timestamp_ms = 0;
  hedge_pos_noaddress_msg.x_m = 0.0;
  hedge_pos_noaddress_msg.y_m = 0.0;
  hedge_pos_noaddress_msg.z_m = 0.0;
  hedge_pos_noaddress_msg.flags = (1<<0);// 'data not available' flag
  
  beacon_pos_msg.address= 0;
  beacon_pos_msg.x_m = 0.0;
  beacon_pos_msg.y_m = 0.0;
  beacon_pos_msg.z_m = 0.0;

  
  while (ros::ok())
  {
    if (hedge->terminationRequired)
      {
		  break;
      }	  
	  
    if (hedgeReceiveCheck())
     {// hedgehog data received
		ROS_INFO("Address: %d, timestamp: %d, %d, X=%.3f  Y= %.3f  Z=%.3f   flags=%d", 	
				(int) hedge_pos_msg.address,
				(int) hedge_pos_msg.timestamp_ms, 
				(int) (hedge_pos_msg.timestamp_ms - hedge_timestamp_prev),
				(float) hedge_pos_msg.x_m, (float) hedge_pos_msg.y_m, (float) hedge_pos_msg.z_m,  
				(int) hedge_pos_msg.flags);
        hedge_pos_publisher.publish(hedge_pos_msg);
        hedge_pos_noaddress_publisher.publish(hedge_pos_noaddress_msg);
        
        hedge_timestamp_prev= hedge_pos_msg.timestamp_ms;
     }   
     
    beaconReadIterations= 0; 
    while(beaconReceiveCheck())
     {// stationary beacons data received
		ROS_INFO("Stationary beacon: Address: %d, X=%.3f  Y= %.3f  Z=%.3f", 	
				(int) beacon_pos_msg.address,
				(float) beacon_pos_msg.x_m, (float) beacon_pos_msg.y_m, (float) beacon_pos_msg.z_m);
        beacons_pos_publisher.publish(beacon_pos_msg);
        
        if ((beaconReadIterations++)>4)
          break;
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
