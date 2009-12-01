/*
 * rflex ROS Package
 * Copyright (C) 2009 David Lu!! (davidlu@wustl.edu)
 * Modeled after erratic_player
 */

#include <assert.h>

// For core Player stuff (message queues, config file objects, etc.)
#include <libplayercore/playercore.h>
// TODO: remove XDR dependency
#include <libplayerxdr/playerxdr.h>

// roscpp
#include <ros/ros.h>
//rosTF
#include "tf/transform_broadcaster.h"

// Messages that I need
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <pr2_msgs/PowerState.h>
#include <rflex/rflex.h>
#include <boost/thread.hpp>

#define PLAYER_QUEUE_LEN 32

class RFLEXNode
{
  public:
    QueuePointer q;
    ros::NodeHandle node_;

    tf::TransformBroadcaster tf_;

    ros::Publisher odom_pub_, battery_pub_;
    ros::Subscriber cmd_vel_sub_;
  
    RFLEXNode(char* configfile) : watts_charging_(10), watts_unplugged_(-10), charging_threshold_(12.98)
    {
      // libplayercore boiler plate
      player_globals_init();
      puts("init");
      itable_init();
      
      // TODO: remove XDR dependency
      playerxdr_ftable_init();

      //create publishers for odometry and battery information
      odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom", 1);
      battery_pub_ = node_.advertise<pr2_msgs::PowerState>("battery_state", 1);

      // The Player address that will be assigned to this device.  The format
      // is interface:index.  The interface must match what the driver is
      // expecting to provide.  The value of the index doesn't really matter, 
      // but 0 is most common.
      const char* player_addr_pos = "position2d:0";
      const char* player_addr_power = "power:0";

      // Create a ConfigFile object, into which we'll stuff parameters.
      // Drivers assume that this object will persist throughout execution
      // (e.g., they store pointers to data inside it).  So it must NOT be
      // deleted until after the driver is shut down.
      this->cf = new ConfigFile();
      this->cf->Load(configfile);

      // Create an instance of the driver, passing it the ConfigFile object.
      // The -1 tells it to look into the "global" section of the ConfigFile,
      // which is where ConfigFile::InsertFieldValue() put the parameters.
      this->driver = new RFLEX(cf, -1);
      assert(this->driver);

      // Print out warnings about parameters that were set, but which the
      // driver never looked at.
      cf->WarnUnused();

      // Grab from the global deviceTable a pointer to the Device that was 
      // created as part of the driver's initialization.
      assert((this->pos_device = deviceTable->GetDevice(player_addr_pos,false)));
      assert((this->power_device = deviceTable->GetDevice(player_addr_power,false)));

      // Create a message queue
      this->q = QueuePointer(false,PLAYER_QUEUE_LEN);
    }

    ~RFLEXNode()
    {
      delete cf;
      player_globals_fini();
    }

    int start()
    {
      // Subscribe to device, which causes it to startup
      if((this->pos_device->Subscribe(this->q) != 0) || 
         (this->power_device->Subscribe(this->q) != 0))
      {
        puts("Failed to subscribe the driver");
        return(-1);
      }
      else
      {
        cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&RFLEXNode::cmdvelReceived, this, _1));
        return(0);
      }
    }

    int stop()
    {
      int ret=0;
      // Unsubscribe from the device, which causes it to shutdown
      if(this->pos_device->Unsubscribe(this->q) != 0)
      {
        puts("Failed to stop the driver");
        ret=-1;
      }
      if(this->power_device->Unsubscribe(this->q) != 0)
      {
        puts("Failed to stop the driver");
        ret=-1;
      }

      // Give the driver a chance to shutdown.  Wish there were a way to
      // detect when that happened.
      usleep(1000000);
      return(ret);
    }

    int setMotorState(uint8_t state)
    {
      Message* msg;
      // Enable the motors
      player_position2d_power_config_t motorconfig;
      motorconfig.state = state;
      if(!(msg = this->pos_device->Request(this->q,
                                           PLAYER_MSGTYPE_REQ,
                                           PLAYER_POSITION2D_REQ_MOTOR_POWER,
                                           (void*)&motorconfig,
                                           sizeof(motorconfig), NULL, false)))
      {
        return(-1);
      }
      else
      {
        delete msg;
        return(0);
      }
    }

    void getCenter()
    {
      Message* msg = NULL;
      //Wait until there is a message for geometry
      while (!(msg = this->pos_device->Request(this->q, PLAYER_MSGTYPE_REQ,
					       PLAYER_POSITION2D_REQ_GET_GEOM,
					       NULL, 0, NULL, false))) 
      {
	ROS_ERROR("No geom for the robot from player (yet). Waiting one second and trying again.");
	ros::Duration(1, 0).sleep();
      }
      player_position2d_geom_t* geomconfig = (player_position2d_geom_t*)msg->GetPayload(); 
      center_x_ = geomconfig->pose.px;
      center_y_ = geomconfig->pose.py;
      center_yaw_ = geomconfig->pose.pyaw;
      ROS_INFO("Robot center at (%fm %fm), yaw %f radians. Width %fm, length %fm", center_x_, center_y_, center_yaw_, geomconfig->size.sw, geomconfig->size.sl);
      delete msg;
    }

    void cmdvelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
      /*
      printf("received cmd: (%.3f,%.3f,%.3f)\n",
             cmd_vel->linear.x,
             cmd_vel->linear.y,
             cmd_vel->angular.z);
             */

      player_position2d_cmd_vel_t cmd;
      memset(&cmd, 0, sizeof(cmd));

      cmd.vel.px = cmd_vel->linear.x;
      cmd.vel.py = 0.0;
      cmd.vel.pa = cmd_vel->angular.z;
      cmd.state = 1;


      this->pos_device->PutMsg(this->q,
                               PLAYER_MSGTYPE_CMD,
                               PLAYER_POSITION2D_CMD_VEL,
                               (void*)&cmd,sizeof(cmd),NULL);
    }


    void doUpdate() 
    {
      Message* msg = NULL;
      // Block until there's a message on the queue 
      q->Wait(); 
      
      // Pop off one message (we own the resulting memory) 
      assert((msg = q->Pop())); 
      
      // Is the message one we care about? 
      player_msghdr_t* hdr = msg->GetHeader(); 
      if((hdr->type == PLAYER_MSGTYPE_DATA) &&  
	 (hdr->subtype == PLAYER_POSITION2D_DATA_STATE) && 
	 (hdr->addr.interf == PLAYER_POSITION2D_CODE)) 
      { 
	// Cast the message payload appropriately  
	player_position2d_data_t* pdata = (player_position2d_data_t*)msg->GetPayload(); 
	
  nav_msgs::Odometry odom;

	// Translate from Player data to ROS data 
	odom.pose.pose.position.x = pdata->pos.px; 
	odom.pose.pose.position.y = pdata->pos.py; 
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pdata->pos.pa); 
	odom.twist.twist.linear.x = pdata->vel.px; 
	odom.twist.twist.linear.y = pdata->vel.py; 
	odom.twist.twist.angular.z = pdata->vel.pa; 

  //@todo TODO: Think about publishing stall information with odometry or on a separate topic
	//odom.stall = pdata->stall; 
	
	odom.header.frame_id = "odom"; 
	
	odom.header.stamp.sec = (long long unsigned int)floor(hdr->timestamp); 
	odom.header.stamp.nsec = (long long unsigned int)((hdr->timestamp - floor(hdr->timestamp)) * 1000000000ULL); 
	
	
	// Publish the new data 
	odom_pub_.publish(odom); 
	
	tf_.sendTransform(tf::Transform(tf::createQuaternionFromYaw(pdata->pos.pa), 
				       tf::Point(pdata->pos.px, 
						 pdata->pos.py, 
						 0.0) 
				       ),
			 ros::Time((long long unsigned int)floor(hdr->timestamp), 
				   (long long unsigned int)((hdr->timestamp - floor(hdr->timestamp)) * 1000000000ULL)), 
			 "base_link_offset",
			 "odom"); 
	tf_.sendTransform(tf::Transform(tf::createQuaternionFromYaw(center_yaw_),  
				       tf::Point(center_x_,  
						 center_y_,  
						 0.0)  
				       ).inverse(),  
			 ros::Time((long long unsigned int)floor(hdr->timestamp),  
				   (long long unsigned int)((hdr->timestamp - floor(hdr->timestamp)) * 1000000000ULL)),  
			 "base_link",
			 "base_link_offset");  
	tf_.sendTransform(tf::Transform(tf::createQuaternionFromYaw(0.0),  
					tf::Point(0.0, 0.0, 0.0)).inverse(),  
			 ros::Time((long long unsigned int)floor(hdr->timestamp),  
				   (long long unsigned int)((hdr->timestamp - floor(hdr->timestamp)) * 1000000000ULL)),  
			 "base_laser_link",
			 "base_link_offset");  
	
	
	//printf("Published new odom: (%.3f,%.3f,%.3f)\n",  
	//odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.th); 
      } 
      else if((hdr->type == PLAYER_MSGTYPE_DATA) &&   
	      (hdr->subtype == PLAYER_POWER_DATA_STATE) &&  
	      (hdr->addr.interf == PLAYER_POWER_CODE))  
      {
	player_power_data_t* pdata = (player_power_data_t*)msg->GetPayload();  
	pr2_msgs::PowerState state;
	state.header.stamp = ros::Time::now();
	state.time_remaining = (pdata->volts < 11.5) ? 0:60; //need to calculate the remaing runtime based on the batteries discharge curve, for now stop when voltage is below 11.5. -Curt
	state.power_consumption = (pdata->volts > charging_threshold_) ? watts_charging_ : watts_unplugged_; //Does not work, as they don't publish this.
  state.AC_present = (pdata->volts > charging_threshold_) ? 1 :0; // are we charging?
	battery_pub_.publish(state);
      }
      else
      { 
	ROS_WARN("Unhandled Player message %d:%d:%d:%d", 
		 hdr->type, 
		 hdr->subtype, 
		 hdr->addr.interf, 
		 hdr->addr.index); 
	
      } 
      
      // We're done with the message now 
      delete msg; 
    }

  private:
    double center_x_, center_y_, center_yaw_;
    double watts_charging_, watts_unplugged_, charging_threshold_;
    Driver* driver;
    Device* pos_device;
    Device* power_device;
    ConfigFile* cf;
};

void spinThread(){
  ros::spin();
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "RFLEX_player");

  RFLEXNode en("configfile");

  ros::NodeHandle n;
  boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

  // Start up the robot
  if(en.start() != 0)
    exit(-1);

  // Enable the motors
  if(en.setMotorState(1) < 0)
    puts("failed to enable motors");

  en.getCenter();

  /////////////////////////////////////////////////////////////////
  // Main loop; grab messages off our queue and republish them via ROS
  while(n.ok())
  {
    en.doUpdate();
  }
  /////////////////////////////////////////////////////////////////

  // Stop the robot
  en.stop();

  spin_thread.join();

  // To quote Morgan, Hooray!
  return(0);
}
