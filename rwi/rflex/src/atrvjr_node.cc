#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rflex/atrvjr_driver.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

/**
 *  \brief ATRV-JR Node for ROS
 *  By Mikhail Medvedev 02/2012
 *  Modified from B21 node originally written by David Lu
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
class ATRVJRNode {
    private:
        ATRVJR driver;

        ros::Subscriber subs[4];			///< Subscriber handles (cmd_vel, cmd_accel, cmd_sonar_power, cmd_brake_power)
        ros::Publisher base_sonar_pub;		///< Sonar Publisher for Base Sonars (sonar_cloud_base)
        ros::Publisher body_sonar_pub;		///< Sonar Publisher for Body Sonars (sonar_cloud_body)
        ros::Publisher voltage_pub;			///< Voltage Publisher (voltage)
        ros::Publisher brake_power_pub;		///< Brake Power Publisher (brake_power)
        ros::Publisher sonar_power_pub;		///< Sonar Power Publisher (sonar_power)
        ros::Publisher odom_pub;			///< Odometry Publisher (odom)
        ros::Publisher plugged_pub;			///< Plugged In Publisher (plugged_in)
        ros::Publisher joint_pub; ///< Joint State Publisher (state)
        ros::Publisher bump_pub; ///< Bump Publisher (bumps)
        tf::TransformBroadcaster broadcaster; ///< Transform Broadcaster (for odom)

        bool isSonarOn, isBrakeOn;
        float acceleration;
        float last_distance, last_bearing;
        float x_odo, y_odo, a_odo;
        float cmdTranslation, cmdRotation;
        bool brake_dirty, sonar_dirty;
        bool initialized;
        float first_bearing;
        int updateTimer;
        int prev_bumps;
        bool sonar_just_on;

        void publishOdometry();
        void publishSonar();
        void publishBumps();

    public:
        ros::NodeHandle n;
        ATRVJRNode();
        ~ATRVJRNode();
        int initialize(const char* port);
        void spinOnce();

        // Message Listeners
        void NewCommand      (const geometry_msgs::Twist::ConstPtr& msg);
        void SetAcceleration (const std_msgs::Float32   ::ConstPtr& msg);
        void ToggleSonarPower(const std_msgs::Bool      ::ConstPtr& msg);
        void ToggleBrakePower(const std_msgs::Bool      ::ConstPtr& msg);
};

ATRVJRNode::ATRVJRNode() : n ("~") {
    isSonarOn = isBrakeOn = false;
    brake_dirty = sonar_dirty = false;
    sonar_just_on = false;
    cmdTranslation = cmdRotation = 0.0;
    updateTimer = 99;
    initialized = false;
    prev_bumps = 0;
    subs[0] = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1,   &ATRVJRNode::NewCommand, this);
    subs[1] = n.subscribe<std_msgs::Float32>("cmd_accel", 1,     &ATRVJRNode::SetAcceleration, this);
    subs[2] = n.subscribe<std_msgs::Bool>("cmd_sonar_power", 1, &ATRVJRNode::ToggleSonarPower, this);
    subs[3] = n.subscribe<std_msgs::Bool>("cmd_brake_power", 1, &ATRVJRNode::ToggleBrakePower, this);
    acceleration = 0.7;

    base_sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar_cloud_base", 50);
    body_sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar_cloud_body", 50);
    sonar_power_pub = n.advertise<std_msgs::Bool>("sonar_power", 1);
    brake_power_pub = n.advertise<std_msgs::Bool>("brake_power", 1);
    voltage_pub = n.advertise<std_msgs::Float32>("voltage", 1);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    plugged_pub = n.advertise<std_msgs::Bool>("plugged_in", 1);
    joint_pub = n.advertise<sensor_msgs::JointState>("state", 1);
    bump_pub = n.advertise<sensor_msgs::PointCloud>("bump", 5);
}

int ATRVJRNode::initialize(const char* port) {
    int ret = driver.initialize(port);
    if (ret < 0)
        return ret;

    driver.setOdometryPeriod (100000);
    driver.setDigitalIoPeriod(100000);
    driver.motionSetDefaults();
    return 0;
}

ATRVJRNode::~ATRVJRNode() {
    driver.motionSetDefaults();
    driver.setOdometryPeriod(0);
    driver.setDigitalIoPeriod(0);
    driver.setSonarPower(false);
    driver.setIrPower(false);
}

/// cmd_vel callback
void ATRVJRNode::NewCommand(const geometry_msgs::Twist::ConstPtr& msg) {
    cmdTranslation = msg->linear.x;
    cmdRotation = msg->angular.z;
}

/// cmd_acceleration callback
void ATRVJRNode::SetAcceleration (const std_msgs::Float32::ConstPtr& msg) {
    acceleration = msg->data;
}

/// cmd_sonar_power callback
void ATRVJRNode::ToggleSonarPower(const std_msgs::Bool::ConstPtr& msg) {
    isSonarOn=msg->data;
    sonar_dirty = true;
}

/// cmd_brake_power callback
void ATRVJRNode::ToggleBrakePower(const std_msgs::Bool::ConstPtr& msg) {
    isBrakeOn = msg->data;
    brake_dirty = true;
}

void ATRVJRNode::spinOnce() {
    // Sending the status command too often overwhelms the driver
    if (updateTimer>=100) {
        driver.sendSystemStatusCommand();
        updateTimer = 0;
    }
    updateTimer++;

    if (cmdTranslation != 0 || cmdRotation != 0)
        driver.setMovement(cmdTranslation, cmdRotation, acceleration);

    if (sonar_dirty) {
        driver.setSonarPower(isSonarOn);
        sonar_dirty = false;
        driver.sendSystemStatusCommand();
    }
    if (brake_dirty) {
        driver.setBrakePower(isBrakeOn);
        brake_dirty = false;
        updateTimer = 99;
    }

    std_msgs::Bool bmsg;
    bmsg.data = isSonarOn;
    sonar_power_pub.publish(bmsg);
    bmsg.data = driver.getBrakePower();
    brake_power_pub.publish(bmsg);
    bmsg.data = driver.isPluggedIn();
    plugged_pub.publish(bmsg);
    std_msgs::Float32 vmsg;
    vmsg.data = driver.getVoltage();
    voltage_pub.publish(vmsg);

    publishOdometry();
    publishSonar();
    publishBumps();
}

/** Integrates over the lastest raw odometry readings from
 * the driver to get x, y and theta */
void ATRVJRNode::publishOdometry() {
    if (!driver.isOdomReady()) {
        return;
    }

    float distance = driver.getDistance();
    float true_bearing = angles::normalize_angle(driver.getBearing());

    if (!initialized) {
        initialized = true;
        first_bearing = true_bearing;
        x_odo = 0;
        y_odo = 0;
        a_odo = 0*true_bearing;
    } else {
        float bearing = true_bearing - first_bearing;
        float d_dist = distance-last_distance;
        float d_bearing = bearing - last_bearing;

        if (d_dist > 50 || d_dist < -50)
            return;

        a_odo += d_bearing;
        a_odo = angles::normalize_angle(a_odo);

        //integrate latest motion into odometry
        x_odo += d_dist * cos(a_odo);
        y_odo += d_dist * sin(a_odo);
    }
    last_distance = distance;
    last_bearing = true_bearing - first_bearing;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(last_bearing);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_odo;
    odom_trans.transform.translation.y = y_odo;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x_odo;
    odom.pose.pose.position.y = y_odo;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    float tvel = driver.getTranslationalVelocity();
    odom.twist.twist.linear.x = tvel*cos(a_odo);
    odom.twist.twist.linear.y = tvel*sin(a_odo);
    odom.twist.twist.angular.z = driver.getRotationalVelocity();

    //publish the message
    odom_pub.publish(odom);

    // finally, publish the joint state
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(1);
    joint_state.position.resize(1);
    joint_state.name[0] = "lewis_twist";
    joint_state.position[0] = true_bearing;

    joint_pub.publish(joint_state);

}

void ATRVJRNode::publishSonar() {
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "base_link";

    if (isSonarOn) {
        driver.getBaseSonarPoints(&cloud);
        base_sonar_pub.publish(cloud);

        driver.getBodySonarPoints(&cloud);
        cloud.header.frame_id = "body";
        body_sonar_pub.publish(cloud);

    } else if (sonar_just_on) {
        base_sonar_pub.publish(cloud);
        cloud.header.frame_id = "body";
        body_sonar_pub.publish(cloud);
    }
}

void ATRVJRNode::publishBumps() {
    sensor_msgs::PointCloud cloud1, cloud2;
    cloud1.header.stamp = ros::Time::now();
    cloud2.header.stamp = ros::Time::now();
    cloud1.header.frame_id = "base_link";
    cloud2.header.frame_id = "body";
    int bumps = driver.getBaseBumps(&cloud1) +
                driver.getBodyBumps(&cloud2);

    if (bumps>0 || prev_bumps>0) {
        bump_pub.publish(cloud1);
        bump_pub.publish(cloud2);
    }
    prev_bumps = bumps;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "atrvjr");
    ATRVJRNode node;
    std::string port;
    node.n.param<std::string>("port", port, "/dev/ttyUSB0");
    ROS_INFO("Attempting to connect to %s", port.c_str());
    if (node.initialize(port.c_str())<0) {
        ROS_ERROR("Could not initialize RFLEX driver!\n");
        return 0;
    }
    ROS_INFO("Connected!");


    int hz;
    node.n.param("rate", hz, 10);
    ros::Rate loop_rate(hz);

    while (ros::ok()) {
        node.spinOnce();
        // Process a round of subscription messages
        ros::spinOnce();
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}
