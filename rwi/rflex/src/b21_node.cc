/* B21 Node for ROS
 * David Lu!! - 2/2010
 */
#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rflex/b21_driver.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

#define BASE_OFFSET 0.15
#define BODY_OFFSET 0.485
#define LASER_OFFSET -0.275
#define PTU_X_OFFSET 0.09
#define PTU_Z_OFFSET .755

class B21Node {
    private:
        B21* driver;

        ros::Subscriber subs[4];
        ros::Publisher base_sonar_pub;
        ros::Publisher body_sonar_pub;
        ros::Publisher voltage_pub, brake_power_pub, sonar_power_pub;
        ros::Publisher odom_pub;
        tf::TransformBroadcaster broadcaster;

        bool isSonarOn, isBrakeOn;
        float acceleration;
        float last_distance, last_bearing;
        float x_odo, y_odo, a_odo;
        float cmd_t, cmd_r;
        bool brake_dirty, sonar_dirty;
        bool initialized;
        int updateTimer;

        void publishOdometry();
        void publishTransforms();
        void publishSonar();

    public:
        ros::NodeHandle n;
        B21Node();
        ~B21Node();
        int initialize(const char* port);
        void spinOnce();

        // Message Listeners
        void NewCommand(const geometry_msgs::Twist::ConstPtr& msg);
        void SetAcceleration (const std_msgs::Int64::ConstPtr& msg);
        void ToggleSonarPower(const std_msgs::Bool::ConstPtr& msg);
        void ToggleBrakePower(const std_msgs::Bool::ConstPtr& msg);
};

B21Node::B21Node() : n ("~") {
    isSonarOn = isBrakeOn = false;
    brake_dirty = sonar_dirty = false;
    cmd_t = cmd_r = 0.0;
    updateTimer = 0;
    initialized = false;
    subs[0] = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &B21Node::NewCommand, this);
    subs[1] = n.subscribe<std_msgs::Int64>("cmd_acceleration", 1, &B21Node::SetAcceleration, this);
    subs[2] = n.subscribe<std_msgs::Bool>("cmd_sonar_power", 1, &B21Node::ToggleSonarPower, this);
    subs[3] = n.subscribe<std_msgs::Bool>("cmd_brake_power", 1, &B21Node::ToggleBrakePower, this);
    acceleration = 0.7;

    base_sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar_cloud_base", 50);
    body_sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar_cloud_body", 50);
    sonar_power_pub = n.advertise<std_msgs::Bool>("sonar_power", 1);
    brake_power_pub = n.advertise<std_msgs::Bool>("brake_power", 1);
    voltage_pub = n.advertise<std_msgs::Float32>("voltage", 1);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
}

int B21Node::initialize(const char* port) {
    driver = new B21();
    int ret = driver->initialize(port);
    if (ret < 0) return ret;
    driver->setOdometryPeriod (100000);
    driver->setDigitalIoPeriod(100000);
    driver->motionSetDefaults();
    return 0;
}

B21Node::~B21Node() {
    driver->motionSetDefaults();
    driver->setOdometryPeriod(0);
    driver->setDigitalIoPeriod(0);
    driver->setSonarPower(false);
    driver->setIrPower(false);

    driver->closeConnection();
}

// cmd_vel callback
void B21Node::NewCommand(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_t = msg->linear.x;
    cmd_r = msg->angular.z;
}

// cmd_acceleration callback
void B21Node::SetAcceleration (const std_msgs::Int64::ConstPtr& msg) {
    acceleration = msg->data;
}

// cmd_sonar_power callback
void B21Node::ToggleSonarPower(const std_msgs::Bool::ConstPtr& msg) {
    isSonarOn=msg->data;
    sonar_dirty = true;
}

// cmd_brake_power callback
void B21Node::ToggleBrakePower(const std_msgs::Bool::ConstPtr& msg) {
    isBrakeOn = msg->data;
    brake_dirty = true;
}

void B21Node::spinOnce() {
    driver->parsePackets();
    if (updateTimer++==100) {
        driver->sendSystemStatusCommand();
        updateTimer = 0;
    }

    driver->setMovement(cmd_t, cmd_r, acceleration);
    if (sonar_dirty) {
        driver->setSonarPower(isSonarOn);
        sonar_dirty = false;
    }
    if (brake_dirty) {
        driver->setBrakePower(isBrakeOn);
        brake_dirty = false;
    }

    std_msgs::Bool bmsg;
    bmsg.data = isSonarOn;
    sonar_power_pub.publish(bmsg);
    bmsg.data = driver->getBrakePower();
    brake_power_pub.publish(bmsg);
    std_msgs::Float32 vmsg;
    vmsg.data = driver->getVoltage();
    voltage_pub.publish(vmsg);

    publishOdometry();
    publishTransforms();
    publishSonar();
}

void B21Node::publishOdometry() {
    float distance = driver->getDistance();
    float bearing = driver->getBearing();

    if (!initialized) {
        initialized = true;
    } else {
        a_odo += bearing - last_bearing;
        a_odo = angles::normalize_angle(a_odo);
        float m_displacement = distance-last_distance;

        //integrate latest motion into odometry
        x_odo += m_displacement * cos(a_odo);
        y_odo += m_displacement * sin(a_odo);

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base";

        odom_trans.transform.translation.x = x_odo;
        odom_trans.transform.translation.y = y_odo;
        odom_trans.transform.translation.z = BASE_OFFSET;
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
        odom.pose.pose.position.z = BASE_OFFSET;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base";
        float tvel = driver->getTranslationalVelocity();
        odom.twist.twist.linear.x = tvel*cos(a_odo);
        odom.twist.twist.linear.y = tvel*sin(a_odo);
        odom.twist.twist.angular.z = driver->getRotationalVelocity();

        //publish the message
        odom_pub.publish(odom);
    }
    last_distance = distance;
    last_bearing = bearing;
}
void B21Node::publishTransforms() {
    geometry_msgs::TransformStamped basebody_trans;
    basebody_trans.header.stamp = ros::Time::now();
    basebody_trans.header.frame_id = "base";
    basebody_trans.child_frame_id = "body";
    basebody_trans.transform.translation.z = BODY_OFFSET;
    basebody_trans.transform.rotation = tf::createQuaternionMsgFromYaw(last_bearing);
    broadcaster.sendTransform(basebody_trans);

    geometry_msgs::TransformStamped other_trans;
    other_trans.header.stamp = ros::Time::now();
    other_trans.header.frame_id = "body";
    other_trans.child_frame_id = "laser";
    other_trans.transform.translation.z = LASER_OFFSET;
    other_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
    broadcaster.sendTransform(other_trans);

    other_trans.child_frame_id = "ptu_base";
    other_trans.transform.translation.y = PTU_X_OFFSET;
    other_trans.transform.translation.z = PTU_Z_OFFSET;
    broadcaster.sendTransform(other_trans);
}

void B21Node::publishSonar() {
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = "base";
    driver->getBaseSonarPoints(&cloud);
    base_sonar_pub.publish(cloud);

    driver->getBodySonarPoints(&cloud);
    cloud.header.frame_id = "body";
    body_sonar_pub.publish(cloud);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "B21");
    B21Node* node = new B21Node();
    std::string port;
    node->n.param<std::string>("port", port, "/dev/ttyUSB0");
    ROS_INFO("Attempting to connect to %s", port.c_str());
    if (node->initialize(port.c_str())<0) {
        ROS_ERROR("Could not initialize RFLEX driver!\n");
    }
    ROS_INFO("Connected!");


    int hz;
    node->n.param("rate", hz, 10);
    ros::Rate loop_rate(hz);

    while (ros::ok()) {
        node->spinOnce();
        // Process a round of subscription messages
        ros::spinOnce();
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}
