#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rflex/rflex_commands.h>
#include <rflex/rflex_configs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

const char* SONAR_CLOUD_NAMES[] = {"sonar_cloud_body", "sonar_cloud_base"};
const char* SONAR_FRAMES[] = {"b21_body", "b21_base"};
class RFlexNode {
    private:
        RFLEX* rflex;
        ros::NodeHandle n;
        ros::Publisher pub;
        long acceleration;
        ros::Publisher sonar_pub[SONAR_RING_COUNT];
        bool isSonarOn, isBrakeOn;
        ros::Publisher voltage_pub, brake_power_pub, sonar_power_pub;
        ros::Publisher odom_pub;
        tf::TransformBroadcaster broadcaster;
        float last_distance, last_bearing;
        float x_odo, y_odo, a_odo, tvel, rvel;
        bool initialized;

        void publishOdometry();
        void publishTransforms();
        void publishSonar();

    public:
        RFlexNode();
        ~RFlexNode();
        int initialize(char* port);
        void spinOnce();

        // Message Listeners
        void NewCommand(const geometry_msgs::Pose2D::ConstPtr& msg);
        void SetAcceleration (const std_msgs::Int64::ConstPtr& msg);
        void ToggleSonarPower(const std_msgs::Bool::ConstPtr& msg);
        void ToggleBrakePower(const std_msgs::Bool::ConstPtr& msg);
};

RFlexNode::RFlexNode() {
    isSonarOn = isBrakeOn = false;
    initialized = false;
    n.subscribe<geometry_msgs::Pose2D>("cmd_vel", 1, &RFlexNode::NewCommand, this);
    n.subscribe<std_msgs::Int64>("cmd_acceleration", 1, &RFlexNode::SetAcceleration, this);
    n.subscribe<std_msgs::Bool>("rflex_sonar_power", 1, &RFlexNode::ToggleSonarPower, this);
    n.subscribe<std_msgs::Bool>("rflex_brake_power", 1, &RFlexNode::ToggleBrakePower, this);
    acceleration = DEFAULT_TRANS_ACCELERATION;

    for (int i=0;i<SONAR_RING_COUNT;i++)
        sonar_pub[i] = n.advertise<sensor_msgs::PointCloud>(SONAR_CLOUD_NAMES[i], 50);
    sonar_power_pub = n.advertise<std_msgs::Bool>("sonar_power", 1);
    brake_power_pub = n.advertise<std_msgs::Bool>("brake_power", 1);
    voltage_pub = n.advertise<std_msgs::Float32>("voltage", 1);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    //n.param("/ptu/max_pan", pmax, 90.);
}

int RFlexNode::initialize(char* port) {
    int ret;
    ret = rflex->initialize(port);
    if (ret < 0) //TODO Remove if nothing else here
        return ret;
    return 0;
}

RFlexNode::~RFlexNode() {
    rflex->close_connection();
}

void RFlexNode::NewCommand(const geometry_msgs::Pose2D::ConstPtr& msg) {
    rflex->set_velocity(msg->x, msg->theta, acceleration);
}

void RFlexNode::SetAcceleration (const std_msgs::Int64::ConstPtr& msg) {
    acceleration = msg->data;
}

void RFlexNode::ToggleSonarPower(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data)
        rflex->sonars_on();
    else
        rflex->sonars_off();
}

void RFlexNode::ToggleBrakePower(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data)
        rflex->brake_on();
    else
        rflex->brake_off();
}

void RFlexNode::spinOnce() {
    publishTransforms();
    // Publish Sonar Messages
    if (isSonarOn) {
        publishSonar();
    }

    int batt, brake;
    rflex->update_system(&batt, &brake);


    std_msgs::Bool bmsg;
    bmsg.data = isSonarOn;
    sonar_power_pub.publish(bmsg);
    bmsg.data = isBrakeOn;
    brake_power_pub.publish(bmsg);
    std_msgs::Float32 vmsg;
    vmsg.data = batt/100.0 + POWER_OFFSET;
    voltage_pub.publish(vmsg);

}

void RFlexNode::publishOdometry() {
    float distance, bearing;
    rflex->update_status(&distance, &bearing, &tvel, &rvel);

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
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(a_odo);

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
        odom.twist.twist.linear.x = tvel*cos(a_odo);
        odom.twist.twist.linear.y = tvel*sin(a_odo);
        odom.twist.twist.angular.z = rvel; //??

        //publish the message
        odom_pub.publish(odom);
    }
    last_distance = distance;
    last_bearing = bearing;
}
void RFlexNode::publishTransforms() {
    geometry_msgs::TransformStamped basebody_trans;
    basebody_trans.header.stamp = ros::Time::now();
    basebody_trans.header.frame_id = "base";
    basebody_trans.child_frame_id = "body";
    basebody_trans.transform.translation.z = BODY_OFFSET;
    basebody_trans.transform.rotation = tf::createQuaternionMsgFromYaw(a_odo);
    broadcaster.sendTransform(basebody_trans);

    geometry_msgs::TransformStamped other_trans;
    other_trans.header.frame_id = "body";
    other_trans.child_frame_id = "laser";
    other_trans.transform.translation.z = LASER_OFFSET;
    broadcaster.sendTransform(other_trans);

    other_trans.child_frame_id = "ptu_base";
    other_trans.transform.translation.x = PTU_X_OFFSET;
    other_trans.transform.translation.z = PTU_Z_OFFSET;
    broadcaster.sendTransform(other_trans);
}

void RFlexNode::publishSonar() {
    float** rings = new float*[SONAR_RING_COUNT];
    int i;
    for (i=0;i<SONAR_RING_COUNT;i++)
        rings[i] = new float[SONARS_PER_RING[i]];
    rflex->update_sonar(rings);
    for (int ringi=0;i<SONAR_RING_COUNT;i++) {
        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = SONAR_FRAMES[ringi];
        cloud.set_points_size(SONARS_PER_RING[ringi]);

        cloud.set_channels_size(0);
        //cloud.channels[0].name = "intensities";
        //cloud.channels[0].set_values_size(num_points);

        double height = SONAR_RING_HEIGHT[ringi];
        for (i = 0; i < SONARS_PER_RING[ringi]; ++i) {
            double angle = SONAR_RING_START_ANGLE[ringi] + SONAR_RING_ANGLE_INC[ringi]*i;
            angle *= M_PI / 180;
            double d = SONAR_RING_DIAMETER[ringi] + rings[ringi][i];
            cloud.points[i].x = cos(angle)*d;
            cloud.points[i].y = sin(angle)*d;
            cloud.points[i].z = height;
        }

        sonar_pub[ringi].publish(cloud);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rflex");
    RFlexNode* node = new RFlexNode();
    if (node->initialize("/dev/ttyUSB0")<0) {
        ROS_ERROR("Could not initialize driver!\n");
    }

    int hz = 30;
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
