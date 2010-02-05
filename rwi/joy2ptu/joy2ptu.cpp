#include <string>
#include <ros/ros.h>
#include <joy/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub, pub2;

double tmin, tmax, pmin, pmax;
double tsmin, tsmax, psmin, psmax;
float pcur, tcur;
bool mode = true;

double max_speed = 0.500; // m/second
double max_turn = 60.0*M_PI/180.0; // rad/second

void posRcvd(const sensor_msgs::JointState::ConstPtr& msg) {
    pcur = msg->position[0];
    tcur = msg->position[1];
}

void joyRcvd(const joy::Joy::ConstPtr& msg) {
    float x = msg->axes[0], y = msg->axes[1];
    float pan, pvel, tilt, tvel;
    float dead = 0.25;
    if (msg->buttons[1]) {
        mode = !mode;
    }
    if (mode) {
        if (msg->buttons[0]) {
            pan = 0;
            tilt = 0;
            pvel = psmax;
            tvel = tsmax;
        } else {
            if (x<-dead)
                pan = pmin;
            else if (x>dead)
                pan = pmax;
            else
                pan = pcur;

            if (y<-dead)
                tilt = tmin;
            else if (y>dead)
                tilt = tmax;
            else
                tilt = tcur;

            float xx = x*x, yy = y*y;
            pvel = xx*(psmax-psmin)+psmin;
            tvel = yy*(tsmax-tsmin)+tsmin;
        }
        sensor_msgs::JointState joint_cmd;
        joint_cmd.header.stamp = ros::Time::now();
        joint_cmd.set_name_size(2);
        joint_cmd.set_position_size(2);
        joint_cmd.set_velocity_size(2);
        joint_cmd.name[0] ="head_pan_joint";
        joint_cmd.position[0] = pan;
        joint_cmd.velocity[0] = pvel;
        joint_cmd.name[1] ="head_tilt_joint";
        joint_cmd.position[1] = tilt;
        joint_cmd.velocity[1] = tvel;
        pub.publish(joint_cmd) ;
    } else {
		geometry_msgs::Twist cmdvel;
		float speed, turn;
		if(x<dead && x>-dead)
			turn = 0;
		else
			turn = max_turn * x;
		if(y<dead && y>-dead)
			speed = 0;
		else
			speed = max_speed * y;
		cmdvel.linear.x = speed;
        cmdvel.angular.z = turn;
		pub2.publish(cmdvel);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy2ptu");
    ros::NodeHandle n;

    int hz = 30;
    ros::Rate loop_rate(hz);
    pub = n.advertise<sensor_msgs::JointState>("ptu_cmd", 1);
    pub2 = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    n.param("/ptu/max_pan", pmax, 90.);
    n.param("/ptu/min_pan", pmin, -90.);
    n.param("/ptu/max_tilt", tmax, 30.);
    n.param("/ptu/min_tilt", tmin, -30.);
    n.param("/ptu/min_tilt_speed", tsmin, 4.);
    n.param("/ptu/max_tilt_speed", tsmax, 140.);
    n.param("/ptu/min_pan_speed", psmin, 4.);
    n.param("/ptu/max_pan_speed", psmax, 140.);
    pcur = tcur = 0;

    ros::Subscriber sub =
        n.subscribe<joy::Joy>("joy", 1, joyRcvd);
    ros::Subscriber sub2=
        n.subscribe<sensor_msgs::JointState>("ptu_state", 1, posRcvd);

    while (ros::ok()) {

        // Process a round of subscription messages
        ros::spinOnce();

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}
