 #include <mimic.hpp>

Mimic::Mimic(ros::NodeHandle& nh) : mimic_(nh){
    leg11 = 0;
    leg12 = 0;
    leg21 = 0;
    leg22 = 0;
    leg31 = 0;
    leg32 = 0;
    leg41 = 0;
    leg42 = 0;
    link1_ = 0.09;
    link2_ = 0.165;
    initialAngle_.data = 0;
    initialAngle_ = calculateBeta(0, 0, 1);

    ROS_WARN_STREAM("Initialized Mimic Joints");

    leg13Pub = mimic_.advertise<std_msgs::Float64>("/doggo_description/leg_controller1_mimic/command", 100);
    leg14Pub = mimic_.advertise<std_msgs::Float64>("/doggo_description/leg_controller2_mimic/command", 100);
    leg23Pub = mimic_.advertise<std_msgs::Float64>("/doggo_description/leg_controller3_mimic/command", 100);
    leg24Pub = mimic_.advertise<std_msgs::Float64>("/doggo_description/leg_controller4_mimic/command", 100);
    leg33Pub = mimic_.advertise<std_msgs::Float64>("/doggo_description/leg_controller5_mimic/command", 100);
    leg34Pub = mimic_.advertise<std_msgs::Float64>("/doggo_description/leg_controller6_mimic/command", 100);
    leg43Pub = mimic_.advertise<std_msgs::Float64>("/doggo_description/leg_controller7_mimic/command", 100);
    leg44Pub = mimic_.advertise<std_msgs::Float64>("/doggo_description/leg_controller8_mimic/command", 100);;

    leg11Sub = mimic_.subscribe("/doggo_description/leg_controller1/command", 100, &Mimic::leg11Callback, this);
    leg12Sub = mimic_.subscribe("/doggo_description/leg_controller2/command", 100, &Mimic::leg12Callback, this);
    leg21Sub = mimic_.subscribe("/doggo_description/leg_controller3/command", 100, &Mimic::leg21Callback, this);
    leg22Sub = mimic_.subscribe("/doggo_description/leg_controller4/command", 100, &Mimic::leg22Callback, this);
    leg31Sub = mimic_.subscribe("/doggo_description/leg_controller5/command", 100, &Mimic::leg31Callback, this);
    leg32Sub = mimic_.subscribe("/doggo_description/leg_controller6/command", 100, &Mimic::leg32Callback, this);
    leg41Sub = mimic_.subscribe("/doggo_description/leg_controller7/command", 100, &Mimic::leg41Callback, this);
    leg42Sub = mimic_.subscribe("/doggo_description/leg_controller8/command", 100, &Mimic::leg42Callback, this);
}

Mimic::~Mimic() {
}

void Mimic::leg11Callback(std_msgs::Float64 msg) {
	leg11 = msg.data;
    ROS_INFO_STREAM("Callback");
    ROS_INFO_STREAM(leg11);
}

void Mimic::leg12Callback(std_msgs::Float64 msg) {
    leg12 = msg.data;
}

void Mimic::leg21Callback(std_msgs::Float64 msg) {
    leg21 = msg.data;
}

void Mimic::leg22Callback(std_msgs::Float64 msg) {
    leg22 = msg.data;
}

void Mimic::leg31Callback(std_msgs::Float64 msg) {
    leg31 = msg.data;
}

void Mimic::leg32Callback(std_msgs::Float64 msg) {
    leg32 = msg.data;
}

void Mimic::leg41Callback(std_msgs::Float64 msg) {
    leg41 = msg.data;
}

void Mimic::leg42Callback(std_msgs::Float64 msg) {
    leg42 = msg.data;
}

std_msgs::Float64 Mimic::calculateBeta(double frontJoint, double rearJoint, int leg){
    std_msgs::Float64 beta;
    double a = (PI_ - frontJoint + rearJoint)/2;
    double c = asin((sin(a) * 0.09)/0.165);
    beta.data = leg * ((PI_ - a - c) - initialAngle_.data);
    return beta;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mimic");
    ros::NodeHandle nh;
    Mimic joints(nh);
    ros::Rate loop(2000);
    while (ros::ok()) {

        joints.leg13Pub.publish(joints.calculateBeta(joints.leg11, joints.leg12, -1));
        joints.leg14Pub.publish(joints.calculateBeta(joints.leg11, joints.leg12, 1));
        joints.leg23Pub.publish(joints.calculateBeta(joints.leg21, joints.leg22, -1));
        joints.leg24Pub.publish(joints.calculateBeta(joints.leg21, joints.leg22, 1));

        // Mirrored Configuration
        joints.leg33Pub.publish(joints.calculateBeta(joints.leg32, joints.leg31, 1));
        joints.leg34Pub.publish(joints.calculateBeta(joints.leg32, joints.leg31, -1));
        joints.leg43Pub.publish(joints.calculateBeta(joints.leg42, joints.leg41, 1));
        joints.leg44Pub.publish(joints.calculateBeta(joints.leg42, joints.leg41, -1));
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}