# Пример написания узлов для управления мобильной платформой и манипулятором

В данном документе представлена инструкция по написанию программного обеспечения для платформы KUKA youBot.

Для работы потребуется создать пакет где будят хранится исходники файлов.

Откройте рабочую область и создайте пакет с необходимыми зависимостями:

```console
cd ~/catkin_ws/src/
catkin_create_pkg youbot_control geometry_msgs roscpp brics_actuator pr2_msgs std_msgs youbot_driver
```
 
Пакет готов для написания кода.

##  Пример написания управления скоростью

Создайте c++ файл:

```console
cd ~/catkin_ws/src/youbot_control/src
touch teleop_twist_keyboard.cpp
```

Откройте созданный файл в любом редакторе кода. Пример кода представлен ниже.

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 

#include <stdlib.h> 

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "youbot_control");
    ros::NodeHandle nh;

    //Ceates the publisher, and tells it to publish
    //to the aaddscmd_vel topic, with a queue size of 1
    ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    double vx = 0.0
    double vy = 0.0
    double th = 0.0

    //Sets the loop to publish at a rate of 10Hz
    ros::Rate rate(10);


    while(ros::ok()) {

        cout << "Please type value of  vx" << endl;
        cin >> readValue;
        cout << "Please type value of  vy" << endl;
        cin >> readValue;
        cout << "Please type value of  th" << endl;
        cin >> readValue;

        //Declares the message to be sent
        geometry_msgs::Twist msg;
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.angular.z = th;

        //Publish the message
        pub.publish(msg);

        //Delays untill it is time to send another message
        rate.sleep();
        }
}
```

##  Пример написания управления манипулятором

```cpp
#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"

#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_arm_test");
	ros::NodeHandle n;
	ros::Publisher armPositionsPublisher;
	ros::Publisher gripperPositionPublisher;

	armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
	gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

	ros::Rate rate(20); //Hz
	double readValue;
	static const int numberOfArmJoints = 5;
	static const int numberOfGripperJoints = 2;
	while (n.ok()) {
		brics_actuator::JointPositions command;
		vector <brics_actuator::JointValue> armJointPositions;
		vector <brics_actuator::JointValue> gripperJointPositions;

		armJointPositions.resize(numberOfArmJoints); 
		gripperJointPositions.resize(numberOfGripperJoints);

		std::stringstream jointName;


		// ::io::base_unit_info <boost::units::si::angular_velocity>).name();
		for (int i = 0; i < numberOfArmJoints; ++i) {
			cout << "Please type in value for joint " << i + 1 << endl;
			cin >> readValue;

			jointName.str("");
			jointName << "arm_joint_" << (i + 1);

			armJointPositions[i].joint_uri = jointName.str();
			armJointPositions[i].value = readValue;

			armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;

		};

		cout << "Please type in value for a left jaw of the gripper " << endl;
		cin >> readValue;
		gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
		gripperJointPositions[0].value = readValue;
		gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

		cout << "Please type in value for a right jaw of the gripper " << endl;
		cin >> readValue;
		gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
		gripperJointPositions[1].value = readValue;
		gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

		cout << "sending command ..." << endl;

		command.positions = armJointPositions;
		armPositionsPublisher.publish(command);

		command.positions = gripperJointPositions;
		gripperPositionPublisher.publish(command);

		cout << "--------------------" << endl;
		ros::spinOnce();
		rate.sleep();

	}

	return 0;
}
```