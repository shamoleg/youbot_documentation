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

    //Sets up the random number generator
    srand(time(0));

    double vx = 0.3
    double vy = 0.3
    double th = 0.1

    //Sets the loop to publish at a rate of 10Hz
    ros::Rate rate(10);


    while(ros::ok()) {
        //Declares the message to be sent
        geometry_msgs::Twist msg;
        //Random x value between -2 and 2
        msg.linear.x = vx;
        //Random y value between -3 and 3
        msg.linear.y = vy;
        //Random y value between -3 and 3
        msg.angular.z = 0.1;
        //Publish the message
        pub.publish(msg);

        //Delays untill it is time to send another message
        rate.sleep();
        }
}
```

