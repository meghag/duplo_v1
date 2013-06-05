#ifndef MOVE_ARM_H
#define MOVE_ARM_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

int move_arm(geometry_msgs::Point go_to);

int move_arm_spread(geometry_msgs::Point go_to, int plan_id);

int move_arm_tumble(geometry_msgs::Point go_to, int plan_id);

#endif