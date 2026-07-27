/**
 * @file microros_app.c
 * @brief micro-ROS stub implementation for testing without micro-ROS
 */

#include "microros_app.h"
#include <stdio.h>

bool MicroROS_Init(void)
{
    printf("[micro-ROS Stub] micro-ROS is disabled for testing.\r\n");
    return false;
}

void MicroROS_PublishJointStates(void)
{
    // Stub
}

void MicroROS_Spin(void)
{
    // Stub
}

void MicroROS_Fini(void)
{
    // Stub
}

bool MicroROS_IsInitialized(void)
{
    return false;
}

