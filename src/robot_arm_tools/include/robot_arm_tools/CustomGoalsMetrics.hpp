/**
 * \file CustomGoalsMetrics.hpp
 * \brief Header file for custom inverse kinematics resolution goals metrics.
 *
 */

#pragma once

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>

static double visibilityMetric(const collision_detection::Contact& contact)
{
    double depth = contact.depth;
    return(1.0 - fmin(depth, 1.0));
}

static double collisionAvoidanceMetric(bool collision)
{
    return(collision ? 10.0 : 0.0);
}

static double boundsAvoidanceMetric(bool satisfiedBounds)
{
    return(satisfiedBounds ? 0.0 : 10.0);
}