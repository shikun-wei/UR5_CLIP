/**
 * \file RobotTrajectories.h
 * \brief Header file
 *
 * Defines the functions used to generate several types of robot arm end effector trajectories
 *
 */

#pragma once

#include <geometry_msgs/Pose.h>

/*!
 * \brief Generates a normal oriented pose on a sphere.
 * \param spherePose The position of the center of the sphere.
 * \param sphereRadius The radius of the sphere.
 * \param poseInclination The inclination of the pose (i.e. the polar angle theta in ISO80000).
 * \param poseAzimuth The azimuth of the pose (i.e. the azimuthal angle phi in ISO80000).
 * \param inward Wether to consider the inward normal to the sphere (true), or the outward one (false).
 * \param yawOffset The eventual yaw offset around the pose normal (by default, the x axis is directed upwards).
*/
geometry_msgs::Pose sphericPose(geometry_msgs::Pose spherePose, double sphereRadius, double poseInclination, double poseAzimuth, bool inward = true, double yawOffset = 0);

//TODO Re-order arguments to have waypoints first !
/*!
 * \brief Generates a normal oriented trajectory on a sphere at a constant inclination angle.
 * \param spherePose The position of the center of the sphere.
 * \param sphereRadius The radius of the sphere.
 * \param inclination The inclination of the trajectory (i.e. the polar angle theta in ISO80000).
 * \param azimuthMin The minimal azimuth of the trajectory (i.e. the azimuthal angle phi in ISO80000).
 * \param azimuthMax The maximal azimuth of the trajectory (i.e. the azimuthal angle phi in ISO80000).
 * \param waypointsNumber The number of waypoints in the trajectory.
 * \param waypoints The vector containing the waypoints.
 * \param inward Wether to consider the inward normal to the sphere (true), or the outward one (false).
 * \param yawOffset The eventual yaw offset around the pose normal (by default, the x axis is directed upwards).
*/
void sphericInclinationTrajectory(geometry_msgs::Pose spherePose, double sphereRadius, double inclination, double azimuthMin, double azimuthMax, 
                                    int waypointsNumber, std::vector<geometry_msgs::Pose>& waypoints, bool inward = true, double yawOffset = 0);

/*!
 * \brief Generates a normal oriented trajectory on a sphere at a constant azimuthal angle.
 * \param spherePose The position of the center of the sphere.
 * \param sphereRadius The radius of the sphere.
 * \param inclination The inclination of the trajectory (i.e. the polar angle theta in ISO80000).
 * \param azimuthMin The minimal azimuth of the trajectory (i.e. the azimuthal angle phi in ISO80000).
 * \param azimuthMax The maximal azimuth of the trajectory (i.e. the azimuthal angle phi in ISO80000).
 * \param waypointsNumber The number of waypoints in the trajectory.
 * \param waypoints The vector containing the waypoints.
 * \param inward Wether to consider the inward normal to the sphere (true), or the outward one (false).
 * \param yawOffset The eventual yaw offset around the pose normal (by default, the x axis is directed upwards).
*/
void sphericAzimuthTrajectory(geometry_msgs::Pose spherePose, double sphereRadius, double azimuth, double inclinationMin, double inclinationMax, 
                             int waypointsNumber, std::vector<geometry_msgs::Pose>& waypoints, bool inward = true, double yawOffset = 0);

/*!
 * \brief Generates a straight trajectory between two poses.
 * \param startingPose The starting pose.
 * \param endingPose The ending pose.
 * \param waypointsNumber The number of waypoints in the trajectory.
 * \param waypoints The vector containing the waypoints.
*/
void straightTrajectory(geometry_msgs::Pose startingPose, geometry_msgs::Pose endingPose, int waypointsNumber, std::vector<geometry_msgs::Pose>& waypoints);

/*!
 * \brief Rotates a trajectory.
 * \param waypoints The waypoints of the trajectory.
 * \param rotationCenter The rotation center point
 * \param rotationRoll The rotation roll angle.
 * \param rotationPitch The rotation pitch angle.
 * \param rotationYaw The rotation yaw angle.
*/
void rotateTrajectory(std::vector<geometry_msgs::Pose>& waypoints, geometry_msgs::Point rotationCenter, double rotationRoll, double rotationPitch, double rotationYaw);

/*!
 * \brief Translates a trajectory.
 * \param waypoints The waypoints of the trajectory.
 * \param translationX The translation along the x axis;
 * \param translationY The translation along the y axis;
 * \param translationZ The translation along the z axis;
*/
void translateTrajectory(std::vector<geometry_msgs::Pose>& waypoints, double translationX, double translationY, double translationZ);

/*!
 * \brief Creates a trajectory from a .yaml file containing waypoints poses.
 * \param waypoints The waypoints of the trajectory.
 * \param filePath The path to the .yaml file containing waypoints poses.
 * \param waypointsKey The key of the poses waypoints in the .yaml file.
 * \param startIndex The index at which to start the waypoints.
*/
void trajectoryFromFile(std::vector<geometry_msgs::Pose>& waypoints, std::string filePath, int startIndex = 0, std::string waypointsKey = "poses");

/*!
 * \brief Creates a trajectory from a ROS parameter containing waypoints poses.
 * \param waypoints The waypoints of the trajectory.
 * \param parameterName The name of the ROS parameter containing waypoints poses.
 * \param startIndex The index at which to start the waypoints.
*/
void trajectoryFromParameter(std::vector<geometry_msgs::Pose>& waypoints, std::string parameterName = "poses", int startIndex = 0);

/*!
 * \brief Creates a trajectory from a .yaml file containing waypoints joints values.
 * \param waypoints The waypoints of the trajectory.
 * \param filePath The path to the .yaml file containing waypoints joints values.
 * \param waypointsKey The key of the joints values waypoints in the .yaml file.
 * \param startIndex The index at which to start the waypoints.
*/
void trajectoryFromFile(std::vector<std::vector<double>>& waypoints, std::string filePath, int startIndex = 0, std::string waypointsKey = "jointsValues");

/*!
 * \brief Creates a trajectory from a ROS parameter containing waypoints joints values.
 * \param waypoints The waypoints of the trajectory.
 * \param parameterName The name of the ROS parameter containing waypoints joints values.
 * \param startIndex The index at which to start the waypoints.
*/
void trajectoryFromParameter(std::vector<std::vector<double>>& waypoints, std::string parameterName = "jointsValues", int startIndex = 0);