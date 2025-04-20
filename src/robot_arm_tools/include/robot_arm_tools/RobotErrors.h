#include <exception>
#include <string.h>

#pragma once 

//TODO Integration !!

class InvalidParameter : public std::exception {
    public:
        char * what () 
        {
            return strdup("Invalid ROS parameter");
        }
};

class UndefinedParameter : public std::exception {
    public:
        char * what () 
        {
            return strdup("Undefined ROS parameter");
        }
};

class ExecutionError : public std::exception {
    public:
        char * what () 
        {
            return strdup("Error during motion execution");
        }
};

class PlanningError : public std::exception {
    public:
        char * what () 
        {
            return strdup("Error during motion planning");
        }
};

class KinematicsError : public std::exception {
    public:
        char * what () 
        {
            return strdup("Error while solving kinematics");
        }
};