/**
 * \file OKMeasurementServer.h
 * \brief Header file of the OKMeasurementServer class
 *
 * Header file of the OKMeasurementServer class - Defines a dummy MeasurementServer, just waiting for an user input
 *
 */

#pragma once

#include "robot_arm_tools/MeasurementServer.h"

 /*! \class OKMeasurementServer
  * \brief Defines a dummy MeasurementServer, just waiting for an user input
  */
class OKMeasurementServer : public MeasurementServer
{
    public:
        /*!
         *  \brief Constructor
         */
        OKMeasurementServer() : MeasurementServer() {};

        /*!
         *  \brief Destructor
         */
        ~OKMeasurementServer(){};

        /*!
         *  \brief Measure - Simply waits for user input
         */
        bool measure();
};
