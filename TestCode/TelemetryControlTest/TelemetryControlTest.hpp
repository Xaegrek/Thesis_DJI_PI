//
// Created by xaegrek on 2/6/18.
//

#ifndef THESIS_DJI_PI_TELEMETRYCONTROLTEST_HPP
#define THESIS_DJI_PI_TELEMETRYCONTROLTEST_HPP


// System Includes
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <vector>

// DJI OSDK includes
#include <dji_status.hpp>
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>
#include <memory>

#include "../../acel-base-master/src/utilities/logging/logger.hpp"

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252

//!@note: All the default timeout parameters are for acknowledgement packets
//! from the aircraft.

/*! Waypoint Controller Test
    This takes in an array of waypoints, and controlls position-based control between each waypoint.
    waypoints are in [x,y,z,yaw] format.
!*/
bool
trajectoryWaypointControllerTest(DJI::OSDK::Vehicle *vehicle, std::vector<std::vector<float>> way, int timeout = 1);

bool trajectoryWaypointControllerTestTimer(DJI::OSDK::Vehicle *vehicle, std::vector<std::vector<float>> way,
                                           int timeout = 1);

/*! Waypoint Controller Test
    This takes in an array of waypoints, and controlls position-based on offset from current body position.
    waypoints are in [x,y,z,yaw] format.
!*/
bool trajectoryWaypointOffsetControllerTest(DJI::OSDK::Vehicle *vehicle, std::vector<std::vector<float>> way,
                                            int timeout = 1);

/*! Trajectory Controller Test Crude
    This forwards function to another file, cause "undefined refererce to fcn(DJI::OSDK::Vehicle*, int)" issues
    This implementation of a trajectory controller takes in polynomial values from a
    trajectory planner, and implements them into a system to run using the telemetry test thrust controller.
    May want to migrate this control to use the api thrust, and do checking internally / with
    another function.
    This can be used for 1 part of a piecewise trajectory, and can be readily scaled, (and probably is/will be
    in this code.  fStyle true runs for thrust and attitude, while false uses position control.
!*/
bool trajectoryControllerTestCrude(DJI::OSDK::Vehicle *vehicle, double aMan[], double bMan[], double cMan[],
                                   double tTrajEnd, int nDim, bool fStyle, int timeout = 1);


/*! Monitored Takeoff
    This implementation of takeoff  with monitoring makes sure your aircraft
    actually took off and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool monitoredTakeoff(DJI::OSDK::Vehicle *vehiclePtr, int timeout = 1);

// Examples of commonly used Flight Mode APIs

/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there.
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control.
!*/
bool moveByPositionOffset(DJI::OSDK::Vehicle *vehicle, float xOffsetDesired,
                          float yOffsetDesired, float zOffsetDesired,
                          float yawDesired, float posThresholdInM = 0.2,
                          float yawThresholdInDeg = 1.0);

/*! Velocity Control. Allows you to set your current velocity.
    The aircraft will adjust to that velocity and maintain it.
    Typical use would be as a building block in an outer loop that does not
    require many incredibly fast changes, such as a trajectory follower.
!*/
bool moveByVelocityRequest(DJI::OSDK::Vehicle *vehicle, float xVelocityDesired,
                          float yVelocityDesired, float zVelocityDesired,
                          float yawRateDesired, float posThresholdInM = 0.2,
                          float yawThresholdInDeg = 1.0);
/*! Altitude-Thrust Control. Allows you to set an attitude and thrust
    values. The aircraft will change to that state and maintain it.
    Typical use would be as a building block in an outer loop that does
    require semi-fast change.
!*/
bool moveByAttitudeThrust(DJI::OSDK::Vehicle *vehicle, float xRoll,
                          float yPitch, float zThrust,
                          float yawDesired, float attThresholdInDeg = 1.0,
                          float thrustThreshold = 1.0);

/*! Monitored Landing (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.

!*/
bool monitoredLanding(DJI::OSDK::Vehicle *vehiclePtr, int timeout = 1);

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
 * coordinates.
 *
 * Accurate when distances are small.
!*/
void localOffsetFromGpsOffset(DJI::OSDK::Vehicle *vehicle,
                              DJI::OSDK::Telemetry::Vector3f &deltaNed,
                              void *target, void *origin);

DJI::OSDK::Telemetry::Vector3f toEulerAngle(void *quaternionData);

#endif //THESIS_DJI_PI_TELEMETRYCONTROLTEST_HPP
