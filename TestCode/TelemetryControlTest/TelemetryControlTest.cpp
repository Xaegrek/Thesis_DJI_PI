/*! xaegrek
 */

#include <cstdio>
#include "TelemetryControlTest.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
FILE * outputfile;


/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool
monitoredTakeoff(Vehicle* vehicle, int timeout)
{
    //! Setup logging file for quaternions, etc



    //@todo: remove this once the getErrorCode function signature changes
    char func[50];
    int  pkgIndex;

    /*! Verify Telemetry Subscription */

    // Start takeoff
    ACK::ErrorCode takeoffStatus = vehicle->control->takeoff(timeout);
    if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(takeoffStatus, func);
        return false;
    }

    // First check: Motors started
    int motorsNotStarted = 0;
    int timeoutCycles    = 20;


    while ((vehicle->broadcast->getStatus().flight <
            DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF) &&
           motorsNotStarted < timeoutCycles)
    {
        motorsNotStarted++;
        usleep(100000);
    }

    if (motorsNotStarted < timeoutCycles)
    {
        std::cout << "Successful TakeOff!" << std::endl;
    }


    // Second check: In air
    int stillOnGround = 0;
    timeoutCycles     = 110;


    while ((vehicle->broadcast->getStatus().flight !=
            DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY) &&
           stillOnGround < timeoutCycles)
    {
        stillOnGround++;
        usleep(100000);
    }

    if (stillOnGround < timeoutCycles)
    {
        std::cout << "Aircraft in air!" << std::endl;
    }


    // Final check: Finished takeoff

    float32_t                 delta;
    Telemetry::GlobalPosition currentHeight;
    Telemetry::GlobalPosition deltaHeight =
            vehicle->broadcast->getGlobalPosition();

    do
    {
        sleep(3);
        currentHeight = vehicle->broadcast->getGlobalPosition();
        delta         = fabs(currentHeight.altitude - deltaHeight.altitude);
        deltaHeight.altitude = currentHeight.altitude;
    } while (delta >= 0.009);

    std::cout << "Aircraft hovering at " << currentHeight.altitude << "m!\n";

    return true;
}

/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there.
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control.
!*/
bool
moveByPositionOffset(Vehicle *vehicle, float xOffsetDesired,
                     float yOffsetDesired, float zOffsetDesired,
                     float yawDesired, float posThresholdInM,
                     float yawThresholdInDeg)
{
    // Set timeout: this timeout is the time you allow the drone to take to finish
    // the
    // mission
    int responseTimeout              = 1;
    int timeoutInMilSec              = 10000;
    int controlFreqInHz              = 50; // Hz
    int cycleTimeInMs                = 1000 / controlFreqInHz;
    int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
    int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
    int pkgIndex;

    //@todo: remove this once the getErrorCode function signature changes
    char func[50];

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        // Telemetry: Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(responseTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            return false;
        }

        // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
        // Hz
        pkgIndex                  = 0;
        int       freq            = 50;
        TopicName topicList50Hz[] = { TOPIC_QUATERNION, TOPIC_GPS_FUSED };
        int       numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
        bool      enableTimestamp = false;

        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
                pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
        if (!(pkgStatus))
        {
            return pkgStatus;
        }
        subscribeStatus =
                vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            // Cleanup before return
            vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
            return false;
        }
    }

    // Wait for data to come in
    sleep(1);

    // Get data

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition currentBroadcastGP;
    Telemetry::GlobalPosition originBroadcastGP;

    // Convert position offset from first position to local coordinates
    Telemetry::Vector3f localOffset;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        originSubscriptionGPS  = currentSubscriptionGPS;
        localOffsetFromGpsOffset(vehicle, localOffset,
                                 static_cast<void*>(&currentSubscriptionGPS),
                                 static_cast<void*>(&originSubscriptionGPS));
    }
    else
    {
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
        originBroadcastGP  = currentBroadcastGP;
        localOffsetFromGpsOffset(vehicle, localOffset,
                                 static_cast<void*>(&currentBroadcastGP),
                                 static_cast<void*>(&originBroadcastGP));
    }

    // Get initial offset. We will update this in a loop later.
    double xOffsetRemaining = xOffsetDesired - localOffset.x;
    double yOffsetRemaining = yOffsetDesired - localOffset.y;
    double zOffsetRemaining = zOffsetDesired - (-localOffset.z);

    // Conversions
    double yawDesiredRad     = DEG2RAD * yawDesired;
    double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

    //! Get Euler angle

    // Quaternion retrieved via subscription
    Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
    // Quaternion retrieved via broadcast
    Telemetry::Quaternion broadcastQ;

    double yawInRad;
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
        yawInRad = toEulerAngle((static_cast<void*>(&subscriptionQ))).z / DEG2RAD;
    }
    else
    {
        broadcastQ = vehicle->broadcast->getQuaternion();
        yawInRad   = toEulerAngle((static_cast<void*>(&broadcastQ))).z / DEG2RAD;
    }

    int   elapsedTimeInMs     = 0;
    int   withinBoundsCounter = 0;
    int   outOfBounds         = 0;
    int   brakeCounter        = 0;
    int   speedFactor         = 2;
    float xCmd, yCmd, zCmd;
    // There is a deadband in position control
    // the z cmd is absolute height
    // while x and y are in relative
    float zDeadband = 0.12;

    if (vehicle->isM100() || vehicle->isLegacyM600())
    {
        zDeadband = 0.12 * 10;
    }

    /*! Calculate the inputs to send the position controller. We implement basic
     *  receding setpoint position control and the setpoint is always 1 m away
     *  from the current position - until we get within a threshold of the goal.
     *  From that point on, we send the remaining distance as the setpoint.
     */
    if (xOffsetDesired > 0)
        xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
    else if (xOffsetDesired < 0)
        xCmd =
                (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
    else
        xCmd = 0;

    if (yOffsetDesired > 0)
        yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
    else if (yOffsetDesired < 0)
        yCmd =
                (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
    else
        yCmd = 0;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        zCmd = currentSubscriptionGPS.altitude + zOffsetDesired;
    }
    else
    {
        zCmd = currentBroadcastGP.altitude + zOffsetDesired;
    }

    //! Main closed-loop receding setpoint position control
    while (elapsedTimeInMs < timeoutInMilSec)
    {

        vehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd,
                                             yawDesiredRad / DEG2RAD);

        usleep(cycleTimeInMs * 1000);
        elapsedTimeInMs += cycleTimeInMs;

        //! Get current position in required coordinates and units
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
            subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
            yawInRad      = toEulerAngle((static_cast<void*>(&subscriptionQ))).z;
            currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
            localOffsetFromGpsOffset(vehicle, localOffset,
                                     static_cast<void*>(&currentSubscriptionGPS),
                                     static_cast<void*>(&originSubscriptionGPS));
        }
        else
        {
            broadcastQ         = vehicle->broadcast->getQuaternion();
            yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
            currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
            localOffsetFromGpsOffset(vehicle, localOffset,
                                     static_cast<void*>(&currentBroadcastGP),
                                     static_cast<void*>(&originBroadcastGP));
        }

        //! See how much farther we have to go
        xOffsetRemaining = xOffsetDesired - localOffset.x;
        yOffsetRemaining = yOffsetDesired - localOffset.y;
        zOffsetRemaining = zOffsetDesired - (-localOffset.z);

        //! See if we need to modify the setpoint
        if (std::abs(xOffsetRemaining) < speedFactor)
            xCmd = xOffsetRemaining;
        if (std::abs(yOffsetRemaining) < speedFactor)
            yCmd = yOffsetRemaining;

        if (vehicle->isM100() && std::abs(xOffsetRemaining) < posThresholdInM &&
            std::abs(yOffsetRemaining) < posThresholdInM &&
            std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
        {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
        }
        else if (std::abs(xOffsetRemaining) < posThresholdInM &&
                 std::abs(yOffsetRemaining) < posThresholdInM &&
                 std::abs(zOffsetRemaining) < zDeadband &&
                 std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
        {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
        }
        else
        {
            if (withinBoundsCounter != 0)
            {
                //! 2. Start incrementing an out-of-bounds counter
                outOfBounds += cycleTimeInMs;
            }
        }
        //! 3. Reset withinBoundsCounter if necessary
        if (outOfBounds > outOfControlBoundsTimeLimit)
        {
            withinBoundsCounter = 0;
            outOfBounds         = 0;
        }
        //! 4. If within bounds, set flag and break
        if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
        {
            break;
        }
    }

    //! Set velocity to zero, to prevent any residual velocity from position
    //! command
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        while (brakeCounter < withinControlBoundsTimeReqmt)
        {
            vehicle->control->emergencyBrake();
            usleep(cycleTimeInMs);
            brakeCounter += cycleTimeInMs;
        }
    }

    if (elapsedTimeInMs >= timeoutInMilSec)
    {
        std::cout << "Task timeout!\n";
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
            ACK::ErrorCode ack =
                    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
            if (ACK::getError(ack))
            {
                std::cout << "Error unsubscribing; please restart the drone/FC to get "
                        "back to a clean state.\n";
            }
        }
        return ACK::FAIL;
    }

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        ACK::ErrorCode ack =
                vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        if (ACK::getError(ack))
        {
            std::cout
                    << "Error unsubscribing; please restart the drone/FC to get back "
                            "to a clean state.\n";
        }
    }

    return ACK::SUCCESS;
}

bool
moveByAttitudeThrust(Vehicle *vehicle, float xRoll,
                     float yPitch, float zThrust,
                     float yawDesired, float attThresholdInDeg,
                     float thrustThreshold)
{
    // Set timeout: this timeout is the time you allow the drone to take to finish
    // the
    // mission
    int responseTimeout              = 1;
    int timeoutInMilSec              = 3500;
    // Telemetry Tracking Deleted - desined for non-M100

    // Wait for data to come in
    sleep(1);

    // Get data
    int controlFreqInHz              = 50; // Hz
    int cycleTimeInMs                = 1000 / controlFreqInHz;
    int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
    int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
    int pkgIndex;

    std::ofstream outfile;
    outfile.open ("QuaterionRecent.txt", std::ofstream::app);
    outfile << "\n new control for (roll, pitch,roll,thrust)" << xRoll << yPitch << yawDesired << zThrust << std::endl;

    //@todo: remove this once the getErrorCode function signature changes
    char func[50];


    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition currentBroadcastGP;
    Telemetry::GlobalPosition originBroadcastGP;

    // Convert position offset from first position to local coordinates
    Telemetry::Vector3f localOffset;


    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    originBroadcastGP  = currentBroadcastGP;
    localOffsetFromGpsOffset(vehicle, localOffset,
                             static_cast<void*>(&currentBroadcastGP),
                             static_cast<void*>(&originBroadcastGP));


    /*! Get initial offset. We will update this in a loop later.*/
/*
    double xOffsetRemaining = xOffsetDesired - localOffset.x;
    double yOffsetRemaining = yOffsetDesired - localOffset.y;
    double zOffsetRemaining = zOffsetDesired - (-localOffset.z);
*/

    float xCmd, yCmd, zCmd;
    // There is a deadband in position control
    // the z cmd is 0.0-100.0 thrust percentage
    // while x and y are in relative
    float zDeadband = 0.1;

    /*! set deadband here if doing that */


    //! Sets controls to those from Function Input
    xCmd = xRoll; //degrees
    yCmd = yPitch;
    zCmd = zThrust; //percent [0, 100]

/*    //! Set systm limits for []Cmd inputs
    float zLimit      = 33;
    float xLimit      = 7;
    float yLimit      = 7;
    if (zCmd > zLimit) {
        zCmd = zLimit;  //! How does this affect attitude, @todo very bad probably
    }
    if (abs(xCmd) > xLimit)
    {
        if (std::signbit(xCmd))
        {
            xCmd = -xLimit;
        }
        else
        {
            xCmd = xLimit;
        }
    }
    if (abs(yCmd) > yLimit)
    {
        if (std::signbit(yCmd))
        {
            yCmd = -yLimit;
        }
        else
        {
            yCmd = yLimit;
        }
    }*/

    // Conversions
    double yawDesiredRad     = DEG2RAD * yawDesired;
    double yawThresholdInRad = DEG2RAD * attThresholdInDeg;

    double xDesiredRad       = DEG2RAD * xRoll;
    double yDesiredRad       = DEG2RAD * yPitch;
    //! Get Euler angle
    // Quaternion retrieved via subscription
    Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
    // Quaternion retrieved via broadcast
    Telemetry::Quaternion broadcastQ;

    // converts and defines the current body yaw angle from Quaterions to Eulers angles, in radians
    double yawInRad; double pitchInRad; double rollInRad;

    broadcastQ = vehicle->broadcast->getQuaternion();
    yawInRad   = toEulerAngle((static_cast<void*>(&broadcastQ))).z / DEG2RAD;
    pitchInRad = toEulerAngle((static_cast<void*>(&broadcastQ))).y / DEG2RAD;
    rollInRad  = toEulerAngle((static_cast<void*>(&broadcastQ))).x / DEG2RAD;


    int   elapsedTimeInMs     = 0;
    int   withinBoundsCounter = 0;
    int   outOfBounds         = 0;
    int   brakeCounter        = 0;
    int   speedFactor         = 2;
    //std::ofstream outfile;

    //! Main closed-loop attitude thrust control
    while (elapsedTimeInMs < timeoutInMilSec)
    {
        //!ostringstream not to file, testing this method
        std::ostringstream osstemp; std::string quaternionWrite;
        osstemp << "Attitude Quaternion   (w,x,y,z)       = " << broadcastQ.q0
                  << ", " << broadcastQ.q1 << ", " << broadcastQ.q2 << ", "
                  << broadcastQ.q3 << "\n";
        quaternionWrite = osstemp.str();
        std::cout << quaternionWrite << std::endl;
        outfile << "Attitude Quaternion   (w,x,y,z)       = " << broadcastQ.q0
                   << ", " << broadcastQ.q1 << ", " << broadcastQ.q2 << ", "
                   << broadcastQ.q3 << "\n" << std::endl;
        //fprintf(outputfile, "%s", osstemp.str());

        vehicle->control->attitudeAndVertThrCtrl(xCmd, yCmd, zCmd,
                                             yawDesiredRad / DEG2RAD);

        usleep(cycleTimeInMs * 1000);
        elapsedTimeInMs += cycleTimeInMs;

       //! Get current position in required coordinates and units

            broadcastQ         = vehicle->broadcast->getQuaternion();
            yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
            pitchInRad         = toEulerAngle((static_cast<void*>(&broadcastQ))).y;  // @todo check this is RADian
            rollInRad          = toEulerAngle((static_cast<void*>(&broadcastQ))).x;
            currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
            localOffsetFromGpsOffset(vehicle, localOffset,
                                     static_cast<void*>(&currentBroadcastGP),
                                     static_cast<void*>(&originBroadcastGP));

        //! Checked current position(->attitude) vs desired
        //! May want it to check verse desired here, or in seperate thread
        //! Actually might check attitude error vs current to handle wind effects

        float xErr; float yErr;
        xErr = (xDesiredRad - rollInRad);
        yErr = (yDesiredRad - pitchInRad);

        xCmd = (xDesiredRad - xErr) / DEG2RAD;
        yCmd = (yDesiredRad - yErr) / DEG2RAD;

        if (vehicle->isM100() &&
            std::abs(xErr) < attThresholdInDeg * DEG2RAD &&
            std::abs(yErr) < attThresholdInDeg * DEG2RAD &&
            std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad) //! Likely can't detect zThrust Values/ do deadbanding
        {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
            std::cout << "1. We are within bounds; start incrementing our in-bound counter "<<std::endl;
        }
        else
        {
            if (withinBoundsCounter != 0)
            {
                //! 2. Start incrementing an out-of-bounds counter
                outOfBounds += cycleTimeInMs;
                std::cout << "2. Start incrementing an out-of-bounds counter" <<std::endl;
            }
        }
        //! 3. Reset withinBoundsCounter if necessary
        if (outOfBounds > outOfControlBoundsTimeLimit)
        {
            withinBoundsCounter = 0;
            outOfBounds         = 0;
            std::cout<< " 3. Reset withinBoundsCounter if necessary" <<std::endl;
        }
        //! 4. If within bounds, set flag and break
        if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
        {
            std::cout << "4. If within bounds, set flag and break" <<std::endl;
            break;
        }
    }

    //! Set velocity to zero, to prevent any residual velocity from position
    //! command  Needs to be set for M100

    if (elapsedTimeInMs >= timeoutInMilSec)
    {
        std::cout << "Task timeout!\n";
        //! API failure transmitted to vehicle hear, likely not supported in M100
    }

    //! Error for lost data connection hear, likely not supported in M100
    outfile << "elapsed time was " << elapsedTimeInMs << std::endl;
    outfile.close();
    return ACK::SUCCESS;
}


/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool
monitoredLanding(Vehicle* vehicle, int timeout)
{
    //@todo: remove this once the getErrorCode function signature changes
    char func[50];
    int  pkgIndex;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        // Telemetry: Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            return false;
        }

        // Telemetry: Subscribe to flight status and mode at freq 10 Hz
        pkgIndex                  = 0;
        int       freq            = 10;
        TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                      TOPIC_STATUS_DISPLAYMODE };
        int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
        bool enableTimestamp = false;

        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
                pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);

        if (!(pkgStatus))
        {
            return pkgStatus;
        }

        subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            // Cleanup before return
            vehicle->subscribe->removePackage(pkgIndex, timeout);
            return false;
        }
    }

    // Start landing
    ACK::ErrorCode landingStatus = vehicle->control->land(timeout);
    if (ACK::getError(landingStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(landingStatus, func);
        return false;
    }

    // First check: Landing started
    int landingNotStarted = 0;
    int timeoutCycles     = 20;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
               VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
               landingNotStarted < timeoutCycles)
        {
            landingNotStarted++;
            usleep(100000);
        }
    }
    else if (vehicle->isM100())
    {
        while (vehicle->broadcast->getStatus().flight !=
               DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING &&
               landingNotStarted < timeoutCycles)
        {
            landingNotStarted++;
            usleep(100000);
        }
    }

    if (landingNotStarted == timeoutCycles)
    {
        std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
        // Cleanup before return
        ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack))
        {
            std::cout << "Error unsubscribing; please restart the drone/FC to get "
                    "back to a clean state.\n";
        }
        return false;
    }
    else
    {
        std::cout << "Landing...\n";
    }

    // Second check: Finished landing
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
               vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
               VehicleStatus::FlightStatus::IN_AIR)
        {
            sleep(1);
        }

        if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_P_GPS ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ATTITUDE)
        {
            std::cout << "Successful landing!\n";
        }
        else
        {
            std::cout
                    << "Landing finished, but the aircraft is in an unexpected mode. "
                            "Please connect DJI GO.\n";
            ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);

            if (ACK::getError(ack))
            {
                std::cout << "Error unsubscribing; please restart the drone/FC to get "
                        "back to a clean state.\n";
            }
            return false;
        }
    }
    else if (vehicle->isLegacyM600())
    {
        while (vehicle->broadcast->getStatus().flight >
               DJI::OSDK::VehicleStatus::FlightStatus::STOPED)
        {
            sleep(1);
        }

        Telemetry::GlobalPosition gp;
        do
        {
            sleep(2);
            gp = vehicle->broadcast->getGlobalPosition();
        } while (gp.altitude != 0);

        if (gp.altitude != 0)
        {
            std::cout
                    << "Landing finished, but the aircraft is in an unexpected mode. "
                            "Please connect DJI GO.\n";
            return false;
        }
        else
        {
            std::cout << "Successful landing!\n";
        }
    }
    else // M100
    {
        while (vehicle->broadcast->getStatus().flight ==
               DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING)
        {
            sleep(1);
        }

        Telemetry::GlobalPosition gp;
        do
        {
            sleep(2);
            gp = vehicle->broadcast->getGlobalPosition();
        } while (gp.altitude != 0);

        if (gp.altitude != 0)
        {
            std::cout
                    << "Landing finished, but the aircraft is in an unexpected mode. "
                            "Please connect DJI GO.\n";
            return false;
        }
        else
        {
            std::cout << "Successful landing!\n";
        }
    }

    // Cleanup
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);

        if (ACK::getError(ack))
        {
            std::cout
                    << "Error unsubscribing; please restart the drone/FC to get back "
                            "to a clean state.\n";
        }
    }

    return true;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates.
    Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed,
                         void* target, void* origin)
{
    Telemetry::GPSFused*       subscriptionTarget;
    Telemetry::GPSFused*       subscriptionOrigin;
    Telemetry::GlobalPosition* broadcastTarget;
    Telemetry::GlobalPosition* broadcastOrigin;
    double                     deltaLon;
    double                     deltaLat;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        subscriptionTarget = (Telemetry::GPSFused*)target;
        subscriptionOrigin = (Telemetry::GPSFused*)origin;
        deltaLon   = subscriptionTarget->longitude - subscriptionOrigin->longitude;
        deltaLat   = subscriptionTarget->latitude - subscriptionOrigin->latitude;
        deltaNed.x = deltaLat * C_EARTH;
        deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
        deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
    }
    else
    {
        broadcastTarget = (Telemetry::GlobalPosition*)target;
        broadcastOrigin = (Telemetry::GlobalPosition*)origin;
        deltaLon        = broadcastTarget->longitude - broadcastOrigin->longitude;
        deltaLat        = broadcastTarget->latitude - broadcastOrigin->latitude;
        deltaNed.x      = deltaLat * C_EARTH;
        deltaNed.y      = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
        deltaNed.z      = broadcastTarget->altitude - broadcastOrigin->altitude;
    }
}

Telemetry::Vector3f
toEulerAngle(void* quaternionData)
{
    Telemetry::Vector3f    ans;
    Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

    double q2sqr = quaternion->q2 * quaternion->q2;
    double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
    double t1 =
            +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
    double t2 =
            -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
    double t3 =
            +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
    double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;

    ans.x = asin(t2);
    ans.y = atan2(t3, t4);
    ans.z = atan2(t1, t0);

    return ans;
}
