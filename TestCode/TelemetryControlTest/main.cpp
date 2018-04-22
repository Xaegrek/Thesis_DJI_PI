//
// Created by xaegrek on 2/6/18.
//

// System Includes
#include "TelemetryControlTest.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

/*! main, based on code from OSDK Sample flight-control
 *
 */
int
main(int argc, char **argv) {
    std::ofstream outfile;
    //std::ofstream outfile ("QuaterionRecent.txt");
    outfile.open("QuaterionRecent.txt", std::ofstream::app);
    outfile << "\n \n New Test" << std::endl;
    outfile.close();

    //FILE * outputfile;
    //outputfile = fopen("QuaterionRecent.txt","a+");
    //fprintf(outputfile,"\n \n \n \n New Test");

    // Test Output
    std::cout << "Dialogue Test" << std::endl;
    // Initialize variables
    int functionTimeout = 1;

    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle *vehicle = linuxEnvironment.getVehicle();
    if (vehicle == NULL) {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }

    // Obtain Control Authority
    vehicle->obtainCtrlAuthority(functionTimeout);

    // Display interactive prompt
    std::cout
            << "| Available commands:                                             |"
            << std::endl;
    std::cout
            << "| [a] Monitored Takeoff + Landing                                 |"
            << std::endl;
    std::cout
            << "| [b] Monitored Takeoff + Position Control + Landing              |"
            << std::endl;
    std::cout
            << "| [c] Monitored Takeoff + Attitude and Thrust Control + Landing   |"
            << std::endl;
    std::cout
            << "| [d] Takeoff + Attitude and Thrust  + Landing with custom values |"
            << std::endl;
    std::cout
            << "| [e] Takeoff + Attitude and Thrust  + Landing with short timeouts|"
            << std::endl;
    std::cout
            << "| [f] Takeoff + polynomial follow (position) + landing at 0,0     |"
            << std::endl;
    std::cout
            << "| [F] Takeoff + polynomial follow (velocity) + landing at 0,0     |"
            << std::endl;
    std::cout
            << "| [t] Takeoff + polynomial follow (bothish) + landing at 0,0     |"
            << std::endl;
    std::cout
            << "| [g] Takeoff + waypoint-style offset pathing + landing           |"
            << std::endl;
    std::cout
            << "| [h] Takeoff + waypoint following + landing                      |"
            << std::endl;
    char inputChar;
    std::cin >> inputChar;

    //! Test polynomial values - DO NOT RUN WITH THESE< CAUSE NOT GOOD - FIXED WING
    bool ctrlStyleThrust;
    double aMan[] = {0, 3.9691e-1,0};
    double bMan[] = {5.1440, -2.0635e-1,0};
    double cMan[] = {-1.6764e0, -2.1535e-1,0};
    double timeTrajEnd = 30;
    int     nDim = sizeof(aMan)/ sizeof(aMan[0]);

    //! Test Waypoint flight points - expandable for nx4
    std::vector<std::vector<float>> waypoints;
    waypoints.push_back({0, 0, 5, 0});
    waypoints.push_back({0, 0, 7, 0});
    waypoints.push_back({0, 5, 7, 0});
    waypoints.push_back({-5, 10, 7, 0});
    waypoints.push_back({-5, 5, 7, 0});
    waypoints.push_back({0, 0, 5, 0});
    int nn;
    float xOf;
    float yOf;
    float zOf;
    float yawSpe;

    switch (inputChar) {
        case 'a':
            monitoredTakeoff(vehicle);
            monitoredLanding(vehicle);
            break;
        case 'b':
            monitoredTakeoff(vehicle);
            moveByPositionOffset(vehicle, 0, 6, 6, 30);
            moveByPositionOffset(vehicle, 6, 0, -3, -30);
            moveByPositionOffset(vehicle, -6, -6, 0, 0);
            monitoredLanding(vehicle);
            break;
        case 'c':
            monitoredTakeoff(vehicle);
            moveByPositionOffset(vehicle, 0, 0, 5, 0);
            moveByAttitudeThrust(vehicle, 1, 1, 20, 90);
            monitoredLanding(vehicle);
            break;
        case '#':
            std::cin >> inputChar;
            if (inputChar == 'c') {
                monitoredTakeoff(vehicle);
                moveByPositionOffset(vehicle, 0, 0, 5, 0);
                moveByAttitudeThrust(vehicle, 1, 1, 40, 90); //change to lower value for real system
                monitoredLanding(vehicle);
            }
            break;
        case 'd':
            std::cout << "May need to adjust attitude threshold " << std::endl;
            std::cout << "Roll Angle Degrees" << std::endl;
            float iRol;
            std::cin >> iRol;
            std::cout << "Pitch Angle Degrees" << std::endl;
            float iPit;
            std::cin >> iPit;
            std::cout << "Yaw Angle Degrees" << std::endl;
            float iYaw;
            std::cin >> iYaw;
            std::cout << "Thrust Percentage, from 0 - 100: Hover near 25" << std::endl;
            float iThr;
            std::cin >> iThr;

            monitoredTakeoff(vehicle);
            moveByPositionOffset(vehicle, 0, 0, 5, 0); //! hover to selttle sisystem
            moveByAttitudeThrust(vehicle, iRol, iPit, iThr, iYaw);
            monitoredLanding(vehicle);
            break;
        case 'e':
            monitoredTakeoff(vehicle);
            moveByPositionOffset(vehicle, 0, 0, 5, 0); //! hover to settle system
            moveByAttitudeThrust(vehicle, 5, 5, 38, 90);
            moveByAttitudeThrust(vehicle, -5, -5, 38, 180);
            moveByAttitudeThrust(vehicle, 0, 0, 45, 0);
            monitoredLanding(vehicle);
            break;
        case 'f':
            ctrlStyleThrust = false;
            monitoredTakeoff(vehicle);
            //  moveByPositionOffset(vehicle, 0, 0, 5, 0);
            trajectoryControllerTestCrude(vehicle,aMan,bMan,cMan, timeTrajEnd, nDim, ctrlStyleThrust);
            monitoredLanding(vehicle);
            break;
        case 'F':
            ctrlStyleThrust = true;
            monitoredTakeoff(vehicle);
            moveByPositionOffset(vehicle, 0, 0, 5, 0);
            trajectoryControllerTestCrude(vehicle,aMan,bMan,cMan, timeTrajEnd, nDim, ctrlStyleThrust);
            monitoredLanding(vehicle);
            break;
        case 't':
            ctrlStyleThrust = true;
            monitoredTakeoff(vehicle);
            moveByPositionOffset(vehicle, 0, 0, 5, 0);
            trajectoryControllerTestCrude(vehicle,aMan,bMan,cMan, timeTrajEnd, nDim, ctrlStyleThrust);
            ctrlStyleThrust = false;
            trajectoryControllerTestCrude(vehicle,aMan,bMan,cMan, 15, nDim, ctrlStyleThrust);
            monitoredLanding(vehicle);
        case 'g':
            for (nn = 0; nn < waypoints.size(); nn = nn + 1) {
                xOf = waypoints[nn][0];
                yOf = waypoints[nn][1];
                zOf = waypoints[nn][2];
                yawSpe = waypoints[nn][3];
                std::cout << "x= " << xOf << " ;y= " << yOf << " ;z= " << zOf << " ;yaw = " << yawSpe << std::endl;
            }
            trajectoryWaypointOffsetControllerTest(vehicle, waypoints);
            break;
        case 'h':
            for (nn = 0; nn < waypoints.size(); nn = nn + 1) {
                xOf = waypoints[nn][0];
                yOf = waypoints[nn][1];
                zOf = waypoints[nn][2];
                yawSpe = waypoints[nn][3];
                std::cout << "x= " << xOf << " ;y= " << yOf << " ;z= " << zOf << " ;yaw = " << yawSpe << std::endl;
            }
            trajectoryWaypointControllerTestTimer(vehicle, waypoints);
            break;
        case 'H':
            for (nn = 0; nn < waypoints.size(); nn = nn + 1) {
                xOf = waypoints[nn][0];
                yOf = waypoints[nn][1];
                zOf = waypoints[nn][2];
                yawSpe = waypoints[nn][3];
                std::cout << "x= " << xOf << " ;y= " << yOf << " ;z= " << zOf << " ;yaw = " << yawSpe << std::endl;
            }
            trajectoryWaypointControllerTest(vehicle, waypoints);
            break;
        default:
            break;
    }
    std::cout << "End" << std::endl;
//    outfile.close();
    //fclose(outputfile);
    return 0;
}
