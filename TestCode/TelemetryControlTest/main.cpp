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
main(int argc, char** argv)
{
    std::ofstream outfile;
    //std::ofstream outfile ("QuaterionRecent.txt");
    outfile.open ("QuaterionRecent.txt", std::ofstream::app);
    outfile << "\n \n New Test" <<std::endl;
    outfile.close();

    //FILE * outputfile;
    //outputfile = fopen("QuaterionRecent.txt","a+");
    //fprintf(outputfile,"\n \n \n \n New Test");

    // Test Output
    std::cout<< "Dialogue Test" <<std::endl;
    // Initialize variables
    int functionTimeout = 1;

    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle* vehicle = linuxEnvironment.getVehicle();
    if (vehicle == NULL)
    {
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
            << "| [f] Takeoff + polynomial follow                                 |"
            << std::endl;
    std::cout
            << "| [g] Takeoff + waypoint following                                |"
            << std::endl;
    char inputChar;
    std::cin >> inputChar;

    //! Test polynomial values - DO NOT RUN WITH THESE< CAUSE NOT GOOD - FIXED WING
    double aMan[] = {0, 3.9691e1, 4.7873e-1, -2.8244e-5, -2.2783e-4, -1.2762e-3, 8.7194e-5};
    double bMan[] = {9.1440, 1.0635e1, -2.7946, 2.0438e-1, -2.5241e-2, 4.3437e-3, -2.2276e-4};
    double cMan[] = {-1.6764e1, 2.1535, 2.0075e-1, -2.4922e-2, 1.5728e-3, -4.6684e-4, 3.1117e-5};

    //! Test Waypoint flight points - expandable for nx4
    std::vector<std::vector<float>> waypoints;
    waypoints.push_back({0,0,5,0});
    waypoints.push_back({0,0,7,0});
    waypoints.push_back({0,5,7,0});
    waypoints.push_back({-5,5,7,0});
    waypoints.push_back({-5,0,7,0});
    waypoints.push_back({0,0,5,0});

    switch (inputChar)
    {
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
            std::cout<<"May need to adjust attitude threshold "<<std::endl;
            std::cout<<"Roll Angle Degrees"<<std::endl;
            float iRol;
            std::cin >> iRol;
            std::cout<<"Pitch Angle Degrees"<<std::endl;
            float iPit;
            std::cin >> iPit;
            std::cout<<"Yaw Angle Degrees"<<std::endl;
            float iYaw;
            std::cin >> iYaw;
            std::cout<<"Thrust Percentage, from 0 - 100: Hover near 25"<<std::endl;
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
            //trajectoryControllerTestCrude(vehicle,aMan,bMan,cMan);
            monitoredLanding(vehicle);
            break;
        case 'g':
            for (int nn=0;nn<=waypoints.size();nn=nn+1)
            {
                float xOf = waypoints[nn][0]; float yOf = waypoints[nn][1]; float zOf = waypoints[nn][2];
                float yawSpe = waypoints[nn][3];
                std::cout << "x= "<<xOf<< " ;y= "<<yOf<< " ;z= "<<zOf<<" ;yaw = "<< yawSpe << std::endl;
            }
            trajectoryWaypointControllerTest(vehicle,waypoints);
            break;
        default:
            break;
    }
    std::cout << "End" << std::endl;
//    outfile.close();
    //fclose(outputfile);
    return 0;
}
