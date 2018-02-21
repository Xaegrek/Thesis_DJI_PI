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
    outfile << "\n \n New Test";
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
    Vehicle*   vehicle = linuxEnvironment.getVehicle();
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
    char inputChar;
    std::cin >> inputChar;

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
            moveByAttitudeThrust(vehicle, 1, 1, 20, 90, 3500);
            monitoredLanding(vehicle);
            break;
        case '#':
        std::cin >> inputChar;
            if (inputChar == 'c') {
                monitoredTakeoff(vehicle);
                moveByPositionOffset(vehicle, 0, 0, 5, 0);
                moveByAttitudeThrust(vehicle, 1, 1, 40, 90, 3500); //change to lower value for real system
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
            std::cout<<"Timeout duration for action, default was 5000, changed to 3500 worked well"<<std::endl;
            int iTim;
            std::cin >> iTim;

            monitoredTakeoff(vehicle);
            moveByPositionOffset(vehicle, 0, 0, 5, 0); //! hover to selttle sisystem
            moveByAttitudeThrust(vehicle, iRol, iPit, iThr, iYaw, iTim);
            monitoredLanding(vehicle);
            break;
        case 'e':
            monitoredTakeoff(vehicle);
            moveByPositionOffset(vehicle, 0, 0, 5, 0); //! hover to settle system
            moveByAttitudeThrust(vehicle, 5, 5, 38, 90, 1000);
            moveByAttitudeThrust(vehicle, -5, -5, 38, 180, 1000);
            moveByAttitudeThrust(vehicle, 0, 0, 45, 0, 500);
            monitoredLanding(vehicle);
            break;
        default:
            break;
    }
    std::cout << "End" << std::endl;
//    outfile.close();
    //fclose(outputfile);
    return 0;
}
