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
    // Test Output
    std::cout<< "Test Output" <<std::endl;
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
            << "| Available commands:                                            |"
            << std::endl;
    std::cout
            << "| [a] Monitored Takeoff + Landing                                |"
            << std::endl;
    std::cout
            << "| [b] Monitored Takeoff + Position Control + Landing             |"
            << std::endl;
    std::cout
            << "| [c] Monitored Takeoff + Attitude and Thrust Control + Landing  |"
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
            moveByAttitudeThrust(vehicle, 10, 1, 1, 20);
            monitoredLanding(vehicle);
            break;
        default:
            break;
    }

    return 0;
}
