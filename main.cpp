// Xaegrek
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <unistd.h>
#include <math.h>

typedef std::chrono::high_resolution_clock Clock;

int main() {
    std::cout << "Hello, World!" << std::endl;
    std::cout << "Test Files" << std::endl;

    std::ofstream outfile ("UserConfig.txt");

    outfile << "app_id : 1051117\n"
            "app_key : 8de7ffac1a632b8b6e21df0026532e12e621b2ab4193c7dc65265a8f061655e5\n"
            "device : /dev/serial0\n"
            "baudrate : 230400"<<std::endl;
    outfile.close();

    std::ostringstream osstemp;std::string test; float x=0.01; int y=3;
    osstemp << "Test string, followed by float and int: " << x << " " << y << std::endl;
    test = osstemp.str();
    std::cout << test;


    osstemp << "Test string, followed by float and int: " << x << " " << y << std::endl;
    test = osstemp.str();

    std::ofstream outfile2 ("QuaterionRecent.txt", std::ofstream::app);
    outfile2 << "test" << std::endl;
    outfile2 << "retest" << std::endl;
    outfile2 << x << std::endl;
    outfile2.close();


    double aMan[] = {0, 3.9691e1, 4.7873e-1, -2.8244e-5, -2.2783e-4, -1.2762e-3, 8.7194e-5};
    double bMan[] = {9.1440, 1.0635e1, -2.7946, 2.0438e-1, -2.5241e-2, 4.3437e-3, -2.2276e-4};
    double cMan[] = {-1.6764e1, 2.1535, 2.0075e-1, -2.4922e-2, 1.5728e-3, -4.6684e-4, 3.1117e-5};

    int nDim = sizeof(aMan)/ sizeof(aMan[0]);
    /*! Takeoff and time set
     *
     !*/

    auto tTrajOrig   = Clock::now();                                // Initialization Time


    auto tTraj       = Clock::now();                            // Current run time
    std::chrono::duration<double> tTrajTemp = tTraj - tTrajOrig;    // Time since begining
    auto tTrajN = tTrajTemp.count();
    double xTr; double yTr; double zTr;
    for (int nn = 0; nn <= nDim; nn = nn +1)
    {
        xTr = xTr + aMan[nn] * pow(tTrajN,nn);
        yTr = yTr + bMan[nn] * pow(tTrajN,nn);
        zTr = zTr + cMan[nn] * pow(tTrajN,nn);
        std::cout<<nn<<std::endl;
    }

    std::cout<< "x "<<xTr<<std::endl<< "y "<<yTr<<std::endl<< "z "<<zTr<<std::endl;
    return 0;
}