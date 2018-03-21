// Xaegrek
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <unistd.h>
#include <math.h>
#include <vector>

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

    std::ostringstream osstemp;std::string test; float x=0.01; int y=0;
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


    std::vector<std::vector<float>> waypoints;
    waypoints.push_back({0,0,5,0});
    waypoints.push_back({0,0,7,0});
    waypoints.push_back({0,5,7,0});
    waypoints.push_back({-5,5,7,0});
    waypoints.push_back({-5,0,7,0});
    waypoints.push_back({0,0,5,0});
    unsigned long nDim = waypoints.size();

    for (int nn=0;nn<nDim;nn=nn+1)
    {

        std::cout << waypoints[nn][0]<< " n is " << nn << 2e6<<std::endl;
    }
    //waypoints[0] = {6,3,3,2};
    //waypoints[1] = {1.2,3.4,2.2,14.3};
    //std::cout << waypoints[0] << std::endl << waypoints[1][3] <<std::endl;
    double aMan[] = {0, 3.9691e1, 4.7873e-1, -2.8244e-5, -2.2783e-4, -1.2762e-3, 8.7194e-5};
    double bMan[] = {9.1440, 1.0635e1, -2.7946, 2.0438e-1, -2.5241e-2, 4.3437e-3, -2.2276e-4};
    double cMan[] = {-1.6764e1, 2.1535, 2.0075e-1, -2.4922e-2, 1.5728e-3, -4.6684e-4, 3.1117e-5};

    nDim = sizeof(aMan)/ sizeof(aMan[0]);
    /*

    *//*! Takeoff and time set
     *
     !*//*

    auto tTrajOrig   = Clock::now();                                // Initialization Time

    while (y<15) {
        auto tTraj = Clock::now();                            // Current run time
        std::chrono::duration<double> tTrajTemp = tTraj - tTrajOrig;    // Time since begining
        auto tTrajN = tTrajTemp.count();
        double xTr;
        double yTr;
        double zTr;
        for (int nn = 0; nn <= nDim; nn = nn +1)
        {
            xTr         = xTr + aMan[nn] * pow(tTrajN,nn);
            yTr         = yTr + bMan[nn] * pow(tTrajN,nn);
            zTr         = zTr + cMan[nn] * pow(tTrajN,nn);
        }
        double xdTr; double ydTr; double zdTr;
        for (int nn = 1; nn <= nDim; nn = nn +1)
        {
            xdTr        = xdTr + nn * aMan[nn] * pow(tTrajN,nn-1);
            ydTr        = ydTr + nn * bMan[nn] * pow(tTrajN,nn-1);
            zdTr        = zdTr + nn * cMan[nn] * pow(tTrajN,nn-1);
        }
        double xddTr; double yddTr; double zddTr;
        for (int nn = 2; nn <= nDim; nn = nn +1)
        {
            xddTr       = xddTr + nn * (nn-1) * aMan[nn] * pow(tTrajN,nn-2);
            yddTr       = yddTr + nn * (nn-1) * bMan[nn] * pow(tTrajN,nn-2);
            zddTr       = zddTr + nn * (nn-1) * cMan[nn] * pow(tTrajN,nn-2);
        }
        double psiTr    = atan2(ydTr,xdTr);
        double psidTr   = (yddTr * xdTr - ydTr * xddTr) / ( pow(xdTr,2) + pow(ydTr,2));
        double gamTr    = atan2(-zdTr,sqrt(pow(xdTr,2) + pow(ydTr,2)));
        double gamdTr   = (zdTr * (xdTr*xddTr + ydTr*yddTr) - zddTr * (pow(xdTr,2) + pow(ydTr,2))) /
                          (sqrt(pow(xdTr,2) + pow(ydTr,2)) * (pow(xdTr,2) + pow(ydTr,2) + pow(zdTr,2)));


        std::cout << "x " << xTr << std::endl << "yd " << ydTr << std::endl << "zdd " << zddTr << std::endl;
        std::cout << "psi " << psiTr <<" psid " <<psidTr << " gam " << gamTr << " gamd " << gamdTr <<std::endl;
        usleep(1e6); y++;
    }*/
    return 0;
}