// Xaegrek
#include <iostream>
#include <fstream>
#include <sstream>

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

    std::ofstream outfile2 ("QuaterionRecent.txt");
    outfile2.close();
    return 0;
}