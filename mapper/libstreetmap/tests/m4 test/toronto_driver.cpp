#include <iostream>
#include <string>
#include <random>
#include <unittest++/UnitTest++.h>

#include "m1.h"
#include "m2.h"

std::string map_name = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";

int main(int argc, char** argv) {
    bool load_success = load_map(map_name);

    if(!load_success) {
        std::cout << "ERROR: Could not load map file: '" << map_name << "'!";
        std::cout << " Subsequent tests will likely fail." << std::endl;
        //Don't abort tests, since we still want to show that all
        //tests fail.
    }

    //Run the unit tests
    int num_failures = UnitTest::RunAllTests();

    close_map();

    return num_failures;
}
