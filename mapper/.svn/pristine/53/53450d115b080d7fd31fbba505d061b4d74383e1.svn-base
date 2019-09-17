/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <algorithm>
#include <set>
#include <unittest++/UnitTest++.h>
#include "m1.h"
#include "unit_test_util.h"
#include "StreetsDatabaseAPI.h"

using ece297test::relative_error;

struct MapFixture {
    MapFixture() {
        //Load the map
        load_map("/cad2/ece297s/public/maps/toronto_canada.streets.bin");
        
        //Initialize random number generators
        //Change seed of rng every time for true randomness
        rng = std::minstd_rand(5);
        rand_intersection = std::uniform_int_distribution<unsigned>(0, getNumberOfIntersections()-1);
        rand_street = std::uniform_int_distribution<unsigned>(1, getNumberOfStreets()-1);
        rand_segment = std::uniform_int_distribution<unsigned>(0, getNumberOfStreetSegments()-1);
        rand_poi = std::uniform_int_distribution<unsigned>(0, getNumberOfPointsOfInterest()-1);
        rand_lat = std::uniform_real_distribution<double>(43.48, 43.91998);
        rand_lon = std::uniform_real_distribution<double>(-79.78998, -79.00001);
    }

    ~MapFixture() {
        //Clean-up
        close_map();
    }
    std::minstd_rand rng;
    std::uniform_int_distribution<unsigned> rand_intersection;
    std::uniform_int_distribution<unsigned> rand_street;
    std::uniform_int_distribution<unsigned> rand_segment;
    std::uniform_int_distribution<unsigned> rand_poi;
    std::uniform_real_distribution<double> rand_lat;
    std::uniform_real_distribution<double> rand_lon;
};


SUITE(distance_time_queries_public_toronto_canada) {
    
    TEST_FIXTURE(MapFixture, find_street_ids) {
        std::vector<unsigned> expected;
        std::vector<unsigned> actual;
        std::string streetName;
        
        //Generate random inputs
        unsigned num = 1000000;
        std::vector<std::string> street_names;
        for(unsigned i = 0; i < num; i++) {
            street_names.push_back(getStreetName(rand_street(rng)));
        }
        
        for(unsigned i = 0; i < num; i++){
            actual = find_street_ids_from_name(street_names[i]);
            streetName = getStreetName(actual[0]);
            CHECK_EQUAL(street_names[i], streetName); 
        }
        
        {
            //Timed Test
            ECE297_TIME_CONSTRAINT(250);
            std::vector<unsigned> result;
            for(size_t i = 0; i < num; i++) {
                result = find_street_ids_from_name(street_names[i]);
            }
        }
       
    }
}
