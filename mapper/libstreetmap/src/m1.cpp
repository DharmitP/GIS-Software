/*
 
 ECE297: MILESTONE 1
 * 
 * Team 043
 * 
 * Daniyal Qureshi
 * Dharmit Patel
 * Xingyu Ma
 *  
 */


#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <map>

const double KM_PER_HR_TO_METRES_PER_SECOND_FACTOR = 1000.0/3600;

//Processing functions
void street_segment_processing();
void combined_processing();


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

std::string cityName;

/************************** GLOBAL DATA STRUCTURES **************************/
//Street segments incident to each intersection
std::vector<std::vector<unsigned>> intersection_street_segments;

//Street segments valid to travel on that are incident to each intersection
std::vector<std::vector<unsigned>> valid_street_segments_to_intersection;

//Intersection street names
std::vector<std::vector<std::string>> intersection_street_names;

//Street segments for a given street
std::vector<std::vector<unsigned>> street_street_segments;

//The length of each street segment
std::vector<double> street_segment_lengths;

//Intersections for each street
std::vector<std::vector<unsigned>> street_street_intersections;

//Adjacent intersections to each 
std::vector<std::vector<unsigned>> intersection_adjacent;

//Adjacent intersections to each, including duplicates
std::vector<std::vector<unsigned>> intersection_adjacent_dup;

//The length of each street segment
std::vector<double> street_lengths;

//Map with street name as key and vector of street ID's as values
std::unordered_map<std::string, std::vector<unsigned>> street_id_from_name;

//The travel time for each street segment
std::vector<double> street_segment_time;

//Map with street name as key and vector of intersection ID's as values
std::unordered_map<std::string, std::vector<unsigned>> intersection_ids_from_street_name;

//LatLon positions for curve points, From, To for each street segment
std::vector<std::vector<LatLon>> street_segment_positions;

//The index where the longest curve point starts for all street segments
std::vector<unsigned> street_segment_max_curve;

//Street segment information structures in a vector
std::vector<StreetSegmentInfo> street_segment_info;

//Intersection names
std::vector<std::string> intersection_names;

//Intersection positions
std::vector<LatLon> intersection_positions;

//Street names
std::vector<std::string> street_names;

//Map with POI name as key and vector of POI ID's as values
std::unordered_map<std::string, std::vector<unsigned>> poi_id_from_name;

//Point of interest type
std::vector<std::string> POI_types;

//Point of interest names
std::vector<std::string> POI_names;

//Point of interest position
std::vector<LatLon> POI_positions;

//Feature names
std::vector<std::string> feature_names;

//Feature types for all features
std::vector<FeatureType> feature_types;

//Feature positions (all LatLons for polygon or multisegment line)
std::vector<std::vector<LatLon>> feature_positions;

//Area of each feature (sorted in descending order of area)
std::multimap<float, unsigned, std::greater<float>> feature_ids_by_areas;

//Our custom structure holding the location and ID of each point
struct interPoint {
    double x, y;
    unsigned id;
};

// Register the point type in the open-source Boost library
BOOST_GEOMETRY_REGISTER_POINT_2D(interPoint, double,  boost::geometry::cs::cartesian, x, y)

//Define and create two R-Trees to hold these geographic points
typedef bgi::rtree<interPoint, bgi::linear<16>> rtree_t;
rtree_t treeInt;
rtree_t treePOI;

/*******************************************************************/

unsigned numberOfIntersections;
unsigned numberOfSegments;
unsigned numberOfStreets;
unsigned numberOfPOI;
unsigned numberOfFeatures;

double minLat;
double minLon;
double maxLat;
double maxLon;

double AvgLat;

bool map_loaded = false;

//Processes information from the map at startup
bool load_map(std::string map_path) {
    //Load your map related data structures here
    //Initial load of the map
    if (!loadStreetsDatabaseBIN(map_path))
        return false;
    
    map_loaded = true;
    cityName = map_path;
    
    //Read these from map
    numberOfIntersections = getNumberOfIntersections();
    numberOfSegments = getNumberOfStreetSegments();
    numberOfStreets = getNumberOfStreets();
    numberOfPOI = getNumberOfPointsOfInterest();
    numberOfFeatures = getNumberOfFeatures();
        
    //Initialize these values before we find them in the processing 
    minLat = getIntersectionPosition(0).lat();
    minLon = getIntersectionPosition(0).lon();
  
    maxLat = getIntersectionPosition(0).lat();
    maxLon = getIntersectionPosition(0).lon();

    /*------------RESIZE ALL DATA STRUCTURES-------------------------*/
    
    intersection_street_names.resize(numberOfIntersections);
    
    intersection_street_segments.resize(numberOfIntersections);
    
    intersection_adjacent.resize(numberOfIntersections);
    
    intersection_adjacent_dup.resize(numberOfIntersections);
    
    valid_street_segments_to_intersection.resize(numberOfIntersections);
    
    street_segment_time.resize(numberOfSegments);
    
    street_segment_lengths.resize(numberOfSegments);
    
    street_street_segments.resize(numberOfStreets);
    
    street_street_intersections.resize(numberOfStreets);    

    street_lengths.resize(numberOfStreets);

    street_segment_positions.resize(numberOfSegments);
    
    street_segment_info.resize(numberOfSegments);
    
    intersection_names.resize(numberOfIntersections);
    
    intersection_positions.resize(numberOfIntersections);
    
    street_names.resize(numberOfStreets);
    
    POI_types.resize(numberOfPOI);
    
    POI_names.resize(numberOfPOI);
    
    POI_positions.resize(numberOfPOI);
    
    feature_names.resize(numberOfFeatures);
    
    feature_types.resize(numberOfFeatures);
    
    feature_positions.resize(numberOfFeatures);
     
    /*----------------------------------------------------------------*/

    //Fill up our data structures
    street_segment_processing();
    combined_processing();
    
    //Calculate average latitude once processing complete
    AvgLat = (maxLat + minLat)*0.5;
    
    return true;
}

//Empties all data structures before exiting
void close_map() {
    if (map_loaded){
        //Clean-up your map related data structures here
        /*****************************************************/
        closeStreetDatabase();

        intersection_street_segments.clear();
        
        valid_street_segments_to_intersection.clear();

        intersection_street_names.clear();

        street_street_segments.clear();

        street_segment_lengths.clear();

        street_street_intersections.clear();

        intersection_adjacent.clear();
        
        intersection_adjacent_dup.clear();

        street_lengths.clear();

        street_id_from_name.clear();

        intersection_ids_from_street_name.clear();

        street_segment_positions.clear();

        street_segment_info.clear();

        intersection_names.clear();

        intersection_positions.clear();

        street_names.clear();
        
        POI_types.clear();

        POI_names.clear();

        POI_positions.clear();

        feature_names.clear();

        feature_types.clear();

        feature_positions.clear();

        feature_ids_by_areas.clear();
        /*****************************************************/    
    }
}

//Returns street id(s) for the given street name
//If no street with this name exists, returns a 0-length vector.
std::vector<unsigned> find_street_ids_from_name(std::string street_name){

    std::vector<unsigned> s_ids;
    //Tries to access the name, if doesn't exist returns empty vector
    try {
        street_id_from_name.at(street_name);
    } catch (std::out_of_range) {
        return s_ids;
    }
    s_ids = street_id_from_name.at(street_name);
    return s_ids;
}

//Returns the street segments for the given intersection 
std::vector<unsigned> find_intersection_street_segments(unsigned intersection_id){
    return intersection_street_segments[intersection_id];
}

//Returns the street names at the given intersection (includes duplicate street names in returned vector)
std::vector<std::string> find_intersection_street_names(unsigned intersection_id){
    return intersection_street_names[intersection_id];
}

//Returns true if you can get from intersection1 to intersection2 using a single street segment (hint: check for 1-way streets too)
//corner case: an intersection is considered to be connected to itself
bool are_directly_connected(unsigned intersection_id1, unsigned intersection_id2){
    
    //Get all adjacent intersections
    std::vector<unsigned> adj_ints = find_adjacent_intersections(intersection_id1);
    
    //Corner case: if same intersection, return true
    if (intersection_id1 == intersection_id2)
        return true;
    
    //If other intersection is adjacent (in the vector), return true
    return !(std::find(adj_ints.begin(),adj_ints.end(),intersection_id2) == adj_ints.end());
    
}

//Returns all intersections reachable by traveling down one street segment 
//from given intersection (hint: you can't travel the wrong way on a 1-way street)
//the returned vector should NOT contain duplicate intersections
std::vector<unsigned> find_adjacent_intersections(unsigned intersection_id){
    return intersection_adjacent[intersection_id];
}

//Returns all street segments for the given street
std::vector<unsigned> find_street_street_segments(unsigned street_id){
    return street_street_segments[street_id];
}

//Returns all intersections along the a given street
std::vector<unsigned> find_all_street_intersections(unsigned street_id){
    return street_street_intersections[street_id];
}

//Return all intersection ids for two intersecting streets
//This function will typically return one intersection id.
//However street names are not guarenteed to be unique, so more than 1 intersection id may exist
std::vector<unsigned> find_intersection_ids_from_street_names(std::string street_name1, std::string street_name2){

    std::vector<unsigned> intersection_ids;
    
    //Tries to access the vectors of IDs of each street name, if not found - return empty vector
    try {
        intersection_ids_from_street_name.at(street_name1);
        intersection_ids_from_street_name.at(street_name2);
    } catch (std::out_of_range) {
        return intersection_ids;
    }
    
    //Store and sort vectors of IDs
    std::vector<unsigned> i_id1 = intersection_ids_from_street_name.at(street_name1);
    std::vector<unsigned> i_id2 = intersection_ids_from_street_name.at(street_name2);
    std::sort(i_id1.begin(), i_id1.end());
    std::sort(i_id2.begin(), i_id2.end());
    //Fill intersection_ids with the common elements of both
    std::set_intersection(i_id1.begin(), i_id1.end(), i_id2.begin(), i_id2.end(), std::back_inserter(intersection_ids));
    return intersection_ids;
 
}

//Returns the distance between two coordinates in meters
double find_distance_between_two_points(LatLon point1, LatLon point2){
    //Find average latitude point as (lat1+lat2)/2
    double latavg = (point1.lat() + point2.lat())*0.5;
    
    //Find x and y coordinates for first LatLon point
    double x1 = (point1.lon()*DEG_TO_RAD)*(cos(DEG_TO_RAD*latavg));
    double y1 = DEG_TO_RAD*point1.lat();
    
    //Find x and y coordinates for second LatLon point
    double x2 = (point2.lon()*DEG_TO_RAD)*(cos(DEG_TO_RAD*latavg));
    double y2 = DEG_TO_RAD*point2.lat();
    
    //Find Pythagorean distance and return value
    double distance = EARTH_RADIUS_IN_METERS*sqrt(pow(y2-y1,2) + pow(x2-x1,2));
    return distance;
}

//Returns the length of the given street segment in meters
double find_street_segment_length(unsigned street_segment_id){
    return street_segment_lengths[street_segment_id];
}

//Returns the length of the specified street in meters
double find_street_length(unsigned street_id){
    return street_lengths[street_id];
}

//Returns the travel time to drive a street segment in seconds (time = distance/speed_limit)
double find_street_segment_travel_time(unsigned street_segment_id){
    return street_segment_time[street_segment_id];
}

//Returns the nearest point of interest to the given position
unsigned find_closest_point_of_interest(LatLon my_position){
    
    //A vector to store our closest points
    std::vector<interPoint> near;
    interPoint pt;
    pt.x = my_position.lat();
    pt.y = my_position.lon();
    
    //We find the closest 400 points in the R-Tree 
    treePOI.query(bgi::nearest<interPoint>(pt, 400), std::back_inserter(near));
    
    //However, these distances are Cartesian so they may not be accurate
    //We perform our own search for the minimum distance in this smaller collection of points
    
    unsigned closestID = 0;
    double closest_distance = find_distance_between_two_points(my_position, getPointOfInterestPosition(0));
    for (unsigned i=0; i < near.size(); i++){
        double current_distance = find_distance_between_two_points(my_position, getPointOfInterestPosition(near[i].id));
        if (current_distance < closest_distance){
            closest_distance = current_distance;
            closestID = near[i].id;
        }
    }
    
    return closestID;  
}

//Returns the the nearest intersection to the given position
unsigned find_closest_intersection(LatLon my_position){

    //A vector to store our closest intersections
    std::vector<interPoint> near;
    interPoint pt;
    pt.x = my_position.lat();
    pt.y = my_position.lon();
    
    //We find the closest 1000 intersections in the R-Tree 
    treeInt.query(bgi::nearest<interPoint>(pt, 1000), std::back_inserter(near));
    
    //However, these distances are Cartesian so they may not be accurate
    //We perform our own search for the minimum distance in this smaller collection of points
    
    unsigned closestID = 0;
    double closest_distance = find_distance_between_two_points(my_position, intersection_positions[0]);
    for (unsigned i=0; i < near.size(); i++){
        double current_distance = find_distance_between_two_points(my_position, intersection_positions[near[i].id]);
        if (current_distance < closest_distance){
            closest_distance = current_distance;
            closestID = near[i].id;
        }
    }
    
    return closestID;
}

//Reads all street segment data from the map and inserts into related data structures
void street_segment_processing(){
    for (unsigned i = 0; i < numberOfSegments; ++i) {
        
        StreetSegmentInfo const ssinfo = getStreetSegmentInfo(i);
        street_segment_info[i] = ssinfo;
        
        double length = 0;

        LatLon lfrom = getIntersectionPosition(street_segment_info[i].from);
        LatLon lto = getIntersectionPosition(street_segment_info[i].to);
        LatLon temp_to;
        LatLon temp_from = lfrom;

        unsigned curvePoints = street_segment_info[i].curvePointCount;
        
        street_segment_positions[i].push_back(lfrom);
        
        double max_length = 0;
        unsigned index_max = 0;
        
        if (curvePoints == 0){ 
            length = find_distance_between_two_points(lfrom, lto);
        }
        else {
            for (unsigned j = 0; j < curvePoints; j++) {
                temp_to = getStreetSegmentCurvePoint(i, j);
                double distance = find_distance_between_two_points(temp_from, temp_to);
                if (distance > max_length) {
                    max_length = distance;
                    index_max = j;
                }
                length += find_distance_between_two_points(temp_from, temp_to);
                temp_from = temp_to;
                
                street_segment_positions[i].push_back(temp_to);
            }
            double distance = find_distance_between_two_points(temp_from, lto);
            if (distance > max_length) {
                max_length = distance;
                index_max = curvePoints - 1;
            }
            length += find_distance_between_two_points(temp_from, lto);
        }
        
        street_segment_max_curve.push_back(index_max);
        
        street_segment_lengths[i] = length;
        street_segment_positions[i].push_back(lto);
        
        //Speed limit of street segment in metres per second
        float speed_limit = (street_segment_info[i].speedLimit) *
                    KM_PER_HR_TO_METRES_PER_SECOND_FACTOR;

        //Find and return travel time of street segment
        street_segment_time[i] = street_segment_lengths[i] / speed_limit;
    
        unsigned street = street_segment_info[i].streetID;
        street_street_segments[street].push_back(i);    
    }
}

//Reads all street, intersection, POI and feature data from the map and inserts into related data structures
void combined_processing(){

    //Find the largest value
    unsigned max_quantity = 0;
    if (numberOfIntersections > numberOfPOI)
        max_quantity = numberOfIntersections;
    else
        max_quantity = numberOfPOI;
    if (numberOfFeatures > max_quantity)
        max_quantity = numberOfFeatures;
    if (numberOfStreets > max_quantity)
        max_quantity = numberOfStreets;
    
    //Iterate to the largest value
    for (unsigned i = 0; i < max_quantity; i++){
        
        //Intersection processing
        if (i < numberOfIntersections){
            //STREET SEGMENTS 

            //Resize inner vector containing street names and street segment ids
            intersection_street_names[i].resize(getIntersectionStreetSegmentCount(i));
            intersection_street_segments[i].resize(getIntersectionStreetSegmentCount(i));
            intersection_adjacent[i].reserve(getIntersectionStreetSegmentCount(i));
            intersection_adjacent_dup[i].reserve(getIntersectionStreetSegmentCount(i));
            valid_street_segments_to_intersection[i].reserve(getIntersectionStreetSegmentCount(i));

            //Iterate through street segments incident to each intersection to get each name
            for(unsigned j = 0; j < getIntersectionStreetSegmentCount(i); ++j){
                auto const ss_name = getStreetName(street_segment_info[getIntersectionStreetSegment(i, j)].streetID);
                intersection_street_names[i][j] = ss_name;
                auto const ss_id = getIntersectionStreetSegment(i, j);
                intersection_street_segments[i][j] = ss_id;

                unsigned destID = 0;
                bool valid = true;

                //If this is the origin, the intersection at the endpoint is a valid destination
                if (i == street_segment_info[intersection_street_segments[i][j]].from)
                    destID = street_segment_info[intersection_street_segments[i][j]].to;

                //If this is the endpoint, this intersection at the origin is the destination
                //But is only valid if this is not a one-way street
                if (i == street_segment_info[intersection_street_segments[i][j]].to){
                    destID = street_segment_info[intersection_street_segments[i][j]].from;
                    valid = !street_segment_info[intersection_street_segments[i][j]].oneWay;
                }

                //We make one last check to ensure we have not already added this intersection and add if not
                if(std::find(intersection_adjacent[i].begin(), intersection_adjacent[i].end(), destID) ==
                       intersection_adjacent[i].end() && valid){
                    intersection_adjacent[i].emplace_back(destID);
                }
                if (valid){
                    valid_street_segments_to_intersection[i].emplace_back(ss_id);
                    intersection_adjacent_dup[i].emplace_back(destID);
                }

            }
            
            //Add intersection positions
            LatLon const i_position = getIntersectionPosition(i);
            intersection_positions[i] = i_position;

            //Finding the maximum and minimum latitude/longitude of the intersections
            if (i_position.lon() < minLon)
                minLon = i_position.lon();
            if (i_position.lon() > maxLon)
                maxLon = i_position.lon();

            if (i_position.lat() < minLat)
                minLat = i_position.lat();
            if (i_position.lat() > maxLat)
                maxLat = i_position.lat();

            //Add the intersection to our R-Tree
            
            interPoint ip;

            ip.id = i;
            ip.x = i_position.lat();
            ip.y = i_position.lon();

            treeInt.insert(ip);

            //Add intersection names
            std::string const i_name = getIntersectionName(i);
            intersection_names[i] = i_name;
            
        }
        
        //POI processing
        if (i < numberOfPOI){
            
            //POI position insertion
            LatLon const position = getPointOfInterestPosition(i);
            POI_positions[i] = position;

            //Add the point of interest to our R-Tree
            interPoint ip;

            ip.id = i;
            ip.x = position.lat();
            ip.y = position.lon();

            treePOI.insert(ip);

            //POI type insertion
            std::string const type = getPointOfInterestType(i);
            POI_types[i] = type;

            //POI name insertion
            std::string const name = getPointOfInterestName(i);
            POI_names[i] = name;

            // Name exists already
            try {
                poi_id_from_name.at(name).push_back(i);
            }
            catch (std::out_of_range) { //Name does not exist
                std::vector<unsigned> p_ids;
                p_ids.push_back(i);
                poi_id_from_name.insert({name, p_ids});
            }
        }
        
        //Feature processing
        if (i < numberOfFeatures){

            //Add feature names to vector
            std::string const name = getFeatureName(i);
            feature_names[i] = name;

            //Add feature types to vector
            FeatureType const type = getFeatureType(i);
            feature_types[i] = type;

            //Determine if feature is closed
            bool isClosed = false;
            LatLon start = getFeaturePoint(i, 0);
            LatLon end = getFeaturePoint(i, getFeaturePointCount(i)-1);
            if (start.lat() == end.lat() && start.lon() == end.lon())
                isClosed = true;        

            feature_positions[i].resize(getFeaturePointCount(i));
            //Temporary variables for use of the shoelace method in calculating area
            LatLon firstPos;
            LatLon nextPos;
            float areaTemp = 0;

            //Add LatLons for all points of a feature
            for(unsigned count = 0; count < getFeaturePointCount(i); count++){
                LatLon const position = getFeaturePoint(i, count);
                feature_positions[i][count] = position;

                //For closed features (with area) - Shoelace Method for area
                if (isClosed) {
                    //The first point should be saved
                    if (count == 0)
                        firstPos = getFeaturePoint(i, count);
                    //This is because the last point's "next" point is the first
                    if (count == getFeaturePointCount(i) - 1)
                        nextPos = firstPos;
                    else
                        nextPos = getFeaturePoint(i, count + 1);
                    //Perform a "shoelace" calculation between this and next point
                    areaTemp += (position.lon() * DEG_TO_RAD * cos(AvgLat*DEG_TO_RAD)) *
                            (nextPos.lat() * DEG_TO_RAD);
                    areaTemp -= (nextPos.lon() * DEG_TO_RAD * cos(AvgLat*DEG_TO_RAD)) *
                            (position.lat() * DEG_TO_RAD);
                }
            }

            //Calculate area of polygon and insert into multimap
            if(isClosed) {
                float area = fabs(areaTemp*0.5);
                feature_ids_by_areas.emplace(area, i);
            }
            else
                feature_ids_by_areas.emplace(0, i);
        }
        
        //Street processing
        if (i < numberOfStreets){
            double total_length = 0;
            std::set<unsigned> street_intersections;
            //We go through all the street segments and add the from and to intersection IDs without repeating
            for (unsigned j = 0; j < street_street_segments[i].size(); j++){
                    street_intersections.emplace(street_segment_info[street_street_segments[i][j]].from);
                    street_intersections.emplace(street_segment_info[street_street_segments[i][j]].to);

                // calculate length of street
                total_length += find_street_segment_length(street_street_segments[i][j]);
            }
            std::vector<unsigned> street_intersections_vec(street_intersections.begin(), street_intersections.end());
            street_street_intersections[i] = street_intersections_vec;
            street_lengths[i] = total_length; // add length of street to vector


            std::string name = getStreetName(i);

            //Street names insertion
            street_names[i] = name;

            //Insert new name if doesn't exist already
            try {
                street_id_from_name.at(name).push_back(i);
            }
            catch (std::out_of_range) {
                std::vector<unsigned> s_ids;
                s_ids.push_back(i);
                street_id_from_name.insert({name, s_ids});    
            }

            //Intersection ids from street name insertion into map
            std::vector<unsigned> i_ids = street_street_intersections[i];
            try {
                std::vector<unsigned> union_v;
                std::set_union(i_ids.begin(), i_ids.end(), intersection_ids_from_street_name.at(name).begin(),
                    intersection_ids_from_street_name.at(name).end(), std::inserter(union_v, union_v.begin()));
                intersection_ids_from_street_name.at(name).swap(union_v);
            }
            catch (std::out_of_range) {
                intersection_ids_from_street_name.insert({name, i_ids});    
            }            
        }
        
    }
}