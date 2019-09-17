#include "m1.h"
#include <unordered_map>
#include "m3.h"
#include "StreetsDatabaseAPI.h"
#include <queue>
#include <limits>
#include <algorithm>
#include <iostream>

extern unsigned numberOfIntersections;
extern std::vector<StreetSegmentInfo> street_segment_info;
extern std::unordered_map<std::string, std::vector<unsigned>> poi_id_from_name;
extern std::vector<double> street_segment_time;
extern std::vector<std::vector<unsigned>> intersection_adjacent_dup;
extern std::vector<std::vector<unsigned>> valid_street_segments_to_intersection;

// function to compute shortest path using Dijkstra's algorithm
std::vector<unsigned> pathfinder(const unsigned intersect_id_start, 
                                 const unsigned intersect_id_end,
                                 const double turn_penalty);

std::vector<unsigned> pathfinder_POI(const unsigned intersect_id_start, 
                                 const double turn_penalty,
                                 std::vector<unsigned> intersections);

// Returns the time required to travel along the path specified, in seconds. 
// The path is given as a vector of street segment ids, and this function 
// can assume the vector either forms a legal path or has size == 0.
// The travel time is the sum of the length/speed-limit of each street 
// segment, plus the given turn_penalty (in seconds) per turn implied by the path. 
// A turn occurs when two consecutive street segments have different street IDs.
double compute_path_travel_time(const std::vector<unsigned>& path, 
                                const double turn_penalty){
    double time_tot = 0; // travel time, initially zero
    
    // if path is empty, return travel time (initially zero)
    if(path.size() == 0) //empty vector
        return time_tot;
    
    // iterate through path to compute travel time
    for(unsigned id = 0; id < path.size() - 1; id++){
        // add travel time of individual street segment to total travel time
        time_tot += street_segment_time[path[id]];
        
        // check if current street segment and next street segment are the same
        // street, if not, apply turn penalty
        if(street_segment_info[path[id]].streetID != street_segment_info[path[id + 1]].streetID)
            time_tot += turn_penalty;
    }
    
    // add final travel time for final street segment
    time_tot += street_segment_time[path[path.size() - 1]];
    
    return time_tot;
}


// Returns a path (route) between the start intersection and the end 
// intersection, if one exists. This routine should return the shortest path
// between the given intersections when the time penalty to turn (change
// street IDs) is given by turn_penalty (in seconds).
// If no path exists, this routine returns an empty (size == 0) vector. 
// If more than one path exists, the path with the shortest travel time is 
// returned. The path is returned as a vector of street segment ids; traversing 
// these street segments, in the returned order, would take one from the start 
// to the end intersection.
std::vector<unsigned> find_path_between_intersections(const unsigned intersect_id_start, 
                                                      const unsigned intersect_id_end,
                                                      const double turn_penalty){
    return pathfinder(intersect_id_start, intersect_id_end, turn_penalty);
}


// Returns the shortest travel time path (vector of street segments) from 
// the start intersection to a point of interest with the specified name.
// The path will begin at the specified intersection, and end on the 
// intersection that is closest (in Euclidean distance) to the point of 
// interest.
// If no such path exists, returns an empty (size == 0) vector.
std::vector<unsigned> find_path_to_point_of_interest(const unsigned intersect_id_start, 
                                               const std::string point_of_interest_name,
                                               const double turn_penalty){
   
    //path vector 
    std::vector<unsigned> path;
 
    //A vector to hold the ID's of all the points of interest with the specified name
    std::vector<unsigned> poi_ids;
    
    std::vector<unsigned> int_ids;
    
    //Grab the list of IDS
    try {
        poi_ids = poi_id_from_name.at(point_of_interest_name);
    }
    //If not found, return an empty vector (can't reach this place as it doesn't exist)
    catch (std::out_of_range) {
        return {};
    }
    
    //Now iterate through the POI
    for (unsigned i = 0; i < poi_ids.size(); i++){
  
        //Find the closest intersection to this POI
        unsigned closest_intersection = find_closest_intersection(getPointOfInterestPosition(poi_ids[i]));
              
        //If there is a POI right where we are, return an empty vector (we're here!)
        if (closest_intersection == intersect_id_start)
            return {};
        
            int_ids.push_back(closest_intersection);
    }
    
    path = pathfinder_POI(intersect_id_start, turn_penalty, int_ids);
    
    //Return the closest path
    return path;
}



std::vector<unsigned> pathfinder(const unsigned intersect_id_start, 
                                 const unsigned intersect_id_end,
                                 const double turn_penalty){
    // path vector
    std::vector<unsigned> path;
    
    // path containing street segment ids that lead to an intersection
    std::vector<unsigned> street_segment_came_from(numberOfIntersections, std::numeric_limits<unsigned>::max());
    
    // path containing intersection ids that lead to an adjacent intersection
    std::vector<unsigned> intersection_came_from(numberOfIntersections, std::numeric_limits<unsigned>::max());
    
    // priority queue containing potential intersection with minimal travel time
    std::priority_queue<std::pair<double, unsigned>, std::vector<std::pair<double, unsigned>>, std::greater<std::pair<double, unsigned>>> frontier;
    
    // open set containing list of intersections and their travel times
    // initially set to infinity, this means intersection is not yet examined or
    // not yet in open set
    std::vector<double> open_set(numberOfIntersections, std::numeric_limits<double>::infinity());
    
    // add start intersection to priority queue and open set
    // initial travel time is zero
    frontier.emplace(std::make_pair(0, intersect_id_start));
    open_set[intersect_id_start] = 0;
        
    // iterate until priority queue is empty
    while(!frontier.empty()){
        // get intersection from top of priority queue
        unsigned current_intersect = frontier.top().second;
        
        // remove top intersection off priority queue
        frontier.pop();
                
        // street segment id that lead to current street segment
        unsigned prev_ss = street_segment_came_from[current_intersect];
        
        if(current_intersect == intersect_id_end)
            break;
        
        // iterate through adjacent intersections to current intersection
        for(unsigned adj_id = 0; adj_id < intersection_adjacent_dup[current_intersect].size(); adj_id++){
            // get adjacent intersection id
            unsigned adjacent_intersect = intersection_adjacent_dup[current_intersect][adj_id];
            
            // get current street segment connecting current intersection to adjacent
            unsigned connect_ss = valid_street_segments_to_intersection[current_intersect][adj_id];
            
            // calculate new travel time to get to adjacent intersection
            double time = 0;
            if(prev_ss == std::numeric_limits<unsigned>::max() || street_segment_info[prev_ss].streetID == street_segment_info[connect_ss].streetID)
                time = open_set[current_intersect] + street_segment_time[connect_ss];
            else
                time = open_set[current_intersect] + street_segment_time[connect_ss] + turn_penalty;
            
            // check if adjacent intersection is in open set, if not update travel time for
            // adjacent intersection and add to priority queue
            if(open_set[adjacent_intersect] == std::numeric_limits<double>::infinity()){
                frontier.emplace(std::make_pair(time, adjacent_intersect));
                open_set[adjacent_intersect] = time;
                street_segment_came_from[adjacent_intersect] = connect_ss;
                intersection_came_from[adjacent_intersect] = current_intersect;
            }
            
            // intersection is in open set, check if current time bests the time recorded in open set
            // if true, update open set
            if(time < open_set[adjacent_intersect]){
                frontier.emplace(std::make_pair(time, adjacent_intersect));
                open_set[adjacent_intersect] = time;
                street_segment_came_from[adjacent_intersect] = connect_ss;
                intersection_came_from[adjacent_intersect] = current_intersect;
            }
        }
    }
    
    // reconstruct path by starting from destination and retracing street_segment_came_from
    unsigned i_id = intersect_id_end;
    unsigned ss_id = street_segment_came_from[i_id];
    while(ss_id != std::numeric_limits<unsigned>::max()){
        path.insert(path.begin(), ss_id);
        i_id = intersection_came_from[i_id];
        ss_id = street_segment_came_from[i_id];  
    }    
    
    return path;
}

std::vector<unsigned> pathfinder_POI(const unsigned intersect_id_start, 
                                 const double turn_penalty,
                                 std::vector<unsigned> intersections){
    // path vector
    std::vector<unsigned> path;
    
    // current intersection id to examine in the algorithm
    unsigned current_intersect;
    
    // path containing street segment ids that lead to an intersection
    std::vector<unsigned> street_segment_came_from(numberOfIntersections, std::numeric_limits<unsigned>::max());
    
    // path containing intersection ids that lead to an adjacent intersection
    std::vector<unsigned> intersection_came_from(numberOfIntersections, std::numeric_limits<unsigned>::max());
    
    // priority queue containing potential intersection with minimal travel time
    std::priority_queue<std::pair<double, unsigned>, std::vector<std::pair<double, unsigned>>, std::greater<std::pair<double, unsigned>>> frontier;
    
    // open set containing list of intersections and their travel times
    // initially set to infinity, this means intersection is not yet examined or
    // not yet in open set
    std::vector<double> open_set(numberOfIntersections, std::numeric_limits<double>::infinity());
    
    // closed set containing list of intersections already examined
    // intersection is examined if its id matches corresponding index, otherwise index
    // is set to infinity
    std::vector<unsigned> closed_set(numberOfIntersections, std::numeric_limits<unsigned>::max());
    
    // add start intersection to priority queue and open set
    // initial travel time is zero
    frontier.emplace(std::make_pair(0, intersect_id_start));
    open_set[intersect_id_start] = 0;
        
    // iterate until priority queue is empty
    while(!frontier.empty()){
        // get intersection from top of priority queue
        current_intersect = frontier.top().second;
        
        // remove top intersection off priority queue
        frontier.pop();
        
        // add current intersection to closed set
        closed_set.emplace_back(current_intersect);
            
        // street segment id that lead to current street segment
        unsigned prev_ss = street_segment_came_from[current_intersect];
        
        // if we find the first closest intersection to the POI, we break and return the path
        if(std::find(intersections.begin(), intersections.end(), current_intersect) != intersections.end())
            break;
        
        // iterate through adjacent intersections to current intersection
        for(unsigned adj_id = 0; adj_id < intersection_adjacent_dup[current_intersect].size(); adj_id++){
            // get adjacent intersection id
            unsigned adjacent_intersect = intersection_adjacent_dup[current_intersect][adj_id];
            
            // get current street segment connecting current intersection to adjacent
            unsigned connect_ss = valid_street_segments_to_intersection[current_intersect][adj_id];
            
            // calculate new travel time to get to adjacent intersection
            double time = 0;
            if(prev_ss == std::numeric_limits<unsigned>::max() || street_segment_info[prev_ss].streetID == street_segment_info[connect_ss].streetID)
                time = open_set[current_intersect] + street_segment_time[connect_ss];
            else
                time = open_set[current_intersect] + street_segment_time[connect_ss] + turn_penalty;
            
            // check if adjacent intersection is in closed set, if true, continue to next adjacent intersection
            if(closed_set[adjacent_intersect] == adjacent_intersect)
                continue;
            
            // check if adjacent intersection is in open set, if not update travel time for
            // adjacent intersection and add to priority queue
            if(open_set[adjacent_intersect] == std::numeric_limits<double>::infinity()){
                frontier.emplace(std::make_pair(time, adjacent_intersect));
                open_set[adjacent_intersect] = time;
                street_segment_came_from[adjacent_intersect] = connect_ss;
                intersection_came_from[adjacent_intersect] = current_intersect;
            }
            
            if(time >= open_set[adjacent_intersect])
                continue;
            
            // intersection is in open set, check if current time bests the time recorded in open set
            // if true, update open set
            if(time < open_set[adjacent_intersect]){
                frontier.emplace(std::make_pair(time, adjacent_intersect));
                open_set[adjacent_intersect] = time;
                street_segment_came_from[adjacent_intersect] = connect_ss;
                intersection_came_from[adjacent_intersect] = current_intersect;
            }
        }
    }
    
    
    
    // reconstruct path by starting from destination and retracing street_segment_came_from
    unsigned i_id = current_intersect;
    unsigned ss_id = street_segment_came_from[i_id];
    while(ss_id != std::numeric_limits<unsigned>::max()){
        path.insert(path.begin(), ss_id);
        i_id = intersection_came_from[i_id];
        ss_id = street_segment_came_from[i_id];  
    }    
    
    return path;
}
