#include <chrono>
#include <thread>
#include <cstdlib>
#include <vector>
#include <string>
#include "string.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include <cmath>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <unordered_map>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <iostream>
#include <fstream>
#include "graphics.h"
#include "easygl_constants.h"
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "featureTypes.h"
#include <map>
#include <fstream>

#define BUTTON_NUM 4
#define ON true
#define OFF false

#define RAD_TO_DEG 57.2957795131
#define SCOPE 10000000

#define TURN_PENALTY 0

using namespace std;

extern string cityName;
extern bool map_loaded;


/*********************DATA STRUCTURES FROM M1**************************/

extern vector<vector<LatLon>> street_segment_positions;

extern vector<string> feature_names;

extern vector<FeatureType> feature_types;
extern vector<vector<LatLon>> feature_positions;
extern std::vector<StreetSegmentInfo> street_segment_info;
extern std::vector<double> street_segment_lengths;
extern std::vector<std::string> street_names;
extern std::vector<std::string> intersection_names;
extern std::multimap<float, unsigned, std::greater<float>> feature_ids_by_areas;
extern std::vector<LatLon> intersection_positions;

extern std::vector<unsigned> street_segment_max_curve;
extern std::unordered_map<std::string, std::vector<unsigned>> poi_id_from_name;

/**********************************************************************/
extern double minLon;
extern double maxLon;
extern double minLat;
extern double maxLat;
extern double AvgLat;


// Max area in world coordinates
t_bound_box max_world_bound;


//Map with OSMID's as key and pointers to OSMNodes as values
std::unordered_map<OSMID, const OSMNode*> osm_nodes;

//Map with OSMID's as key and pointers to OSMWays as values
std::unordered_map<OSMID, const OSMWay*> osm_ways;

//Functions for gathering OSM data and clearing it
void osm_processing();
void osm_cleanup();

//Members for drawing intersections
bool in_find_state = false;
std::vector<unsigned> list_intersections_onscreen;
void act_on_find_button(void (*drawscreen_ptr) (void));
void draw_intersections();

bool firstRun = true;
//A variable to check if we've clicked the marker once
bool firstDistance = false; 


//Members for drawing POIs
void act_on_poi_button(void (*drawscreen_ptr) (void));
bool in_poi_state = false;
std::vector<unsigned> list_poi_onscreen;
void draw_poi();

//Members for drawing highlighted (found) streets
void act_on_street_button(void (*drawscreen_ptr) (void));
bool in_street_state = false;
string highlighted_street = "";

//Members for filtering POIs
void act_on_filter_button(void (*drawscreen_ptr) (void));
bool in_filter_state = false;
std::vector<string> show_poi_types = {"restaurant", "education", "healthcare",
            "busstop", "miscellaneous", "worship", "post"};
std::vector<string> available_types;

//Data structure storing the type of each street segment
std::vector<string> street_segment_roadtypes;

//Data structure storing the type of each POI
std::vector<string> poi_types;

//Processing functions for both types
void amenity_road_processing();

//Icons for each amenity
Surface icon_restaurant;
Surface icon_education;
Surface icon_healthcare;
Surface icon_busstop;
Surface icon_post;
Surface icon_worship;
Surface icon_miscellaneous;

//Icons for help button
std::string help = "";
Surface icon_help;
void draw_help();
void print_help();
t_bound_box help_box(10,300,50,340);

//Members for onscreen textbox
bool textbox_onscreen = true;
Surface icon_textbox;
Surface icon_searchtag;
Surface icon_directionstag;
std::string textbox_text_1 = "";
std::string textbox_text_2 = "";
std::string textbox_text_3 = "";
t_bound_box textbox(10,450,310,520);
t_bound_box closebox(310, 450 ,360, 520);
void display_textbox();

//Icons for filtering POI
Surface icon_filterpoibutton;
void draw_filterpoi();
void draw_filterpoi_menu();
void print_filterpoi();
t_bound_box filterpoi_box(10,341,50,380);

bool filter_menu_on = false;
Surface icon_filtermenupoi;
t_bound_box filtermenu_box(60,341,106,510);
t_bound_box filtermenu_confirm_box(64,484,102,507);
bool filter_restaurant = false;
Surface icon_filter_restaurant;
Surface icon_filter_restaurant_selected;
t_bound_box restaurant_box(74,354,92,371);
bool filter_education = false;
Surface icon_filter_education;
Surface icon_filter_education_selected;
t_bound_box education_box(74,444,92,461);
bool filter_healthcare = false;
Surface icon_filter_healthcare;
Surface icon_filter_healthcare_selected;
t_bound_box healthcare_box(74,390,92,407);
bool filter_busstop = false;
Surface icon_filter_busstop;
Surface icon_filter_busstop_selected;
t_bound_box busstop_box(74,372,92,389);
bool filter_post = false;
Surface icon_filter_post;
Surface icon_filter_post_selected;
t_bound_box post_box(74,408,92,425);
bool filter_worship = false;
Surface icon_filter_worship;
Surface icon_filter_worship_selected;
t_bound_box worship_box(74,426,92,443);
bool filter_miscellaneous = false;
Surface icon_filter_miscellaneous;
Surface icon_filter_miscellaneous_selected;
t_bound_box miscellaneous_box(74,462,92,480);

//Icons for search bar
Surface icon_searchbar;
Surface icon_searchbar_selected;
Surface icon_searchselectdirections;
Surface icon_searchselectsearch;

t_bound_box modemenu_directions_box(370,10,405,35);
t_bound_box modemenu_search_box(406,10,440,35);

void find_intersections();
void find_poi();
void find_streets();

Surface icon_searchmenu;
Surface icon_searchmenuint;
Surface icon_searchmenupoi;
Surface icon_searchmenustreet;

void path_int();
void path_poi();

void draw_path();
void print_path();

Surface icon_directionmenu;
Surface icon_directionmenuint;
Surface icon_directionmenupoi;

//Search settings
bool in_searching_mode = true;
bool search_street = false;
bool search_poi = false;
bool search_int = false;
t_bound_box findmenu_street_box(10,71,50,120);
t_bound_box findmenu_poi_box(10,121,50,160);
t_bound_box findmenu_int_box(10,161,50,200);

//Direction settings
bool in_directions_mode = false;
std::vector<unsigned> direction_path;
bool point1_inputted = false;
bool direction_poi = false;
bool direction_int = false;
bool in_path_state = false;
std::string point_dir1 = "";
std::string point_dir2 = "";
t_bound_box directionmenu_poi_box(10,120,50,160);
t_bound_box directionmenu_int_box(10,161,50,200);

//Functions for the search bar
void draw_searchbar();
void draw_text_overlay();
t_bound_box searchbar_box(10,10,355,45);
t_bound_box no_overlap(0,0,380,65);
t_bound_box searchbutton_box(370,45,410,10);
bool searchbar_selected = false;
bool input_complete = false;
std::string searchbox_input = "";

//Functions for find menu
void draw_findmenu();


static void drawscreen();
void act_on_button_press(float x, float y, t_event_buttonPressed event);
void act_on_mouse_move(float x, float y);
void act_on_key_press(char c, int keysym);

//static void draw_all_street_segmentsB();
static void draw_all_street_segments();
static void draw_all_features();
static void draw_all_POI();
void drawPosition (LatLon location, string type);
static void draw_all_marks();
double find_distance_closest_point_of_interest(unsigned ID);

t_point LatLon_to_XY (LatLon location);
LatLon XY_to_LatLon (t_point coords);
void drawFeatureLine(unsigned id, const t_color& color, int linestyle, int linewidth);
void drawFeatureAreaB(unsigned id, const t_color& color, int linestyle);
void drawFeatureArea(unsigned id, const t_color& color, int linestyle);
bool isFeatureClosed(unsigned id);
void drawSegment (LatLon from, LatLon to);

double getZoomLevel();

/*************************HELPER FUNCTIONS FOR TEXT****************************/
double angle_street_name(float x1, float y1, float x2, float y2);
t_point midpoint(LatLon l1, LatLon l2);
void draw_street_name(float x1, float y1, float x2, float y2, unsigned id, unsigned point);

/******************************************************************************/

/*******************************DRAWING MARKERS********************************/
bool marker_onscreen = false;
t_point marker_location;
t_point past_location;
unsigned current_id = 0;
unsigned past_id = 0;
void draw_marker();
void calculate_distance();
/******************************************************************************/

//Global Variables for screen size (function: set_visible_world)
double latAvg;
double lonAvg;

const t_bound_box initial_coords = t_bound_box(0, 0, 1100, 1150);
t_bound_box screen_bound;

//Our actual window
void draw_map() {
    //Initialize background
    init_graphics(cityName, t_color(228, 230, 229));
    set_drawing_buffer(OFF_SCREEN);
    set_visible_world(minLon * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat) * SCOPE,
                      minLat * DEG_TO_RAD * SCOPE,
                      maxLon * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat) * SCOPE,
                      maxLat * DEG_TO_RAD * SCOPE);
    max_world_bound = get_visible_world();
    create_all_buttons();
    wait_for_load_map();
    
    //Process OSM data
    osm_processing();
    amenity_road_processing();
    
    //Read from the help file
    ifstream helpfile;
    helpfile.open("libstreetmap/resources/helpfile");
    std::string help_line;
    if (helpfile.is_open()){
        while (getline(helpfile, help_line)){
            help = help + help_line + "\n";
        }
        helpfile.close();
    }
    
    //Load all icons
    icon_restaurant = load_png_from_file("libstreetmap/resources/icon_restaurant.png");
    icon_education = load_png_from_file("libstreetmap/resources/icon_education.png");
    icon_healthcare = load_png_from_file("libstreetmap/resources/icon_healthcare.png");
    icon_busstop = load_png_from_file("libstreetmap/resources/icon_busstop.png");
    icon_post = load_png_from_file("libstreetmap/resources/icon_post.png");
    icon_worship = load_png_from_file("libstreetmap/resources/icon_worship.png");
    icon_miscellaneous = load_png_from_file("libstreetmap/resources/icon_miscellaneous.png");   
    
    icon_filtermenupoi = load_png_from_file("libstreetmap/resources/filterpoimenu.png");
    icon_filter_restaurant = load_png_from_file("libstreetmap/resources/select_restaurant.png");
    icon_filter_restaurant_selected = load_png_from_file("libstreetmap/resources/select_restaurant_selected.png");
    icon_filter_education = load_png_from_file("libstreetmap/resources/select_education.png");
    icon_filter_education_selected = load_png_from_file("libstreetmap/resources/select_education_selected.png");
    icon_filter_healthcare = load_png_from_file("libstreetmap/resources/select_healthcare.png");
    icon_filter_healthcare_selected = load_png_from_file("libstreetmap/resources/select_healthcare_selected.png");
    icon_filter_busstop = load_png_from_file("libstreetmap/resources/select_busstop.png");
    icon_filter_busstop_selected = load_png_from_file("libstreetmap/resources/select_busstop_selected.png");
    icon_filter_post = load_png_from_file("libstreetmap/resources/select_post.png");
    icon_filter_post_selected = load_png_from_file("libstreetmap/resources/select_post_selected.png");
    icon_filter_worship = load_png_from_file("libstreetmap/resources/select_worship.png");
    icon_filter_worship_selected = load_png_from_file("libstreetmap/resources/select_worship_selected.png");
    icon_filter_miscellaneous = load_png_from_file("libstreetmap/resources/select_miscellaneous.png");
    icon_filter_miscellaneous_selected = load_png_from_file("libstreetmap/resources/select_miscellaneous_selected.png");
    
    icon_help = load_png_from_file("libstreetmap/resources/help.png"); 
    
    icon_filterpoibutton = load_png_from_file("libstreetmap/resources/filterpoibutton.png");
    
    icon_textbox = load_png_from_file("libstreetmap/resources/text_area.png");
    icon_searchtag = load_png_from_file("libstreetmap/resources/search_tag.png");
    icon_directionstag = load_png_from_file("libstreetmap/resources/directions_tag.png");
    
    icon_searchbar = load_png_from_file("libstreetmap/resources/searchbar.png"); 
    icon_searchbar_selected = load_png_from_file("libstreetmap/resources/searchbarselected.png");
    icon_searchselectsearch = load_png_from_file("libstreetmap/resources/searchselectsearch.png");     
    icon_searchselectdirections = load_png_from_file("libstreetmap/resources/searchselectdirections.png");     
    
    icon_searchmenu = load_png_from_file("libstreetmap/resources/searchmenu.png");
    icon_searchmenuint = load_png_from_file("libstreetmap/resources/searchmenuint.png");
    icon_searchmenupoi = load_png_from_file("libstreetmap/resources/searchmenupoi.png");
    icon_searchmenustreet = load_png_from_file("libstreetmap/resources/searchmenustreet.png");
    
    icon_directionmenu = load_png_from_file("libstreetmap/resources/directionmenu.png");
    icon_directionmenuint = load_png_from_file("libstreetmap/resources/directionmenuint.png");
    icon_directionmenupoi = load_png_from_file("libstreetmap/resources/directionmenupoi.png");    
    
    set_keypress_input(true);
    set_mouse_move_input(true);
    set_drawing_buffer(ON_SCREEN);
    event_loop(act_on_button_press, act_on_mouse_move, act_on_key_press, drawscreen);
    
    //Clean up after loop is over
    close_graphics();
    closeOSMDatabase();    
    osm_cleanup();
}


void drawscreen(void){
    set_drawing_buffer(OFF_SCREEN);
    screen_bound = get_visible_world();
    clearscreen();
    draw_all_features();
    //draw_all_street_segmentsB();
    draw_all_street_segments();
    draw_all_POI();
    draw_intersections();
    draw_path();
    draw_all_marks();
    draw_poi();
    draw_marker();
    draw_findmenu();
    draw_searchbar();
    draw_findmenu();
    draw_help();
    draw_filterpoi();
    draw_filterpoi_menu();
    draw_text_overlay();
    display_textbox();
    if(screen_bound.area() > max_world_bound.area())
        set_visible_world(max_world_bound);
    copy_off_screen_buffer_to_screen();
}

void create_all_buttons(){
/*
    create_button("Zoom Fit", "Find Int", act_on_find_button); 
    
    create_button("Find Int", "Find POI", act_on_poi_button); 
    
    create_button("Find POI", "Find Street", act_on_street_button);
   
    create_button("Find Street", "Filter POI", act_on_filter_button); 
*/
}

/******************************************************************************/

/**************************** FIND BUTTONS ************************************/

//Filter button
void act_on_filter_button(void (*drawscreen_ptr) (void)){
    /*
    if (!in_filter_state){
        
        in_filter_state = true;
        
        //Clear the input stream for the first run
        if (firstRun) {
            cin.ignore(1000, '\n');
            firstRun = false;
        }
        
        string input = " ";

        while (input != "display") {
            cout << "\nNOT DISPLAYED: ";
            for (unsigned i = 0; i < available_types.size(); i++)
                cout << available_types[i] << " ";
            cout << "\nCURRENTLY DISPLAYING: ";
            for (unsigned i = 0; i < show_poi_types.size(); i++)
                cout << show_poi_types[i] << " ";
            cout << "\nEnter a type to add it or remove it from current display (enter \'display\' to exit): ";
            
            getline(cin, input);
            
            if (input == "display"){
                //Do nothing
            }
            //If this is a currently displayed value
            else if (std::find(show_poi_types.begin(), show_poi_types.end(), input) != show_poi_types.end()) {
                //Remove it from our onscreen list
                auto itr = std::find(show_poi_types.begin(), show_poi_types.end(), input);
                std::swap(*itr, show_poi_types.back());
                show_poi_types.pop_back();
                
                //Place this into our available list
                available_types.push_back(input);
            }
            //If this is an available value
            else if (std::find(available_types.begin(), available_types.end(), input) != available_types.end()) {
                //Remove it from our available list
                auto itr = std::find(available_types.begin(), available_types.end(), input);
                std::swap(*itr, available_types.back());
                available_types.pop_back();
                
                //Place this into our available list
                show_poi_types.push_back(input);
            }
            //Otherwise there is not valid input
            else {
                cout << "\"" << input << "\" is not a valid type.\n";
            }
        }
        draw_all_POI();
    }
    else {
        in_filter_state = false;
        drawscreen();
    }
    */
}

//Find button for street
void act_on_street_button(void (*drawscreen_ptr) (void)){
    /*
    if (!in_street_state){
        
        in_street_state = true;
        
        //Clear the input stream for the first run
        if (firstRun) {
            cin.ignore(1000, '\n');
            firstRun = false;
        }
        
        //Grab the streets from the user
        string street_name;
        cout << "Enter name of street: ";
        getline(cin, street_name);
        
        
        std::vector<unsigned> list_highlighted_onscreen = find_street_ids_from_name(street_name);
        if (list_highlighted_onscreen.empty()) {
            cout << "No streets were found called \"" << street_name << "\".\n";
            in_street_state = false;
        }
        else {
            cout << "Street(s) found! Drawing onscreen.\n";
            highlighted_street = street_name;
        }
        list_highlighted_onscreen.clear();
        list_highlighted_onscreen.shrink_to_fit();
    }
    else {
        in_street_state = false;
        drawscreen();
    }
    */
}

void find_intersections () {
    if (!in_find_state){
        in_find_state = true;
        
        auto split = searchbox_input.find("&");
        string street1 = searchbox_input.substr(0,split);
        string street2 = searchbox_input.substr(split+1,searchbox_input.back());
        boost::trim(street1);
        boost::trim(street2);
        
        
        //Obtain the list of intersections;
        list_intersections_onscreen = find_intersection_ids_from_street_names(street1, street2);
        
        if (list_intersections_onscreen.empty()) {
            textbox_text_2 = "ALERT: Not found";
            textbox_text_1 = "";
            textbox_text_3 = "";
            textbox_onscreen = true;
            display_textbox();
            copy_off_screen_buffer_to_screen();
            in_find_state = false;
        }
        else {
            textbox_text_2 = searchbox_input;
            textbox_text_1 = "";
            textbox_text_3 = "";
            textbox_onscreen = true;
            display_textbox();
            copy_off_screen_buffer_to_screen();
            draw_intersections();
        }        
    }
    else {
        in_find_state = false;
        drawscreen();
    }
}

void find_poi () {
    if (!in_poi_state) {

        in_poi_state = true;

        //Grab the name of the POI from user
        string poi = searchbox_input;
        boost::trim(searchbox_input);

        bool poi_name_notfound = false;
        //Tries to access the name, if doesn't exist returns empty vector
        try {
            poi_id_from_name.at(poi);
        } catch (std::out_of_range) {
            poi_name_notfound = true;
        }

        if (!poi_name_notfound) {
            list_poi_onscreen = poi_id_from_name.at(poi);
            textbox_text_2 = searchbox_input;

            textbox_text_1 = "";
            textbox_text_3 = "";
            textbox_onscreen = true;
            display_textbox();
            draw_poi();
            copy_off_screen_buffer_to_screen();
        } 
        else {
            in_poi_state = false;
            textbox_text_2 = "ALERT: Not found";

            textbox_text_1 = "";
            textbox_text_3 = "";
            textbox_onscreen = true;
            display_textbox();
            copy_off_screen_buffer_to_screen();
        }        
        
    }
    else {
        in_poi_state = false;
        drawscreen();
    }
}

void find_streets(){
    if (!in_street_state){
        
        in_street_state = true;
        
        //Grab the streets from the user
        string street_name = searchbox_input;
        boost::trim(street_name);
        
        
        std::vector<unsigned> list_highlighted_onscreen = find_street_ids_from_name(street_name);
        
        if (!list_highlighted_onscreen.empty()) {
            textbox_text_2 = searchbox_input;
            highlighted_street = street_name;
            textbox_text_1 = "";
            textbox_text_3 = "";
            textbox_onscreen = true;
            display_textbox();
            copy_off_screen_buffer_to_screen();
        } 
        else {
            in_street_state = false;
            textbox_text_2 = "ALERT: Not found";
            textbox_text_1 = "";
            textbox_text_3 = "";
            textbox_onscreen = true;
            display_textbox();
            copy_off_screen_buffer_to_screen();
        }                
        list_highlighted_onscreen.clear();
        list_highlighted_onscreen.shrink_to_fit();
        drawscreen();
    }
    else {
        in_street_state = false;
        drawscreen();
    }
}

void path_int(){
    if (!in_path_state){
        in_path_state = true;
        
        //Take in point1, get the ID(s) of that intersection
        //
        //Then get ID(s) of point2
        //
        //Print out error if any intersections not found
        //
        //Iterate through these IDs
        //      For each of them, see the distance from point1[i]
        //      Save the IDs of the shortest distance

        //Run function and grab vector of street segments, save to direction_path vector
                
        auto split = point_dir1.find("&");
        if (split == std::string::npos) {
            in_path_state = false;
            return;
        }
        string street1 = point_dir1.substr(0,split);
        string street2 = point_dir1.substr(split+1,point_dir1.back());
        
        split = point_dir2.find("&");
        if (split == std::string::npos) {
            in_path_state = false;
            return;
        }
        string street3 = point_dir2.substr(0,split);
        string street4 = point_dir2.substr(split+1,point_dir2.back());        
        
        boost::trim(street1);
        boost::trim(street2);
        boost::trim(street3);
        boost::trim(street4);
        
        //Obtain the list of intersections;
        std::vector<unsigned> inter_1 = find_intersection_ids_from_street_names(street1, street2); 
        std::vector<unsigned> inter_2 = find_intersection_ids_from_street_names(street3, street4); 
        
        //If any of them don't exist, no path to draw
        if (inter_1.empty() || inter_2.empty()){
            in_path_state = false;
            textbox_text_1 = "ALERT: Not found!";
            textbox_text_2 = "";
            textbox_text_3 = "";
            textbox_onscreen = true;

            display_textbox();
            copy_off_screen_buffer_to_screen();
        }
        else {
            //Need to find closest intersection-intersection path
            //Go through all the intersections with first name
            double min_distance = std::numeric_limits<double>::max();
            unsigned final_start = 0;
            unsigned final_destination = 0;
            for (unsigned matches_1 = 0; matches_1 < inter_1.size(); matches_1++){
                
                //Go through the other intersections
                for (unsigned matches_2 = 0; matches_2 < inter_2.size(); matches_2++){

                    //Obtain a path to that and check if its faster than what we have so far
                    std::vector<unsigned> this_path = find_path_between_intersections(inter_1[matches_1], inter_2[matches_2], TURN_PENALTY);
                    double distance = compute_path_travel_time(this_path, TURN_PENALTY);
                    if (distance < min_distance){
                        direction_path = this_path;
                        min_distance = distance;
                        final_start = inter_1[matches_1];
                        final_destination = inter_2[matches_2];
                    }
                }
            }
            
            //Show our destination location            
            marker_onscreen = true;
            marker_location = LatLon_to_XY(intersection_positions[final_destination]);
            
            print_path();
            
            if (direction_path.size() == 0) {
                textbox_text_1 = "ALERT: Nowhere to go!";
                textbox_text_2 = "";
                textbox_text_3 = "";
                textbox_onscreen = true;
            }
            else {
                textbox_text_1 = "START: " + intersection_names[final_start];
                textbox_text_2 = "END: " + intersection_names[final_destination];
                textbox_text_3 = "Time: " + std::to_string(compute_path_travel_time(direction_path, TURN_PENALTY)) + "s";
                textbox_onscreen = true;
            }
            //Now we can draw out our direction path
            drawscreen();
            
        }
        
    }
    else {
        in_path_state = false;
        drawscreen();
    }
}

void path_poi(){
    if (!in_path_state){
        
        in_path_state = true;
        
        //Take in point1, get the ID(s) of that intersection
        //
        //Then get ID(s) of point2
        //
        //Print out error if any intersections not found
        //
        //Iterate through these POIs
        //      For each of them, see the distance from point1[i]
        //      Save the IDs of the shortest distance
        //Find the closest intersection to POI, grab its ID
        //
        //Run function and grab vector of street segments, save to direction_path vector

        auto split = point_dir1.find("&");
        if (split == std::string::npos) {
            in_path_state = false;
            return;
        }
        string street1 = point_dir1.substr(0,split);
        string street2 = point_dir1.substr(split+1,point_dir1.back());      
        
        string poi_dest = point_dir2;
        boost::trim(street1);
        boost::trim(street2);
        boost::trim(poi_dest);
        
        //Obtain the list of intersections;
        std::vector<unsigned> inter_1 = find_intersection_ids_from_street_names(street1, street2); 
        
        //Check to see if there are any POI's with this 
        try {
            std::vector<unsigned> poi_ids = poi_id_from_name.at(poi_dest);
        }
        //If no POIs, nothing to draw
        catch (std::out_of_range) {
            textbox_text_1 = "ALERT: No POIs found with this name!";
            textbox_text_2 = "";
            textbox_text_3 = "";
            textbox_onscreen = true;
            display_textbox();
            copy_off_screen_buffer_to_screen();
            in_path_state = false;
            return;
        }
        //If no intersections, nothing to draw
        if (inter_1.empty()){
            in_path_state = false;
            textbox_text_1 = "ALERT: No intersections with this name!";
            textbox_text_2 = "";
            textbox_text_3 = "";
            textbox_onscreen = true;
            display_textbox();
            copy_off_screen_buffer_to_screen();
            return;
        }
        else {
            //Need to find closest intersection-POI path
            //Go through all the intersections with first name
            double min_distance = std::numeric_limits<double>::max();
            unsigned closest_int = 0;
            for (unsigned matches_1 = 0; matches_1 < inter_1.size(); matches_1++) {
                //Obtain a path to that and check if its faster than what we have so far
                std::vector<unsigned> this_path = find_path_to_point_of_interest(inter_1[matches_1], poi_dest, TURN_PENALTY);
                double distance = compute_path_travel_time(this_path, TURN_PENALTY);
                if (distance < min_distance && distance !=0) {
                    direction_path = this_path;
                    min_distance = distance;
                    closest_int = inter_1[matches_1];
                }
            }
                        
            //Show our destination location (and all other POIs of the same name)           
            list_poi_onscreen = poi_id_from_name.at(point_dir2);
            
            print_path();
            
            if (direction_path.size() == 0) {
                textbox_text_1 = "ALERT: Nowhere to go!";
                textbox_text_2 = "";
                textbox_text_3 = "";
                textbox_onscreen = true;
            }
            else {
                textbox_text_1 = "START: " + intersection_names[closest_int];
                textbox_text_2 = "END: " + poi_dest;
                textbox_text_3 = "Time: " + std::to_string(compute_path_travel_time(direction_path, TURN_PENALTY)) + "s";
                textbox_onscreen = true;
            }
            
            //Now we can draw out our direction path
            drawscreen();
            
        }
        
    }
    else {
        in_path_state = false;
        drawscreen();
    }
}

//Find button for intersections
void act_on_find_button(void (*drawscreen_ptr) (void)){
    /*
    if (!in_find_state){
        
        in_find_state = true;
        
        //Clear the input stream for the first run
        if (firstRun) {
            cin.ignore(1000, '\n');
            firstRun = false;
        }
        //Grab the streets from the user
        string street1, street2;
        cout << "Enter street 1: ";
        getline(cin, street1);
        cout << "Enter street 2: ";
        getline(cin, street2);
        
        list_intersections_onscreen.clear();
        list_intersections_onscreen.shrink_to_fit();
        
        //Obtain the list of intersections;
        list_intersections_onscreen = find_intersection_ids_from_street_names(street1, street2);
        
        if (list_intersections_onscreen.empty()) {
            cout << "No intersections were found between \"" << street1 << "\" and \"" << street2 << "\".\n";
            in_find_state = false;
        }
        else {
            cout << "Intersections found! Drawing onscreen.\n";
            draw_intersections();
        }
    }
    else {
        in_find_state = false;
        drawscreen();
    }
    */
}

//Draws intersections onto the screen
void draw_intersections() {
    if (in_find_state){
        setcolor(PURPLE);
        for (unsigned i = 0; i < list_intersections_onscreen.size(); i++) {
            const t_point locInt = LatLon_to_XY(getIntersectionPosition(list_intersections_onscreen[i]));
            t_point scrnLocInt = world_to_scrn(locInt);
            set_coordinate_system(GL_SCREEN);
            fillarc(scrnLocInt, 7, 0, 360);
            set_coordinate_system(GL_WORLD);
        }
    }
}

//Find button for POI
void act_on_poi_button(void (*drawscreen_ptr) (void)){
    /*
    if (!in_poi_state) {

        in_poi_state = true;
        
        //Clear the input stream for the first run
        if (firstRun) {
            cin.ignore(1000, '\n');
            firstRun = false;
        }
        //Grab the name of the POI from user
        string poi;
        cout << "Enter the name of the point of interest: ";
        getline(cin, poi);

        bool poi_name_notfound = false;
        //Tries to access the name, if doesn't exist returns empty vector
        try {
            poi_id_from_name.at(poi);
        } catch (std::out_of_range) {
            poi_name_notfound = true;
        }
        
        if (!poi_name_notfound){
            list_poi_onscreen = poi_id_from_name.at(poi);
            cout << "Points of interest found! Drawing onscreen.\n";
            
        }
        else {
            cout << "No points of interest were found with the name \"" << poi << "\".\n";
            in_poi_state = false;
            draw_poi();
        }
        
    }
    else {
        in_poi_state = false;
        drawscreen();
    }
    */
}

//Draws POI onto the screen
void draw_poi() {
    if (in_poi_state) {
        for (unsigned i = 0; i < list_poi_onscreen.size(); i++) {
            const t_point locInt = LatLon_to_XY(getPointOfInterestPosition(list_poi_onscreen[i]));
            t_point scrnLocInt = world_to_scrn(locInt);
            set_coordinate_system(GL_SCREEN);
            
            //Setting the screen bound (a small window of the world bound)
            //For scoping (see draw POI)
            screen_bound.left() = 0.5*max_world_bound.left();
            screen_bound.bottom() = 0.5*max_world_bound.bottom();
            screen_bound.right() = 0.5*max_world_bound.right();
            screen_bound.top() = 0.5*max_world_bound.top();

            t_point redball = scrnLocInt;
            redball.y -= 30;
            t_point whiteball = redball;
            t_point orangeball = redball;
            t_point orangeball2 = redball;
            //We scope the size of the target coloured marker depending on the zoom
            if (getZoomLevel() < 8.5) {
                //Draw the POI
                setcolor(BLACK);
                fillarc(scrnLocInt, 10, 0, 360);
                setcolor(WHITE);
                fillarc(scrnLocInt, 8, 0, 360);
                setcolor(BLACK);
                fillarc(scrnLocInt, 3, 0, 360);
                //draw needle
                setcolor(t_color(150, 160, 164));
                setlinewidth(3);
                drawline(scrnLocInt, redball);
                //draw ball
                setcolor(BLACK);
                fillarc(redball, 14.5, 0, 360);
                setcolor(t_color(63,193,72));
                fillarc(redball, 14, 0, 360);
                setcolor(t_color(89,204,97));
                orangeball2.offset(-1.5, -1.5);
                fillarc(orangeball2, 9.5, 0, 360);
                setcolor(t_color(109,214,116));
                orangeball.offset(-3.5, -3.5);
                fillarc(orangeball, 8, 0, 360);
                //draw light
                setcolor(WHITE);
                whiteball.offset(-4.7, -4.7);
                fillarc(whiteball, 5.3, 0, 360);
            } else {
                //Draw the POI
                setcolor(BLACK);
                fillarc(scrnLocInt, 1.4*getZoomLevel(), 0, 360);
                setcolor(WHITE);
                fillarc(scrnLocInt, 1.0*getZoomLevel(), 0, 360);
                setcolor(BLACK);
                fillarc(scrnLocInt, 0.5*getZoomLevel(), 0, 360);

                redball.y -= 2 * getZoomLevel();
                whiteball = redball;
                orangeball = redball;
                orangeball2 = redball;
                //draw needle
                setcolor(t_color(150, 160, 164));
                setlinewidth(3 + 0.2 * getZoomLevel());
                drawline(scrnLocInt, redball);
                //draw ball
                setcolor(BLACK);
                fillarc(redball, 2 * getZoomLevel(), 0, 360);
                setcolor(63,193,72);
                fillarc(redball, 1.9 * getZoomLevel(), 0, 360);
                setcolor(t_color(89,204,97));
                orangeball2.offset(-4 + 0.1*getZoomLevel(), -4 + 0.1*getZoomLevel());
                fillarc(orangeball2, 1.3 * getZoomLevel(), 0, 360);
                setcolor(t_color(109,214,116));
                orangeball.offset(-6.5 + 0.1*getZoomLevel(), -6.5 + 0.1*getZoomLevel());
                fillarc(orangeball, 1.1 * getZoomLevel(), 0, 360);
                //draw light
                setcolor(WHITE);
                whiteball.offset(-8.5 + 0.1*getZoomLevel(), -8.5 + 0.1*getZoomLevel());
                fillarc(whiteball, 0.62 * getZoomLevel(), 0, 360);
            }
            set_coordinate_system(GL_WORLD);
            
        }    
    }  
}

/******************************************************************************/

/*************************** MOUSE & MARKER ***********************************/

//Clicking on an intersection onscreen - draws marker
void act_on_button_press(float x, float y, t_event_buttonPressed event) {    

    //Check if we clicked on the search bar
    if (searchbar_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))) {
        searchbar_selected = true;
        draw_searchbar();
        draw_text_overlay();
        copy_off_screen_buffer_to_screen();        
    }

    else if (help_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
        print_help();
    }
    
    else if (textbox.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
        //Nothing to do
    }
    
    else if (closebox.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
        textbox_onscreen = false;
        drawscreen();
        copy_off_screen_buffer_to_screen();                
    }
    
    else if (filterpoi_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
        filter_menu_on = true;
        draw_filterpoi_menu();
        copy_off_screen_buffer_to_screen();        
    } 
    
    else if (filter_menu_on && filtermenu_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
        if (restaurant_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
            filter_restaurant = !filter_restaurant;
        }
        if (healthcare_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
            filter_healthcare = !filter_healthcare;
        }
        if (post_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
            filter_post = !filter_post;
        }
        if (education_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
            filter_education = !filter_education;
        }
        if (busstop_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
            filter_busstop = !filter_busstop;
        }
        if (worship_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
            filter_worship= !filter_worship;
        }
        if (miscellaneous_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
            filter_miscellaneous = !filter_miscellaneous;
        }
        if (filtermenu_confirm_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
            filter_menu_on = false;
        }
        drawscreen();
        copy_off_screen_buffer_to_screen();
    }
    
    else if (modemenu_search_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
        in_searching_mode = true;
        in_directions_mode = false;
        textbox_onscreen = false;
        draw_searchbar();
        draw_findmenu();
        draw_text_overlay();
        copy_off_screen_buffer_to_screen();
    }
    else if (modemenu_directions_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
        in_searching_mode = false;
        in_directions_mode = true;
        textbox_onscreen = false;
        drawscreen();
        copy_off_screen_buffer_to_screen();
    }
    
    else if (findmenu_street_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
        if (in_searching_mode){
            search_poi = false;
            search_int = false;
            if (!search_street){         
                search_street = true;
            }
            else {
                search_street = false;
                in_street_state = false;
            }
            drawscreen();
            copy_off_screen_buffer_to_screen();
        }
    }

    else if (findmenu_int_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
        if (in_searching_mode){
            search_poi = false;
            search_street = false;
            if (!search_int){         
                search_int = true;
            }
            else {
                search_int = false;
                in_poi_state = false;
            }
            drawscreen();
            copy_off_screen_buffer_to_screen();
        }
        else {
            direction_int = true;
            direction_poi = false;
            drawscreen();
            copy_off_screen_buffer_to_screen();
        }
    }
    else if (findmenu_poi_box.intersects(xworld_to_scrn_fl(x), yworld_to_scrn_fl(y))){
        if (in_searching_mode){
            search_street = false;
            search_int = false;
            if (!search_poi){         
                search_poi = true;
            }
            else {
                search_poi = false;
                in_find_state = false;
            }
            drawscreen();
            copy_off_screen_buffer_to_screen();
        }
        else {
            direction_int = false;
            direction_poi = true;
            drawscreen();
            copy_off_screen_buffer_to_screen();
        }
    }
    
    else {
        searchbar_selected = false;
        //Draw the marker or wipe the screen at that intersection (when clicking somewhere else)
        if (!marker_onscreen && !no_overlap.intersects(xworld_to_scrn_fl(x),yworld_to_scrn_fl(y))){
            //Convert clicked x and y to lat and lon
            LatLon const clicked_world_coords = XY_to_LatLon(t_point(x, y));

            //If this is not the first time we used the marker, fill up past location
            if (firstDistance) {
                past_location = marker_location;
                past_id = current_id;
            }
            
            //Query r tree search to find and return id of closest intersection
            unsigned const closest_intersection = find_closest_intersection(clicked_world_coords);
            current_id = closest_intersection;

            if (in_directions_mode)
                in_path_state = true;                
            
            //Using id, find and print intersection name
            marker_location = LatLon_to_XY(getIntersectionPosition(closest_intersection));
            marker_onscreen = true;     //Draw only the marker for now
            
            draw_marker();
            draw_searchbar();
            draw_text_overlay();
            draw_findmenu();
            draw_help();
            draw_filterpoi();
            draw_filterpoi_menu(); 
            
            calculate_distance();       //Print out distance from last location

            //After the first click, we can begin calculating distance
            if (!firstDistance)
                firstDistance = true;
            copy_off_screen_buffer_to_screen();
        }
        else {
            if (in_directions_mode)
                in_path_state = false;     
            marker_onscreen = false;    //We've deselected, wipe screen
            drawscreen();
        }
    }
}

//Draws the marker onto screen if on
void draw_marker(){
    if (marker_onscreen){
        t_point scrnLocInt = world_to_scrn(marker_location);
        set_coordinate_system(GL_SCREEN);
        t_point redball = scrnLocInt;
        redball.y -= 30;
        t_point whiteball = redball;
        t_point orangeball = redball;
        t_point orangeball2 = redball;
        //We scope the size of the target coloured marker depending on the zoom
        if (getZoomLevel() < 8.5) {
            //Draw the POI
            setcolor(BLACK);
            fillarc(scrnLocInt, 10, 0, 360);
            setcolor(WHITE);
            fillarc(scrnLocInt, 8, 0, 360);
            setcolor(BLACK);
            fillarc(scrnLocInt, 3, 0, 360);
            
            setcolor(t_color(150, 160, 164));
            setlinewidth(3);
            drawline(scrnLocInt, redball);
            setcolor(BLACK);
            fillarc(redball, 14.5, 0, 360);
            setcolor(RED);
            fillarc(redball, 14, 0, 360);
            setcolor(t_color(237, 44, 23));
            orangeball2.offset(-1.5, -1.5);
            fillarc(orangeball2, 9.5, 0, 360);
            setcolor(t_color(200, 90, 70));
            orangeball.offset(-3.5, -3.5);
            fillarc(orangeball, 8, 0, 360);
            setcolor(WHITE);
            whiteball.offset(-4.7, -4.7);
            fillarc(whiteball, 5.3, 0, 360);
        } else {
            //Draw the POI
            setcolor(BLACK);
            fillarc(scrnLocInt, 1.4*getZoomLevel(), 0, 360);
            setcolor(WHITE);
            fillarc(scrnLocInt, 1.0*getZoomLevel(), 0, 360);
            setcolor(BLACK);
            fillarc(scrnLocInt, 0.5*getZoomLevel(), 0, 360);
            
            setcolor(t_color(150, 160, 164));
            setlinewidth(3 + 0.2 * getZoomLevel());
            redball.y -= 2 * getZoomLevel();
            whiteball = redball;
            orangeball = redball;
            orangeball2 = redball;
            
            drawline(scrnLocInt, redball);
            setcolor(BLACK);
            fillarc(redball, 2.0 * getZoomLevel(), 0, 360);
            setcolor(RED);
            fillarc(redball, 1.9 * getZoomLevel(), 0, 360);
            setcolor(t_color(237, 44, 23));
            orangeball2.offset(-4 + 0.1*getZoomLevel(), -4 + 0.1*getZoomLevel());
            fillarc(orangeball2, 1.3 * getZoomLevel(), 0, 360);
            setcolor(t_color(200, 90, 70));
            orangeball.offset(-6.5 + 0.1*getZoomLevel(), -6.5 + 0.1*getZoomLevel());
            fillarc(orangeball, 1.1 * getZoomLevel(), 0, 360);
            setcolor(WHITE);
            whiteball.offset(-8.5 + 0.1*getZoomLevel(), -8.5 + 0.1*getZoomLevel());
            fillarc(whiteball, 0.62 * getZoomLevel(), 0, 360);
        }
            
        set_coordinate_system(GL_WORLD);       
    }  
}

void draw_path(){
    if (in_path_state){
            
        for (unsigned i = 0; i < direction_path.size(); i++){
            unsigned segment = direction_path[i];
            
            for (unsigned point = 0; point < street_segment_positions[segment].size()-1; point++){
                //Get the endpoints of this street segment
                float x1 = street_segment_positions[segment][point].lon() * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat);
                float y1 = street_segment_positions[segment][point].lat() * DEG_TO_RAD;

                float x2 = street_segment_positions[segment][point+1].lon() * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat);
                float y2 = street_segment_positions[segment][point+1].lat() * DEG_TO_RAD;
                //bright purple for streets
                setcolor(t_color(192,73,221));
                setlinewidth(1.2 * getZoomLevel());
                //Draw this component of the street segment
                drawline(x1 * SCOPE, y1 * SCOPE, x2 * SCOPE, y2 * SCOPE);  
            }for (unsigned point = 0; point < street_segment_positions[segment].size()-1; point++){
                //Get the endpoints of this street segment
                float x1 = street_segment_positions[segment][point].lon() * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat);
                float y1 = street_segment_positions[segment][point].lat() * DEG_TO_RAD;

                float x2 = street_segment_positions[segment][point+1].lon() * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat);
                float y2 = street_segment_positions[segment][point+1].lat() * DEG_TO_RAD;
                //bright purple for streets
                setcolor(t_color(253,137,255));
                setlinewidth(0.8 * getZoomLevel());
                //Draw this component of the street segment
                drawline(x1 * SCOPE, y1 * SCOPE, x2 * SCOPE, y2 * SCOPE);  
            }
        }
    }
}

//Prints out the path taken to console
void print_path(){
    if (in_path_state){
        //A variable to keep in mind what our previous street was
        unsigned previous_road;
        unsigned direction_count = 0;
        double distance_current = 0; 
        double distance_total = 0;
        if (direction_path.size() != 0)
            cout << "\n\n****************---DIRECTIONS---****************\n";
        for (unsigned i = 0; i < direction_path.size(); i++){
            
            //Check to see if we are still on the same street; if yes, add more distance
            if (i == 0 || street_segment_info[direction_path[i]].streetID == previous_road){
                distance_current += street_segment_lengths[direction_path[i]];
                previous_road = street_segment_info[direction_path[i]].streetID;
            }
            else {
                direction_count++;
                cout << direction_count << ". Travel along " << street_names[previous_road] << " for ";
                unsigned kilometres = distance_current / 1000.0;
                if (kilometres >= 1)
                    cout << distance_current/1000.0 << " km.\n";
                else
                    cout << distance_current << " m.\n";
                distance_total += distance_current;
                distance_current = 0;
                direction_count++;
                cout << direction_count <<
                        ". Turn onto " << street_names[street_segment_info[direction_path[i]].streetID] << ".\n";
                previous_road = street_segment_info[direction_path[i]].streetID;
                distance_current = street_segment_lengths[direction_path[i]];
            }
            
            //Check if we are at the final leg - we must print then
            if (i == direction_path.size()-1){
                direction_count++;
                if (i != 0)
                    distance_current += street_segment_lengths[direction_path[i]];
                distance_total += distance_current;
 
                cout << direction_count << ". Travel along " << street_names[street_segment_info[direction_path[i]].streetID]
                        << " for ";
                unsigned kilometres = distance_current / 1000.0;
                if (kilometres >= 1)
                    cout << distance_current/1000.0 << " km.\n";
                else
                    cout << distance_current << " m.\n";

                
                cout << "ARRIVAL AT DESTINATION\n";
                
                //Print out total distance
                kilometres = distance_total / 1000.0;
                cout << "Approximate distance: ";
                if (kilometres >= 1)
                    cout << distance_total/1000.0 << " km.\n";
                else
                    cout << distance_total << " m.\n";
                        
                //Print out total time
                double total_time = compute_path_travel_time(direction_path, TURN_PENALTY);
                int hours = total_time / 3600;
                int minutes = (total_time - hours*3600) / 60;
                double seconds = total_time - hours*3600 - minutes*60;  
                cout << "Estimated Travel Time: ";
                if (hours > 0)
                    cout << hours << "h ";
                if (minutes > 0)
                    cout << minutes << "m ";
                cout << seconds << "s. \n";                        
            }
        }
        if (direction_path.size() != 0)
            cout << "\n\n************************************************\n";
    }
}

//Outputs the coordinates on lower status bar
void act_on_mouse_move(float x, float y) {
    //Display Lat lon coordinates on status bar
    LatLon const position = XY_to_LatLon(t_point(x,y));
    update_message("Lat: " + to_string(position.lat()) + " Lon: " + to_string(position.lon()));
}


#include <X11/keysym.h>

void act_on_key_press(char c, int keysym) {
    // function to handle keyboard press evenst, the ASCII character is returned
    // along with an extended code (keysym) on X11 to represent non-ASCII
    // characters like the arrow keys.
    //Update our input string if we have the search bar clicked
    if (searchbar_selected){  
        //Read in a backspace, remove our input's last character
        if (keysym == 65288){
            if (searchbox_input.size() > 0)
                searchbox_input.pop_back();
        }
        //Add our new character
        else if ((keysym >= 97 && keysym <= 122) || (keysym >= 65 && keysym <= 90) || keysym == 38 || keysym == 32 
                    || keysym == 39 || keysym == 46 || keysym == 63){
            if ('a' <= c <= 'z')
                searchbox_input = searchbox_input + c;
            else {}
        }
        //If they hit the Enter key, we set our input complete flag high
        else if (keysym == 65293){
            input_complete = true;
            if (in_searching_mode){
                if (search_int)
                    find_intersections();
                else if (search_poi)
                    find_poi();
                else 
                    find_streets();
            }
            else {
                if (!point1_inputted){
                    point_dir1 = searchbox_input;
                    boost::trim(point_dir1);
                    searchbox_input = "";
                    point1_inputted = true;
                }
                else {
                    point_dir2 = searchbox_input;
                    boost::trim(point_dir2);
                    if (direction_int)    
                        path_int();
                    else
                        path_poi();
                    point1_inputted = false;
                }
            }
        }
        //Invalid character; do nothing
        else { }
        draw_searchbar();
        draw_text_overlay();
        copy_off_screen_buffer_to_screen();
    }

//Use #ifdef X11, #endif and XK_Left, XK_Down, etc. for arrow keys
}

void draw_searchbar(){
    set_coordinate_system(GL_SCREEN); 
    if (searchbar_selected)
        draw_surface(icon_searchbar_selected, 10, 10);
    else
        draw_surface(icon_searchbar, 10, 10);
    if (in_searching_mode)
        draw_surface(icon_searchselectsearch, 370, 10);
    else
        draw_surface(icon_searchselectdirections, 370, 10);        
    set_coordinate_system(GL_WORLD);     
}

void draw_text_overlay(){
    set_coordinate_system(GL_SCREEN); 
    settextrotation(0);
    setfontsize(9);
    setcolor(BLACK);
    if (in_directions_mode){
        if (!point1_inputted)
            drawtext_in_sb(searchbar_box, "Start: " + searchbox_input);
        else
            drawtext_in_sb(searchbar_box, "To: " + searchbox_input);  
        }
    else{
        drawtext_in_sb(searchbar_box, searchbox_input);          
    }
    set_coordinate_system(GL_WORLD); 
}

void draw_findmenu(){
    set_coordinate_system(GL_SCREEN); 
    if (in_searching_mode){
        if (search_street)
            draw_surface(icon_searchmenustreet, 10, 70);
        else if (search_poi)
            draw_surface(icon_searchmenupoi, 10, 70);
        else if (search_int)
            draw_surface(icon_searchmenuint, 10, 70);
        else
            draw_surface(icon_searchmenu, 10, 70);
    }
    else {
        if (direction_int)
            draw_surface(icon_directionmenuint, 10, 110);
        else
            draw_surface(icon_directionmenupoi, 10, 110);            
    }
    set_coordinate_system(GL_WORLD);         
}

void draw_help(){
    set_coordinate_system(GL_SCREEN); 
    draw_surface(icon_help, 10, 300);            
    set_coordinate_system(GL_WORLD);             
}

void print_help(){
    cout << help;
}

void draw_filterpoi(){
    set_coordinate_system(GL_SCREEN); 
    draw_surface(icon_filterpoibutton, 10, 350);            
    set_coordinate_system(GL_WORLD);      
}

void draw_filterpoi_menu(){
    if (filter_menu_on){
        set_coordinate_system(GL_SCREEN); 
        draw_surface(icon_filtermenupoi, 60, 350);       
        if (filter_restaurant)
            draw_surface(icon_filter_restaurant, 74, 354);
        else
            draw_surface(icon_filter_restaurant_selected, 74, 354);
        if (filter_busstop)
            draw_surface(icon_filter_busstop, 74, 372);
        else
            draw_surface(icon_filter_busstop_selected, 74, 372);
        if (filter_healthcare)
            draw_surface(icon_filter_healthcare, 74, 390);
        else
            draw_surface(icon_filter_healthcare_selected, 74, 390);
        if (filter_post)
            draw_surface(icon_filter_post, 74, 408);
        else
            draw_surface(icon_filter_post_selected, 74, 408);
        if (filter_worship)
            draw_surface(icon_filter_worship, 74, 426);
        else
            draw_surface(icon_filter_worship_selected, 74, 426);
        if (filter_education)
            draw_surface(icon_filter_education, 74, 444);
        else
            draw_surface(icon_filter_education_selected, 74, 444);
        if (filter_miscellaneous)
            draw_surface(icon_filter_miscellaneous, 74,462);
        else
            draw_surface(icon_filter_miscellaneous_selected, 74,462);
        set_coordinate_system(GL_WORLD);          
    }
}

//Draws the text box with info
void display_textbox(){
    if (textbox_onscreen){
        set_coordinate_system(GL_SCREEN);
        draw_surface(icon_textbox, 10, 450);   
        if (in_directions_mode)
           draw_surface(icon_directionstag, 310,450); 
        else
           draw_surface(icon_searchtag, 310,450);   
        setcolor(BLACK);
        setfontsize(8);
        drawtext(t_point(textbox.get_xcenter(),textbox.bottom()+10), textbox_text_1);
        drawtext(t_point(textbox.get_xcenter(),textbox.bottom()+30), textbox_text_2);
        drawtext(t_point(textbox.get_xcenter(),textbox.bottom()+50), textbox_text_3);
        set_coordinate_system(GL_WORLD);
    }
}

void print_filterpoi(){
    if (!in_filter_state){
        
        in_filter_state = true;
        
        //Clear the input stream for the first run
        if (firstRun) {
            cin.ignore(1000, '\n');
            firstRun = false;
        }
        
        string input = " ";

        while (input != "display") {
            cout << "\nNOT DISPLAYED: ";
            for (unsigned i = 0; i < available_types.size(); i++)
                cout << available_types[i] << " ";
            cout << "\nCURRENTLY DISPLAYING: ";
            for (unsigned i = 0; i < show_poi_types.size(); i++)
                cout << show_poi_types[i] << " ";
            cout << "\nEnter a type to add it or remove it from current display (enter \'display\' to exit): ";
            
            getline(cin, input);
            
            if (input == "display"){
                //Do nothing
            }
            //If this is a currently displayed value
            else if (std::find(show_poi_types.begin(), show_poi_types.end(), input) != show_poi_types.end()) {
                //Remove it from our onscreen list
                auto itr = std::find(show_poi_types.begin(), show_poi_types.end(), input);
                std::swap(*itr, show_poi_types.back());
                show_poi_types.pop_back();
                
                //Place this into our available list
                available_types.push_back(input);
            }
            //If this is an available value
            else if (std::find(available_types.begin(), available_types.end(), input) != available_types.end()) {
                //Remove it from our available list
                auto itr = std::find(available_types.begin(), available_types.end(), input);
                std::swap(*itr, available_types.back());
                available_types.pop_back();
                
                //Place this into our available list
                show_poi_types.push_back(input);
            }
            //Otherwise there is not valid input
            else {
                cout << "\"" << input << "\" is not a valid type.\n";
            }
        }
        draw_all_POI();
    }
    else {
        in_filter_state = false;
        drawscreen();
    }
}

/******************************************************************************/

/*************************** DRAWING STREETS ***********************************/

//Draws all streets in black first
//static void draw_all_street_segmentsB(){
//    
//    setlinestyle(SOLID, ROUND);
//    
//    for (unsigned segment = 0; segment < street_segment_positions.size(); segment++){
//        bool draw = true;
//
//        string roadType = street_segment_roadtypes[segment];
//        
//        
//        
//        setcolor(173,171,171);
//        if (getZoomLevel() > 6.5){
//            if (roadType == "motorway" || roadType == "motorway_link") {
//                draw = true;
//                setcolor(232,195,127);
//                setlinewidth(1.6 * getZoomLevel());
//            }
//            else if (roadType == "trunk" || roadType == "trunk_link") {
//                draw = true;
//                setcolor(232,195,127);
//                setlinewidth(1.6 * getZoomLevel());
//            }
//            else if (roadType == "primary" || roadType == "primary_link"){
//                draw = true;
//                setcolor(173,171,171);
//                setlinewidth(1.75 * getZoomLevel());
//            }
//            else if (roadType == "secondary" || roadType == "secondary_link"){
//                draw = true;
//                setcolor(173,171,171);
//                setlinewidth(1.5 * getZoomLevel());
//            }
//            else if (roadType == "tertiary" || roadType == "tertiary_link") {
//                draw = true;
//                setcolor(173,171,171);
//                setlinewidth(1.25 * getZoomLevel());
//            }
//            else if(roadType == "track"){
//                draw = true;
//                setlinestyle(DASHED, BUTT);
//                setcolor(173,171,171);
//                setlinewidth(1.25 * getZoomLevel());
//            }
//            else {
//                draw = true;
//                setlinewidth(1 * getZoomLevel());
//            }
//        }        
//        else if (getZoomLevel() > 2.7 && getZoomLevel() < 6.5){
//            if (roadType == "motorway" || roadType == "motorway_link") {
//                draw = true;
//                setcolor(232,195,127);
//                setlinewidth(1.35 * getZoomLevel());
//            }
//            else if (roadType == "trunk" || roadType == "trunk_link") {
//                draw = true;
//                setcolor(232,195,127);
//                setlinewidth(1.35 * getZoomLevel());
//            }
//            else if (roadType == "primary" || roadType == "primary_link"){
//                draw = true;
//                setcolor(173,171,171);
//                setlinewidth(1.5 * getZoomLevel());
//            }
//            else if (roadType == "secondary" || roadType == "secondary_link"){
//                draw = true;
//                setcolor(173,171,171);
//                setlinewidth(1.25 * getZoomLevel());
//            }
//            else if (roadType == "tertiary" || roadType == "tertiary_link") {
//                draw = true;
//                setcolor(173,171,171);
//                setlinewidth(1 * getZoomLevel());
//            }
//            else if(roadType == "track"){
//                draw = true;
//                setlinestyle(DASHED, BUTT);
//                setcolor(173,171,171);
//                setlinewidth(1 * getZoomLevel());
//            }
//            else {
//                draw = false;
//            }
//        }
//        else if (getZoomLevel() > 1.4 && getZoomLevel() < 2.7){
//            if (roadType == "motorway" || roadType == "motorway_link") {
//                draw = true;
//                setcolor(232,195,127);
//                setlinewidth(1.15 * getZoomLevel());
//            }
//            else if (roadType == "trunk" || roadType == "trunk_link") {
//                draw = true;
//                setcolor(232,195,127);
//                setlinewidth(1.15 * getZoomLevel());
//            }
//            else if (roadType == "primary" || roadType == "primary_link"){
//                draw = true;
//                setcolor(173,171,171);   
//                setlinewidth(1.25 * getZoomLevel());
//            }
//            else if (roadType == "secondary" || roadType == "secondary_link"){
//                draw = true;
//                setcolor(173,171,171);
//                setlinewidth(1 * getZoomLevel());
//            }
//            else {
//                draw = false;
//            }
//        }
//        else if (getZoomLevel() > 0.8 && getZoomLevel() < 1.4){
//            if (roadType == "motorway" || roadType == "motorway_link") {
//                draw = true;
//                setcolor(232,195,127);
//                setlinewidth(1 * getZoomLevel());
//            }
//            else if (roadType == "trunk" || roadType == "trunk_link") {
//                draw = true;
//                setcolor(232,195,127);
//                setlinewidth(1 * getZoomLevel());
//            }
//            else if (roadType == "primary" || roadType == "primary_link"){
//                draw = true;
//                setcolor(173,171,171);
//                setlinewidth(1 * getZoomLevel());
//            }
//            else {
//                draw = false;
//            }
//        }
//        else{
//            if (roadType == "motorway" || roadType == "motorway_link") {
//                draw = true;
//                setcolor(232,195,127);
//                setlinewidth(0.9 * getZoomLevel());
//            }
//            else if (roadType == "trunk" || roadType == "trunk_link") {
//                draw = true;
//                setcolor(232,195,127);
//                setlinewidth(0.9 * getZoomLevel());
//            }
//            else {
//                draw = false;
//            }
//        }
//        
//        
//        //Check to see if this is the street we must highlight
//        bool highlighted = ((highlighted_street == street_names[street_segment_info[segment].streetID]) && in_street_state);
//        
//        //Now that we know if its a major street or not, we can draw it        
//        if (draw || highlighted){
//            if (highlighted) {
//                setcolor(163,52,59);
//                setlinewidth(1.5 * getZoomLevel());
//            }
//            for (unsigned point = 0; point < street_segment_positions[segment].size()-1; point++){
//                //Get the endpoints of this street segment
//                float x1 = street_segment_positions[segment][point].lon() * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat);
//                float y1 = street_segment_positions[segment][point].lat() * DEG_TO_RAD;
//
//                float x2 = street_segment_positions[segment][point+1].lon() * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat);
//                float y2 = street_segment_positions[segment][point+1].lat() * DEG_TO_RAD;
//
//                //Draw this component of the street segment
//                drawline(x1 * SCOPE, y1 * SCOPE, x2 * SCOPE, y2 * SCOPE);                
//            }
//        }
//    }
//}

//Draws all streets
static void draw_all_street_segments(){
    
    setlinestyle(SOLID, ROUND);
    
    for (unsigned segment = 0; segment < street_segment_positions.size(); segment++){
        bool draw = true;

        string roadType = street_segment_roadtypes[segment];
        
        /*
         * We get the type of road each of these are and with each of them we 
         * decide whether to draw or not and what thickness to draw them at
         */
        
        /*----------------------ZOOM ADJUSTMENTS------------------------------*/
        if (getZoomLevel() > 6.5){
            if (roadType == "motorway" || roadType == "motorway_link") {
                draw = true;
                setcolor(245, 213, 154);
                setlinewidth(1.4 * getZoomLevel());
            }
            else if (roadType == "trunk" || roadType == "trunk_link") {
                draw = true;
                setcolor(254, 239, 188);
                setlinewidth(1.4 * getZoomLevel());
            }
            else if (roadType == "primary" || roadType == "primary_link"){
                draw = true;
                setcolor(YELLOW);
                setlinewidth(1.65 * getZoomLevel());
            }
            else if (roadType == "secondary" || roadType == "secondary_link"){
                draw = true;
                setcolor(WHITE);
                setlinewidth(1.4 * getZoomLevel());
            }
            else if (roadType == "tertiary" || roadType == "tertiary_link") {
                draw = true;
                setcolor(WHITE);
                setlinewidth(1.15 * getZoomLevel());
            }
            else if(roadType == "track"){
                draw = true;
                setcolor(SADDLEBROWN);
                setlinestyle(DASHED, BUTT);
                setlinewidth(1.15 * getZoomLevel());
            }
            else {
                draw = true;
                setcolor(WHITE);
                setlinewidth(0.9 * getZoomLevel());
            }
        }        
        else if (getZoomLevel() > 2.7 && getZoomLevel() < 6.5){
            if (roadType == "motorway" || roadType == "motorway_link") {
                draw = true;
                setcolor(245, 213, 154);
                setlinewidth(1.15 * getZoomLevel());
            }
            else if (roadType == "trunk" || roadType == "trunk_link") {
                draw = true;
                setcolor(254, 239, 188);
                setlinewidth(1.15 * getZoomLevel());
            }
            else if (roadType == "primary" || roadType == "primary_link"){
                draw = true;
                setcolor(YELLOW);
                setlinewidth(1.5 * getZoomLevel());
            }
            else if (roadType == "secondary" || roadType == "secondary_link"){
                draw = true;
                setcolor(WHITE);
                setlinewidth(1.15 * getZoomLevel());
            }
            else if (roadType == "tertiary" || roadType == "tertiary_link") {
                draw = true;
                setcolor(WHITE);
                setlinewidth(0.9 * getZoomLevel());
            }
            else if(roadType == "track"){
                draw = true;
                setcolor(SADDLEBROWN);
                setlinestyle(DASHED, BUTT);
                setlinewidth(0.9 * getZoomLevel());
            }
            else {
                draw = false;
            }
        }
        else if (getZoomLevel() > 1.4 && getZoomLevel() < 2.7){
            if (roadType == "motorway" || roadType == "motorway_link") {
                draw = true;
                setcolor(245, 213, 154);
                setlinewidth(0.9 * getZoomLevel());
            }
            else if (roadType == "trunk" || roadType == "trunk_link") {
                draw = true;
                setcolor(254, 239, 188);
                setlinewidth(0.9 * getZoomLevel());
            }
            else if (roadType == "primary" || roadType == "primary_link"){
                draw = true;
                setcolor(YELLOW);
                setlinewidth(1.25 * getZoomLevel());
            }
            else if (roadType == "secondary" || roadType == "secondary_link"){
                draw = true;
                setcolor(WHITE);
                setlinewidth(0.9 * getZoomLevel());
            }
            else {
                draw = false;
            }
        }
        else if (getZoomLevel() > 0.8 && getZoomLevel() < 1.4){
            if (roadType == "motorway" || roadType == "motorway_link") {
                draw = true;
                setcolor(245, 213, 154);
                setlinewidth(0.7 * getZoomLevel());
            }
            else if (roadType == "trunk" || roadType == "trunk_link") {
                draw = true;
                setcolor(254, 239, 188);
                setlinewidth(0.7 * getZoomLevel());
            }
            else if (roadType == "primary" || roadType == "primary_link"){
                draw = true;
                setcolor(YELLOW);
                setlinewidth(0.9 * getZoomLevel());
            }
            else {
                draw = false;
            }
        }
        else{
            if (roadType == "motorway" || roadType == "motorway_link") {
                draw = true;
                setcolor(245, 213, 154);
                setlinewidth(0.6 * getZoomLevel());
            }
            else if (roadType == "trunk" || roadType == "trunk_link") {
                draw = true;
                setcolor(254, 239, 188);
                setlinewidth(0.6 * getZoomLevel());
            }
            else {
                draw = false;
            }
        }
        /*--------------------------------------------------------------------*/
        
        //Check to see if this is the street we must highlight
        bool highlighted = ((highlighted_street == street_names[street_segment_info[segment].streetID]) && in_street_state);
        
        //Now that we know if its a major street or not, we can draw it        
        if (draw || highlighted){
            if (highlighted) {
                setcolor(RED);
                setlinewidth(1.1 * getZoomLevel());
            }
            for (unsigned point = 0; point < street_segment_positions[segment].size()-1; point++){
                //Get the endpoints of this street segment
                float x1 = street_segment_positions[segment][point].lon() * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat);
                float y1 = street_segment_positions[segment][point].lat() * DEG_TO_RAD;

                float x2 = street_segment_positions[segment][point+1].lon() * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat);
                float y2 = street_segment_positions[segment][point+1].lat() * DEG_TO_RAD;

                //Draw this component of the street segment
                drawline(x1 * SCOPE, y1 * SCOPE, x2 * SCOPE, y2 * SCOPE);                
            }
        }
    }
}

/******************************************************************************/

/************************** DRAWING FEATURES **********************************/

//Draws the outline of a single feature
void drawFeatureLine(unsigned id, const t_color& color, int linestyle, int linewidth)
{
    unsigned numPoints = getFeaturePointCount(id);
    setlinewidth (linewidth);
    setcolor(color);
    setlinestyle(linestyle);
    LatLon point1 = getFeaturePoint(id, 0);//the first one is should be id, second one feature point idx
    LatLon point2;
    for(unsigned j = 1; j < numPoints ; j++)
    {
        point2 = getFeaturePoint(id, j);
        drawSegment(point1, point2);
        point1 = point2;
    }
}

//Draws a single line segment 
void drawSegment (LatLon from, LatLon to)
{
    t_point p1 = LatLon_to_XY(from);
    t_point p2 = LatLon_to_XY(to);
    drawline(p1, p2);
}

//Draws a single geographic feature (building))
void drawFeatureAreaB(unsigned id, const t_color& color, int linestyle)
{
    
    setcolor(163, 124, 97);
    setlinestyle(linestyle);
    unsigned numPoints = getFeaturePointCount(id);
    t_point points[numPoints];
    t_point jb[numPoints];

    for (unsigned j = 0; j < numPoints ; j++)
    {   
        points[j] = LatLon_to_XY(getFeaturePoint(id, j));
        jb[j].x = points[j].x;
        jb[j].y = points[j].y;
        //draw different 3D shadow for position on screen
        jb[j].offset(-1.2,1.2);
       
    }
    fillpoly(points, numPoints);
    setcolor(color);
    fillpoly(jb, numPoints);
    
}

//Draws a single geographic feature
void drawFeatureArea(unsigned id, const t_color& color, int linestyle)
{
    setcolor(color);
    setlinestyle(linestyle);
    unsigned numPoints = getFeaturePointCount(id);
    t_point points[numPoints];
    for (unsigned j = 0; j < numPoints ; j++)
    {   
        points[j] = LatLon_to_XY(getFeaturePoint(id, j));
    }
    fillpoly(points, numPoints);
}

//Draws all the geographic features on the map
static void draw_all_features(){
    
        /*
        LatLon location = getPointOfInterestPosition(i);
        t_point mylocation=LatLon_to_XY(location);
       
        unsigned boundx, boundy;
        string type = poi_types[i];
        
        //bool onscreen = (std::find(show_poi_types.begin(), show_poi_types.end(), type) != show_poi_types.end());
        
        bool onscreen = true;
        
        //Only the points of interest that are inside the screen will be presented, speeds up drawing time
        if(onscreen && screen_bound.intersects(mylocation)&&screen_bound.get_height()<=boundy&&screen_bound.get_width()<=boundx)
        */
    
    //Draw multisegment line features first
    for(auto i = feature_ids_by_areas.begin(); i != feature_ids_by_areas.end() ; i++){
        int feature_attri=feature_types[i -> second];
        if(i -> first == 0){
            //Park
            if(feature_attri==1){
                if(getZoomLevel() > 1.4)
                    drawFeatureArea(i -> second, t_color(197, 223, 158),SOLID);
            }
            //Beach
            else if(feature_attri==2){
                if(getZoomLevel() > 1.4)
                    drawFeatureArea(i -> second, t_color(250, 242, 199),SOLID);
            }
            //Lake
            else if(feature_attri==3)
                drawFeatureArea(i -> second,t_color(164, 198, 226),SOLID);
            //Island
            else if(feature_attri==5) 
                drawFeatureArea(i -> second,ORANGE,SOLID);
            //Shoreline
            else if(feature_attri==6) 
                drawFeatureArea(i -> second,BISQUE,SOLID);
            //Building
            else if(feature_attri==7){
                if(getZoomLevel() > 7.8)
                    drawFeatureAreaB(i -> second, t_color(230, 202, 162),SOLID);
            }
            //Greenspace
            else if(feature_attri==8)
                drawFeatureArea(i -> second,t_color(147,216,104),SOLID);
            //River
            else if(feature_attri==4)
                drawFeatureLine(i -> second, t_color(164, 198, 226), SOLID, 3);
            //Stream
            else if(feature_attri==9){
                if(getZoomLevel() > 1.)
                    drawFeatureLine(i -> second, t_color(164, 198, 226), SOLID, 1);
            }
            //Unknown
            else{
                if(getZoomLevel() > 7.8)
                    drawFeatureLine(i -> second, LIGHTGREY, SOLID, 1);
            }
        }
    }
    
    //Draw closed features next, in order from largest area to smallest
    for(auto i = feature_ids_by_areas.begin(); i != feature_ids_by_areas.end() ; i++){
        int feature_attri=feature_types[i -> second];

        if(i -> first != 0) 
        {
            //Park
            if(feature_attri==1){
                if(getZoomLevel() > 1.4)
                    drawFeatureArea(i -> second, t_color(197, 223, 158),SOLID);
            }
            //Beach
            else if(feature_attri==2){
                if(getZoomLevel() > 1.4)
                    drawFeatureArea(i -> second, t_color(250, 242, 199),SOLID);
            }
            //Lake
            else if(feature_attri==3)
                drawFeatureArea(i -> second, t_color(164, 198, 226),SOLID);
            //Island
            else if(feature_attri==5) 
                drawFeatureArea(i -> second, LIGHTGREY,SOLID);
            //Shoreline
            else if(feature_attri==6) 
                drawFeatureArea(i -> second, BISQUE,SOLID);
            //Building
            else if(feature_attri==7){ 
                if(getZoomLevel() > 7.8)
                    drawFeatureAreaB(i -> second, t_color(230, 202, 162),SOLID);
            }
            //Greenspace
            else if(feature_attri==8)
                drawFeatureArea(i -> second, t_color(147,216,104),SOLID);
            //River
            else if(feature_attri==4)
                drawFeatureLine(i -> second, t_color(164, 198, 226), SOLID, 3);
            //Stream
            else if(feature_attri==9){
                if(getZoomLevel() > 1.4)
                    drawFeatureLine(i -> second, t_color(164, 198, 226), SOLID, 1);
            }
            //Unknown
            else{         
                if(getZoomLevel() > 7.8)
                    drawFeatureArea(i -> second, LIGHTGREY,SOLID);
            }
        }
    }
}

/******************************************************************************/

/********************* DRAWING POINTS OF INTEREST *****************************/

//Draws a point of of interest on the map
void drawPosition (LatLon location, string type)
{
    t_point coord = LatLon_to_XY(location);
    if (type == "restaurant") {
        draw_surface(icon_restaurant, coord.x - 7, coord.y - 7);
    }
    else if (type == "education"){
        draw_surface(icon_education, coord.x - 7, coord.y - 7);      
    }
    else if (type == "healthcare"){
        draw_surface(icon_healthcare, coord.x - 7, coord.y - 7);      
    }
    else if (type == "busstop"){
        draw_surface(icon_busstop, coord.x - 7, coord.y - 7);      
    }
    else if (type == "post"){
        draw_surface(icon_post, coord.x - 7, coord.y - 7);      
    }
    else if (type == "worship"){
        draw_surface(icon_worship, coord.x - 7, coord.y - 7);      
    }
    else {
        draw_surface(icon_miscellaneous, coord.x - 7, coord.y - 7);      
    }
}

//Calculates the distance from the nearest point of interest
double find_distance_closest_point_of_interest(unsigned ID)
{
    LatLon location = getPointOfInterestPosition(ID);    
    unsigned closest = find_closest_point_of_interest(getPointOfInterestPosition(ID));
    
    LatLon otherslocation = getPointOfInterestPosition(closest);
    return find_distance_between_two_points(location, otherslocation);
    
}

//Draws all points of interest on the map
static void draw_all_POI(){
    
    settextrotation(0);
    for(unsigned i = 0; i < getNumberOfPointsOfInterest(); i++)
    {
        LatLon location = getPointOfInterestPosition(i);
        t_point mylocation=LatLon_to_XY(location);
       
        unsigned boundx, boundy;
        string type = poi_types[i];
        
        //bool onscreen = (std::find(show_poi_types.begin(), show_poi_types.end(), type) != show_poi_types.end());
        
        bool onscreen = true;
        
        //Check what type of POI we have and if it is currently being filtered
        if (type == "restaurant")
            onscreen = !filter_restaurant;
        else if (type == "healthcare")
            onscreen = !filter_healthcare;
        else if (type == "busstop")
            onscreen = !filter_busstop;
        else if (type == "education")
            onscreen = !filter_education;
        else if (type == "post")
            onscreen = !filter_post;
        else if (type == "worship")
            onscreen = !filter_worship;
        else
            onscreen = !filter_miscellaneous;
        
        //The areas in which different types of POI appear (by importance)
        if (type == "restaurant" || type == "healthcare" || type == "busstop") {
            boundx = 3640;
            boundy = 2315;
        }
        else if (type == "education" || type == "post" || type == "worship"){
            boundx = 2640;
            boundy = 1315;
        }
        else {
            boundx = 1640;
            boundy = 1315;
        }

        //Only the points of interest that are inside the screen will be presented, speeds up drawing time
        if(onscreen && screen_bound.intersects(mylocation)&&screen_bound.get_height()<=boundy&&screen_bound.get_width()<=boundx)
        {
            
            
            //create different layer for the name of point of interests
            if(getZoomLevel() >= 10)
            {
                drawPosition (location, type);
                setfontsize(0.9*getZoomLevel());
                if(getZoomLevel() >= 12 && (type == "post"))
                {
                    t_point p1 = LatLon_to_XY(location);
                    setcolor(BLACK);
                    drawtext(p1, getPointOfInterestName(i), 1000, 100);
                }
                else if(getZoomLevel() >= 14 && (type == "busstop")){
                    t_point p1 = LatLon_to_XY(location);
                    setcolor(BLACK);
                    drawtext(p1, getPointOfInterestName(i), 1000, 100);
                }
                else if(getZoomLevel() >= 16 && (type == "restaurant")){
                    t_point p1 = LatLon_to_XY(location);
                    setcolor(BLACK);
                    drawtext(p1, getPointOfInterestName(i), 1000, 100);
                }
                else if(getZoomLevel() >= 18 && (type == "healthcare")){
                    t_point p1 = LatLon_to_XY(location);
                    setcolor(BLACK);
                    drawtext(p1, getPointOfInterestName(i), 1000, 100);
                }
                else if(getZoomLevel() >= 20 && (type == "education" || type == "worship")){
                    t_point p1 = LatLon_to_XY(location);
                    setcolor(BLACK);
                    drawtext(p1, getPointOfInterestName(i), 1000, 100);
                }
                else if(getZoomLevel() >= 22){
                    t_point p1 = LatLon_to_XY(location);
                    setcolor(BLACK);
                    drawtext(p1, getPointOfInterestName(i), 1000, 100);
                }
            }
        }
    }
}

/******************************************************************************/

/***************************** DRAWING TEXT ***********************************/

//Draws a single street name on a segment portion
void draw_street_name(float x1, float y1, float x2, float y2, unsigned id, unsigned point) {
    float temp = 0;
    if (x1 > x2) {
        temp = x1;
        x1 = x2;
        x2 = temp;

        temp = y1;
        y1 = y2;
        y2 = temp;
    }
    
    unsigned longest_curve = street_segment_max_curve[id];
    string s_name = street_names[street_segment_info[id].streetID];
    t_color curColor = getcolor();
    
    if(getZoomLevel() > 5){
         if (s_name != "<unknown>" && point == longest_curve) {
                  int angle = angle_street_name(x1, y1, x2, y2);
                  setfontsize(1.2*getZoomLevel());
                  settextrotation(angle);
                  t_point mid = midpoint(XY_to_LatLon(t_point(x1,y1)), XY_to_LatLon(t_point(x2,y2)));
                  setcolor(BLACK);
                  float lengthSeg =  find_street_segment_length(id);
                  bool oneway = street_segment_info[id].oneWay;
                  if(oneway){
                           unsigned id1 = street_segment_info[id].from;
                           unsigned id2 = street_segment_info[id].to;

                           if(LatLon_to_XY(intersection_positions[id2]).x > LatLon_to_XY(intersection_positions[id1]).x)
                                    s_name = s_name + " → ";
                           else 
                                    s_name =  " ← " + s_name;   
                  }
                  drawtext(mid, s_name, lengthSeg, lengthSeg);
        }  
    }
     
    setcolor(curColor);
}

//Draws all the text labels on the map
static void draw_all_marks() {
    for (unsigned segment = 0; segment < street_segment_positions.size(); segment++){
        bool draw = true;
        
        string roadType = street_segment_roadtypes[segment];
        
        /*----------------------ZOOM ADJUSTMENTS------------------------------*/
        if (getZoomLevel() > 6.5){
            draw = true;
        }        
        else if (getZoomLevel() > 6 && getZoomLevel() < 6.5){
            draw = (roadType == "motorway" || roadType == "motorway_link" || roadType == "trunk"
                    || roadType == "trunk_link" || roadType == "primary" || roadType == "primary_link"
                    || roadType == "secondary" || roadType == "secondary_link"  || roadType == "tertiary" 
                    || roadType == "tertiary_link");
        }
        else if (getZoomLevel() > 5.8 && getZoomLevel() < 6){
            draw = (roadType == "motorway" || roadType == "motorway_link" || roadType == "trunk"
                    || roadType == "trunk_link" || roadType == "primary" || roadType == "primary_link"
                    || roadType == "secondary" || roadType == "secondary_link" );
        }
        else if (getZoomLevel() > 5.5 && getZoomLevel() < 5.8){
            draw = (roadType == "motorway" || roadType == "motorway_link" || roadType == "trunk"
                    || roadType == "trunk_link" || roadType == "primary" || roadType == "primary_link");
        }
        else{
            draw = (roadType == "motorway" || roadType == "motorway_link" || roadType == "trunk" || roadType == "trunk_link");
        }

        /*--------------------------------------------------------------------*/
        
        //Check to see if this is the street we must highlight
        bool highlighted = ((highlighted_street == street_names[street_segment_info[segment].streetID]) && in_street_state);
        
        //Now that we know if its a major street or not, we can draw it        
        if (draw || highlighted){
            for (unsigned point = 0; point < street_segment_positions[segment].size()-1; point++){
                //Get the endpoints of this street segment
                float x1 = street_segment_positions[segment][point].lon() * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat);
                float y1 = street_segment_positions[segment][point].lat() * DEG_TO_RAD;

                float x2 = street_segment_positions[segment][point+1].lon() * DEG_TO_RAD * std::cos(DEG_TO_RAD*AvgLat);
                float y2 = street_segment_positions[segment][point+1].lat() * DEG_TO_RAD;
      
                draw_street_name(x1 * SCOPE, y1 * SCOPE, x2 * SCOPE, y2 * SCOPE, segment, point);
            }
        }
    
    }
}

/******************************************************************************/

/********************** MISCELLANEOUS HELPER FUNCTIONS ************************/

//Holds all buttons until the map is loaded
void wait_for_load_map() {
    //disenable all buttons except load and exit
  //  for (int i = 0; i < BUTTON_NUM - 1; i++)
    //    enable_or_disable_button(i, ON);
}

//Calculates the angle a street is curved by
double angle_street_name(float x1, float y1, float x2, float y2){
    return atan2(y2-y1,x2-x1) * RAD_TO_DEG;
}

//Finding the middle of street segment ends 
t_point midpoint(LatLon l1, LatLon l2){
    LatLon middle((l2.lat() - l1.lat())*0.5 + l1.lat(), (l2.lon() - l1.lon())*0.5 + l1.lon());
    t_point p = LatLon_to_XY(middle);
    
    return p;
}

//Converts t_points to LatLon
LatLon XY_to_LatLon (t_point coords){
    float lon = coords.x / (SCOPE * DEG_TO_RAD * cos(AvgLat * DEG_TO_RAD));
    float lat = coords.y / (SCOPE * DEG_TO_RAD);
    
    return LatLon(lat, lon);
}

//Converts LatLon to t_points
t_point LatLon_to_XY (LatLon location)
{
    t_point result;
    result.x = location.lon() * DEG_TO_RAD * cos(AvgLat*DEG_TO_RAD) * SCOPE;
    result.y = location.lat() * DEG_TO_RAD * SCOPE;
    
    return result;
}

//Gets the current zoom level
double getZoomLevel(){
    
    t_bound_box currentWorld = get_visible_world();
    double oldHeight = max_world_bound.top() - max_world_bound.bottom();
    double currentHeight = currentWorld.get_height();
    return 0.8 * pow(oldHeight / currentHeight, 0.6);
    
}

//Prints out the distance from the last marker location
void calculate_distance() {
    if (in_directions_mode){
        if (firstDistance && in_path_state){
            
            cout << past_id << " to " << current_id << "\n";
                        
            //Here run the function to update directions_path vector
            direction_path = find_path_between_intersections(past_id, current_id, TURN_PENALTY);
            textbox_text_1 = "START: " + intersection_names[past_id];
            textbox_text_2 = "END: " + intersection_names[current_id];
            textbox_text_3 = "Time: " + std::to_string(compute_path_travel_time(direction_path, TURN_PENALTY)) + "s";
            textbox_onscreen = true;
            print_path();
            draw_path();
            display_textbox();
        }
    }
    else {
        textbox_text_1 = "";
        textbox_text_2 = intersection_names[current_id];
        textbox_text_3 = "";
        textbox_onscreen = true;
        display_textbox();
        copy_off_screen_buffer_to_screen();        
    }
}

/******************************************************************************/

/*************************** OSM DATA PROCESSING ******************************/

//Reads all OSM data into relevant structures
void osm_processing(){
    //Find the largest quantity
    unsigned noNodes = getNumberOfNodes();
    unsigned noWays = getNumberOfWays();
    unsigned max_quantity = 0;
    if (noNodes > noWays)
        max_quantity = noNodes;
    else
        max_quantity = noWays;
    
    for (unsigned i = 0; i < max_quantity; i++){
        if (i < noNodes){
            const OSMNode *o = getNodeByIndex(i);
            osm_nodes.insert(std::make_pair<OSMID, const OSMNode*>(o->id(), &*o));
        }
        if (i < noWays){
            const OSMWay *o = getWayByIndex(i);
            osm_ways.insert(std::make_pair<OSMID, const OSMWay*>(o->id(), &*o));           
        }
    }
}

//Cleans up OSM data structures
void osm_cleanup(){
    osm_nodes.clear();
    osm_ways.clear();
}


//Parses the OSM data on POI nodes & street segment ways so we can classify their types
void amenity_road_processing(){
    unsigned noPOI = getNumberOfPointsOfInterest();
    unsigned noSegs = getNumberOfStreetSegments();
    unsigned max_quantity = 0;
    if (noPOI > noSegs)
        max_quantity = noPOI;
    else
        max_quantity = noSegs;
    
    //Iterate through all types
    for (unsigned i = 0; i < max_quantity; i++){
        //Amenity processing
        if (i < noPOI){
            //Get the tag for what type of POI this is
            try {
                const OSMNode *n = osm_nodes.at(getPointOfInterestOSMNodeID(i));
                bool found = false;
                std::pair<std::string, std::string> designation;
                for (unsigned j = 0; j< getTagCount(n); j++){
                    std::pair<std::string, std::string> title = getTagPair(n, j);
                    if (title.first == "amenity"){
                        designation = title;
                        found = true;
                        break;
                    }            
                }

                //If it didn't have the tag, it's unknown
                string poiType;
                if (found) {
                    string type = designation.second;
                    if (type == "restaurant" || type == "bar" || type == "cafe" || type == "fast_food" ||
                            type == "food_court" || type == "pub") {
                        poiType = "restaurant";
                    }
                    else if (type == "college" || type == "kindergarten" || type == "school" ||
                            type == "music_school" || type == "driving_school" || type == "language_school" ||
                            type == "university" || type == "library"){
                        poiType = "education"; 
                    }
                    else if (type == "clinic" || type == "dentist" || type == "doctors" ||
                            type == "hospital" || type == "nursing_home" || type == "veterinary" ||
                            type == "pharmacy"){
                        poiType = "healthcare";
                    }
                    else if (type == "bus_station"){
                        poiType = "busstop";
                    }
                    else if (type == "post_box" || type == "post_office"){
                        poiType = "post";
                    }
                    else if (type == "place_of_worship"){
                        poiType = "worship";
                    }
                    else {
                        poiType = "miscellaneous";
                    }            
                }
                else
                    poiType = "unknown";
                poi_types.push_back(poiType);            
            }
            catch (std::out_of_range) {
                poi_types.push_back("unknown");   
            }            
        }
        
        //Road processing
        if (i < noSegs){
            try {
                //Get the tag for the type of street this is
                const OSMWay *w = osm_ways.at(street_segment_info[i].wayOSMID);
                bool found = false;
                std::pair<std::string, std::string> designation;
                for (unsigned j = 0; j < getTagCount(w); j++){
                    std::pair<std::string, std::string> title = getTagPair(w, j);
                    if (title.first == "highway"){
                        designation = title;
                        found = true;
                        break;
                    }
                }

                //If it didn't have the tag, it's unknown
                string roadType;
                if (found)
                    roadType = designation.second;
                else
                    roadType = "unknown";
                street_segment_roadtypes.push_back(roadType);            
            }
            catch (std::out_of_range){
                street_segment_roadtypes.push_back("unknown");                        
            }            

        }
    }
    
}

/******************************************************************************/


