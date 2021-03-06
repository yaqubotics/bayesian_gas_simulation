#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <gaden_player/GasPosition.h>
#include <gaden_player/WindPosition.h>

#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
// CLASS for every simulation to run. If two gas sources are needed, just create 2 instances!

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <bits/stdc++.h>

#include "nav_msgs/OccupancyGrid.h"

using namespace std;
using namespace cv;

class sim_obj
{
public:
    sim_obj(std::string filepath, bool load_wind_info);
    ~sim_obj();

    std::string     gas_type;
    std::string     simulation_filename;
    int             environment_cells_x, environment_cells_y, environment_cells_z;
    double          environment_cell_size;
    double          source_pos_x, source_pos_y, source_pos_z;
    double          env_min_x;              //[m]
    double          env_max_x;              //[m]
    double          env_min_y;              //[m]
    double          env_max_y;              //[m]
    double          env_min_z;              //[m]
    double          env_max_z;              //[m]
    bool            load_wind_data;
    std::vector<std::vector<std::vector<double> > > C;  //3D Gas concentration
    std::vector<std::vector<std::vector<double> > > U;  //3D Wind U
    std::vector<std::vector<std::vector<double> > > V;  //3D Wind V
    std::vector<std::vector<std::vector<double> > > W;  //3D Wind W
    bool            first_reading;

    //methods
    void configure_environment();
    void load_data_from_logfile(int sim_iteration);
    void read_csv_file(int sim_iteration);
    void get_gas_concentration(float x, float y, float z, std::string &gas_name, double &gas_conc);
    void get_wind_value(float x, float y, float z, double &u, double &v, double &w);
    void get_concentration_as_markers(visualization_msgs::Marker &mkr_points);
    void display_pdf_gas_distribution();
};



// ----------------------  MAIN--------------------//

// Parameters
double                          player_freq;
int                             num_simulators;
bool                            verbose;
std::vector<std::string>        simulation_data;
std::vector<sim_obj>            player_instances;          //To handle N simulations at a time.
std::vector<std::string>        srv_response_gas_types;
std::vector<double>             srv_response_gas_concs;
int                             initial_iteration, loop_from_iteration, loop_to_iteration;
bool                            allow_looping;
std::string                     occupancy3D_data;

int map_width;
int map_height;
std::vector<std::vector<int>> map_data;
bool map_received = false;

//Visualization
ros::Publisher                  marker_pub;
ros::Publisher                  marker_pdf_pub;
ros::Publisher                  time_marker_pub;
ros::Publisher                  iteration_truth_pub;
visualization_msgs::Marker      mkr_gas_points;                  //We will create an array of particles according to cell concentration
visualization_msgs::Marker      mkr_pdf_gas_points;                  //We will create an array of particles according to cell concentration
std_msgs::Int32                 iteration_truth;                  //We will create an array of particles according to cell concentration
//functions:
void loadNodeParameters(ros::NodeHandle private_nh);
void init_all_simulation_instances();
void load_all_data_from_logfiles(int sim_iteration);
void read_csv_files(int sim_iteration);
void display_current_gas_distribution();



