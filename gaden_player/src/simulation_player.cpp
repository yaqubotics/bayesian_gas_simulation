/*--------------------------------------------------------------------------------
 * Pkg for playing the simulation results of the "filament_simulator" pkg.
 * It allows to run on real time, and provide services to simulated sensors (gas, wind)
 * It supports loading several simulations at a time, which allows multiple gas sources and gas types
 * It also generates a point cloud representing the gas concentration [ppm] on the 3D environment
 --------------------------------------------------------------------------------*/

#include "simulation_player.h"

void map_cb(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if(!map_received)
    {
        cout << msg->data[0] << "\n";
        std_msgs::Header header = msg->header;
        nav_msgs::MapMetaData info = msg->info;
        ROS_INFO("Got map %d %d", info.width, info.height);
        vector<vector<int>> local_map_data;
        map_width = info.width;
        map_height = info.height;
        
        for(int i=0;i<map_width;i++)
        {
            vector<int> temp_vector;
            for(int j=0;j<map_height;j++)
            {
                temp_vector.push_back(msg->data[i+map_width*j]);
            }
            local_map_data.push_back(temp_vector);
        }
        map_data = local_map_data;
        map_received = true;
    }
}

//--------------- SERVICES CALLBACKS----------------------//
bool get_gas_value_srv(gaden_player::GasPosition::Request  &req, gaden_player::GasPosition::Response &res)
{
    //ROS_INFO("[Player] Request for gas concentration at location [%.2f, %.2f, %.2f]m",req.x, req.y, req.z);

    //Get all gas concentrations and gas types (from all instances)
    for (int i=0;i<num_simulators; i++)
        player_instances[i].get_gas_concentration(req.x, req.y, req.z, srv_response_gas_types[i], srv_response_gas_concs[i]);
    //std::cout << "get_gas_value_srv 1\n";
    //Return gas concentration for each gas_type.
    //If we have multiple instances with the same gas, then add their concentrations.
    std::map<std::string,double> mymap;
    for (int i=0;i<num_simulators; i++)
    {
        if (mymap.find(srv_response_gas_types[i]) == mymap.end() )
            mymap[srv_response_gas_types[i]] = srv_response_gas_concs[i];
        else
            mymap[srv_response_gas_types[i]] += srv_response_gas_concs[i];
    }

    //std::cout << "get_gas_value_srv 2\n";
    //Configure Response
    res.gas_conc.clear();
    res.gas_type.clear();
    //std::cout << "get_gas_value_srv 3\n";
    for (std::map<std::string,double>::iterator it=mymap.begin(); it!=mymap.end(); ++it)
    {
        res.gas_type.push_back(it->first);
        res.gas_conc.push_back(it->second);
    }
    //std::cout << "get_gas_value_srv 4\n";

    return true;
}


bool get_wind_value_srv(gaden_player::WindPosition::Request  &req, gaden_player::WindPosition::Response &res)
{
    //std::cout<< "get wind 0\n";
    //Since the wind fields are identical among different instances, return just the information from instance[0]
    player_instances[0].get_wind_value(req.x, req.y, req.z, res.u, res.v, res.w);
    //std::cout<< "get wind 1\n";
    return true;
}

#include <sstream>

string ZeroPadNumber(int num)
{
    stringstream ss;
    
    // the number is converted to string with the help of stringstream
    ss << num; 
    string ret;
    ss >> ret;
    
    // Append zero chars
    int str_length = ret.length();
    for (int i = 0; i < 2 - str_length; i++)
        ret = "0" + ret;
    return ret;
}

//------------------------ MAIN --------------------------//

int main( int argc, char** argv )
{
    ros::init(argc, argv, "simulation_player");
	ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //Read Node Parameters
    loadNodeParameters(pn);	
	
    //Publishers
    marker_pub = n.advertise<visualization_msgs::Marker>("Gas_Distribution", 1);
    marker_pdf_pub = n.advertise<visualization_msgs::Marker>("Gas_PDF_Distribution", 1);
    iteration_truth_pub = n.advertise<std_msgs::Int32>("iteration_truth", 1);

    //Services offered
    ros::ServiceServer serviceGas = n.advertiseService("odor_value", get_gas_value_srv);
    ros::ServiceServer serviceWind = n.advertiseService("wind_value", get_wind_value_srv);
    ros::Subscriber sub_map = n.subscribe("/map", 1, map_cb);

    while(!map_received) ros::spinOnce();

    //Init variables        
    init_all_simulation_instances();
    ros::Time time_last_loaded_file = ros::Time::now();
    srand(time(NULL));// initialize random seed
    ros::Duration(20).sleep();

    //Init Markers for RVIZ visualization
    mkr_gas_points.header.frame_id = "/map";
    mkr_gas_points.header.stamp = ros::Time::now();
    mkr_gas_points.ns = "Gas_Dispersion";
    mkr_gas_points.action = visualization_msgs::Marker::ADD;
    mkr_gas_points.type = visualization_msgs::Marker::POINTS;   //Marker type
    mkr_gas_points.id = 0;                                      //One marker with multiple points.
    mkr_gas_points.scale.x = 0.025;//0.3;//0.1
    mkr_gas_points.scale.y = 0.025;//0.3;//0.1
    mkr_gas_points.scale.z = 0.025;//0.3;//0.1
    mkr_gas_points.pose.orientation.w = 1.0;

    //Init Markers for RVIZ visualization
    mkr_pdf_gas_points.header.frame_id = "/map";
    mkr_pdf_gas_points.header.stamp = ros::Time::now();
    mkr_pdf_gas_points.ns = "Gas_Dispersion_2";
    mkr_pdf_gas_points.action = visualization_msgs::Marker::ADD;
    mkr_pdf_gas_points.type = visualization_msgs::Marker::POINTS;   //Marker type
    mkr_pdf_gas_points.id = 0;                                      //One marker with multiple points.
    mkr_pdf_gas_points.scale.x = 0.2;//0.3;//0.1
    mkr_pdf_gas_points.scale.y = 0.2;//0.3;//0.1
    mkr_pdf_gas_points.scale.z = 0.2;//0.3;//0.1
    mkr_pdf_gas_points.pose.orientation.w = 1.0;
    // Loop	
    ros::Rate r(100); //Set max rate at 100Hz (for handling services - Top Speed!!)
    int iteration_counter = initial_iteration;


    time_marker_pub = n.advertise<visualization_msgs::Marker>("time_marker", 1);

    visualization_msgs::Marker time_marker;
    time_marker.header.frame_id = "/map";
    time_marker.header.stamp = ros::Time::now();
    time_marker.ns = "basic_shapes";
    time_marker.id = 1;
    time_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    time_marker.action = visualization_msgs::Marker::ADD;

    time_marker.pose.position.x = -8.0;
    time_marker.pose.position.y = 15.0;
    time_marker.pose.position.z = 1.0;
    time_marker.pose.orientation.x = 0.0;
    time_marker.pose.orientation.y = 0.0;
    time_marker.pose.orientation.z = 0.0;
    time_marker.pose.orientation.w = 1.0;

    time_marker.text = "blablabla";

    time_marker.scale.x = 3.0;
    time_marker.scale.y = 3.0;
    time_marker.scale.z = 1.0;

    time_marker.color.r = 0.0f;
    time_marker.color.g = 0.0f;
    time_marker.color.b = 0.0f;
    time_marker.color.a = 1.0;
    std::cout << "Start Gaden Player\n";
    while (ros::ok())
    {        
        //std::cout << "test\n";
        if( (ros::Time::now() - time_last_loaded_file).toSec() >= 1/player_freq )
        {
            //time_marker.id++;
            time_marker.text ="Time: "+ZeroPadNumber(0)+":"+ZeroPadNumber((iteration_counter+1)/60)+":"+ZeroPadNumber((iteration_counter+1)%60);
            time_marker_pub.publish(time_marker);
            if (verbose)
                ROS_INFO("[Player] Playing simulation iteration %i", iteration_counter);
            iteration_truth.data = iteration_counter;
            iteration_truth_pub.publish(iteration_truth);
            //Read Gas and Wind data from log_files
            ros::Time time_begin_load = ros::Time::now();
            //load_all_data_from_logfiles(iteration_counter); //On the first time, we configure gas type, source pos, etc.
            //std::cout << "Start reading csv\n";
            read_csv_files(iteration_counter);
            //std::cout << "Reading csv DONE\n";
            
            ros::Time time_begin_display = ros::Time::now();

            //display_current_gas_distribution();    //Rviz visualization
            
            //std::cout << "Display pdf gas distribution Start\n";
            player_instances[0].display_pdf_gas_distribution();
            //std::cout << "Display pdf gas distribution Done\n";
            if(verbose)ROS_INFO("time load: %f s, display: %f s",(ros::Time::now()-time_begin_load).toSec(),(ros::Time::now()-time_begin_display).toSec());
            iteration_counter++;

            //Looping?
            if (allow_looping)
            {
               if (iteration_counter >= loop_to_iteration)
               {
                   iteration_counter = loop_from_iteration;
                   if (verbose)
                       ROS_INFO("[Player] Looping");
               }
            }
            time_last_loaded_file = ros::Time::now();
            //std::cout << "End of iteration\n";
        }

        //Attend service request at max rate!
        //This allows sensors to have higher sampling rates than the simulation update
        ros::spinOnce();
        //std::cout << "test 1\n";

        r.sleep();
        //std::cout << "test 2\n";

    }
}


//Load Node parameters
void loadNodeParameters(ros::NodeHandle private_nh)
{
    //player_freq
    private_nh.param<bool>("verbose", verbose, false);
    
    //player_freq
    private_nh.param<double>("player_freq", player_freq, 1);  //Hz

    //Number of simulators to load (For simulating multiple gases and multiple sources)
    private_nh.param<int>("num_simulators", num_simulators, 1);

    if (verbose)
    {
        ROS_INFO("[Player] player_freq %.2f", player_freq);
        ROS_INFO("[Player] num_simulators:  %i", num_simulators);
    }


    //FilePath for simulated data
    simulation_data.resize(num_simulators);
    for (int i=0;i<num_simulators; i++)
    {
        //Get location of simulation data for instance (i)
        std::string paramName = boost::str( boost::format("simulation_data_%i") % i);
        private_nh.param<std::string>(paramName.c_str(),simulation_data[i], "");
        if (verbose)
            ROS_INFO("[Player] simulation_data_%i:  %s", i, simulation_data[i].c_str());
    }
    
    // Initial iteration
    private_nh.param<int>("initial_iteration", initial_iteration, 1);
    // Loop
    private_nh.param<bool>("allow_looping", allow_looping, false);
    private_nh.param<int>("loop_from_iteration", loop_from_iteration, 1);
    private_nh.param<int>("loop_to_iteration", loop_to_iteration, 1);
    private_nh.param<std::string>("occupancy3D_data", occupancy3D_data, "");

}


//Init
void init_all_simulation_instances()
{
    ROS_INFO("[Player] Initializing %i instances",num_simulators);

    // At least one instance is needed which loads the wind field data!
    sim_obj so(simulation_data[0], true);
    player_instances.push_back(so);

    //Create other instances, but do not save wind information! It is the same for all instances
    for (int i=1;i<num_simulators;i++)
    {
        sim_obj so(simulation_data[i], false);
        player_instances.push_back(so);
    }

    //Set size for service responses
    srv_response_gas_types.resize(num_simulators);
    srv_response_gas_concs.resize(num_simulators);
}


//Load new Iteration of the Gas&Wind State on the 3d environment
void load_all_data_from_logfiles(int sim_iteration)
{    
    //Load corresponding data for each instance (i.e for every gas source)
    for (int i=0;i<num_simulators;i++)
    {
        if (verbose)
            ROS_INFO("[Player] Loading new data to instance %i (iteration %i)",i,sim_iteration);
        player_instances[i].load_data_from_logfile(sim_iteration);
    }
}

void read_csv_files(int sim_iteration)
{    
    //Load corresponding data for each instance (i.e for every gas source)
    for (int i=0;i<num_simulators;i++)
    {
        if (verbose)
            ROS_INFO("[Player] Loading new data to instance %i (iteration %i)",i,sim_iteration);
        player_instances[i].read_csv_file(sim_iteration);
    }
}

//Display in RVIZ the gas distribution
void display_current_gas_distribution()
{
    //Remove previous data points
    mkr_gas_points.points.clear();
    mkr_gas_points.colors.clear();
    for (int i=0;i<num_simulators;i++)
    {
        player_instances[i].get_concentration_as_markers(mkr_gas_points);
    }
    //Display particles
    marker_pub.publish(mkr_gas_points);
}

//Display in RVIZ the gas distribution
void sim_obj::display_pdf_gas_distribution()
{
    //Remove previous data points
    visualization_msgs::Marker      mkr_pdf_gas_points;
    mkr_pdf_gas_points.header.frame_id = "/map";
    mkr_pdf_gas_points.header.stamp = ros::Time::now();
    mkr_pdf_gas_points.ns = "Gas_Dispersion_2";
    mkr_pdf_gas_points.action = visualization_msgs::Marker::ADD;
    mkr_pdf_gas_points.type = visualization_msgs::Marker::POINTS;   //Marker type
    mkr_pdf_gas_points.id = 0;                                      //One marker with multiple points.
    mkr_pdf_gas_points.scale.x = environment_cell_size;//0.3;//0.1
    mkr_pdf_gas_points.scale.y = environment_cell_size;//0.3;//0.1
    mkr_pdf_gas_points.scale.z = 0.1;//0.3;//0.1
    mkr_pdf_gas_points.pose.orientation.w = 1.0;
    mkr_pdf_gas_points.points.clear();
    mkr_pdf_gas_points.colors.clear();
    //std::cout << "display marker pdf\n";
    //std::cout << environment_cells_x <<" "<< environment_cells_y <<" "<< environment_cells_z <<" "<< environment_cell_size <<" "<<env_min_x<<" "<<env_min_y<<" "<<std::endl;
    //std::cout << env_min_x << std::endl;
    //std::cout << env_min_y << std::endl;
    //float C_norm[environment_cells_x][environment_cells_y];
    float max = 0;
    float logbase = 100000;
    int z=0; // sensor altitude
    for(int i=0;i<environment_cells_x;i++)
    {
        for(int j=0;j<environment_cells_y;j++)
        {
            if(map_data[i][j] == 0)
            {
                if(max < log(C[i][j][z]+1)/log(logbase))
                    max = log(C[i][j][z]+1)/log(logbase);
            }
        }
    }
    for(int i=0;i<environment_cells_x;i++)
    {
        for(int j=0;j<environment_cells_y;j++)
        {
            if(map_data[i][j] == 0)
            {
                int zz;
                zz = (int)ceil((0.5 - env_min_z)/environment_cell_size);
        
                //Get gas concentration from that cell
                //yy = 111;
                //std::cout << i << " " << j << std::endl;
                //std::cout << xx << " " << yy << " " << zz << " " << environment_cell_size << " " << env_max_y << std::endl;
                double gas_conc;

                gas_conc = (log(C[i][j][z]+1)/log(logbase))/max;
                if(C[i][j][z] == 0)
                    gas_conc = 0.0;
                geometry_msgs::Point p; //Location of point
                std_msgs::ColorRGBA color;  //Color of point

                p.x = env_min_x + (i+0.5)*environment_cell_size; //+ ((rand()%100)/100.0f)*environment_cell_size;
                p.y = env_min_y + (j+0.5)*environment_cell_size; //+ ((rand()%100)/100.0f)*environment_cell_size;
                p.z = 0.0; //env_min_z + (zz+0.5)*environment_cell_size; //+ ((rand()%100)/100.0f)*environment_cell_size;;

                // color map jet
                if((gas_conc >= 0.75) && (gas_conc <= 1.0))
                {
                    color.r = 1;
                    color.g = -4*gas_conc+4;
                    color.b = 0.0;
                    color.a = 1.0;
                    //std::cout << "gas value:" << gas_conc << p.x << p.y << std::endl; 
                }
                else if((gas_conc >= 0.5) && (gas_conc < 0.75))
                {
                    color.r = 4*gas_conc-2;
                    color.g = 1;
                    color.b = 0;
                    color.a = 1.0;
                }
                else if((gas_conc >= 0.25) && (gas_conc < 0.5))
                {
                    color.r = 0;
                    color.g = 1;
                    color.b = -4*gas_conc+2;
                    color.a = 1.0;
                }
                else
                {
                    color.r = 0;
                    color.g = 4*gas_conc;
                    color.b = 1;
                    color.a = 1.0;
                }
                mkr_pdf_gas_points.points.push_back(p);
                mkr_pdf_gas_points.colors.push_back(color);
                mkr_pdf_gas_points.id++;
            }
        }
    }
    //Display particles
    marker_pdf_pub.publish(mkr_pdf_gas_points);
}
//==================================== SIM_OBJ ==============================//


// Constructor
sim_obj::sim_obj(std::string filepath, bool load_wind_info)
{
    gas_type = "unknown";
    simulation_filename = filepath;
    environment_cells_x = environment_cells_y = environment_cells_z = 0;
    environment_cell_size = 0.0; //m
    source_pos_x = source_pos_y = source_pos_z = 0.0; //m
    load_wind_data = load_wind_info;
    first_reading = true;

    std::ifstream infile(occupancy3D_data.c_str());
    std::string line;
    //Line 1 (min values of environment)
    std::getline(infile, line);
    size_t pos = line.find(" ");
    line.erase(0, pos+1);
    pos = line.find(" ");
    env_min_x = atof(line.substr(0, pos).c_str());
    line.erase(0, pos+1);
    pos = line.find(" ");
    env_min_y = atof(line.substr(0, pos).c_str());
    env_min_z = atof(line.substr(pos+1).c_str());


    //Line 2 (max values of environment)
    std::getline(infile, line);
    pos = line.find(" ");
    line.erase(0, pos+1);
    pos = line.find(" ");
    env_max_x = atof(line.substr(0, pos).c_str());
    line.erase(0, pos+1);
    pos = line.find(" ");
    env_max_y = atof(line.substr(0, pos).c_str());
    env_max_z = atof(line.substr(pos+1).c_str());

    std::getline(infile, line);
    pos = line.find(" ");
    line.erase(0, pos+1);
    pos = line.find(" ");
    environment_cells_x = atoi(line.substr(0, pos).c_str());
    line.erase(0, pos+1);
    pos = line.find(" ");
    environment_cells_y = atof(line.substr(0, pos).c_str());
    environment_cells_z = atof(line.substr(pos+1).c_str());

    std::getline(infile, line);
    pos = line.find(" ");
    environment_cell_size = atof(line.substr(pos+1).c_str());
    //std::cout << environment_cells_x << " " << environment_cells_y << " " << environment_cells_z << std::endl;
    configure_environment();
}

sim_obj::~sim_obj(){}

void sim_obj::read_csv_file(int sim_iteration)
{
    
    std::string line;
    std::fstream fin;
    std::string filename = boost::str( boost::format("%s%4i.csv") % simulation_filename.c_str() % sim_iteration);
    std::cout << filename << endl;
    fin.open(filename,ios::in);
    //std::cout<< "open DONE\n";
    string word;
    int idx_x = 0;
    int idx_y = 0;
    while(std::getline(fin,line))
    {
        
        //std::cout<< "getline 1 DONE\n";

        stringstream strstream(line.substr(0,line.size()));
        //std::cout<< "stringstream DONE "<<line<<"\n";
        idx_y = 0;
        while(std::getline(strstream,word,' '))
        {
            //std::cout<< "inside while "<<word<<" "<<idx_x<<" "<<idx_y<<"\n";
            C[idx_x][idx_y][0] = std::stod(word);
            //std::cout<< "done stod\n";
            //if(C[idx_x][idx_y][0] > 0)
            //    std::cout << C[idx_x][idx_y][0] << endl;
            idx_y++;
        }
        idx_x++;
    }
    fin.close();
    //std::cout<< "close DONE\n";

}

//Load a new file with Gas+Wind data
void sim_obj::load_data_from_logfile(int sim_iteration)
{
    std::string line;
    int line_counter = 0;
    std::string filename = boost::str( boost::format("%s%i") % simulation_filename.c_str() % sim_iteration);

    //Open file
    if (FILE *file = fopen(filename.c_str(), "r"))
    {
        //File exists!, keep going!
        fclose(file);
    }else{
        std::cout<< "File " << filename << " does not exist\n";
    }

    std::ifstream infile(filename.c_str(), std::ios_base::binary);
    boost::iostreams::filtering_istream inbuf;
    inbuf.push(boost::iostreams::zlib_decompressor());
    inbuf.push(infile);
    size_t pos;
    double conc, u, v, w;
    int x, y, z;

    while (std::getline(inbuf, line))
    {
        line_counter++;
        //ROS_INFO("Reading Line %i", line_counter);
        //ROS_INFO("%s",line.c_str());

        if (first_reading && (line_counter == 1))
        {
            //Line 1 (min values of environment)
            size_t pos = line.find(" ");
            line.erase(0, pos + 1);
            pos = line.find(" ");
            env_min_x = atof(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);
            pos = line.find(" ");
            env_min_y = atof(line.substr(0, pos).c_str());
            env_min_z = atof(line.substr(pos + 1).c_str());
        }
        else if (first_reading && (line_counter == 2))
        {
            //Line 2 (max values of environment)
            pos = line.find(" ");
            line.erase(0, pos + 1);
            pos = line.find(" ");
            env_max_x = atof(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);
            pos = line.find(" ");
            env_max_y = atof(line.substr(0, pos).c_str());
            env_max_z = atof(line.substr(pos + 1).c_str());
        }
        else if (first_reading && (line_counter == 3))
        {
            //Get Number of cells (X,Y,Z)
            pos = line.find(" ");
            line.erase(0, pos + 1);

            pos = line.find(" ");
            environment_cells_x = atoi(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);
            pos = line.find(" ");
            environment_cells_y = atoi(line.substr(0, pos).c_str());
            environment_cells_z = atoi(line.substr(pos + 1).c_str());
        }
        else if (first_reading && (line_counter == 4))
        {
            //Get Cell_size
            pos = line.find(" ");
            line.erase(0, pos + 1);

            pos = line.find(" ");
            environment_cell_size = atof(line.substr(0, pos).c_str());
        }
        else if (first_reading && (line_counter == 5))
        {
            //Get GasSourceLocation
            pos = line.find(" ");
            line.erase(0, pos + 1);

            pos = line.find(" ");
            source_pos_x = atof(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);

            pos = line.find(" ");
            source_pos_y = atof(line.substr(0, pos).c_str());
            source_pos_z = atof(line.substr(pos + 1).c_str());
        }
        else if (first_reading && (line_counter == 6))
        {
            //Get Gas_Type
            pos = line.find(" ");
            gas_type = line.substr(pos + 1);
            //Configure instances
            configure_environment();
        }
        else if (line_counter > 7)
        {
            //A line has the format x y z conc u v w
            pos = line.find(" ");
            x = atoi(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);

            pos = line.find(" ");
            y = atoi(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);

            pos = line.find(" ");
            z = atoi(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);

            pos = line.find(" ");
            conc = atof(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);

            pos = line.find(" ");
            u = atof(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);

            pos = line.find(" ");
            v = atof(line.substr(0, pos).c_str());
            w = atof(line.substr(pos + 1).c_str());

            //Save data to internal storage
            C[x][y][z] = conc / 1000;
            if (load_wind_data)
            {
                U[x][y][z] = u / 1000;
                V[x][y][z] = v / 1000;
                W[x][y][z] = w / 1000;
            }
        }
    }
    infile.close();
    if (first_reading)
        first_reading = false;
}


//Get Gas concentration at lcoation (x,y,z)
void sim_obj::get_gas_concentration(float x, float y, float z, std::string &gas_name, double &gas_conc)
{
    //Get cell idx from point location
    int xx,yy,zz;
    xx = (int)ceil((x - env_min_x)/environment_cell_size);
    yy = (int)ceil((y - env_min_y)/environment_cell_size);
    zz = (int)ceil((z - env_min_z)/environment_cell_size);

    //Get gas concentration from that cell
    gas_conc = C[xx][yy][zz];
    gas_name = gas_type;
}
/*
void sim_obj::get_2D_gas_map()
{
    for(float i=0;i<map_size_x;i=i+0.2)
    {
        for(float j=0;j<map_size_y;j=j+0.2)
        {
            int xx,yy,zz;
            xx = (int)ceil((x - env_min_x)/environment_cell_size);
            yy = (int)ceil((y - env_min_y)/environment_cell_size);
            zz = (int)ceil((z - env_min_z)/environment_cell_size);

            
        }
    }
}
*/
//Get Wind concentration at lcoation (x,y,z)
void sim_obj::get_wind_value(float x, float y, float z, double &u, double &v, double &w)
{
    if (load_wind_data)
    {
        //Get cell idx from point location
        int xx,yy,zz;
        xx = (int)ceil((x - env_min_x)/environment_cell_size);
        yy = (int)ceil((y - env_min_y)/environment_cell_size);
        zz = (int)ceil((z - env_min_z)/environment_cell_size);

        //Set wind vectors from that cell
        u = U[xx][yy][zz];
        v = V[xx][yy][zz];
        w = W[xx][yy][zz];
    }
    else
    {
        if (verbose)
            ROS_WARN("[Plyer] Request to provide Wind information when No Wind data is available!!");
    }
}


//Init instances (for running multiple simulations)
void sim_obj::configure_environment()
{
    //ROS_INFO("Configuring Enviroment");
    //ROS_INFO("\t\t Dimension: [%i,%i,%i] cells", environment_cells_x, environment_cells_y, environment_cells_z);
    //ROS_INFO("\t\t Gas_Type: %s", gas_type.c_str());
    //ROS_INFO("\t\t FilePath: %s", simulation_filename.c_str());
    //ROS_INFO("\t\t Source at location: [%.4f,%.4f,%.4f][m] ", source_pos_x, source_pos_y, source_pos_z);
    //ROS_INFO("\t\t Loading Wind Data: %i", load_wind_data);


    //Resize Gas Concentration container
    C.resize(environment_cells_x);
    for (int i = 0; i < environment_cells_x; ++i)
    {
        C[i].resize(environment_cells_y);
        for (int j = 0; j < environment_cells_y; ++j)
        {
            C[i][j].resize(environment_cells_z);
        }
    }

    //Resize Wind info container (if necessary)
    if (load_wind_data)
    {

        U.resize(environment_cells_x);
        for (int i = 0; i < environment_cells_x; ++i)
        {
            U[i].resize(environment_cells_y);
            for (int j = 0; j < environment_cells_y; ++j)
            {
                U[i][j].resize(environment_cells_z);
            }
        }

        V.resize(environment_cells_x);
        for (int i = 0; i < environment_cells_x; ++i)
        {
            V[i].resize(environment_cells_y);
            for (int j = 0; j < environment_cells_y; ++j)
            {
                V[i][j].resize(environment_cells_z);
            }
        }

        W.resize(environment_cells_x);
        for (int i = 0; i < environment_cells_x; ++i)
        {
            W[i].resize(environment_cells_y);
            for (int j = 0; j < environment_cells_y; ++j)
            {
                W[i][j].resize(environment_cells_z);
            }
        }
    }
}


void sim_obj::get_concentration_as_markers(visualization_msgs::Marker &mkr_points)
{
    //For every cell, generate as much "marker points" as [ppm]
    for (int i=0;i<environment_cells_x;i++)
    {
        for (int j=0;j<environment_cells_y;j++)
        {
            for (int k=0;k<environment_cells_z;k++)
            {
                geometry_msgs::Point p; //Location of point
                std_msgs::ColorRGBA color;  //Color of point

                double gas_value = C[i][j][k]*1;

                for (int N=0;N<(int)ceil(gas_value/2000);N++)
                {
                    //Set point position (corner of the cell + random)
                    p.x = env_min_x + (i+0.5)*environment_cell_size + ((rand()%100)/100.0f)*environment_cell_size;
                    p.y = env_min_y + (j+0.5)*environment_cell_size + ((rand()%100)/100.0f)*environment_cell_size;
                    p.z = env_min_z + (k+0.5)*environment_cell_size + ((rand()%100)/100.0f)*environment_cell_size;
                    //std::cout << "Molecule "<<N<<" coordinate" << std::endl;
                    //std::cout << "Concentration "<< gas_value << std::endl;
                    //std::cout << p.x << "," << p.y << "," << p.z << std::endl;
                    //Set color of particle according to gas type
                    color.a = 1.0;
                    if (!strcmp(gas_type.c_str(),"ethanol"))
                    {
                        color.r=0.2; color.g=0.9; color.b=0;
                    }
                    else if (!strcmp(gas_type.c_str(),"methane"))
                    {
                        color.r=0.9; color.g=0.1; color.b=0.1;
                    }
                    else if (!strcmp(gas_type.c_str(),"hydrogen"))
                    {
                        color.r=0.2; color.g=0.1; color.b=0.9;
                    }
                    else if (!strcmp(gas_type.c_str(),"propanol"))
                    {
                        color.r=0.8; color.g=0.8; color.b=0;
                    }
                    else if (!strcmp(gas_type.c_str(),"chlorine"))
                    {
                        color.r=0.8; color.g=0; color.b=0.8;
                    }
                    else if (!strcmp(gas_type.c_str(),"flurorine"))
                    {
                        color.r=0.0; color.g=0.8; color.b=0.8;
                    }
                    else if (!strcmp(gas_type.c_str(),"acetone"))
                    {
                        color.r=0.9; color.g=0.2; color.b=0.2;
                    }
                    else if (!strcmp(gas_type.c_str(),"neon"))
                    {
                        color.r=0.9; color.g=0; color.b=0;
                    }
                    else if (!strcmp(gas_type.c_str(),"helium"))
                    {
                        color.r=0.9; color.g=0; color.b=0;
                    }
                    else if (!strcmp(gas_type.c_str(),"hot_air"))
                    {
                        color.r=0.9; color.g=0; color.b=0;
                    }
                    else
                    {
                        //ROS_INFO("[player] Setting Default Color");
                        color.r = 0.9; color.g = 0;color.b = 0;
                    }

                    //Add particle marker
                    mkr_points.points.push_back(p);
                    mkr_points.colors.push_back(color);
                }

            }
        }
    }

}


