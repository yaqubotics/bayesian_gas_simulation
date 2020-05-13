/*---------------------------------------------------------------------------------------
 * MAIN Node for the simulation of gas dispersal using a Filament-based approach.
 * This node loads the wind field (usually from CFD simulation), and simulates over it
 * different filaments to spread gas particles.
 *
 * Each filament is composed of a fixed number of gas molecules (Q)
 * Each filament is determined by its center position and width.
 * The width of a filament increases over time (Turbulent and molecular difussion)
 * The position of a filament is updated with the wind.
 *
 * The gas concentration at a given point is the sum of the concentration of all filaments.
 *
 * Thus, the gas concentration at the source location is determined by the number of molecules/filament and the number of filaments.
 *
 * A log file is recorded for every snapshot (time-step) with information about the gas
 * concentration and wind vector for every cell (3D) of the environment.
 *
 * The node implements the filament-base gas dispersal simulation. At each time step, the puffs
 * of filaments are sequentially released at a source location. Each puff is composed of n filaments.
 * Filaments are affected by turbulence and molecular diffusion along its path while being transported
 * by advection with the wind. The 3-dimensional positions of these filaments are represented by the points
 * of the “visualization msgs/markers”. At each time step, “Dispersal_Simulation” node calculates or
 * determines the positions of n filaments. Gas plumes are simulated with or without acceleration.
 *
 * It is very time consuming, and currently it runs in just one thread (designed to run offline).
 *
 * TODO: Cambiar std::vector por std::list para los FILAMENTOS
 ---------------------------------------------------------------------------------------*/


#include "filament_simulator/filament_simulator.h"


//==========================//
//      Constructor         //
//==========================//
CFilamentSimulator::CFilamentSimulator()
{
	//Read parameters
	loadNodeParameters();

    //Create directory to save results (if needed)
    //if (save_results && !boost::filesystem::exists(results_location))
    //   if (!boost::filesystem::create_directories(results_location))
    //       ROS_ERROR("[filament] Could not create result directory: %s", results_location.c_str());

	//Set Publishers and Subscribers
	//-------------------------------
	marker_pub = n.advertise<visualization_msgs::Marker>("filament_visualization", 1);

    // Wait preprocessing Node to finish?
    preprocessing_done = false;
    if(wait_preprocessing)
    {
        prepro_sub = n.subscribe("preprocessing_done", 1, &CFilamentSimulator::preprocessingCB, this);
        while(ros::ok() && !preprocessing_done)
        {
            ros::Duration(0.5).sleep();
            ros::spinOnce();
            if (verbose) ROS_INFO("[filament] Waiting for node GADEN_preprocessing to end.");
        }
    }

	//Init variables
	//-----------------
    sim_time = 0.0;                         //Start at time = 0(sec)
    sim_time_last_wind = -2*windTime_step;  //Force to load wind-data on startup
    current_wind_snapshot = 0;              //Start with wind_iter= 0;
    current_simulation_step = 0;            //Start with iter= 0;
	//last_saved_step = -1;
	last_saved_step = results_min_time/results_time_step;
    wind_notified = false;               //To warm the user (only once) that no more wind data is found!

	//Init the Simulator
	initSimulator();


	//Fluid Dynamics Eq
	/*/-----------------
	* Ideal gas equation:
	* PV = nRT
	* P is the pressure of the gas (atm)
	* V is the volume of the gas (cm^3)
    * n is the amount of substance of gas (mol) = m/M where m=mass of the gas [g] and M is the molar mass
	* R is the ideal, or universal, gas constant, equal to the product of the Boltzmann constant and the Avogadro constant. (82.057338 cm^3·atm/mol·k)
	* T is the temperature of the gas (kelvin)
	*/
	double R = 82.057338;												   //[cm³·atm/mol·K] Gas Constant
	filament_initial_vol = pow(6*filament_initial_std,3);				   //[cm³] -> We approximate the infinite volumen of the 3DGaussian as 6 sigmas.
	env_cell_vol = pow(cell_size*100,3);									//[cm³] Volumen of a cell
	filament_numMoles = (envPressure*filament_initial_vol)/(R*envTemperature);//[mol] Num of moles of Air in that volume
	env_cell_numMoles = (envPressure*env_cell_vol)/(R*envTemperature);		//[mol] Num of moles of Air in that volume


	//The moles of target_gas in a Filament are distributted following a 3D Gaussian
	//Given the ppm value at the center of the filament, we approximate the total number of gas moles in that filament.
	filament_numMoles_of_gas = ( sqrt(8*pow(3.14159,3))*pow(filament_initial_std,3) ) * filament_ppm_center/pow(10,6);   //[cm³]
	double numMoles_in_cm3 = envPressure/(R*envTemperature);   //[mol/cm³]
	filament_numMoles_of_gas = filament_numMoles_of_gas * numMoles_in_cm3; //[moles_target_gas/filament] This is a CTE parameter!!

    if (verbose) ROS_INFO("[filament] filament_initial_vol [cm3]: %f",filament_initial_vol);
    if (verbose) ROS_INFO("[filament] env_cell_vol [cm3]: %f",env_cell_vol);
    if (verbose) ROS_INFO("[filament] filament_numMoles [mol]: %E",filament_numMoles);
    if (verbose) ROS_INFO("[filament] env_cell_numMoles [mol]: %E",env_cell_numMoles);
    if (verbose) ROS_INFO("[filament] filament_numMoles_of_gas [mol]: %E",filament_numMoles_of_gas);

	// Molecular gas mass [g/mol]
	// air : 28.97 g/mol
	// ethanol : 30.07 g/mol
	// methane : 16.04 g/mol
	// SpecificGravity(Air) = 1 (as reference)
	// Specific gravity is the ratio of the density of a substance to the density of a reference substance; equivalently,
	// it is the ratio of the mass of a substance to the mass of a reference substance for the same given volume.
	float air_weight = 28.97;
	SpecificGravity[0] = 1.0378;	  //ethanol   (heavier than air)
    SpecificGravity[1] = 0.5537;	  //methane   (lighter than air)
	SpecificGravity[2] = 0.0696;	  //hydrogen  (lighter than air)
	SpecificGravity[6] = 1.4529;	  //acetone   (heavier than air)

	//To be updated
	SpecificGravity[3] = 58.124;	 //propanol   //gases heavier then air
	SpecificGravity[4] = 70.906;	 //chlorine
	SpecificGravity[5] = 37.996;	 //fluorine
	SpecificGravity[7] = 20.179;	 //neon	   //gases lighter than air
	SpecificGravity[8] = 4.002602;   //helium
	SpecificGravity[9] = 26.966;	 //hot_air
	SpecificGravity[10] = 44.0095/air_weight; // co2


	//Init visualization
	//-------------------
	filament_marker.header.frame_id = fixed_frame;
	filament_marker.ns = "filaments";
	filament_marker.action = visualization_msgs::Marker::ADD;
	filament_marker.id = 0;
	filament_marker.type = visualization_msgs::Marker::POINTS;
	filament_marker.color.a = 1;
	
	
}



CFilamentSimulator::~CFilamentSimulator()
{
}


//==============================//
//      GADEN_preprocessing CB  //
//==============================//
void CFilamentSimulator::preprocessingCB(const std_msgs::Bool& b)
{
    preprocessing_done = true;
}


//==========================//
//      Load Params         //
//==========================//
void CFilamentSimulator::loadNodeParameters()
{
	ros::NodeHandle private_nh("~");
    
    // Verbose
    private_nh.param<bool>("verbose", verbose, false);
    private_nh.param<bool>("debug", debug, false);

    // Wait PreProcessing
    private_nh.param<bool>("wait_preprocessing", wait_preprocessing, false);

    // Simulation Time (sec)
	private_nh.param<double>("sim_time", max_sim_time, 20.0);
	private_nh.param<double>("disperse_time", disperse_time, 20.0);
    // Time increment between Gas snapshots (sec)
	private_nh.param<double>("time_step", time_step, 1.0);

    // Number of iterations to carry on = max_sim_time/time_step
	numSteps = floor(max_sim_time/time_step);
	numDisperseSteps = floor(disperse_time/time_step);

    // Num of filaments/sec
	private_nh.param<int>("num_filaments_sec", numFilaments_sec, 100);
    private_nh.param<bool>("variable_rate", variable_rate, false);
	numFilaments_step = floor(numFilaments_sec * time_step);
	total_number_filaments = numFilaments_step * numSteps;

    // Gas concentration at the filament center - 3D gaussian [ppm]
	private_nh.param<double>("ppm_filament_center", filament_ppm_center, 20);

    // [cm] Sigma of the filament at t=0-> 3DGaussian shape
	private_nh.param<double>("filament_initial_std", filament_initial_std, 1.5);

    // [cm²/s] Growth ratio of the filament_std
	private_nh.param<double>("filament_growth_gamma", filament_growth_gamma, 10.0);

    // [cm] Sigma of the white noise added on each iteration
	private_nh.param<double>("filament_noise_std_small", filament_noise_std_small, 0.1);
	private_nh.param<double>("filament_noise_std_big", filament_noise_std_big, 0.1);

	private_nh.param<double>("filament_color_r", filament_color_r, 0);
	private_nh.param<double>("filament_color_g", filament_color_g, 1);
	private_nh.param<double>("filament_color_b", filament_color_b, 0);

    // Gas Type ID
	private_nh.param<int>("gas_type", gasType, 1);

	// Environment temperature (necessary for molecules/cm3 -> ppm)
	private_nh.param<double>("temperature", envTemperature, 298.0);

	// Enviorment pressure (necessary for molecules/cm3 -> ppm)
	private_nh.param<double>("pressure", envPressure, 1.0);

	// Gas concentration units (0= molecules/cm3,  1=ppm)
	private_nh.param<int>("concentration_unit_choice", gasConc_unit, 1);



	//WIND DATA
	//----------
	//CFD wind files location
	private_nh.param<std::string>("wind_data", wind_files_location, "");
	//(sec) Time increment between Wind snapshots --> Determines when to load a new wind field
	private_nh.param<double>("wind_time_step", windTime_step, 1.0);
	private_nh.param<bool>("zero_wind", zero_wind, false);
    // Loop
    private_nh.param<bool>("allow_looping", allow_looping, false);
    private_nh.param<int>("loop_from_step", loop_from_step, 1);
    private_nh.param<int>("loop_to_step", loop_to_step, 100);



	//ENVIRONMENT
	//-----------
	private_nh.param<bool>("random_flag", random_flag, false);

	// Occupancy gridmap 3D location
	private_nh.param<std::string>("occupancy3D_data", occupancy3D_data, "");
	if(random_flag)
		private_nh.param<std::string>("occupancy3D_rand_data", occupancy3D_rand_data, "");

	//fixed frame (to disaply the gas particles on RVIZ)
	private_nh.param<std::string>("fixed_frame", fixed_frame, "/map");

	//Source postion (x,y,z)
	private_nh.param<double>("source_position_x", gas_source_pos_x, 1.0);
	private_nh.param<double>("source_position_y", gas_source_pos_y, 1.0);
	private_nh.param<double>("source_position_z", gas_source_pos_z, 1.0);

	// Simulation results.
	private_nh.param<int>("save_results", save_results, 1);
	private_nh.param<std::string>("results_location", results_location, "");
	private_nh.param<std::string>("csv_file_name", csv_file_name, "");
    private_nh.param<double>("results_min_time", results_min_time, 0.0);
    private_nh.param<double>("results_time_step", results_time_step, 1.0);


    private_nh.param<int>("save_csv_every_iteration",save_csv_every_iteration,15);

    if (verbose)
    {
        ROS_INFO("[filament] The data provided in the roslaunch file is:");
        ROS_INFO("[filament] Simulation Time        %f(s)",sim_time);
        ROS_INFO("[filament] Gas Time Step:         %f(s)",time_step);
        ROS_INFO("[filament] Num_steps:             %d",numSteps);
        ROS_INFO("[filament] Number of filaments:   %d",numFilaments_sec);
        ROS_INFO("[filament] PPM filament center    %f",filament_ppm_center);
        ROS_INFO("[filament] Gas type:              %d",gasType);
        ROS_INFO("[filament] Concentration unit:    %d",gasConc_unit);
        ROS_INFO("[filament] Wind_time_step:        %f(s)", windTime_step);
        ROS_INFO("[filament] Fixed frame:           %s",fixed_frame.c_str());
        ROS_INFO("[filament] Source position:       (%f,%f,%f)",gas_source_pos_x, gas_source_pos_y, gas_source_pos_z);

        if (save_results)
            ROS_INFO("[filament] Saving results to %s",results_location.c_str());
    }
}



//==========================//
//                          //
//==========================//
void CFilamentSimulator::initSimulator()
{
    if (verbose) ROS_INFO("[filament] Initializing Simulator... Please Wait!");

	//1. Load Environment and Configure Matrices
	if (FILE *file = fopen(occupancy3D_data.c_str(), "r"))
	{
		//Files exist!, keep going!
		fclose(file);
        if (verbose) ROS_INFO("[filament] Loading 3D Occupancy GridMap");
		read_3D_file(occupancy3D_data, Env, true);
		if(random_flag) read_3D_file(occupancy3D_rand_data, Env_rand, true);
	}
	else
	{
		ROS_ERROR("[filament] File %s Does Not Exists!",occupancy3D_data.c_str());
	}


	//2. Load the first Wind snapshot from file (all 3 components U,V,W)
	read_wind_snapshot(current_simulation_step);

	//3. Initialize the filaments vector to its max value (to avoid increasing the size at runtime)
    if (verbose) ROS_INFO("[filament] Initializing Filaments");
	filaments.resize(total_number_filaments, CFilament(0.0, 0.0, 0.0, filament_initial_std));
}



//Resize a 3D Matrix compose of Vectors, This operation is only performed once!
void CFilamentSimulator::configure3DMatrix(std::vector< std::vector< std::vector<double> > > &A)
{
	A.resize(env_cells_x);
	for (int i = 0; i < env_cells_x; ++i)
	{
		A[i].resize(env_cells_y);
		for (int j = 0; j < env_cells_y; ++j)
		{
			A[i][j].resize(env_cells_z);
		}
	}
}



//==========================//
//                          //
//==========================//
void CFilamentSimulator::read_wind_snapshot(int idx)
{

	//configure filenames to read
	std::string U_filemane = boost::str( boost::format("%s%i.csv_U") % wind_files_location % idx );
	std::string V_filemane = boost::str( boost::format("%s%i.csv_V") % wind_files_location % idx );
	std::string W_filemane = boost::str( boost::format("%s%i.csv_W") % wind_files_location % idx );

    if (verbose) ROS_INFO("Reading Wind Snapshot %s",U_filemane.c_str());

    //read data to 3D matrices
	if (FILE *file = fopen(U_filemane.c_str(), "r"))
	{
		//Files exist!, keep going!
		fclose(file);
        if (verbose) ROS_INFO("[filament] Loading Wind Snapshot %i", idx);

		read_3D_file(U_filemane, U, false);
		read_3D_file(V_filemane, V, false);
		read_3D_file(W_filemane, W, false);
	}
	else
	{
		//No more wind data. Keep current info.
		if (!wind_notified)
		{
			ROS_WARN("[filament] File %s Does Not Exists!",U_filemane.c_str());
			ROS_WARN("[filament] No more wind data available. Using last Wind snapshopt as SteadyState.");
			wind_notified = true;
		}
    }
}




//==========================//
//                          //
//==========================//
void CFilamentSimulator::read_3D_file(std::string filename, std::vector< std::vector< std::vector<double> > > &A, bool hasHeader=false)
{
	//open file
	std::ifstream infile(filename.c_str());
	std::string line;
	int line_counter = 0;

	//If header -> read 4 Header lines & configure all matrices to given dimensions!
	if (hasHeader)
	{
		//Line 1 (min values of environment)
		std::getline(infile, line);
		line_counter++;
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
		line_counter++;
		pos = line.find(" ");
		line.erase(0, pos+1);
		pos = line.find(" ");
		env_max_x = atof(line.substr(0, pos).c_str());
		line.erase(0, pos+1);
		pos = line.find(" ");
		env_max_y = atof(line.substr(0, pos).c_str());
		env_max_z = atof(line.substr(pos+1).c_str());

		//Line 3 (Num cells on eahc dimension)
		std::getline(infile, line);
		line_counter++;
		pos = line.find(" ");
		line.erase(0, pos+1);
		pos = line.find(" ");
		env_cells_x = atoi(line.substr(0, pos).c_str());
		line.erase(0, pos+1);
		pos = line.find(" ");
		env_cells_y = atof(line.substr(0, pos).c_str());
		env_cells_z = atof(line.substr(pos+1).c_str());

		//Line 4 cell_size (m)
		std::getline(infile, line);
		line_counter++;
		pos = line.find(" ");
		cell_size = atof(line.substr(pos+1).c_str());

        if (verbose) ROS_INFO("[filament] Env dimensions (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)",env_min_x, env_min_y, env_min_z, env_max_x, env_max_y, env_max_z );
        if (verbose) ROS_INFO("[filament] Env size in cells	 (%d,%d,%d) - with cell size %f [m]",env_cells_x,env_cells_y,env_cells_z, cell_size);

		//Reserve memory for the 3D matrices: U,V,W,C and Env, according to provided num_cells of the environment.
		//It also init them to 0.0 values
		configure3DMatrix(U);
		configure3DMatrix(V);
		configure3DMatrix(W);
		configure3DMatrix(C);
		configure3DMatrix(Env);
		if(random_flag) configure3DMatrix(Env_rand);
	}

    if (verbose) ROS_INFO("Reading File to (%lu,%lu,%lu)Matrix", A.size(), A[0].size(), A[0][0].size());

	//Read file line by line
	int x_idx = 0;
	int y_idx = 0;
	int z_idx = 0;

	while ( std::getline(infile, line) )
	{
		line_counter++;
		std::stringstream ss(line);
		if (z_idx >=env_cells_z)
		{
			ROS_ERROR("Trying to read:[%s]",line.c_str());
		}

		if (line == ";")
		{
			//New Z-layer
			z_idx++;
			x_idx = 0;
			y_idx = 0;
		}
		else
		{   //New line with constant x_idx and all the y_idx values
			while (!ss.fail())
			{
				double f;
				ss >> f;		//get one double value
				if (!ss.fail())
				{
					if(!zero_wind)
						A[x_idx][y_idx][z_idx] = f;
					else
						A[x_idx][y_idx][z_idx] = 0.0;//static_cast <float> (rand()) / static_cast <float> (RAND_MAX) -0.5;
					y_idx++;
				}
			}

			//Line has ended
			x_idx++;
			y_idx = 0;
		}
	}
    //End of file.
    if (verbose) ROS_INFO("End of File");
}



// Add new filaments. On each step add a total of "numFilaments_step"
void CFilamentSimulator::add_new_filaments(double radius_arround_source)
{
    // Release rate
    int filaments_to_release;
    if (variable_rate)
    {
        filaments_to_release = (int) round( random_number(0.0, numFilaments_step) );
    }
    else
    {
        filaments_to_release = numFilaments_step;
    }

    for (int i=0; i<filaments_to_release; i++)
	{
		//Set position of new filament within the especified radius arround the gas source location
		double x = gas_source_pos_x + random_number(-1,1)*radius_arround_source;
		double y = gas_source_pos_y + random_number(-1,1)*radius_arround_source;
		double z = gas_source_pos_z + random_number(-1,1)*radius_arround_source;

		/*Instead of adding new filaments to the filaments vector on each iteration (push_back)
		  we had initially resized the filaments vector to the max number of filaments (numSteps*numFilaments_step)
		  Here we will "activate" just the corresponding filaments for this step.*/
        filaments[current_simulation_step*numFilaments_step+i].activate_filament(x, y, z, sim_time);
	}
}


// Here we estimate the gas concentration on each cell of the 3D env
// based on the active filaments and their 3DGaussian shapes
// For that we employ Farrell's Concentration Eq
void CFilamentSimulator::update_gas_concentration_from_filament(int fil_i)
{
	// We run over all the active filaments, and update the gas concentration of the cells that are close to them.
	// Ideally a filament spreads over the entire environment, but in practice since filaments are modeled as 3Dgaussians
	// We can stablish a cutt_off raduis of 3*sigma.
    // To avoid resolution problems, we evaluate each filament according to the minimum between:
    // the env_cell_size and filament_sigma. This way we ensure a filament is always well evaluated (not only one point).
	
			double grid_size_m = std::min(cell_size, (filaments[fil_i].sigma/100));  //[m] grid size to evaluate the filament
					// Compute at which increments the Filament has to be evaluated.
					// If the sigma of the Filament is very big (i.e. the Filament is very flat), the use the world's cell_size.
					// If the Filament is very small (i.e in only spans one or few world cells), then use increments equal to sigma
					//  in order to have several evaluations fall in the same cell.

			int num_evaluations = ceil(6*(filaments[fil_i].sigma/100) / grid_size_m);
					// How many times the Filament has to be evaluated depends on the final grid_size_m.
					// The filament's grid size is multiplied by 6 because we evaluate it over +-3 sigma
					// If the filament is very small (i.e. grid_size_m = sigma), then the filament is evaluated only 6 times
					// If the filament is very big and spans several cells, then it has to be evaluated for each cell (which will be more than 6)


			// EVALUATE IN ALL THREE AXIS
			for (int i=0; i<=num_evaluations; i++)
			{
				for (int j=0; j<=num_evaluations; j++)
				{
					for (int k=0; k<=num_evaluations; k++)
					{
						//get point to evaluate [m]
						double x = (filaments[fil_i].pose_x - 3*(filaments[fil_i].sigma/100)) + i*grid_size_m;
						double y = (filaments[fil_i].pose_y - 3*(filaments[fil_i].sigma/100)) + j*grid_size_m;
						double z = (filaments[fil_i].pose_z - 3*(filaments[fil_i].sigma/100)) + k*grid_size_m;

						// Disntance from evaluated_point to filament_center (in [cm])
						double distance_cm = 100 * sqrt( pow(x-filaments[fil_i].pose_x,2) + pow(y-filaments[fil_i].pose_y,2) + pow(z-filaments[fil_i].pose_z,2) );

						// FARRELLS Eq.
                        //Evaluate the concentration of filament fil_i at given point (moles/cm³)
						double num_moles_cm3 = (filament_numMoles_of_gas / (sqrt(8*pow(3.14159,3)) * pow(filaments[fil_i].sigma,3) )) * exp( -pow(distance_cm,2)/(2*pow(filaments[fil_i].sigma,2)) );

						//Multiply for the volumen of the grid cell
						double num_moles = num_moles_cm3 * pow(grid_size_m*100,3);  //[moles]


						//Valid point? If either OUT of the environment, or through a wall, treat it as invalid
						bool path_is_obstructed =  check_environment_for_obstacle(filaments[fil_i].pose_x, filaments[fil_i].pose_y, filaments[fil_i].pose_z, x,y,z );

                        if (!path_is_obstructed)
                        {
                            if (path_is_obstructed)
                            {
                                //Point is not valid! Instead of ignoring it, add its concentration to the filament center location
                                //This avoids "loosing" gas concentration as filaments get close to obstacles (e.g. the floor)
                                x = filaments[fil_i].pose_x;
                                y = filaments[fil_i].pose_y;
                                z = filaments[fil_i].pose_z;
                            }

                            //Get 3D cell of the evaluated point
                            int x_idx = floor( (x-env_min_x)/cell_size );
                            int y_idx = floor( (y-env_min_y)/cell_size );
                            int z_idx = floor( (z-env_min_z)/cell_size );
							
                            //Accumulate concentration in corresponding env_cell
                            if (gasConc_unit==0)
							{
								mtx.lock();
                                C[x_idx][y_idx][z_idx] += num_moles;	//moles
								mtx.unlock();
							}
                            else
                            {
								mtx.lock();
                                double num_ppm = (num_moles/env_cell_numMoles)*pow(10,6);   //[ppm]
                                C[x_idx][y_idx][z_idx] += num_ppm;	//ppm
                                //ROS_INFO("ppm concentraton at [%i %i %i]: %f", x_idx,y_idx,z_idx,num_ppm);
								mtx.unlock();
                            }
                        }
					}
				}
	}

	//Update Gasconcentration markers
}

void CFilamentSimulator::write_to_csv()
{
	//std::string out_foldername = boost::str( boost::format("%s/PDF_gasType_%i_sourcePose_%.2f_%.2f_%.2f_iter_%i.csv") % csv_file_name % gasType % gas_source_pos_x % gas_source_pos_y % gas_source_pos_z % last_saved_step);

	std::string out_foldername = boost::str( boost::format("%s_sourcePose_%.2f_%.2f_%.2f") % csv_file_name % gas_source_pos_x % gas_source_pos_y % gas_source_pos_z);
	std::string out_filename = boost::str( boost::format("%s/PDF_gasType_%i_iter_%4.4i.csv") % out_foldername % gasType % last_saved_step);
	//std::ofstream myfile(out_filename);
	const char* cstr = out_foldername.c_str(); // cstr points to: "This is a string.\0"
	
	mkdir(cstr,0777);
	std::ofstream myfile;
	ROS_INFO("save_csv at-%d",last_saved_step);
	//std::cout << cstr << std::endl;
    myfile.open (out_filename);
    /* // OLD FORMAT
    myfile <<"cell_x,cell_y,cell_z,pose_x,pose_y,pose_z,conc,\n";
	for (size_t i=0; i<env_cells_x; i++)
	{
		for (size_t j=0; j<env_cells_y; j++)
		{
			myfile << i << ",";
			myfile << j << ",";
			myfile << 2 << ",";
			myfile << i*cell_size << ",";
			myfile << j*cell_size << ",";
			myfile << 0.6 << ",";
			myfile << C[i][j][2] << ","; // concentration
			myfile << "\n";
			
		}
	}*/
    // NEW_FORMAT
	for (size_t i=0; i<env_cells_x; i++)
	{
		for (size_t j=0; j<env_cells_y; j++)
		{
			myfile << C[i][j][0] << " "; // concentration
		}
		myfile << "\n";
	}
	myfile.close();
	last_saved_step++;

}


//==========================//
//                          //
//==========================//
void CFilamentSimulator::update_gas_concentration_from_filaments(){
	//First, set all cells to 0.0 gas concentration (clear previous state)
	#pragma omp parallel for collapse(3)
	for (size_t i=0; i<env_cells_x; i++)
	{
		for (size_t j=0; j<env_cells_y; j++)
		{
			for (size_t k=0; k<env_cells_z; k++)
			{
				C[i][j][k] = 0.0;
			}
		}
	}

	#pragma omp parallel for
	for (int i = 0; i < current_simulation_step * numFilaments_step; i++)
	{
		if (filaments[i].valid)
		{
			update_gas_concentration_from_filament(i);
		}
	}
}

//Check if a given 3D pose falls in:
// 0 = free space
// 1 = obstacle, wall, or outside the environment
// 2 = outlet (usefull to disable filaments)
int CFilamentSimulator::check_pose_with_environment(double pose_x, double pose_y, double pose_z)
{
	//1.1 Check that pose is within the boundingbox environment
	if (pose_x<env_min_x || pose_x>env_max_x || pose_y<env_min_y || pose_y>env_max_y || pose_z<env_min_z || pose_z>env_max_z)
		return 1;

	//Get 3D cell of the point
	int x_idx = floor( (pose_x-env_min_x)/cell_size );
	int y_idx = floor( (pose_y-env_min_y)/cell_size );
	int z_idx = floor( (pose_z-env_min_z)/cell_size );

	if (x_idx >= env_cells_x || y_idx >= env_cells_y || z_idx >= env_cells_z)
		return 1;

	//1.2. Return cell occupancy (0=free, 1=obstacle, 2=outlet)
	return Env[x_idx][y_idx][z_idx];
}


//==========================//
//                          //
//==========================//
bool CFilamentSimulator::check_environment_for_obstacle(double start_x, double start_y, double start_z,
													   double   end_x, double   end_y, double end_z)
{
	const bool PATH_OBSTRUCTED = true;
	const bool PATH_UNOBSTRUCTED = false;


	// Check whether one of the points is outside the valid environment or is not free
	if(check_pose_with_environment(start_x, start_y, start_z) != 0)   { return PATH_OBSTRUCTED; }
	if(check_pose_with_environment(  end_x, end_y  ,   end_z) != 0)   { return PATH_OBSTRUCTED; }


	// Calculate normal displacement vector
	double vector_x = end_x - start_x;
	double vector_y = end_y - start_y;
	double vector_z = end_z - start_z;
	double distance = sqrt(vector_x*vector_x + vector_y*vector_y + vector_z*vector_z);
	vector_x = vector_x/distance;
	vector_y = vector_y/distance;
	vector_z = vector_z/distance;


	// Traverse path
	int steps = ceil( distance / cell_size );	// Make sure no two iteration steps are separated more than 1 cell
	double increment = distance/steps;

	for(int i=1; i<steps-1; i++)
	{
		// Determine point in space to evaluate
		double pose_x = start_x + vector_x*increment*i;
		double pose_y = start_y + vector_y*increment*i;
		double pose_z = start_z + vector_z*increment*i;


		// Determine cell to evaluate (some cells might get evaluated twice due to the current code
		int x_idx = floor( (pose_x-env_min_x)/cell_size );
		int y_idx = floor( (pose_y-env_min_y)/cell_size );
		int z_idx = floor( (pose_z-env_min_z)/cell_size );


		// Check if the cell is occupied
		if(Env[x_idx][y_idx][z_idx] != 0) { return PATH_OBSTRUCTED; }
	}

	// Direct line of sight confirmed!
	return PATH_UNOBSTRUCTED;
}

bool CFilamentSimulator::check_collision(float a, float b, float c, float d)
{


    int x0 = floor( (a-env_min_x)/cell_size );
	int y0 = floor( (b-env_min_y)/cell_size );
	int x1 = floor( (c-env_min_y)/cell_size );
	int y1 = floor( (d-env_min_y)/cell_size );
    
    int dy=y1-y0;
    int dx=x1-x0;
    int f=0;
    int sx,sy;
    if(dy < 0)
    {
        dy=-dy;
        sy=-1;
    }
    else
    {
        sy=1;
    }
    
    if(dx < 0)
    {
        dx=-dx;
        sx=-1;
    }
    else
    {
        sx=1;
    }

    if(dx >= dy)
    {
        while (x0 != x1)
        {
            f=f+dy;
            if(f >= dx)
            {
                if (Env[x0+((sx-1)/2)][y0+((sy-1)/2)][1] != 0)
                    return false;
                y0 = y0+sy;
                f = f-dx;
            }
            if ((f != 0) && (Env[x0+((sx-1)/2)][y0+((sy-1)/2)][1] != 0))
                return false;
            if ((dy == 0) && (Env[x0+((sx-1)/2)][y0][1] != 0) && (Env[x0+((sx-1)/2)][y0-1][1] != 0))
                return false;
            x0=x0+sx;
        }
    }
    else
    {
        while (y0 != y1)
        {
            f=f+dx;
            if(f >= dy)
            {
                if (Env[x0+((sx-1)/2)][y0+((sy-1)/2)][1] != 0)
                    return false;
                x0 = x0+sx;
                f = f-dy;
            }
            if ((f != 0) && (Env[x0+((sx-1)/2)][y0+((sy-1)/2)][1] != 0))
                return false;
            if ((dx == 0) && (Env[x0][y0+((sy-1)/2)][1] != 0) && (Env[x0-1][y0+((sy-1)/2)][1] != 0))
                return false;
            y0=y0+sy;
        }
    }
	
    return true;
}

//Update the filaments location in the 3D environment
// According to Farrell Filament model, a filament is afected by three components of the wind flow.
// 1. Va (large scale wind) -> Advection (Va) -> Movement of a filament as a whole by wind) -> from CFD
// 2. Vm (middle scale wind)-> Movement of the filament with respect the center of the "plume" -> modeled as white noise
// 3. Vd (small scale wind) -> Difussion or change of the filament shape (growth with time)
// We also consider Gravity and Bouyant Forces given the gas molecular mass
void CFilamentSimulator::update_filament_location(int i)
{
	//Estimte filament acceleration due to gravity & Bouyant force (for the given gas_type):
	double g = 9.8;
	double specific_gravity_air = 1; //[dimensionless]
	double accel = g * ( specific_gravity_air - SpecificGravity[gasType] ) / SpecificGravity[gasType];
	double newpos_x, newpos_y, newpos_z, noise_std;
	mtx.lock();
	CFilament filament= filaments[i];
	mtx.unlock();
	//Update the location of all active filaments
	//ROS_INFO("[filament] Updating %i filaments of %lu",current_simulation_step*numFilaments_step, filaments.size());

			try
			{
				//Get 3D cell of the filament center
				int x_idx = floor( (filament.pose_x-env_min_x)/cell_size );
				int y_idx = floor( (filament.pose_y-env_min_y)/cell_size );
				int z_idx = floor( (filament.pose_z-env_min_z)/cell_size );

				//1. Simulate Advection (Va)
				//   Large scale wind-eddies -> Movement of a filament as a whole by wind
				//------------------------------------------------------------------------
				newpos_x = filament.pose_x + U[x_idx][y_idx][z_idx] * time_step;
				newpos_y = filament.pose_y + V[x_idx][y_idx][z_idx] * time_step;
				newpos_z = filament.pose_z + W[x_idx][y_idx][z_idx] * time_step;
				if(debug)
				{
					std::cout << "fil pose " << filament.pose_x << " " << filament.pose_y << " " << filament.pose_z << "\n";
					std::cout << "wind vel " << U[x_idx][y_idx][z_idx] << " " << V[x_idx][y_idx][z_idx] << " " << W[x_idx][y_idx][z_idx] << "\n";
				}
				//Check filament location
				int valid_location = check_pose_with_environment(newpos_x, newpos_y, newpos_z);
				switch (valid_location)
				{
				case 0:
					//Free and valid location... update filament position
					mtx.lock();
					filaments[i].pose_x = newpos_x;
					filaments[i].pose_y = newpos_y;
					filaments[i].pose_z = newpos_z;
					mtx.unlock();
					break;
				case 2:
					//The location corresponds to an outlet! Delete filament!
					mtx.lock();
					filaments[i].valid = false;
					mtx.unlock();
					break;
				default:
					//The location falls in an obstacle -> Illegal movement (Do not apply advection)
					break;
				}

				//2. Simulate Gravity & Bouyant Force
				//------------------------------------
                //OLD approach: using accelerations (pure gas)
                //newpos_z = filaments[i].pose_z + 0.5*accel*pow(time_step,2);

                //Approximation from "Terminal Velocity of a Bubble Rise in a Liquid Column", World Academy of Science, Engineering and Technology 28 2007
                double ro_air = 1.205;     //[kg/m³] density of air
                double mu = 19*pow(10,-6);    //[kg/s·m] dynamic viscosity of air
                double terminal_buoyancy_velocity = (g * (1-SpecificGravity[gasType])*ro_air * filament_ppm_center*pow(10,-6) ) / (18* mu);
                newpos_z = filaments[i].pose_z + terminal_buoyancy_velocity*time_step;

				mtx.lock();
				CFilament filament= filaments[i];
				mtx.unlock();
				//Check filament location
				if (check_pose_with_environment(filament.pose_x, filament.pose_y, newpos_z ) == 0){
					mtx.lock();
					filaments[i].pose_z = newpos_z;
					mtx.unlock();	
				}


				//3. Add some variability (stochastic process)
				//   Vm (middle scale wind)-> Movement of the filament with respect the center of the "plume" -> modeled as Gaussian white noise
				//----------------------------------------------------------------

				x_idx = floor( (filament.pose_x-env_min_x)/cell_size );
				y_idx = floor( (filament.pose_y-env_min_y)/cell_size );
				z_idx = floor( (filament.pose_z-env_min_z)/cell_size );

				if((random_flag) && (Env_rand[x_idx][y_idx][z_idx] == 2)) //(1=big, 2=small)
					noise_std = filament_noise_std_small;// * time_step;
				else
					noise_std = filament_noise_std_big;// * time_step;

				static RandomGenerator rng(static_cast<unsigned> (time(0)));
				NormalDistribution gaussian_dist(0.0,noise_std);
				GaussianGenerator generator(rng, gaussian_dist);
				mtx.lock();
				float oldpos_x = filaments[i].pose_x;
				float oldpos_y = filaments[i].pose_y;
				newpos_x = filaments[i].pose_x + generator();
				newpos_y = filaments[i].pose_y + generator();
				newpos_z = filaments[i].pose_z + generator();

				//Check filament location
				if (check_pose_with_environment(newpos_x, newpos_y, newpos_z ) == 0)
				{
					if(check_collision(newpos_x, newpos_y, oldpos_x, oldpos_y))
					{
						filaments[i].pose_x = newpos_x;
						filaments[i].pose_y = newpos_y;
						filaments[i].pose_z = newpos_z;
					}
				}




				//4. Filament growth with time (this affects the posterior estimation of gas concentration at each cell)
				//   Vd (small scale wind eddies) -> Difussion or change of the filament shape (growth with time)
				//   R = sigma of a 3D gaussian -> Increasing sigma with time
				//------------------------------------------------------------------------
                filaments[i].sigma = sqrt(pow(filament_initial_std,2) + filament_growth_gamma*(sim_time-filaments[i].birth_time));
				if(verbose)ROS_INFO("Filaments-%d sigma = %f",i,filaments[i].sigma);
				mtx.unlock();
			}catch(...)
			{
				ROS_ERROR("Exception Updating Filaments!");
				return;
			}
}


//==========================//
//                          //
//==========================//
void CFilamentSimulator::update_filaments_location()
{
	#pragma omp parallel for
	for (int i = 0; i < current_simulation_step * numFilaments_step; i++)
	{
		if (filaments[i].valid)
		{
			update_filament_location(i);
		}
	}
}


//==========================//
//                          //
//==========================//
void CFilamentSimulator::publish_markers()
{
	//1. Clean old markers
	filament_marker.points.clear();
	filament_marker.colors.clear();
	filament_marker.header.stamp = ros::Time::now();
	filament_marker.pose.orientation.w = 1.0;

	//width of points: scale.x is point width, scale.y is point height
	filament_marker.scale.x = cell_size/4;
	filament_marker.scale.y = cell_size/4;
	filament_marker.scale.z = cell_size/4;

	//2. Add a marker for each filament!
	for (int i=0; i<current_simulation_step*numFilaments_step; i++)
	{
		geometry_msgs::Point point;
		std_msgs::ColorRGBA color;

		//Set filament pose
		point.x = filaments[i].pose_x;
		point.y = filaments[i].pose_y;
		point.z = filaments[i].pose_z;

		//Set filament color
		color.a = 1;
		if (filaments[i].valid)
		{
			color.r = filament_color_r;
			color.g = filament_color_g;
			color.b = filament_color_b;
		}
		else
		{
			color.r = 1;
			color.g = 1;
			color.b = 1;
		}

		//Add marker
		filament_marker.points.push_back(point);
		filament_marker.colors.push_back(color);
	}

	//Publish marker of the filaments
	marker_pub.publish(filament_marker);
}



//==========================//
//                          //
//==========================//
double CFilamentSimulator::random_number(double min_val, double max_val)
{
	double n = (double)(rand() % 100);  //int random number [0, 100)
	n = n/100.0f;					   //random number [0, 1)
	n = n * (max_val - min_val);		//random number [0, max-min)
	n = n+ min_val;					 //random number [min, max)
	return n;
}


//Saves current Wind + GasConcentration to file
// These files will be later used in the "player" node.
void CFilamentSimulator::save_state_to_file()
{
    //Configure file name for saving the current snapshot
    std::string out_filename = boost::str( boost::format("%s/FilamentSimulation_gasType_%i_sourcePosition_%.2f_%.2f_%.2f_iteration_%i") % results_location % gasType % gas_source_pos_x % gas_source_pos_y % gas_source_pos_z % last_saved_step);
	
    int count=0;
	
	//We don't write directly to the file because we want to compress it,
	//and we dont store every line directly in a stream because that is very slow.
	//The fixed size of the array allows us to significantly speed up the process, but could lead to
	//overflowing errors, so we have to use a very pesimistic estimation of the space we need 
	char* charArray = new char[C.size()*C[0].size()*C[0][0].size()*50];

	//Write header
	//--------------
	//count how many chars have been printed to the array in each step and advance the pointer accordingly
    count += sprintf(&charArray[count], "env_min(m) %.4f %.4f %.4f\n",env_min_x, env_min_y, env_min_z);
    count += sprintf(&charArray[count], "env_max(m) %.4f %.4f %.4f\n",env_max_x, env_max_y, env_max_z);
    count += sprintf(&charArray[count], "NumCells_XYZ %i %i %i\n",env_cells_x,env_cells_y,env_cells_z);
    count += sprintf(&charArray[count], "CellSizes_XYZ[m] %.4f %.4f %.4f\n",cell_size, cell_size, cell_size);
    count += sprintf(&charArray[count], "GasSourceLocation_XYZ[m] %.4f %.4f %.4f\n",gas_source_pos_x, gas_source_pos_y, gas_source_pos_z);
	std::string gas_type_str;
	switch (gasType)
	{
		case 0: gas_type_str = "ethanol"; break;
		case 1: gas_type_str = "methane"; break;
		case 2: gas_type_str = "hydrogen"; break;
		case 3: gas_type_str = "propanol"; break;
		case 4: gas_type_str = "chlorine"; break;
		case 5: gas_type_str = "flurorine"; break;
		case 6: gas_type_str = "acetone"; break;
		case 7: gas_type_str = "neon"; break;
		case 8: gas_type_str = "helium"; break;
		case 9: gas_type_str = "hot_air"; break;
		case 10: gas_type_str = "carbon_dioxide"; break;
		default: gas_type_str = "ethanol";
	}
    count += sprintf(&charArray[count], "GasType %s\n",gas_type_str.c_str());

	std::string gas_units_str;
	if (gasConc_unit == 0)
		gas_units_str = "10^⁻3moles";
	else if (gasConc_unit == 1)
		gas_units_str =  "10^⁻3ppm";
	else
		gas_units_str =  "10^⁻3ppm";
    count += sprintf(&charArray[count], "Cell_x\t Cell_y\t Cell_z\t Gas_conc[%s]\t Wind_u[10^⁻3m/s]\t Wind_v[10^⁻3m/s]\t Wind_w[10^⁻3m/s]\n", gas_units_str.c_str());

	
	// Save to file the Gas concentration and wind vectors of every cell of the environment.
	//-------------------------------------------------------------------------------------
	for (int x = 0; x < env_cells_x; x++)
	{
		for (int y = 0; y < env_cells_y; y++)
		{
			for (int z = 0; z < env_cells_z; z++)
			{
				if(!(C[x][y][z]==0&& U[x][y][z]==0&& V[x][y][z]==0&& W[x][y][z]==0))
				{
					//Save to file! -> "Cell_x Cell_y Cell_z, Gas_conc U V W"
                    count += sprintf(&charArray[count], "%i %i %i %i %i %i %i\n",x, y, z,  (int)(1000* C[x][y][z]),  (int)(1000* U[x][y][z]),  (int)(1000* V[x][y][z]),  (int)(1000* W[x][y][z]));
				}
			}
		}
	}
	
	//now we do have to create a stream, but we pass all the information to it in one step, so we avoid the performance issues
	boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
	std::stringstream ist;
	inbuf.push(boost::iostreams::zlib_compressor());
	inbuf.push(ist);
	ist << charArray;

	std::ofstream fi(out_filename);
	boost::iostreams::copy(inbuf,fi);

	last_saved_step++;
	delete[] charArray;
}


//==============================//
//			MAIN                //
//==============================//
int main(int argc, char **argv)
{
	// Init ROS-NODE
	ros::init(argc, argv, "new_filament_simulator");

	//Create simulator obj and initialize it
	CFilamentSimulator sim;

	
	// Initiate Random Number generator with current time
	srand(time(NULL));

	//--------------
	// LOOP
	//--------------	
    ros::Rate r(100);
    ros::Time time_begin = ros::Time::now();
    ros::Time iter_time_begin = ros::Time::now();
	while (ros::ok() && (sim.current_simulation_step<sim.numSteps) )
	{
		//ROS_INFO("[filament] Simulating step %i (sim_time = %.2f)", sim.current_simulation_step, sim.sim_time);
		
        //0. Load wind snapshot (if necessary and availabe)
        if ( sim.sim_time-sim.sim_time_last_wind >= sim.windTime_step)
        {
            // Time to update wind!
            sim.sim_time_last_wind = sim.sim_time;
            if (sim.allow_looping)
            {
                // Load wind-data
                sim.read_wind_snapshot(sim.current_wind_snapshot);
                // Update idx
                if (sim.current_wind_snapshot >= sim.loop_to_step)
                    sim.current_wind_snapshot = sim.loop_from_step;
                else
                    sim.current_wind_snapshot++;
            }
            else
                sim.read_wind_snapshot(floor(sim.sim_time/sim.windTime_step));  //Alllways increasing
        }

		//1. Create new filaments close to the source location
		//   On each iteration num_filaments (See params) are created
		if(sim.current_simulation_step<sim.numDisperseSteps)
			sim.add_new_filaments(sim.cell_size);

		//2. Update Gas Concentration field		
		sim.update_gas_concentration_from_filaments();

		//3. Publish markers for RVIZ
		//sim.publish_markers();

		//4. Update filament locations
		sim.update_filaments_location();

		//5. Save data (if necessary)
        if ( (sim.save_results==1) && (sim.sim_time>=sim.results_min_time) )
        {
            if ( floor(sim.sim_time/sim.results_time_step) != sim.last_saved_step )
            {
			    //sim.save_state_to_file();
            	
				//ros::Time iter_time_end = ros::Time::now();
				//ros::Duration iter_duration = iter_time_end - iter_time_begin;
				//ros::Duration iter_duration_total = iter_time_end - time_begin;
				//ROS_INFO("Iteration-%d spent time: %lf s, total: %f s", sim.current_simulation_step-1,iter_duration.toSec(),iter_duration_total.toSec());
				//iter_time_begin = ros::Time::now();
			}
			if (sim.last_saved_step % sim.save_csv_every_iteration == 0)
			{
				ros::Time iter_time_end = ros::Time::now();
				ros::Duration iter_duration = iter_time_end - iter_time_begin;
				ros::Duration iter_duration_total = iter_time_end - time_begin;
				ROS_INFO("[Save .csv] Iteration-%d spent time: %lf s, total: %f s", sim.current_simulation_step-1,iter_duration.toSec(),iter_duration_total.toSec());
				iter_time_begin = ros::Time::now();
				
				sim.write_to_csv();
			}
	    }

		//4. Update Simulation state
		sim.sim_time = sim.sim_time + sim.time_step;	//sec
		sim.current_simulation_step++;

		r.sleep();
		ros::spinOnce();
		
	}
	ros::Time time_end = ros::Time::now();
	ros::Duration duration = time_end - time_begin;
	ROS_INFO("The simulation is running for %lf secs", duration.toSec());
}
