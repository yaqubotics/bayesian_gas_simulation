#include "preprocessing_voronoi/preprocessing_voronoi.h"

void map_cb(const nav_msgs::OccupancyGridConstPtr& msg)
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

void clicked_point_cb(const geometry_msgs::PointStamped& msg)
{
    ROS_INFO("clicked");
}

visualization_msgs::Marker add_marker(float x, float y, int idx, int shape,float scale_x,float scale_y,float scale_z,float color_r,float color_g,float color_b)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = idx;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
}

// QItem for current location and distance 
// from source location 
class QItem { 
public: 
    int row; 
    int col; 
    float dist; 
    QItem(int x, int y, float w) 
        : row(x), col(y), dist(w) 
    { 
    } 
}; 
  
int minDistance(vector<vector<int>> grid_map,int grid_x,int grid_y,int dest_x, int dest_y) 
{ 
    QItem source(0, 0, 0); 
  
    // To keep track of visited QItems. Marking 
    // blocked cells as visited. 
    
    vector<vector<bool>> visited(map_width,vector<bool>(map_height,false)); 
    for (int i = 0; i < map_width; i++) { 
        for (int j = 0; j < map_height; j++) 
        { 
            if (grid_map[i][j] == 100)
                visited[i][j] = true; 
        } 
    } 
    
    source.row = grid_x;
    source.col = grid_y;
    // applying BFS on matrix cells starting from source 
    queue<QItem> q; 
    q.push(source); 
    visited[source.row][source.col] = true; 
    while (!q.empty()) { 
        QItem p = q.front(); 
        q.pop(); 
  
        // Destination found; 
        if((p.row == dest_x) && (p.col == dest_y)) 
            return p.dist; 
  
        // moving up 
        if (p.row - 1 >= 0 && 
            visited[p.row - 1][p.col] == false) { 
            q.push(QItem(p.row - 1, p.col, p.dist + 1)); 
            visited[p.row - 1][p.col] = true; 
        } 
  
        // moving down 
        if (p.row + 1 < map_width && 
            visited[p.row + 1][p.col] == false) { 
            q.push(QItem(p.row + 1, p.col, p.dist + 1)); 
            visited[p.row + 1][p.col] = true; 
        } 
  
        // moving left 
        if (p.col - 1 >= 0 && 
            visited[p.row][p.col - 1] == false) { 
            q.push(QItem(p.row, p.col - 1, p.dist + 1)); 
            visited[p.row][p.col - 1] = true; 
        } 
  
         // moving right 
        if (p.col + 1 < map_height && 
            visited[p.row][p.col + 1] == false) { 
            q.push(QItem(p.row, p.col + 1, p.dist + 1)); 
            visited[p.row][p.col + 1] = true; 
        } 
    } 
    return -1; 
} 

void printVectorInt(std::string filename, std::vector<std::vector<int> > env)
{
    std::ofstream outfile(filename.c_str());
    for (int i=0;i<env.size();i++)
    {
        for(int j=0;j<env[0].size();j++)
        {
            outfile << env[i][j] << " ";
        }
        outfile << "\n";
    }
}

void printVectorFloat(std::string filename, std::vector<std::vector<float> > env)
{
    std::ofstream outfile(filename.c_str());
    for (int i=0;i<env.size();i++)
    {
        for(int j=0;j<env[0].size();j++)
        {
            outfile << env[i][j] << " ";
        }
        outfile << "\n";
    }
}


float eu_distance(float x, float y, float p, float q)
{
    return sqrt((x-p)*(x-p)+(y-q)*(y-q));
}

float sumDistanceRegion(vector<vector<int>> voronoi_map,int grid_x,int grid_y,int region_idx) 
{ 
    QItem source(0, 0, 0); 
  
    // To keep track of visited QItems. Marking 
    // blocked cells as visited. 
    //vector<vector<float>> map_distance(map_width,vector<float>(map_height,-1.0));
    vector<vector<bool>> visited(map_width,vector<bool>(map_height,true)); 
    for (int i = 0; i < map_width; i++) { 
        for (int j = 0; j < map_height; j++) 
        { 
            if (voronoi_map[i][j] == region_idx)
                visited[i][j] = false; 
        } 
    } 
    float sum = 0;
    source.row = grid_x;
    source.col = grid_y;
    //map_distance[grid_x][grid_y] = 0;
    // applying BFS on matrix cells starting from source 
    queue<QItem> q; 
    q.push(source); 
    visited[source.row][source.col] = true; 
    while (!q.empty()) { 
        QItem p = q.front(); 
        q.pop(); 
  
        // Destination found; 
        //if((p.row == dest_x) && (p.col == dest_y)) 
        //    return p.dist; 
  
        // moving up 
        if (p.row - 1 >= 0 && 
            visited[p.row - 1][p.col] == false) { 
            q.push(QItem(p.row - 1, p.col, p.dist + 1)); 
            visited[p.row - 1][p.col] = true; 
            //map_distance[p.row-1][p.col] = p.dist+1;
            sum+=eu_distance(p.row-1,p.col,grid_x,grid_y);
        } 

        if (p.row - 1 >= 0 && p.col - 1 >= 0 && 
            visited[p.row - 1][p.col-1] == false) { 
            q.push(QItem(p.row - 1, p.col-1, p.dist + 1.4142)); 
            visited[p.row - 1][p.col-1] = true; 
            //map_distance[p.row-1][p.col-1] = p.dist+1.4142;
            sum+=eu_distance(p.row-1,p.col-1,grid_x,grid_y);
        } 
  
        if (p.row - 1 >= 0 && p.col + 1 < map_height && 
            visited[p.row - 1][p.col+1] == false) { 
            q.push(QItem(p.row - 1, p.col+1, p.dist + 1.4142)); 
            visited[p.row - 1][p.col+1] = true; 
            //map_distance[p.row-1][p.col+1] = p.dist+1.4142;
            sum+=eu_distance(p.row-1,p.col+1,grid_x,grid_y);
        } 

        // moving down 
        if (p.row + 1 < map_width && 
            visited[p.row + 1][p.col] == false) { 
            q.push(QItem(p.row + 1, p.col, p.dist + 1)); 
            visited[p.row + 1][p.col] = true; 
            //map_distance[p.row+1][p.col] = p.dist+1;
            sum+=eu_distance(p.row+1,p.col,grid_x,grid_y);
        } 

        if (p.row + 1 < map_width && p.col - 1 >= 0 && 
            visited[p.row + 1][p.col-1] == false) { 
            q.push(QItem(p.row + 1, p.col-1, p.dist + 1.4142)); 
            visited[p.row + 1][p.col-1] = true; 
            //map_distance[p.row+1][p.col-1] = p.dist+1.4142;
            sum+=eu_distance(p.row+1,p.col-1,grid_x,grid_y);
        } 
  
        if (p.row + 1 < map_width && p.col + 1 < map_height && 
            visited[p.row + 1][p.col+1] == false) { 
            q.push(QItem(p.row + 1, p.col+1, p.dist + 1.4142)); 
            visited[p.row + 1][p.col+1] = true; 
            //map_distance[p.row+1][p.col+1] = p.dist+1.4142;
            sum+=eu_distance(p.row+1,p.col+1,grid_x,grid_y);
        } 
  
        // moving left 
        if (p.col - 1 >= 0 && 
            visited[p.row][p.col - 1] == false) { 
            q.push(QItem(p.row, p.col - 1, p.dist + 1)); 
            visited[p.row][p.col - 1] = true; 
            //map_distance[p.row][p.col-1] = p.dist+1;
            sum+=eu_distance(p.row,p.col-1,grid_x,grid_y);
        } 
  
         // moving right 
        if (p.col + 1 < map_height && 
            visited[p.row][p.col + 1] == false) { 
            q.push(QItem(p.row, p.col + 1, p.dist + 1)); 
            visited[p.row][p.col + 1] = true; 
            //map_distance[p.row][p.col+1] = p.dist+1;
            sum+=eu_distance(p.row,p.col+1,grid_x,grid_y);
        } 
    } 
    return sum; 
} 


vector<vector<float>> getMapDistance(vector<vector<int>> grid_map,int grid_x,int grid_y) 
{ 
    QItem source(0, 0, 0); 
  
    // To keep track of visited QItems. Marking 
    // blocked cells as visited. 
    vector<vector<float>> map_distance(map_width,vector<float>(map_height,-1.0));
    vector<vector<bool>> visited(map_width,vector<bool>(map_height,false)); 
    for (int i = 0; i < map_width; i++) { 
        for (int j = 0; j < map_height; j++) 
        { 
            if (grid_map[i][j] == 100)
                visited[i][j] = true; 
        } 
    } 
    
    source.row = grid_x;
    source.col = grid_y;
    map_distance[grid_x][grid_y] = 0;
    // applying BFS on matrix cells starting from source 
    queue<QItem> q; 
    q.push(source); 
    visited[source.row][source.col] = true; 
    while (!q.empty()) { 
        QItem p = q.front(); 
        q.pop(); 
  
        // Destination found; 
        //if((p.row == dest_x) && (p.col == dest_y)) 
        //    return p.dist; 
  
        // moving up 
        if (p.row - 1 >= 0 && 
            visited[p.row - 1][p.col] == false) { 
            q.push(QItem(p.row - 1, p.col, p.dist + 1)); 
            visited[p.row - 1][p.col] = true; 
            map_distance[p.row-1][p.col] = p.dist+1;
        } 

        if (p.row - 1 >= 0 && p.col - 1 >= 0 && 
            visited[p.row - 1][p.col-1] == false) { 
            q.push(QItem(p.row - 1, p.col-1, p.dist + 1.4142)); 
            visited[p.row - 1][p.col-1] = true; 
            map_distance[p.row-1][p.col-1] = p.dist+1.4142;
        } 
  
        if (p.row - 1 >= 0 && p.col + 1 < map_height && 
            visited[p.row - 1][p.col+1] == false) { 
            q.push(QItem(p.row - 1, p.col+1, p.dist + 1.4142)); 
            visited[p.row - 1][p.col+1] = true; 
            map_distance[p.row-1][p.col+1] = p.dist+1.4142;
        } 

        // moving down 
        if (p.row + 1 < map_width && 
            visited[p.row + 1][p.col] == false) { 
            q.push(QItem(p.row + 1, p.col, p.dist + 1)); 
            visited[p.row + 1][p.col] = true; 
            map_distance[p.row+1][p.col] = p.dist+1;
        } 

        if (p.row + 1 < map_width && p.col - 1 >= 0 && 
            visited[p.row + 1][p.col-1] == false) { 
            q.push(QItem(p.row + 1, p.col-1, p.dist + 1.4142)); 
            visited[p.row + 1][p.col-1] = true; 
            map_distance[p.row+1][p.col-1] = p.dist+1.4142;
        } 
  
        if (p.row + 1 < map_width && p.col + 1 < map_height && 
            visited[p.row + 1][p.col+1] == false) { 
            q.push(QItem(p.row + 1, p.col+1, p.dist + 1.4142)); 
            visited[p.row + 1][p.col+1] = true; 
            map_distance[p.row+1][p.col+1] = p.dist+1.4142;
        } 
  
        // moving left 
        if (p.col - 1 >= 0 && 
            visited[p.row][p.col - 1] == false) { 
            q.push(QItem(p.row, p.col - 1, p.dist + 1)); 
            visited[p.row][p.col - 1] = true; 
            map_distance[p.row][p.col-1] = p.dist+1;
        } 
  
         // moving right 
        if (p.col + 1 < map_height && 
            visited[p.row][p.col + 1] == false) { 
            q.push(QItem(p.row, p.col + 1, p.dist + 1)); 
            visited[p.row][p.col + 1] = true; 
            map_distance[p.row][p.col+1] = p.dist+1;
        } 
    } 
    return map_distance; 
} 

int main(int argc, char **argv){
    ros::init(argc, argv, "preprocessing_voronoi");
    int numModels;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher pub_preprocessing_done = nh.advertise<std_msgs::Bool>("preprocessing_done",5,true);
    ros::Publisher pub_point_of_region = nh.advertise<visualization_msgs::MarkerArray> ("point_of_region", 1);
    ros::Publisher pub_point_of_region_final = nh.advertise<visualization_msgs::MarkerArray> ("point_of_region_final", 1);
    ros::Publisher pub_voronoi_map = nh.advertise<visualization_msgs::MarkerArray> ("voronoi_map", 1);
    ros::Publisher pub_voronoi_boundary = nh.advertise<visualization_msgs::MarkerArray> ("voronoi_boundary", 1);
    double min_distance_point;
    private_nh.param<double>("min_distance_point", min_distance_point, 1); //size of the cells
    private_nh.param<double>("cell_size", cell_size, 1); //size of the cells
    private_nh.param<int>("num_of_region", num_of_region, 5); //size of the cells
    std::string output;
    private_nh.param<std::string>("output_path", output, "");

    ros::Subscriber sub_map = nh.subscribe("/map", 1, map_cb);
    ros::Subscriber sub_clicked_point = nh.subscribe("/clicked_point", 1, clicked_point_cb);

    ROS_INFO("Preprocessing done");
    ros::Duration(10).sleep();

    std_msgs::Bool b;
    b.data=true;

    //ros::spin();

    ros::Rate r(0.1);
    while(!map_received) ros::spinOnce();
    srand((unsigned) time(0));
    
    int idx = 0;
    visualization_msgs::MarkerArray point_of_region;
    visualization_msgs::MarkerArray point_of_region_final;
    vector<vector<int>> voronoi_grid_list;
    vector<vector<int>> point_list;
    while (idx < num_of_region)
    {
        float rand_x = (float)(rand() % map_width);
        float rand_y = (float)(rand() % map_height);
        
        //float rand_x = 0;
        //float rand_y = idx*10;
        cout << rand_x << " " << rand_y << "\n";
        bool coincidence_point = false;
        for(int i=0;i<point_list.size();i++)
        {
            if(eu_distance((float)point_list[i][0],(float)point_list[i][1],rand_x,rand_y) < min_distance_point)
            {
                coincidence_point = true;
                break;
            }
        }
        if((map_data[(int)rand_x][(int)rand_y] == 0) && (!coincidence_point))
        {
            point_of_region.markers.push_back(add_marker(rand_x*cell_size,rand_y*cell_size,idx,visualization_msgs::Marker::SPHERE,5.0,5.0,5.0,0,1,0));
            pub_point_of_region.publish(point_of_region);
            vector<int> voronoi_grid;
            voronoi_grid_list.push_back(voronoi_grid);
            vector<int> point_temp;
            point_temp.push_back((int)rand_x);
            point_temp.push_back((int)rand_y);
            point_list.push_back(point_temp);
            idx++;
        }
        else
        {
            cout << "obstruct\n";
        }
    }
    int voronoi_iteration = 0;
    while(true)
    {
        cout << "Voronoi iteration-"<<voronoi_iteration<<endl;
        voronoi_iteration++;
        visualization_msgs::MarkerArray point_of_region;
        vector<vector<vector<int>>> distance_map(num_of_region,vector<vector<int>>(map_width,vector<int>(map_height,-1)));
        vector<vector<int>> voronoi_map_data(map_width,vector<int>(map_height,-1));
        vector<vector<vector<float>>> all_region_distance(num_of_region,vector<vector<float>>(map_width,vector<float>(map_height,-1.0)));

        for(int a=0;a<num_of_region;a++)
        {
            all_region_distance[a] = getMapDistance(map_data,point_list[a][0],point_list[a][1]);
        }
        cout << "Distance Map Calculated\n";
        vector<vector<float>> region_color(num_of_region,vector<float>(3,0.0));
        for(int i=0;i<num_of_region;i++)
        {
            for(int j=0;j<3;j++)
            {
                region_color[i][j] = (float)(rand() % 255)/255.0;
            }
        }
        visualization_msgs::MarkerArray voronoi_map;
        visualization_msgs::MarkerArray voronoi_boundary;
        int idx_voronoi_grid = 0;
        for(int i=0;i<map_width;i++)
        {
            for(int j=0;j<map_height;j++)
            {
                if(map_data[i][j] == 0)
                {
                    // calculate shortest path
                    vector<float> distance_list;
                    for(int k=0;k<num_of_region;k++)
                    {
                        //cout <<"point: ["<<point_list[k][0]<<" , "<<point_list[k][1]<<"]\n";
                        //cout <<"grid distance: "<< grid_distance <<"\n";
                        distance_list.push_back(all_region_distance[k][i][j]);
                    }
                    vector<float>::iterator minimum_region_idx = min_element(distance_list.begin(), distance_list.end());
                    int region_idx = distance(distance_list.begin(),minimum_region_idx);
                    //cout << "region idx "<<region_idx<<"\n";
                    voronoi_map_data[i][j] = region_idx;
                    voronoi_map.markers.push_back(add_marker(i*cell_size,j*cell_size,idx_voronoi_grid,visualization_msgs::Marker::CUBE,cell_size,cell_size,0.1,region_color[region_idx][0],region_color[region_idx][1],region_color[region_idx][2]));
                    
                    idx_voronoi_grid++;
                }
            }
            //cout<<"row-"<<i<<" done\n";
            pub_voronoi_map.publish(voronoi_map);
        }
        /*
        int boundary_cell_idx = 0;
        for(int i=0;i<map_width;i++)
        {
            for(int j=0;j<map_height;j++)
            {
                if(map_data[i][j] == 0)
                {
                    int r_idx = voronoi_map_data[i][j];

                    //bool b_1 = (voronoi_map_data[i][j] == r_idx);
                    bool b_1 = false;
                    if(j+1 < map_height)
                    {
                        bool b_2 = (voronoi_map_data[i][j+1] != r_idx);
                        b_1 = b_1 || b_2;
                    }
                    if((j-1 >= 0)&&(!b_1))
                    {
                        bool b_3 = (voronoi_map_data[i][j-1] != r_idx);
                        b_1 = b_1 || b_3;
                    }
                    if((i+1 < map_width)&&(!b_1))
                    {
                        bool b_4 = (voronoi_map_data[i+1][j] != r_idx);
                        b_1 = b_1 || b_4;
                    }
                    if(((i+1 < map_width) && (j+1 < map_height))&&(!b_1))
                    {
                        bool b_5 = (voronoi_map_data[i+1][j+1] != r_idx);
                        b_1 = b_1 || b_5;
                    }
                    if(((i+1 < map_width)&&(j-1 >= 0))&&(!b_1))
                    {
                        bool b_6 = (voronoi_map_data[i+1][j-1] != r_idx);
                        b_1 = b_1 || b_6;
                    }
                    if((i-1 >= 0)&&(!b_1))
                    {
                        bool b_7 = (voronoi_map_data[i-1][j] != r_idx);
                        b_1 = b_1 || b_7;
                    }
                    if(((i-1 >= 0)&&(j+1 < map_height))&&(!b_1))
                    {
                        bool b_8 = (voronoi_map_data[i-1][j+1] != r_idx);
                        b_1 = b_1 || b_8;
                    }
                    if(((i-1 >= 0)&&(j-1 >= 0))&&(!b_1))
                    {
                        bool b_9 = (voronoi_map_data[i-1][j-1] != r_idx);
                        b_1 = b_1 || b_9;
                    }
                    if(b_1)       
                    {         
                        voronoi_boundary.markers.push_back(add_marker(i*cell_size,j*cell_size,boundary_cell_idx,visualization_msgs::Marker::CUBE,cell_size,cell_size,0.1,0.0,0.0,1.0));
                        pub_voronoi_boundary.publish(voronoi_boundary);
                        boundary_cell_idx++;
                    }
                }
            }
        }
        */
        printVectorInt(boost::str(boost::format("%s/voronoi_grid.csv") % output.c_str()), voronoi_map_data);
        printVectorFloat(boost::str(boost::format("%s/map0.csv") % output.c_str()), all_region_distance[0]);
        printVectorFloat(boost::str(boost::format("%s/map1.csv") % output.c_str()), all_region_distance[1]);
        printVectorFloat(boost::str(boost::format("%s/map2.csv") % output.c_str()), all_region_distance[2]);
        printVectorInt(boost::str(boost::format("%s/voronoi_init_points.txt") % output.c_str()), point_list);
        cout << "Voronoi Map Calculated\n";

        for(int a=0;a< num_of_region;a++)
        {
            float min_distance = 2147483647;
            int min_point_x;
            int min_point_y;
            for(int i=0;i<map_width;i++)
            {
                for(int j=0;j<map_height;j++)
                {
                    if(voronoi_map_data[i][j] == a)
                    {
                        float distance = sumDistanceRegion(voronoi_map_data,i,j,a);
                        if(distance < min_distance)
                        {
                            min_distance = distance;
                            min_point_x = i;
                            min_point_y = j;
                        }
                        //cout << "[" <<a<<","<<i<<","<<j<<","<<distance<<","<<min_distance<<"]\n";
                    }
                }
            }
            point_list[a][0] = min_point_x;
            point_list[a][1] = min_point_y;
            point_of_region.markers.push_back(add_marker(min_point_x*cell_size,min_point_y*cell_size,a,visualization_msgs::Marker::SPHERE,5.0,5.0,5.0,1,0,0));
            pub_point_of_region.publish(point_of_region);
            //cout << "centroid-"<<a<<" done\n";

        }
        
        printVectorInt(boost::str(boost::format("%s/voronoi_final_points.txt") % output.c_str()), point_list);
    }
    cout << "DONE ALL\n";
    while(ros::ok()){
        ros::Duration(0.2).sleep();
        //ROS_INFO("Waiting for publishers...");
        //pub_preprocessing_done.publish(b);
        pub_point_of_region.publish(point_of_region);
        pub_point_of_region_final.publish(point_of_region_final);
        //r.sleep();
        //cout << map_data[248][73] << " " << map_data[248][93] << "\n";
        ros::spinOnce();
    }
    
}
