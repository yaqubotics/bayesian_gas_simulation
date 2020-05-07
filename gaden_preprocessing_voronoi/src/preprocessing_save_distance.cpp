#include "preprocessing_voronoi/preprocessing_save_distance.h"

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
    int dist; 
    QItem(int x, int y, int w) 
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

  
vector<vector<int>> getMapDistance(vector<vector<int>> grid_map,int grid_x,int grid_y) 
{ 
    QItem source(0, 0, 0); 
  
    // To keep track of visited QItems. Marking 
    // blocked cells as visited. 
    vector<vector<int>> map_distance(map_width,vector<int>(map_height,-1));
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
  
        // moving down 
        if (p.row + 1 < map_width && 
            visited[p.row + 1][p.col] == false) { 
            q.push(QItem(p.row + 1, p.col, p.dist + 1)); 
            visited[p.row + 1][p.col] = true; 
            map_distance[p.row+1][p.col] = p.dist+1;
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
            map_distance[p.row][p.col-1] = p.dist+1;
        } 
    } 
    return map_distance; 
} 
void printVector(std::string filename, std::vector<std::vector<int> > env)
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

    private_nh.param<double>("cell_size", cell_size, 1); //size of the cells
    private_nh.param<int>("num_of_region", num_of_region, 5); //size of the cells
    std::string output;
    private_nh.param<std::string>("output_path", output, "");

    ros::Subscriber sub_map = nh.subscribe("/map", 1, map_cb);
    ros::Subscriber sub_clicked_point = nh.subscribe("/clicked_point", 1, clicked_point_cb);

    ROS_INFO("Preprocessing done");
    std_msgs::Bool b;
    b.data=true;

    //ros::spin();

    ros::Rate r(0.1);
    while(!map_received) ros::spinOnce();

    for(int p=0;p<map_width;p++)
    {
        for(int q=0;q<map_height;q++)
        {
            vector<vector<int>> distance_grid(map_width,vector<int>(map_height,-1));
            if(map_data[p][q] == 0)
            {
                /*
                for(int i=0;i<map_width;i++)
                {
                    for(int j=0;j<map_height;j++)
                    {
                        if(map_data[i][j] == 0)
                        {
                            distance_grid[i][j] = minDistance(map_data,i,j,p,q);
                        }
                    }
                    cout << "["<< p << "," << q << "]" <<" row-" << i << endl;
                }
                */
                cout << "["<< p << "," << q << "]\n";
                vector<vector<int>> distance_grid = getMapDistance(map_data,p,q);
                printVector(boost::str(boost::format("%s/distance_grid/d_g_%4d_%4d.csv") % output.c_str() % p % q), distance_grid);
            }
        }
    }
    cout << "DONE ALL\n";
    while(ros::ok()){
        ros::Duration(0.2).sleep();
        //ROS_INFO("Waiting for publishers...");
        //pub_preprocessing_done.publish(b);
        //r.sleep();
        //cout << map_data[248][73] << " " << map_data[248][93] << "\n";
        ros::spinOnce();
    }
    
}
