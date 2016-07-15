#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <time.h> 
#include <iostream>
#include <ros/console.h>
#include <cstring>
#include <fstream>

using namespace std;

std_msgs::String flag;
float max_dis = 0.8;
string pre_data  = "go";

bool map_init = false;
bool odom_init = false;

int odom_x = 0;
int odom_y = 0;


int map_width = 0;
int map_height = 0;
float map_re = 0;
float map_x = 0;
float map_y = 0;
int8_t* map_data = NULL;

int scan_x = 20;
int scan_y = 20;


string getTime() {
    time_t nowtime;
    time(&nowtime);
    string time_str = ctime(&nowtime);
    string res =  time_str.substr(0,time_str.length()-1);
    return res;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapData)
{
    /*
    nav_msgs::MapMetaData md = mapData->info;

    cout << "map info:  " << endl;
    cout << "width: " << md.width << endl;
    cout << "height: " << md.height << endl;
    cout << "resolution: " << md.resolution << endl;
    geometry_msgs::Pose origin_pose =  md.origin;
    cout << "Point: " << origin_pose.position.x << "," << origin_pose.position.y << ","<< origin_pose.position.z << endl;
    cout << "Quaternion: " << origin_pose.orientation.x << "," << origin_pose.orientation.y << 
        ","<< origin_pose.orientation.z << "," << origin_pose.orientation.w << endl;
    */
}

void globalmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapData)
{
    nav_msgs::MapMetaData md = mapData->info;

    cout << "global map info:  " << endl;
    cout << "width: " << md.width << endl;
    cout << "height: " << md.height << endl;
    cout << "resolution: " << md.resolution << endl;
    geometry_msgs::Pose origin_pose =  md.origin;
    cout << "Point: " << origin_pose.position.x << "," << origin_pose.position.y << ","<< origin_pose.position.z << endl;
    cout << "Quaternion: " << origin_pose.orientation.x << "," << origin_pose.orientation.y << 
        ","<< origin_pose.orientation.z << "," << origin_pose.orientation.w << endl;

    // Get global map info
    if (!map_init){
        map_width = md.width;
        map_height = md.height;
        map_re = md.resolution;
        int array_size = map_width * map_height * sizeof(int8_t);
        map_data = (int8_t*)malloc(array_size);
        //cout << array_size << endl;
        for(int i = 0; i<array_size;++i){
            map_data[i] = mapData->data[i];
        }
        //memcpy(map_data, mapData->data, array_size);
        map_x = origin_pose.orientation.x;
        map_y = origin_pose.orientation.y;
        map_init = true;
    }
    
    /*
    ofstream out("out.txt");  
    for(int i = 0; i < md.height; ++i){
        for(int j = 0; j < md.width; ++j){
            if (mapData->data[i*md.width+j] == 0)
                out << " ";
            else
                out << mapData->data[i*md.width+j];
        }
        out<<endl;
    }
    out.close();
    */
}



void localmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapData)
{
    nav_msgs::MapMetaData md = mapData->info;

    cout << "local map info:  " << endl;
    cout << "width: " << md.width << endl;
    cout << "height: " << md.height << endl;
    cout << "resolution: " << md.resolution << endl;
    geometry_msgs::Pose origin_pose =  md.origin;
    cout << "Point: " << origin_pose.position.x << "," << origin_pose.position.y << ","<< origin_pose.position.z << endl;
    cout << "Quaternion: " << origin_pose.orientation.x << "," << origin_pose.orientation.y << 
        ","<< origin_pose.orientation.z << "," << origin_pose.orientation.w << endl;

    // When local map changed
    if (odom_init){
        // Offset to local map
        float x_local_dis = odom_x - origin_pose.position.x;
        float y_local_dis = odom_y - origin_pose.position.y;
        float x_local_offset0 = x_local_dis / md.resolution;
        float y_local_offset0 = y_local_dis / md.resolution;

        int x_local_offset = int(x_local_offset0);
        int y_local_offset = int(y_local_offset0);
        if (x_local_offset - x_local_offset0 > 0.5)
            ++x_local_offset;
        if (y_local_offset - y_local_offset0 > 0.5)
            ++y_local_offset;

        // Offset to global map
        float x_global_dis = odom_x - map_x;
        float y_global_dis = odom_y - map_y;
        float x_global_offset0 = x_global_dis / map_re;
        float y_global_offset0 = y_global_dis / map_re;

        int x_global_offset = int(x_global_offset0);
        int y_global_offset = int(y_global_offset0);
        if (x_global_offset - x_global_offset0 > 0.5)
            ++x_global_offset;
        if (y_global_offset - y_global_offset0 > 0.5)
            ++y_global_offset;


        // Scan range
        int x_begin = x_local_offset - scan_x;
        if (x_begin < 0)
            x_begin = 0;

        int x_end = x_local_offset + scan_x;
        if (x_end > md.width)
            x_end = md.width;

        int y_begin = y_local_offset - scan_y;
        if (y_begin < 0)
            y_begin = 0;

        int y_end = y_local_offset + scan_y;
        if (y_end > md.height)
            y_end = md.height;

        // Go through the near local map

        bool flag_bool = false;
        for(int i = y_begin; i < y_end; ++i){
            for(int j = x_begin; j < x_end; ++j){
                int local_index = i * md.width + j;
                int global_index = (i - x_local_offset + x_global_offset) * map_width + j - y_local_offset + y_global_offset;
                // If not same with global map
                if ((mapData->data[local_index] != 0) && (map_data[global_index] == 0)){
                    flag_bool = true;
                    break;   
                }
            }
            if (flag_bool)
                break;
        }

        if (flag_bool) {
            if (pre_data == "go") {
                flag.data = "stop";
                pre_data = "stop";
                cout << getTime() << "  stop" << endl;
            }
        }
        else {
            if (pre_data == "stop") {
                flag.data = "go";
                pre_data  = "go";
                cout << getTime() << "  go" << endl;
            }
        }

        // Next odom
        odom_init = false;
    }
}


void odomCallback(const nav_msgs::Odometry::ConstPtr& odomData){
    geometry_msgs::Point p = odomData->pose.pose.position;
    //cout << p.x << " " << p.y << " " << p.z << endl; 

    // Get point of robot
    if (map_init) {
        odom_x = p.x;
        odom_y = p.y;
        odom_init = true;
    }
}


void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
    /*
    // Example of using some of the non-range data-types
    float rangeDataNum =  1 + (laserScanData->angle_max - laserScanData->angle_min) / (laserScanData->angle_increment);

    // Go through the laser data
    bool flag_bool = false;
    for (int j = 0; j < rangeDataNum; ++j) {  
        if ( laserScanData->ranges[j] < max_dis ) {
            flag_bool = true;
            break;   
        }
    }

    if (flag_bool) {
        if (pre_data == "go") {
            flag.data = "stop";
            pre_data = "stop";
            cout << getTime() << "  stop" << endl;
        }
    }
    else {
        if (pre_data == "stop") {
            flag.data = "go";
            pre_data  = "go";
            cout << getTime() << "  go" << endl;
        }
    }
    */
}

int main (int argc, char **argv)
{
    // command line ROS arguments
    ros::init(argc, argv, "pioneer_laser_node");

    // ROS comms access point
    ros::NodeHandle my_handle;

    // topic to public stop_flag
    string stop_flag_topic = "/stop_flag";
    if(my_handle.hasParam("stop_flag_topic"))
        my_handle.getParam("stop_flag_topic",stop_flag_topic);
    else
        my_handle.setParam("stop_flag_topic", stop_flag_topic);

    // topic to get scan data
    string detect_sub_topic = "/scan";
    if(my_handle.hasParam("detect_sub_topic"))
        my_handle.getParam("detect_sub_topic",detect_sub_topic);
    else
        my_handle.setParam("detect_sub_topic", detect_sub_topic);

    // topic to get map data
    string map_sub_topic = "/map";
    if(my_handle.hasParam("map_sub_topic"))
        my_handle.getParam("map_sub_topic",map_sub_topic);
    else
        my_handle.setParam("map_sub_topic", map_sub_topic);

    // topic to get map data
    string global_map_sub_topic = "/move_base/global_costmap/costmap";
    if(my_handle.hasParam("global_map_sub_topic"))
        my_handle.getParam("global_map_sub_topic",global_map_sub_topic);
    else
        my_handle.setParam("global_map_sub_topic", global_map_sub_topic);

    // topic to get map data
    string local_map_sub_topic = "/move_base/local_costmap/costmap";
    if(my_handle.hasParam("local_map_sub_topic"))
        my_handle.getParam("local_map_sub_topic",local_map_sub_topic);
    else
        my_handle.setParam("local_map_sub_topic", local_map_sub_topic);


    // topic to get map data
    string odom_sub_topic = "/odom";
    if(my_handle.hasParam("map_sub_topic"))
        my_handle.getParam("odom_sub_topic",odom_sub_topic);
    else
        my_handle.setParam("odom_sub_topic", odom_sub_topic);

    // advertise to the flag topic
    ros::Publisher pub_object = my_handle.advertise<std_msgs::String>(stop_flag_topic,1);
    ROS_DEBUG("advertise /stop_flag");

    // subscribe to the scan topic and define a callback function to process the data
    //ros::Subscriber laser_sub_object = my_handle.subscribe(detect_sub_topic, 1, laserScanCallback);
    //ROS_DEBUG("subscriber to /scan");

    // subscribe to the map topic and define a callback function to process the data
    //ros::Subscriber map_sub_object = my_handle.subscribe(map_sub_topic, 1, mapCallback);
    //ROS_DEBUG("subscriber to /map");

    // subscribe to the map topic and define a callback function to process the data
    ros::Subscriber map3_sub_object = my_handle.subscribe(global_map_sub_topic, 1, globalmapCallback);
    ROS_DEBUG("subscriber to /move_base/global_costmap/costmap");

    // subscribe to the map topic and define a callback function to process the data
    ros::Subscriber map2_sub_object = my_handle.subscribe(local_map_sub_topic, 1, localmapCallback);
    ROS_DEBUG("subscriber to /move_base/local_costmap/costmap");

    // subscribe to the map topic and define a callback function to process the data
    //ros::Subscriber tf_sub_object = my_handle.subscribe("/tf", 1, tfCallback);
    //ROS_DEBUG("subscriber to /tf");

    // subscribe to the map topic and define a callback function to process the data
    ros::Subscriber odom_sub_object = my_handle.subscribe(odom_sub_topic, 1, odomCallback);
    ROS_DEBUG("subscriber to /odom");

    // loop 10 Hz
    ros::Rate loop_rate(10);

    // get the distance to stop
    if (my_handle.hasParam("max_dis"))
        my_handle.getParam("max_dis",max_dis);
    else
        my_handle.setParam("max_dis", max_dis);

    cout << getTime() << "  go" << endl;

    // publish the velocity set in the call back
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    
        // publish
        pub_object.publish(flag);
    }

    return 0;
}
