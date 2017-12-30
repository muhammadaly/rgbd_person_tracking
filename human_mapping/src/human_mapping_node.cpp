#include <ros/ros.h>
#include <people_msgs/Tracks.h>
#include <people_msgs/Track.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
//#include <stdlib.h>
#include <nav_msgs/Odometry.h>

typedef signed char uint8 ;
#define CELL_RESOLUTION 0.2 // m
#define MaxRangeX 5.0 // m
#define MaxRangeY 5.0 // m
#define NCellsX (float)(MaxRangeX/CELL_RESOLUTION)
#define NCellsY (float)(MaxRangeY/CELL_RESOLUTION)



#define HUMAN_DISTRIBUTION_RANGE 1.0
#define CO_MATRIX_WIDTH (float)(HUMAN_DISTRIBUTION_RANGE/CELL_RESOLUTION)
#define CO_MATRIX_HEIGHT (float)(HUMAN_DISTRIBUTION_RANGE/CELL_RESOLUTION)

ros::Subscriber people_sub;
ros::Publisher map_pub;
ros::Publisher human_odom;
std::vector<uint8> values;
static int seq_number;



std::vector<std::vector<uint8>> convarience_matrix;
void updateMap(std::vector<people_msgs::Track> tracks)
{
  for(int p = 0 ;p < 1 ; p++)
  {
//    ROS_INFO("X : %f , Y : %f , Z : %f" , tracks[p].point3D.x, tracks[p].point3D.y, tracks[p].point3D.z);
    nav_msgs::Odometry msg;
    msg.header.seq = ++seq_number;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera_link";

    msg.child_frame_id = "camera_link";
    msg.pose.pose.position.x = tracks[p].point3D.x;
    msg.pose.pose.position.y = tracks[p].point3D.y;
    msg.pose.pose.position.z = tracks[p].point3D.z;

    human_odom.publish(msg);


//    int xCell = (NCellsX/2);
    int yCell = abs(int(tracks[p].point3D.x/CELL_RESOLUTION));
    int xCell = int(tracks[p].point3D.z/CELL_RESOLUTION);
//    ROS_INFO("col Ind : %i , row Ind : %i" , xCell , yCell);
    values[yCell * NCellsX + xCell] = 255;
//    int co_matrix_x_offset = int(CO_MATRIX_WIDTH/2),
//        co_matrix_y_offset = int(CO_MATRIX_HEIGHT/2);
//    int roi_y_start = yCell - co_matrix_y_offset,
//        roi_y_end = yCell + co_matrix_y_offset,
//        roi_x_start = xCell - co_matrix_x_offset,
//        roi_x_end = xCell + co_matrix_x_offset;
//    int i = 0,j=0 ;
//    for(int rowInd = roi_y_start; rowInd <= roi_y_end ; rowInd ++)
//    {
//      j = 0;
//      for(int colInd = roi_x_start; colInd <= roi_x_end ; colInd ++)
//      {
//        values[(rowInd * NCellsX) + colInd] = convarience_matrix[i][j++] ;
//      }
//      i++;
//    }
  }
}
void fillmapDiagonal()
{
  for(int i = 0 ; i < NCellsY ; i ++)
  {
    values[(i * NCellsX) + i] = 255;
  }
}
void clearMap()
{
  values = std::vector<uint8>(NCellsX * NCellsY, 0) ;
}
void human_mapping_callback(const people_msgs::TracksConstPtr &detected_people)
{
  clearMap();
  if(detected_people->tracks.size() > 0)
  {
//    ROS_INFO("Number of detected humans %i" , (int) _candidates->clusters.clusters.size());
    updateMap(detected_people->tracks);
  }
}

void initializeHumanCovarianceMatrix()
{
  convarience_matrix.clear();
  float varience_x, varience_y, mean_x , mean_y;
  varience_x = varience_y = 0.3;
  mean_x = mean_y = 0.0;
  int startX = (int)((CO_MATRIX_WIDTH)/2.0),
      startY = (int)((CO_MATRIX_HEIGHT)/2.0);
  for(int x = -startX ; x <= startX ; x++)
  {
    std::vector<uint8> tmp;
    for(int y = -startY; y <=startY ; y++)
    {
      tmp.push_back(( (1.0 / (M_PI * varience_x * varience_y))*
                    exp(-( (pow(x - mean_x , 2.0) / (2 * pow(varience_x,2.0))) + (pow(y - mean_y , 2.0) / (2 * pow(varience_y,2.0))) )) ) *
                    255);
    }
    convarience_matrix.push_back(tmp);
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_mapping_node");
  ros::NodeHandle nh;
  seq_number = 0;
  people_sub = nh.subscribe("/tracks", 1, human_mapping_callback);
  map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);
  human_odom = nh.advertise<nav_msgs::Odometry>("/human_pose", 1);
  clearMap();
  ros::Rate r(1);
  initializeHumanCovarianceMatrix();

  while(ros::ok())
  {
    nav_msgs::OccupancyGrid new_map;
    new_map.header.seq = ++seq_number;
    new_map.header.stamp = ros::Time::now();
    new_map.header.frame_id = "camera_link";

    new_map.info.resolution = CELL_RESOLUTION;
    new_map.info.width = NCellsX;
    new_map.info.height = NCellsY;
    new_map.info.origin.position.x = -((NCellsX/2.0) * CELL_RESOLUTION);
    new_map.info.origin.position.y = 0.0;
    new_map.info.origin.position.z = 0.0;

    new_map.info.origin.orientation.x = 0.0;
    new_map.info.origin.orientation.y = 0.0;
    new_map.info.origin.orientation.z = 0.0;
    new_map.info.origin.orientation.w = 0.0;

    new_map.data = values;
    map_pub.publish(new_map);
    ros::spinOnce();
    r.sleep();
  }
//  ros::spin();
}
