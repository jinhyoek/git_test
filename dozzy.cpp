#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#define RAD2DEG(x) ((x)*180./M_PI)
#include <sstream>
#include <algorithm>
float g_optimal;
std::vector<float> distances;
std::vector<float> real_distances;
std::vector< std::vector <float> > tmp;
int g_num;
int g_count;
float smallest;

float find_optimal_degree(const std::vector<float> distances);
float minimal(const std::vector<float> real_distances,int h);

float scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
	g_count = count ;
    float dozzy = 0.8;
    int h = 0;
	distances.clear();
	real_distances.clear();

	for(int i = 0; i <= count /2 ; i++) 
    {
   
      	if (scan->ranges[i] > dozzy || scan->ranges[i]  == 0) 
	  {
		  distances.push_back(1);
	  }                                  
      	else if (scan->ranges[i] > dozzy || scan->ranges[i]  == 0)
      {
		  distances.push_back(0);
      }
      
      	if (scan -> ranges[i] > 0.12)
	{
		real_distances.push_back(scan->ranges[i]);
		h++;
	}
     smallest = minimal(real_distances,h);
    }
	printf("smallest:%f\n",smallest);
    return (find_optimal_degree(distances));
}

std::vector <std::vector <float> > make_second_array(const std::vector<float> distances)
{
	std::vector <float> nums(2);

   std::vector <std::vector<float>> array;
	
   	g_num =0;
	int ranges = 0;
	int first_angle = 0;
	array.clear();
	tmp.clear();
	int i;
	for (i = 1; i <= g_count /2  ; i++) {

        	if ((distances[i - 1] == 1) && (distances[i] == 0) && (ranges > 10) && (first_angle != 0)) {
				nums[0] = ranges;
				nums[1] = first_angle;
				array.push_back(nums);
				g_num ++;

				first_angle = 0;
				ranges = 0;
      }
        	else if ((distances[i - 1] == 0) &&(distances[i] == 0)  ) {
				first_angle = i;
        }
		else if ((distances[i - 1] == 1) && (distances[i] == 1)) {
		ranges += 1;
        }
    }
	tmp = array;
	array.clear();
	return tmp;
}

float select_angle(std::vector< std::vector<float> > array)
{
   float ranges;
   float first_angle;
   int i;
        //배열크기,입력값,최소값,최대값
   ranges = array[0][0];

   first_angle = array[0][1];

      // 배열 첫번째 값을 최소 최대값으로 설정
	for(i=0; i < g_num ;i++)
	{
		if(ranges < array[i][0])
	{
		ranges = array[i][0];
		first_angle = array[i][1];
		std::cout << "range is " << array[i][0] / g_count * 360 << " , first is " << array[i][1] / g_count * 360<< std::endl; 

	}
		else{
		std::cout << "range is " << array[i][0] / g_count * 360 << " , fisrt is " << array[i][1] / g_count * 360<< std::endl; 
}

	}
   	std::cout << "ranges: " << ranges/ g_count * 360 << " first angle: " << first_angle/ g_count * 360 << std::endl;

	ranges = ranges/ g_count * 360;
	first_angle = first_angle/ g_count * 360;

   return (first_angle + (ranges / 2));
}

float find_optimal_degree(const std::vector<float> distances)
{

	float optimal;

   optimal = select_angle(make_second_array(distances));
   printf("optimal: %f\n",  optimal);
  // msg = std::to_string(optimal);
	
   g_optimal = optimal;
   
	
   return (optimal);
}


float minimal(const std::vector<float> real_distances,int h)
{
	smallest = 10;
	for (int i=0; i<h - 1 ; i++)
	{ if (real_distances[i] < smallest)
		{smallest = real_distances[i]; }}
	return smallest;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "seven");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
	ros::Publisher optimal_pub = n.advertise<std_msgs::Float64>("optimal_angle", 1000);
	ros::Publisher miny_pub = n.advertise<std_msgs::Float64>("minimum_distance", 1000);
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		std_msgs::Float64 msg;
		std_msgs::Float64 mini;

        float g_o;
        g_o = g_optimal;
		//std::stringstream ss;
		//ss << g_optimal  ;
		msg.data = g_o;
		mini.data = smallest;

		ROS_INFO("%f", msg.data);
		ROS_INFO("%f", mini.data);

		optimal_pub.publish(msg);
		miny_pub.publish(mini);

		ros::spinOnce();

		loop_rate.sleep();
	}
    return 0;
}


