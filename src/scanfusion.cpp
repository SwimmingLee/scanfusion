#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


int bDepthscan = false;
float depthang_min, depthang_max, depthang_itv = 0;
float lidarang_min, lidarang_max, lidarang_itv = 0;

sensor_msgs::LaserScan depthscan;
sensor_msgs::LaserScan lidarscan;

void depthscanCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if(depthang_itv == 0)
	{
		depthang_min = msg->angle_min;
		depthang_max = msg->angle_max;
		depthang_itv = msg->angle_increment;
	}

	depthscan.ranges = msg->ranges;
	
	bDepthscan = true;
}

void lidarscanCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{

	if(lidarang_itv == 0)
	{
		lidarang_min = msg->angle_min;
		lidarang_max = msg->angle_max;
		lidarang_itv = msg->angle_increment;
		lidarscan.range_min = msg->range_min;
		lidarscan.range_max = msg->range_max;
		lidarscan.angle_min = msg->angle_min;
		lidarscan.angle_max = msg->angle_max;
		lidarscan.angle_increment = msg->angle_increment;
		lidarscan.header.frame_id = msg->header.frame_id;
	}

	lidarscan.ranges =  msg->ranges;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scanfusion");
	ros::NodeHandle nh;

	ros::Publisher pub_fusionscan = nh.advertise<sensor_msgs::LaserScan>("fusionscan", 1);
	ros::Subscriber sub_depthscan = nh.subscribe("depthscan", 1, depthscanCB);
	ros::Subscriber sub_lidarscan = nh.subscribe("lidarscan", 1, lidarscanCB);

	ros::Rate loop_rate(30);

	while(ros::ok())
	{
		if(lidarang_itv != 0 && depthang_itv != 0 && bDepthscan)
		{
			//라이다 스캔 픽셀 개수 360개, 깊이 스캔 픽셀 개수 640개
			int lidar_maxfixel = (int)((lidarang_max - lidarang_min) / lidarang_itv) + 1; 
			int depth_maxfixel = (int)((depthang_max - depthang_min) / depthang_itv) + 1;	

			
			int pre_i = 0;
			for(int n = 0; n < lidar_maxfixel; n++){
				float srt_ang = lidarang_min + lidarang_itv*n;
				if( srt_ang > depthang_max) break;
				if( srt_ang < depthang_min) continue;

				for(int i = pre_i; i < depth_maxfixel; i+=2)
				{
					float dst_ang = depthang_min + depthang_itv*i;
					float depth_length, corr_length;
					
					depth_length = depthscan.ranges[i];
					corr_length = sqrt(0.0324 + depth_length*depth_length + 0.36*depth_length*cos(dst_ang));

					if(dst_ang > 0)
						dst_ang = acos((corr_length*corr_length + 0.0324 - depth_length*depth_length) / (0.36*corr_length));
					else
						dst_ang = -acos((corr_length*corr_length + 0.0324 - depth_length*depth_length) / (0.36*corr_length));
					
					if( srt_ang < dst_ang )
					{
					    std::cout << "pre_t:"<< pre_i << " : " << srt_ang <<" dst_ang:" << dst_ang << std::endl;
					    std::cout << "pre_t:"<< pre_i << " : " << lidarscan.ranges[n] <<" dst:" << corr_length << std::endl;
					   // float depth_length, corr_length;
					//	depth_length = depthscan.ranges[i];
					//	corr_length = 0.0256 + depth_length*depth_length + 0.32*depth_length*cos(dst_ang);
						//if(lidarscan.ranges[n] > depthscan.ranges[i])
						if(lidarscan.ranges[n] > corr_length && corr_length < 2.5)
							lidarscan.ranges[n] = corr_length;
							//lidarscan.ranges[n] = depthscan.ranges[i];
					    pre_i = i;
					    break;
					}
				}
			}// end of lidar ranges acess
			bDepthscan = false;
			lidarscan.header.stamp = ros::Time::now();
			pub_fusionscan.publish(lidarscan);
		}	


	ros::spinOnce();
	loop_rate.sleep();
	}// end of main loop
	return 0;
}
