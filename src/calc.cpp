#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <aruco_mapping/ArucoMarker.h>

#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <stdlib.h>
#include <math.h>

int rots;

std::vector<Eigen::Quaternionf> imu_readings;
std::vector<Eigen::Quaternionf> marker_readings;

std::vector<Eigen::Vector3f> imu_t_tp1;
std::vector<Eigen::Vector3f> marker_t_tp1;

float iw, ix, iy, iz, mw, mx, my, mz;

void callback(const sensor_msgs::ImuConstPtr& imu_msg, const aruco_mapping::ArucoMarkerConstPtr& marker_msg)
{

	iw = (imu_msg->orientation.w);
	ix = (imu_msg->orientation.x);
	iy = (imu_msg->orientation.y);
	iz = (imu_msg->orientation.z);

	mw = (marker_msg->global_camera_pose.orientation.w);
	mx = (marker_msg->global_camera_pose.orientation.x);
	my = (marker_msg->global_camera_pose.orientation.y);
	mz = (marker_msg->global_camera_pose.orientation.z);

	if(!(mx == 0.0 && my == 0.0 && mz == 0.0 && mw == 0.0))
	{
		imu_readings.push_back(Eigen::Quaternionf(iw, ix, iy, iz));
		marker_readings.push_back(Eigen::Quaternionf(mw, mx, my, mz));
		rots++;
		ROS_INFO_STREAM("Using rotation pair #" << rots);
		std::cout << "\n" << iw << "\t" << ix << "\t" << iy << "\t" << iz<< "\n";
		std::cout << mw << "\t" << mx << "\t" << my << "\t" << mz << "\n";
		std::cout << std::endl;
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "calc_transform");
	ros::NodeHandle nh;

	int no_of_rots;

	if(!ros::param::get("~no_of_rots", no_of_rots))
	{
		ROS_FATAL_STREAM("\"no_of_rots\" parameter is not set!");
		exit(1);
	}

	message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "imu/data", 100);
	message_filters::Subscriber<aruco_mapping::ArucoMarker> marker_sub(nh, "aruco_poses", 100);

	message_filters::TimeSequencer<sensor_msgs::Imu> seq_imu(imu_sub, ros::Duration(0.1), ros::Duration(0.01), 100);
	message_filters::TimeSequencer<aruco_mapping::ArucoMarker> seq_marker(marker_sub, ros::Duration(0.1), ros::Duration(0.01), 100);	

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, aruco_mapping::ArucoMarker> ApproxPolicy;
  
    message_filters::Synchronizer<ApproxPolicy> sync(ApproxPolicy(100), seq_imu, seq_marker);
    sync.registerCallback(boost::bind(&callback, _1, _2));	

	ROS_INFO_STREAM("Beginning to record rotations!\n");
	ros::Duration(1).sleep();

	while(rots < no_of_rots && ros::ok())
	{
		ros::spinOnce();
	}

	for (std::vector<Eigen::Quaternionf>::iterator it = imu_readings.begin(); it < imu_readings.end() - 1; it++)
	{
      	Eigen::AngleAxis<float> a_imu((*it).inverse() * (*(it+1)));
        imu_t_tp1.push_back(Eigen::Vector3f(a_imu.angle() * a_imu.axis()));
  	}

  	for (std::vector<Eigen::Quaternionf>::iterator it = marker_readings.begin(); it < marker_readings.end() - 1; it++)
  	{
    	 Eigen::AngleAxis<float> a_marker((*it).inverse() * (*(it+1)));
    	 marker_t_tp1.push_back(a_marker.angle() * a_marker.axis());
    }

    std::cout << "\n IMU trasnformations!\n";
	
	for(std::vector<Eigen::Vector3f>::const_iterator it = imu_t_tp1.begin(); it < imu_t_tp1.end() - 1; it++)
    {
    	std::cout << (*it).transpose() << std::endl;
    }

    std::cout << "\n Camera Transformations!\n";

    for(std::vector<Eigen::Vector3f>::const_iterator it = marker_t_tp1.begin(); it < marker_t_tp1.end() - 1; it++)
    {
    	std::cout << (*it).transpose() << std::endl;
    }

  	int tsize_imu, tsize_camera;
  	tsize_imu = imu_t_tp1.size();
  	tsize_camera = marker_t_tp1.size();

  	Eigen::Matrix<float, 3, Eigen::Dynamic> transformations_imu;
  	Eigen::Matrix<float, 3, Eigen::Dynamic> transformations_camera;

  	transformations_imu.resize(3, tsize_imu);
  	transformations_camera.resize(3, tsize_camera);

  	for(int i = 0; i < tsize_imu; i++)
  	{
    	transformations_imu.col(i) = imu_t_tp1[i];
    	transformations_camera.col(i) = marker_t_tp1[i];
  	}
  
 	Eigen::Matrix<float, 3, 1> centre_imu = transformations_imu.rowwise().sum()/transformations_imu.cols();
  	Eigen::Matrix<float, 3, 1> centre_camera = transformations_camera.rowwise().sum()/transformations_camera.cols();

  	transformations_imu = transformations_imu.colwise() - centre_imu;
  	transformations_camera = transformations_camera.colwise() - centre_camera;
 
  	Eigen::Matrix<float, 3, 3> covariance_m = transformations_camera*(transformations_imu.transpose());
  	Eigen::JacobiSVD<Eigen::MatrixXf> svd(covariance_m, Eigen::ComputeFullU | Eigen::ComputeFullV);

  	Eigen::Matrix<float, 3,3> rotation;
  	rotation = svd.matrixV()*(svd.matrixU().transpose());

  	std::cout << "\nEstimated Transformation Matrix:\n " << rotation << "\n";

	return 0;
}