#include "utility.h"

class TransformFusion : public ParamServer {
   public:

	ros::Subscriber subImuOdometry;
	ros::Publisher pubImuOdometry;
	ros::Publisher pubImuPath;

	tf::TransformListener tfListener;
	tf::StampedTransform lidar2Baselink;

	TransformFusion() {
		if (lidarFrame != baselinkFrame) {
			try {
				tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
				tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
			} catch (tf::TransformException& ex) {
				ROS_ERROR("%s", ex.what());
			}
		}

		subImuOdometry = nh.subscribe<nav_msgs::Odometry>("/ins/odometry", 2000,
														  &TransformFusion::imuOdometryHandler, this,
														  ros::TransportHints().tcpNoDelay());

		pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
		pubImuPath = nh.advertise<nav_msgs::Path>("lio_sam/imu/path", 1);
	}

	void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {
		// static ros::Time prevTime(0);
		static tf::TransformBroadcaster tfMap2Odom;
		static tf::Transform map_to_odom
			= tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
		tfMap2Odom.sendTransform(
			tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

		// publish latest odometry
 		nav_msgs::Odometry laserOdometry = odomConverter(*odomMsg);
		pubImuOdometry.publish(laserOdometry);

		// publish tf
		static tf::TransformBroadcaster tfOdom2BaseLink;
		tf::Transform tCur;
		tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
		if (lidarFrame != baselinkFrame)
			tCur = tCur * lidar2Baselink;
		tf::StampedTransform odom_2_baselink
			= tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
		tfOdom2BaseLink.sendTransform(odom_2_baselink);

		// publish IMU path
		static nav_msgs::Path imuPath;
		static double last_path_time = -1;
		double imuTime = odomMsg->header.stamp.toSec();
		if (imuTime - last_path_time > 0.1) {
			last_path_time = imuTime;
			geometry_msgs::PoseStamped pose_stamped;
			pose_stamped.header.stamp = odomMsg->header.stamp;
			pose_stamped.header.frame_id = odometryFrame;
			pose_stamped.pose = laserOdometry.pose.pose;
			imuPath.poses.push_back(pose_stamped);
			
			if (pubImuPath.getNumSubscribers() != 0) {
				imuPath.header.stamp = odomMsg->header.stamp;
				imuPath.header.frame_id = odometryFrame;
				pubImuPath.publish(imuPath);
			}
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "roboat_loam");

	TransformFusion TF;

	ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");

	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();

	return 0;
}
