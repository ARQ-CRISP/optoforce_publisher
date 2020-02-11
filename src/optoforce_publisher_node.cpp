// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#include <signal.h>
#include <ros/ros.h>

#include <optoforce_api/OptoDAQ.h>
#include <optoforce_api/OptoDAQDescriptor.h>
#include <optoforce_api/OptoPacket3D.h>
#include <optoforce_api/OptoDAQWatcher.h>

#include <geometry_msgs/WrenchStamped.h>

#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

using namespace std;

OptoDAQ* daqs_;
// count of DAQs
std::size_t count_;
// frequency of packets (Hz)
double pack_freq_ = 1000.0;

vector<ros::Publisher> wrench_publisher_vec_; // to visualize in rviz
tf2_ros::Buffer tf_buffer_;
// external params
bool invert_;
vector<string> frame_names_;

// zeroing
bool zero_;
vector<tf2::Vector3> zero_vec_;

void timerCallback(const ros::TimerEvent&);
void publishForce(const size_t sensor_no, const tf2::Vector3 &force_vec);
bool getParams(ros::NodeHandle nh);
void sigintCallback(int sig);

// TODO: parametrize sensor count
int main(int argc, char **argv){

  ros::init(argc, argv, "optoforce_publisher");
  ROS_INFO("Optoforce publisher node");

  ros::NodeHandle nh;

  // Create an OptoDAQWatcher instance that can enumerate connected DAQs via USB
  OptoDAQWatcher watcher;
	watcher.Start();  // Start the watcher on a different thread

	OptoDAQDescriptor descriptors[16];

  // Trying to get connected DAQs (max.: 16, it can be changed up to 64)
  count_ = watcher.GetConnectedDAQs(descriptors, 16, true);
  while (count_ == 0) {
    count_ = watcher.GetConnectedDAQs(descriptors, 16, true);
  }

  std::string info_str = "";
  // Show information about connected DAQs
	for (std::size_t i = 0; i < count_; ++i) {
		info_str += "Information about Connected DAQ (" + to_string(i + 1) + "):\n";
		info_str += "Connected on port: " + string(descriptors[i].GetAddress()) + "\n";
		info_str += "Protocol version: " + to_string(descriptors[i].GetProtocolVersion()) + "\n";
		info_str += "S/N:" + string(descriptors[i].GetSerialNumber()) + "\n";
		info_str += "Type name:" + string(descriptors[i].GetTypeName()) + "\n";
		info_str += "-----------------------\n";

    ROS_INFO("Optoforce publisher: %s", info_str.c_str());
	}

  // Open all the connected DAQs
	daqs_ = new OptoDAQ[count_];
	for (std::size_t i = 0; i < count_; ++i) {
		daqs_[i].SetOptoDAQDescriptor(descriptors[i]);
		bool success = daqs_[i].Open();
		if (success == false) {
			ROS_ERROR("%d. DAQ could not be opened!", (int)i+1);
			continue;
		}
		OptoConfig config = OptoConfig(pack_freq_, 4, 0);
		success = daqs_[i].SendConfig(config); // Set up the speed to 100 Hz and filtering to 15 Hz
		if (success) {
			ROS_INFO("%d. DAQ successfully configured.", (int)i+1);
		}
		else {
			ROS_ERROR("%d. DAQ could not be configured.", (int)i+1);
			continue;
		}
		daqs_[i].RequestSensitivityReport(); // This call is a must
	}

  // attempt to get ros parameters
  if(!getParams(nh)) return -1;

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  // create wrench publisher to visualize in rviz
  for (int si=0; si<4; si++){
    wrench_publisher_vec_.push_back(
      nh.advertise<geometry_msgs::WrenchStamped>("optoforce_wrench_"+to_string(si), 3));
  }

  // create tf listener
  tf2_ros::TransformListener tfListener(tf_buffer_);

  // start timer loop
  ros::Timer timer = nh.createTimer(ros::Duration(1/pack_freq_), timerCallback);

  // wait for other processes
  ros::spin();

  // clean-up
  delete[] daqs_;

  return 0;

}

// get external parameters if possible
bool getParams(ros::NodeHandle nh){
  // get invert parameter
  if(!ros::param::get("~invert_optoforce", invert_) ){
    ROS_ERROR("Optoforce publisher: Can't find invert_optoforce param.");
    return false;
  }

  // get zero parameter
  if(!ros::param::get("~zero_optoforce", zero_) ){
    ROS_ERROR("Optoforce publisher: Can't find zero_optoforce param.");
    return false;
  }
  zero_vec_.resize(4, tf2::Vector3());

  // get frame name params
  if(!ros::param::get("~frame_names", frame_names_) ){
    ROS_ERROR("Optoforce publisher: Can't find frame_names param.");
    return false;
  }

  return true;

}

// timer loop. reads packets from DAQ and publishes for ROS.
void timerCallback(const ros::TimerEvent&){

  // Get a packet from every opened DAQs
	for (std::size_t i = 0; i < count_; ++i) {
    OptoPacket3D packet;

		if (daqs_[i].IsValid()) {
			daqs_[i].GetLastPacket3D(&packet, true); // non-blocking call

      if (!packet.IsValid()) {
        continue;
      }
		}else{
      ROS_WARN("Optoforce publisher: invalid DAQ!");
      continue;
    }
    // Value to scale and invert (optionally) force
    double scaler = 0.08;
    double inverter = 1.0;
    if(invert_) inverter *= -1.0;
		// Show the captured packet's F values
		std::size_t size = packet.GetSize(); // It should be the number of sensors.
		for (std::size_t j = 0; j < size; ++j) {
      tf2::Vector3 f_vec;
      f_vec.setX(packet.GetFxInCounts(j) * scaler * inverter - zero_vec_[j].getX());
      f_vec.setY(packet.GetFyInCounts(j) * scaler * inverter - zero_vec_[j].getY());
      f_vec.setZ(packet.GetFzInCounts(j) * scaler * inverter - zero_vec_[j].getZ());

      unsigned short pid = packet.GetPacketCounter();
      unsigned short status = packet.GetStatus();

			ROS_DEBUG("Optoforce publisher: Sensor %d Sample %d \nForce (%f, %f, %f) Status %d",
                  (int)j, pid, f_vec.getX(), f_vec.getY(), f_vec.getZ(), status);

      publishForce(j, f_vec);

      // zero the sensors at the first reading
      if(zero_) zero_vec_[j] = f_vec;
		}

    // zero only once
    zero_ = false;
	}
}

void publishForce(const size_t sensor_no, const tf2::Vector3 &force_vec){

  // create wrench message and publish
  geometry_msgs::WrenchStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_names_[sensor_no];

  msg.wrench.force.x = force_vec.getX();
  msg.wrench.force.y = force_vec.getY();
  msg.wrench.force.z = force_vec.getZ();

  msg.wrench.torque.x = 0.0;
  msg.wrench.torque.y = 0.0;
  msg.wrench.torque.z = 0.0;

  wrench_publisher_vec_[sensor_no].publish(msg);

}

// This procedure is called when the ros node is interrupted
void sigintCallback(int sig)
{
  // send and empty wrench message
  for(int si=0; si<4; si++)
    publishForce(si, tf2::Vector3());

  // spin once to hand communication
  ros::spinOnce();

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}
