//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
/**
 * Copy of the CameraSensor/DepthCameraSensor plugin with minor changes
 */

/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// camera stuff
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

#include <tf2_msgs/TFMessage.h>
#include <mrs_msgs/Float64MultiArrayStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <mrs_lib/image_publisher.h>

#include <gazebo/plugins/CameraPlugin.hh>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo/gazebo_config.h>
//#include "ros/ros.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

#define DEBUG true

#define R_THRESH 254
#define B_THRESH 1

namespace gazebo {

template<typename Base>
class GazeboRosThermalCamera_: public Base, GazeboRosCameraUtils {
	/// \brief Constructor
	/// \param parent The parent entity, must be a Model or a Sensor
public:
	GazeboRosThermalCamera_();

	/// \brief Destructor
public:
	~GazeboRosThermalCamera_();

	/// \brief Load the plugin
	/// \param take in SDF root element
public:
	void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
public:
	void LoadImpl(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
	}

private:
  std::string parent_frame_name_;
  std::string base_frame_name_;

  sensors::CameraSensorPtr cameraParentSensor_;

  /// \brief frame transform parameters
private:
  double x_, y_, z_, roll_, pitch_, yaw_;

protected:
  ros::Publisher      tf_pub_;
  tf2_msgs::TFMessage tf_message_;
  ros::WallTimer      timer_;
  boost::thread       load_thread_;
  void                publishStaticTransforms(const ros::WallTimerEvent &event);
  void                createStaticTransforms();
  void                TransformThread();

	/// \brief Update the controller
protected:
	virtual void OnNewFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format);

	/// \brief Update the controller
protected:
	virtual void OnNewImageFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth,
			const std::string &_format);

	/// \brief Put camera data to the ROS topic
protected:
	void PutCameraData(const unsigned char *_src);
protected:
	void PutCameraData(const unsigned char *_src, common::Time &last_update_time);

private:
	double randomTempNoise(double temp);

	ros::Publisher raw_thermal_publisher;
	ros::Publisher raw_thermal_stamped_publisher;
	std::string raw_thermal_topicname;
	double surroundingTemperature;
	double maximalTemperature;
	double minimalTemperatureGreenColor;
	double minimalTemperatureGreenColorIntensity;
	double noiseStdDev;
	double noiseStdDevMaxTemp;

  mrs_lib::ImagePublisher *image_publisher;

  /* ros::Publisher debug_image; */

	std::normal_distribution<double> noise;
	std::default_random_engine generator;

};

template <>
void GazeboRosThermalCamera_<CameraPlugin>::LoadImpl(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  this->camera_ = this->CameraPlugin::camera;
}

template class GazeboRosThermalCamera_<CameraPlugin>;
typedef GazeboRosThermalCamera_<CameraPlugin> GazeboRosThermalCamera;

////////////////////////////////////////////////////////////////////////////////
// Constructor
template<class Base>
GazeboRosThermalCamera_<Base>::GazeboRosThermalCamera_() {
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
template<class Base>
GazeboRosThermalCamera_<Base>::~GazeboRosThermalCamera_() {
}

template<class Base>
void GazeboRosThermalCamera_<Base>::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
	Base::Load(_parent, _sdf);
	// copying from CameraPlugin into GazeboRosCameraUtils
	this->parentSensor_ = this->parentSensor;
	this->cameraParentSensor_ = std::dynamic_pointer_cast<sensors::CameraSensor>(_parent);

	this->width_ = this->width;
	this->height_ = this->height;
	this->depth_ = this->depth;
	this->format_ = this->format;

	this->image_connect_count_ = boost::shared_ptr<int>(new int);
	*this->image_connect_count_ = 0;
	this->image_connect_count_lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);
	this->was_active_ = boost::shared_ptr<bool>(new bool);
	*this->was_active_ = false;
	//ROS_INFO_STREAM("creating mrs thermal camera 3 ");
	LoadImpl(_parent, _sdf);
	GazeboRosCameraUtils::Load(_parent, _sdf);

	surroundingTemperature = 0;
	maximalTemperature = 0;
	minimalTemperatureGreenColor = 0;

	raw_thermal_topicname = _sdf->GetElement("rawTemperatureTopicName")->GetValue()->GetAsString();
	_sdf->GetElement("surroundingTemperature")->GetValue()->Get(surroundingTemperature);
	_sdf->GetElement("maximalTemperature")->GetValue()->Get(maximalTemperature);
	_sdf->GetElement("minimalTemperatureGreenColor")->GetValue()->Get(minimalTemperatureGreenColor);
	_sdf->GetElement("noiseStdDev")->GetValue()->Get(noiseStdDev);
	_sdf->GetElement("noiseStdDevMaxTemp")->GetValue()->Get(noiseStdDevMaxTemp);


	minimalTemperatureGreenColorIntensity = minimalTemperatureGreenColor * 255;

	//noise = std::normal_distribution<double>(0.0, noiseStdDev);
	srand(time(NULL));

	ROS_INFO_STREAM("surroundingTemperature " << surroundingTemperature);


  /* load TF-related parameters //{ */
  
  // Get parameters from sdf
  if (!_sdf->HasElement("parentFrameName")) {
    ROS_INFO_NAMED("thermal", "thermal plugin missing <parentFrameName>, defaults to \"world\"");
    this->parent_frame_name_ = "world";
  } else
    this->parent_frame_name_ = _sdf->Get<std::string>("parentFrameName");

  if (!_sdf->HasElement("sensorBaseFrameName")) {
    ROS_INFO_NAMED("thermal", "thermal plugin missing <sensorBaseFrameName>, defaults to \"sensor_base\"");
    this->base_frame_name_ = "sensor_base";
  } else
    this->base_frame_name_ = _sdf->Get<std::string>("sensorBaseFrameName");
  
  if (!_sdf->HasElement("x")) {
    ROS_INFO_NAMED("thermal", "thermal plugin missing <x>, defaults to 0");
    this->x_ = 0;
  } else
    this->x_ = _sdf->Get<double>("x");
  
  if (!_sdf->HasElement("y")) {
    ROS_INFO_NAMED("thermal", "thermal plugin missing <y>, defaults to 0");
    this->y_ = 0;
  } else
    this->y_ = _sdf->Get<double>("y");
  
  if (!_sdf->HasElement("z")) {
    ROS_INFO_NAMED("thermal", "thermal plugin missing <z>, defaults to 0");
    this->z_ = 0;
  } else
    this->z_ = _sdf->Get<double>("z");
  
  if (!_sdf->HasElement("roll")) {
    ROS_INFO_NAMED("thermal", "thermal plugin missing <roll>, defaults to 0");
    this->roll_ = 0;
  } else
    this->roll_ = _sdf->Get<double>("roll");
  
  if (!_sdf->HasElement("pitch")) {
    ROS_INFO_NAMED("thermal", "thermal plugin missing <pitch>, defaults to 0");
    this->pitch_ = 0;
  } else
    this->pitch_ = _sdf->Get<double>("pitch");
  
  if (!_sdf->HasElement("yaw")) {
    ROS_INFO_NAMED("thermal", "thermal plugin missing <yaw>, defaults to 0");
    this->yaw_ = 0;
  } else
    this->yaw_ = _sdf->Get<double>("yaw");
  
  //}
  this->load_thread_ = boost::thread(boost::bind(&GazeboRosThermalCamera_::TransformThread, this));

  /* ROS_INFO_STREAM("[thermal camera]: Initial FSAA is " << this->cameraParentSensor_->Camera()->RenderTexture()->getFSAA()); */
  /* this->cameraParentSensor_->Camera()->RenderTexture()->unload(); */
  /* this->cameraParentSensor_->Camera()->RenderTexture()->unload(); */
  /* this->cameraParentSensor_->Camera()->RenderTexture()->setFSAA(0, Ogre::StringUtil::BLANK); */
  /* this->cameraParentSensor_->Camera()->RenderTexture()->getBuffer()->getRenderTarget()->removeAllViewports(); */
  /* this->cameraParentSensor_->Camera()->SetRenderTarget(this->cameraParentSensor_->Camera()->RenderTexture()->getBuffer()->getRenderTarget()); */
  /* this->cameraParentSensor_->Camera()->RenderTexture()->load(); */
  /* ROS_INFO_STREAM("[thermal camera]: Usage is " << this->cameraParentSensor_->Camera()->RenderTexture()->getUsage()); */
  /* ROS_INFO_STREAM("[thermal camera]: Usage should be " << (Ogre::TU_RENDERTARGET)); */
  /* ROS_INFO_STREAM("[thermal camera]: After setting FSAA is " << this->cameraParentSensor_->Camera()->RenderTexture()->getFSAA()); */

  sdf::ElementPtr imgElem = _sdf->GetElement("image");
  Ogre::Texture *temp;
  temp = (Ogre::TextureManager::getSingleton().createManual(
      "CUSTOM_RttTex",
      "General",
      Ogre::TEX_TYPE_2D,
      this->width_,
      this->height_,
      0,
      this->cameraParentSensor_->Camera()->RenderTexture()->getFormat(),
      Ogre::TU_RENDERTARGET,
      0,
      false,
      0)).getPointer();

  this->cameraParentSensor_->Camera()->SetRenderTarget(temp->getBuffer()->getRenderTarget());



	this->parentSensor->SetActive(true);
	ros::NodeHandle n;
	raw_thermal_publisher = n.advertise < std_msgs::Float64MultiArray > (raw_thermal_topicname, 1);
  /* debug_image = n.advertise<sensor_msgs::Image>("gazebo_debug_image", 1); */
  image_publisher = new mrs_lib::ImagePublisher(boost::make_shared<ros::NodeHandle>(n));
	std::stringstream ss_stamped_thermal;
	ss_stamped_thermal << raw_thermal_topicname << "_stamped";
	raw_thermal_stamped_publisher = n.advertise < mrs_msgs::Float64MultiArrayStamped > (ss_stamped_thermal.str(), 1);
}

////////////////////////////////////////////////////////////////////////////////
template<class Base>
void GazeboRosThermalCamera_<Base>::TransformThread() {
  while (!this->initialized_) {
    ros::Duration(0.01).sleep();
  }
  ROS_INFO_NAMED("camera", "camera plugin - GazeboRosThermalCamera_ initialized");
  this->tf_pub_ = this->rosnode_->advertise<tf2_msgs::TFMessage>("/tf_gazebo_static", 10, false);

  createStaticTransforms();
  this->timer_ = this->rosnode_->createWallTimer(ros::WallDuration(1.0), &GazeboRosThermalCamera_::publishStaticTransforms, this);
}

////////////////////////////////////////////////////////////////////////////////
template<class Base>
void GazeboRosThermalCamera_<Base>::createStaticTransforms() {
  // frame describing sensor main baseline frame in drone frame
  geometry_msgs::TransformStamped static_transform_base_world;

  // frame describing camera's baseline frames in sensor main baseline
  geometry_msgs::TransformStamped static_transform_camera_base;

  static_transform_base_world.header.stamp            = ros::Time::now();
  static_transform_base_world.header.frame_id         = this->parent_frame_name_;
  static_transform_base_world.child_frame_id          = tf::resolve(std::string(), this->base_frame_name_);
  static_transform_base_world.transform.translation.x = this->x_;
  static_transform_base_world.transform.translation.y = this->y_;
  static_transform_base_world.transform.translation.z = this->z_;
  tf2::Quaternion quat;
  quat.setRPY(this->roll_, this->pitch_, this->yaw_);
  static_transform_base_world.transform.rotation.x = quat.x();
  static_transform_base_world.transform.rotation.y = quat.y();
  static_transform_base_world.transform.rotation.z = quat.z();
  static_transform_base_world.transform.rotation.w = quat.w();

  static_transform_camera_base.header.stamp    = static_transform_base_world.header.stamp;
  static_transform_camera_base.header.frame_id = static_transform_base_world.child_frame_id;
  static_transform_camera_base.child_frame_id  = tf::resolve(std::string(), this->frame_name_);

  quat.setRPY(-M_PI_2, 0, -M_PI_2);
  static_transform_camera_base.transform.rotation.x = quat.x();
  static_transform_camera_base.transform.rotation.y = quat.y();
  static_transform_camera_base.transform.rotation.z = quat.z();
  static_transform_camera_base.transform.rotation.w = quat.w();

  this->tf_message_.transforms.push_back(static_transform_base_world);
  this->tf_message_.transforms.push_back(static_transform_camera_base);
  /* ROS_INFO_NAMED("camera", "created"); */
}

////////////////////////////////////////////////////////////////////////////////
template<class Base>
void GazeboRosThermalCamera_<Base>::publishStaticTransforms(const ros::WallTimerEvent &event) {
  /* ROS_INFO("publishing"); */
  this->tf_pub_.publish(this->tf_message_);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
template<class Base>
void GazeboRosThermalCamera_<Base>::OnNewFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth,
		const std::string &_format) {
	//ROS_INFO_STREAM("OnNewFrame " << this->initialized_);
	if (!this->initialized_ || this->height_ <= 0 || this->width_ <= 0)
		return;

#if (GAZEBO_MAJOR_VERSION > 6)
  this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
#else
	this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
#endif

	if (!this->parentSensor->IsActive()) {
		if ((*this->image_connect_count_) > 0 || this->raw_thermal_publisher.getNumSubscribers() > 0 || this->raw_thermal_stamped_publisher.getNumSubscribers() > 0)
			// do this first so there's chance for sensor to run 1 frame after activate
			this->parentSensor->SetActive(true);
	} else {
		if ((*this->image_connect_count_) > 0 || this->raw_thermal_publisher.getNumSubscribers() > 0 || this->raw_thermal_stamped_publisher.getNumSubscribers() > 0) {
#if (GAZEBO_MAJOR_VERSION >= 8)
      common::Time cur_time = this->world_->SimTime();
#else
			common::Time cur_time = this->world_->GetSimTime();
#endif
			if (cur_time - this->last_update_time_ >= this->update_period_) {
				this->PutCameraData(_image);
				this->PublishCameraInfo();
				this->last_update_time_ = cur_time;
			}
		}
	}
}

template<class Base>
void GazeboRosThermalCamera_<Base>::OnNewImageFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth,
		const std::string &_format) {
	OnNewFrame(_image, _width, _height, _depth, _format);
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
template<class Base>
void GazeboRosThermalCamera_<Base>::PutCameraData(const unsigned char *_src, common::Time &last_update_time) {
	this->sensor_update_time_ = last_update_time;
	this->PutCameraData(_src);
}

template<class Base>
double GazeboRosThermalCamera_<Base>::randomTempNoise(double temp) {
	double rand_range = this->noiseStdDev
			+ std::fabs(this->noiseStdDevMaxTemp - this->noiseStdDev) * (temp - surroundingTemperature) / this->maximalTemperature;

	double rand_temp = ((double) rand() * (rand_range) / (double) RAND_MAX - rand_range / 2.0);
	return rand_temp;
}

template<class Base>
void GazeboRosThermalCamera_<Base>::PutCameraData(const unsigned char *_src) {
	if (!this->initialized_ || this->height_ <= 0 || this->width_ <= 0)
		return;

	this->lock_.lock();

	// copy data into image
	this->image_msg_.header.frame_id = this->frame_name_;
	this->image_msg_.header.stamp.sec = this->sensor_update_time_.sec;
	this->image_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

	/* ROS_INFO_STREAM("this->raw_thermal_publisher.getNumSubscribers() " << this->raw_thermal_publisher.getNumSubscribers()); */

	/// don't bother if there are no subscribers
	if ((*this->image_connect_count_) > 0 || this->raw_thermal_publisher.getNumSubscribers() > 0 || this->raw_thermal_stamped_publisher.getNumSubscribers() > 0) {
		//ROS_INFO_STREAM("PutCameraData " << this->raw_thermal_publisher.getNumSubscribers());

		/* size_t size = this->width_ * this->height_; */

		/* std::vector < uint8_t > &data(this->image_msg_.data); */
    cv::Mat image_vis(cv::Size(this->width_/2, this->height_/2),CV_8UC1);
    /* image_vis.step = image_vis.cols; */
    cv::Mat image_proc(cv::Size(this->width_, this->height_),CV_64FC1);
    /* image_proc.step = image_proc.cols; */

    cv::Mat image_temp(cv::Size(this->width_, this->height_),CV_64FC1);
    /* image_temp.step = image_temp.cols; */
    cv::Mat image_bg(cv::Size(this->width_, this->height_),CV_64FC1);
    /* image_bg.step = image_bg.cols; */
    cv::Mat image_rw_temp(cv::Size(this->width_/2, this->height_/2),CV_64FC1);
    /* image_rw_temp.step = image_rw_temp.cols; */
		/* std::vector<double> data_rw_temp; */
		/* data_rw_temp.resize(size); */

		double maximalTemperature_without_surrounding = maximalTemperature - surroundingTemperature;
		double image_scale = 255.0 / (maximalTemperature + this->noiseStdDevMaxTemp / 2.0);
		size_t img_index = 0;
		double sum_empty = 0;
		int num_empty = 0;
    cv::Mat raw_input(cv::Size(this->width_, this->height_),CV_8UC3);
    for (int j=0; j<image_temp.rows; j++)
      for (int i=0; i<image_temp.cols; i++){
        raw_input.at<cv::Vec3b>(cv::Point(i,j)) = cv::Vec3b(_src[img_index], _src[img_index+1], _src[img_index+2]);
			if ((_src[img_index] > R_THRESH) && (_src[img_index + 1] < minimalTemperatureGreenColorIntensity) && (_src[img_index + 2] < B_THRESH)) {
				//RGB [255,0,0] translates to white (white hot)
				//data[i] = 255 * (1.0 - (double) _src[img_index + 1] / ((double) minimalTemperatureGreenColorIntensity));

				double temp = surroundingTemperature
						+ maximalTemperature_without_surrounding * (1.0 - ((double) _src[img_index + 1] / minimalTemperatureGreenColorIntensity));

				/* image_temp.at<double>(cv::Point(i,j)) = round(temp + randomTempNoise(temp)); */
				image_temp.at<double>(cv::Point(i,j)) = temp;

				//ROS_INFO_STREAM(i << " data[i] " << image_scale * data_rw_temp[i]);

			} else {
				//Everything else is written to the MONO8 output image much darker
				image_bg.at<double>(cv::Point(i,j)) = (double)((3*255) - (_src[img_index] + _src[img_index + 1] + _src[img_index + 2]) )/3.0 / 32.0;
				image_bg.at<double>(cv::Point(i,j)) += surroundingTemperature;
        image_temp.at<double>(cv::Point(i,j)) = 0;
				sum_empty += image_bg.at<double>(cv::Point(i,j));
				num_empty += 1;
			}
			img_index += 3;
		}
    if (DEBUG)
      image_publisher->publish("gazebo_image_debug_raw_" + raw_thermal_topicname , 0.2, raw_input);


    unsigned int dilation_size = 1;
    /* unsigned int dilation_size = 0; */
    cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );
    /* cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, */
    /*                    cv::Size(2, 2), */
    /*                    cv::Point( 1, 1 ) ); */

    /* cv_bridge::CvImage br_image_msg; */
    /* br_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv::Mat(image_temp.rows, image_temp.cols, CV_8UC1, cv::Scalar(0))); */
    cv::Mat image_temp_q;
    image_temp.convertTo(image_temp_q, CV_8U );
    /* debug_image.publish(br_image_msg.toImageMsg()); */
    if (DEBUG)
      image_publisher->publish("gazebo_image_debug_threshold_" + raw_thermal_topicname , 0.2, image_temp_q);

    /* cv::dilate(image_temp,image_proc,element); */
    image_proc = image_temp;
    /* image_proc = image_temp; */
    
    cv::Mat image_proc_downscaled(cv::Size(32,32),CV_64FC1);
    cv::Mat image_bg_downscaled(cv::Size(32,32),CV_64FC1);
    /* image_proc_downscaled.step = image_proc_downscaled.cols; */
    /* image_bg_downscaled.step = image_bg_downscaled.cols; */
    cv::resize(image_proc,image_proc_downscaled,cv::Size(32,32),0,0,cv::INTER_AREA);
    cv::resize(image_bg,image_bg_downscaled,cv::Size(32,32),0,0,cv::INTER_NEAREST);
    /* image_proc_downscaled = image_proc; */
    /* image_bg_downscaled = image_bg; */

		/* double avg_empty = sum_empty / (double) num_empty; */
		//ROS_INFO_STREAM("avg_empty " << avg_empty);
		/* for (size_t i = 0; i < size/4; i+=8) { */
    image_rw_temp = image_proc_downscaled+image_bg_downscaled;
    for (int j=0; j<image_rw_temp.rows; j++)
      for (int i=0; i<image_rw_temp.cols; i++){
			/* if (image_proc_downscaled.at<double>(cv::Point(i,j)) <0.0001) { */
			/* /1* if (image_proc_downscaled.data[i] == 0) { *1/ */
			/* /1* if (false) { *1/ */
			/* 	image_rw_temp.at<double>(cv::Point(i,j)) = image_bg_downscaled.at<double>(cv::Point(i,j)) - avg_empty; */
			/* } else{ */
        /* image_rw_temp.at<double>(cv::Point(i,j))= image_proc_downscaled.at<double>(cv::Point(i,j)); */
      /* } */
      image_rw_temp.at<double>(cv::Point(i,j)) = image_rw_temp.at<double>(cv::Point(i,j))+ randomTempNoise(image_rw_temp.at<double>(cv::Point(i,j)));
      /* image_rw_temp.data[i] = image_rw_temp.data[i] + randomTempNoise(image_rw_temp.data[i]); */
      image_vis.at<unsigned char>(cv::Point(i,j)) = std::max(0.0,std::min(255.0, image_scale * image_rw_temp.at<double>(cv::Point(i,j))));
      /* ROS_INFO_STREAM(image_rw_temp.at<double>(cv::Point(i,j))); */
      /* ROS_INFO_STREAM("BG: " << image_bg_downscaled.at<double>(cv::Point(i,j))); */
      /* ROS_INFO_STREAM("FG: " << image_proc_downscaled.at<double>(cv::Point(i,j))); */
		}
    /* ROS_INFO_STREAM(image_bg); */

    /* ROS_INFO_STREAM(image_raw); */
    /* cv::waitKey(10); */
    /* cv::imshow("image_raw_dilated",image_raw); */
    /* cv::waitKey(10); */

		std_msgs::Float64MultiArray raw_temp_msg;
		raw_temp_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
		raw_temp_msg.layout.dim[0].size = 32;
		raw_temp_msg.layout.dim[0].stride = 1;
		raw_temp_msg.layout.dim[0].label = "x-axis";
		raw_temp_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
		raw_temp_msg.layout.dim[1].size = 32;
		raw_temp_msg.layout.dim[1].stride = 1;
		raw_temp_msg.layout.dim[1].label = "y-axis";
    for (int j=0; j<image_rw_temp.rows; j++)
      for (int i=0; i<image_rw_temp.cols; i++){
      raw_temp_msg.data.push_back(image_rw_temp.at<double>(cv::Point(i,j)));
    }
		/* raw_temp_msg.data = data_rw_temp; */
		this->raw_thermal_publisher.publish(raw_temp_msg);

    mrs_msgs::Float64MultiArrayStamped raw_temp_stamped_msg;
    raw_temp_stamped_msg.header = this->image_msg_.header;
    raw_temp_stamped_msg.matrix = raw_temp_msg;
    this->raw_thermal_stamped_publisher.publish(raw_temp_stamped_msg);

		// publish to ros
		this->image_msg_.width = this->width_/3;
		this->image_msg_.height = this->height_/3;
		this->image_msg_.encoding = sensor_msgs::image_encodings::MONO8;
		this->image_msg_.step = this->image_msg_.width;

    this->image_msg_.data.resize((this->image_msg_.step)*(this->image_msg_.height));
    int index = 0;
    for (int j=0; j<image_rw_temp.rows; j++)
      for (int i=0; i<image_rw_temp.cols; i++){
      /* this->image_msg_.data[i] = image_proc.data[i]; */
      this->image_msg_.data[index] = image_vis.at<unsigned char>(cv::Point(i,j));
      index++;
    }
		this->image_pub_.publish(this->image_msg_);
	}

	this->lock_.unlock();
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosThermalCamera)

}
