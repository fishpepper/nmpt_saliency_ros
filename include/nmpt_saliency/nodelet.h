/******************************************************************************

Copyright 2016  Simon Schulz (University of Bielefeld)
                      [sschulz@techfak.uni-bielefeld.de]

********************************************************************************/

#ifndef INCLUDE_NMPT_SALIENCY_NODELET_H_
#define INCLUDE_NMPT_SALIENCY_NODELET_H_
#include <signal.h>

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <nmpt/BlockTimer.h>
#include <nmpt/FastSalience.h>
#include <nmpt/LQRPointTracker.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <vector>

#include "nmpt_saliency/nmpt_saliencyConfig.h"

namespace nmpt_saliency {

class Nodelet : public nodelet::Nodelet{
 public:
    Nodelet();
    ~Nodelet();

 private:
    void publishSaliencyImage(const cv::Mat &saliency_image, ros::Time timestamp);
    void publishSalientPointsImage(const cv::Mat &saliency_image, ros::Time timestamp);
    virtual void onInit();
    void imageCallback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
    void dynamicReconfigureCallback(const nmpt_saliency::nmpt_saliencyConfig &config,
                                    uint32_t level);
    void connectCb();
    FastSalience salTracker;

    volatile bool running_;
    boost::mutex connect_mutex_;
    dynamic_reconfigure::Server<nmpt_saliency::nmpt_saliencyConfig> *reconfig_server_;
    image_transport::ImageTransport *image_transport_;
    image_transport::CameraSubscriber image_subscriber_;
    // image_transport::Publisher saliency_image_publisher_;
    // image_transport::Publisher salient_spot_image_publisher_;
    ros::Publisher saliency_image_publisher_;
    ros::Publisher salient_spot_image_publisher_;
    ros::Publisher salient_spot_publisher_;

    std::vector<double> lqrpt;
    boost::mutex salient_spot_mutex_;
    boost::shared_ptr<LQRPointTracker> salient_spot_ptr_;
    std::vector<cv::KeyPoint> keypoints_;
    bool color_saliency_image_;
};

}  // namespace nmpt_saliency

#endif  // INCLUDE_NMPT_SALIENCY_NODELET_H_

