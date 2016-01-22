/******************************************************************************

Copyright 2016  Simon Schulz (University of Bielefeld)
                      [sschulz@techfak.uni-bielefeld.de]

All rights reserved.

********************************************************************************/

#include "nmpt_saliency/nodelet.h"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>

#include <string>
#include <vector>

using nmpt_saliency::Nodelet;
using std::vector;

Nodelet::Nodelet() : running_(false), lqrpt(2, 0.5) {
    salient_spot_ptr_ = boost::shared_ptr<LQRPointTracker>(new LQRPointTracker(2, 1.0, 0, .015));
    // nothing to do
    NODELET_INFO("nmpt_saliency: Nodelet initialized");
}

Nodelet::~Nodelet() {
    image_subscriber_.shutdown();
    saliency_image_publisher_.shutdown();
    salient_spot_image_publisher_.shutdown();
    salient_spot_publisher_.shutdown();
}

void Nodelet::imageCallback(const sensor_msgs::ImageConstPtr& image,
                                    const sensor_msgs::CameraInfoConstPtr& info) {
    // extract image from msg:
    cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image);

    if (image->width >= 640) {
        NODELET_DEBUG_ONCE("nmpt_saliency:  WARNING: nmpt saliency - do not use huge images as "
                           "input, use crop_decimate to input smaller images into this nodelet.");
    }

    keypoints_.clear();

    // detect
    salTracker.detect(cv_image->image, keypoints_);

    cv::Mat saliency_image;
    salTracker.getSalImage(saliency_image);

    // fetch salient spots
    double min, max;
    cv::Point minloc, maxloc;

    minMaxLoc(saliency_image, &min, &max, &minloc, &maxloc);

    lqrpt[0] = maxloc.x * 1.0 / saliency_image.cols;
    lqrpt[1] = maxloc.y * 1.0 / saliency_image.rows;

    boost::interprocess::scoped_lock<boost::mutex> lock(salient_spot_mutex_);
    salient_spot_ptr_->setTrackerTarget(lqrpt);
    salient_spot_ptr_->updateTrackerPosition();
    lqrpt = salient_spot_ptr_->getCurrentPosition();
    lock.unlock();

    // publish salient point
    geometry_msgs::PointStamped ps;
    geometry_msgs::Point p;
    double mid_x = lqrpt[0] * saliency_image.cols;
    double mid_y = lqrpt[1] * saliency_image.rows;
    p.x = mid_x;
    p.y = mid_y;
    p.z = keypoints_.size();
    ps.point = p;
    ps.header.stamp = image->header.stamp;
    salient_spot_publisher_.publish(ps);

    // if requested publish saliency debug image
    if (saliency_image_publisher_.getNumSubscribers()) {
        publishSaliencyImage(saliency_image, image->header.stamp);
    }

    // if requested publish salient spots debug image
    if (salient_spot_image_publisher_.getNumSubscribers()) {
        publishSalientPointsImage(cv_image->image, image->header.stamp);
    }
}

void Nodelet::publishSalientPointsImage(const cv::Mat &image, ros::Time timestamp) {
    // there is a subscriber, publish nice debug image
    cv::Mat salient_point_image = image.clone();

    // draw detections:
    for (size_t i = 0; i < keypoints_.size(); i++) {
        cv::circle(salient_point_image, keypoints_[i].pt, 2, CV_RGB(0, 255, 0));
    }

    // draw point of interest:
    cv::Point poi = cv::Point(lqrpt[0] * salient_point_image.cols,
                              lqrpt[1] * salient_point_image.rows);
    circle(salient_point_image, poi, 6, CV_RGB(0, 0, 255));
    circle(salient_point_image, poi, 5, CV_RGB(0, 0, 255));
    circle(salient_point_image, poi, 4, CV_RGB(255, 255, 0));
    circle(salient_point_image, poi, 3, CV_RGB(255, 255, 0));

    std_msgs::Header header;
    header.stamp = timestamp;
    sensor_msgs::ImagePtr saliency_msg = cv_bridge::CvImage(header, "bgr8",
                                                            salient_point_image).toImageMsg();
    salient_spot_image_publisher_.publish(saliency_msg);
}

void Nodelet::publishSaliencyImage(const cv::Mat &saliency_image, ros::Time timestamp) {
    // subscribers to saliency topic -> publish image
    cv::Mat saliency_image_out;

    // convert to nice image:
    double min, max;
    cv::minMaxIdx(saliency_image, &min, &max);

    // histogram Equalization
    cv::Mat tmp;
    float scale = 255 / (max-min);

    std::string encoding;
    if (0) {
        // nice colored image
        // first map to grayscale
        saliency_image.convertTo(tmp, CV_8UC1, scale, -min*scale);
        // and finally add a nice color map
        encoding = "bgr8";
        cv::applyColorMap(tmp, saliency_image_out, cv::COLORMAP_RAINBOW);
    } else {
        // only grayscale
        encoding = "mono8";
        saliency_image.convertTo(saliency_image_out, CV_8UC1, scale, -min*scale);
    }

    std_msgs::Header header;
    header.stamp = timestamp;
    sensor_msgs::ImagePtr saliency_msg = cv_bridge::CvImage(header, encoding,
                                                            saliency_image_out).toImageMsg();
    saliency_image_publisher_.publish(saliency_msg);
}

// Handles (un)subscribing when clients (un)subscribe
void Nodelet::connectCb() {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    if (
            (saliency_image_publisher_.getNumSubscribers() == 0) &&
            (salient_spot_image_publisher_.getNumSubscribers() == 0) &&
            (salient_spot_publisher_.getNumSubscribers() == 0)
            ) {
        NODELET_INFO("nmpt_saliency: no more subscribers on saliency topics, shutting down "
                     "image subscriber");
        image_subscriber_.shutdown();
    } else if (!image_subscriber_) {
        NODELET_INFO("nmpt_saliency: new subscriber on saliency topic, subscribing to image topic");
        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
        image_subscriber_ = image_transport_->subscribeCamera(
                    "image_color", 1, boost::bind(&Nodelet::imageCallback, this, _1, _2));
    }
}

void Nodelet::onInit() {
    NODELET_INFO("nmpt_saliency: Nodelet::onInit()");

    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    ros::NodeHandle node(getNodeHandle());

    image_transport_ = new image_transport::ImageTransport(priv_nh);


    // Monitor whether anyone is subscribed to the output
    image_transport::SubscriberStatusCallback imagetransport_connect_cb =
            boost::bind(&Nodelet::connectCb, this);

    ros::SubscriberStatusCallback connect_cb
            = boost::bind(&Nodelet::connectCb, this);

    // Make sure we don't enter connectCb() between advertising and assigning to pub_
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    saliency_image_publisher_ =
            image_transport_->advertise("image_saliency", 1,
                                        imagetransport_connect_cb, imagetransport_connect_cb);

    salient_spot_image_publisher_ =
            image_transport_->advertise("image_salient_spots", 1,
                                        imagetransport_connect_cb, imagetransport_connect_cb);

    salient_spot_publisher_ =
            priv_nh.advertise<geometry_msgs::PointStamped>("salient_spot", 10,
                                                        connect_cb, connect_cb);

    // attach to dyn reconfig server:
    NODELET_INFO("nmpt_saliency: connecting to dynamic reconfiguration server");
    ros::NodeHandle reconf_node(priv_nh, "parameters");
    reconfig_server_ =
            new dynamic_reconfigure::Server<nmpt_saliency::nmpt_saliencyConfig>(reconf_node);
    reconfig_server_->setCallback(boost::bind(&Nodelet::dynamicReconfigureCallback, this, _1, _2));

    running_ = true;
}


void Nodelet::dynamicReconfigureCallback(const nmpt_saliency::nmpt_saliencyConfig &config,
                                                    uint32_t level) {
    boost::interprocess::scoped_lock<boost::mutex> lock(salient_spot_mutex_);
    salient_spot_ptr_.reset(new LQRPointTracker(
                                2,
                                config.dt,
                                config.drag,
                                config.move_cost));

}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
PLUGINLIB_EXPORT_CLASS(nmpt_saliency::Nodelet, nodelet::Nodelet);
