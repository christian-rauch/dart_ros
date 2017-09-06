#ifndef ROSDEPTHSOURCE_HPP
#define ROSDEPTHSOURCE_HPP

#include <dart/depth_sources/depth_source.h>

#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include <mutex>


namespace dart {

template <typename DepthType, typename ColorType>
class RosDepthSource : public dart::DepthSource<DepthType,ColorType> {
public:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximateTimePolicy;

    RosDepthSource() : it(n) {
        this->_isLive = true;
        this->_hasColor = true;
        this->_hasTimestamps = true;
        this->threshold = 0;
    }

    ~RosDepthSource() {
        sync_conn.disconnect();
#ifdef CUDA_BUILD
        delete _depthData;
#else
        delete [] _depthData;
#endif
        delete [] _colorData;
    }

    uint64_t getDepthTime() const { return _depthTime; }

    uint64_t getColorTime() const { return _colourTime; }

    void setFrame(const uint frame) { }

    void advance() { }

    bool hasRadialDistortionParams() const {
        // DART expects 5 distortion parameters from the 'plumb_bob' distortion model
        return distortion_param.size()==5;
    }

    const float* getRadialDistortionParams() const {
        return distortion_param.data();
    }

#ifdef CUDA_BUILD
    /**
     * @brief getDepth
     * @return pointer to array containing depth values
     */
    const DepthType * getDepth() const {
        return _depthData->hostPtr();
    }

    /**
     * @brief getDeviceDepth
     * @return pointer to array containing depth values
     */
    const DepthType * getDeviceDepth() const {
        return _depthData->devicePtr();
    }
#else
    /**
     * @brief getDepth
     * @return pointer to array containing depth values
     */
    const DepthType * getDepth() const {
        return _depthData;
    }

    /**
     * @brief getDeviceDepth
     * @return pointer to array containing depth values
     */
    const DepthType * getDeviceDepth() const { return 0; }
#endif // CUDA_BUILD

    const ColorType * getColor() const { return _colorData; }

    bool setup(const std::string &caminfo_topic) {
        // wait for camera parameters
        const sensor_msgs::CameraInfoConstPtr ci = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(caminfo_topic);
        setCameraParameter(ci);
        std::cout << "set camera parameter" << std::endl;

        // allocate memory for depth image
#ifdef CUDA_BUILD
        _depthData = new dart::MirroredVector<DepthType>(this->_depthWidth*this->_depthHeight);
#else
        _depthData = new DepthType[this->_depthWidth*this->_depthHeight];
#endif // CUDA_BUILD
        _colorData = new ColorType[this->_colorWidth*this->_colorHeight];

        return true;
    }

    bool subscribe_images(std::string depth_topic, std::string colour_topic) {
        const std::string depth_default_transport = determineDefaultTransport(depth_topic);
        const std::string colour_default_transport = determineDefaultTransport(colour_topic);

        sub_colour.subscribe(it, colour_topic, 1, image_transport::TransportHints(colour_default_transport, ros::TransportHints(), ros::NodeHandle("~/colour")));
        sub_depth.subscribe(it, depth_topic, 1, image_transport::TransportHints(depth_default_transport, ros::TransportHints(), ros::NodeHandle("~/depth")));

        std::cout << "colour transport: " << sub_colour.getTransport() << std::endl;
        std::cout << "depth transport: " << sub_depth.getTransport() << std::endl;

        img_sync = std::make_shared<message_filters::Synchronizer<ApproximateTimePolicy>>(ApproximateTimePolicy(5), sub_colour, sub_depth);
        sync_conn = img_sync->registerCallback(&RosDepthSource::setImageData, this);
    }

    void disconnectSync() {
        sync_conn.disconnect();
    }

    void setDistanceThreshold(const float dist) {
        threshold = dist;
    }

    const std::string &getColourOpticalFrame() const { return camera_colour_frame; }

    const std::string &getDepthOpticalFrame() const { return camera_depth_frame; }

    bool hasPublisher() {
        return (sub_colour.getNumPublishers()!=0) & (sub_depth.getNumPublishers()!=0);
    }

    image_transport::SubscriberFilter& getSubscriberColour() { return sub_colour; }

    image_transport::SubscriberFilter& getSubscriberDepth() { return sub_depth; }

    void setImageData(const sensor_msgs::ImageConstPtr& img_colour, const sensor_msgs::ImageConstPtr& img_depth) {
        cv::Mat img_depth_cv = cv_bridge::toCvShare(img_depth)->image;
        if(img_depth_cv.type()==CV_16UC1) {
            // convert from 16bit uint millimeter to 32bit float meter
            img_depth_cv.convertTo(img_depth_cv, CV_32F);
            img_depth_cv = img_depth_cv/1000.0;
        }
        if(threshold>0) {
            // remove pixels above threshold
            img_depth_cv.setTo(0.0, img_depth_cv>threshold);
        }

        cv::Mat img_colour_cv = cv_bridge::toCvShare(img_colour)->image;
        if(img_colour->encoding=="bgr8") {
            cv::cvtColor(img_colour_cv, img_colour_cv, cv::COLOR_BGR2RGB);
        }

        mutex.lock();
        this->_frame++;
        _depthTime = img_depth->header.stamp.toNSec();
        _colourTime = img_colour->header.stamp.toNSec();
        camera_depth_frame = img_depth->header.frame_id;
        camera_colour_frame = img_colour->header.frame_id;
        std::memcpy(_colorData, img_colour_cv.data, sizeof(ColorType)*this->_colorWidth*this->_colorHeight);
#ifdef CUDA_BUILD
        std::memcpy(_depthData->hostPtr(), img_depth_cv.data, sizeof(DepthType)*_depthData->length());
        _depthData->syncHostToDevice();
#else
        _depthData = (DepthType*)img_depth_cv.data;
#endif // CUDA_BUILD
        mutex.unlock();
    }

private:
    void setCameraParameter(const sensor_msgs::CameraInfoConstPtr caminfo) {
        this->_depthWidth = caminfo->width;
        this->_depthHeight = caminfo->height;
        this->_colorWidth = caminfo->width;
        this->_colorHeight = caminfo->height;

        this->_focalLength.x = caminfo->P[0];
        this->_focalLength.y = caminfo->P[5];
        this->_principalPoint.x = caminfo->P[2];
        this->_principalPoint.y = caminfo->P[6];

        if(caminfo->distortion_model=="plumb_bob") {
            distortion_param = std::vector<float>(caminfo->D.begin(), caminfo->D.end());
        }
        else {
            std::cerr << "distortion model " << caminfo->distortion_model << "is not supported and will be ignored" << std::endl;
        }
    }

    /**
     * @brief determineDefaultTransport get the transport from the topic string
     * e.g. /camera/rgb/compressed will return "compressed", the provided topic string
     * will be changed to the canonical topic name without the transport, e.g. "/camera/rgb"
     * @param image_topic topic of published images
     * @return name of transport
     */
    static
    std::string determineDefaultTransport(std::string &image_topic) {
        std::string default_transport = image_topic.substr(image_topic.rfind("/")+1);
        if(supported_transports.count(default_transport)) {
            // remove transport from topic
            image_topic = image_topic.substr(0, image_topic.rfind("/"));
        }
        else {
            // no transport name found in topic name
            default_transport = "raw";
        }

        return default_transport;
    }

    ros::NodeHandle n;
    image_transport::ImageTransport it;
    std::shared_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> img_sync;
    message_filters::Connection sync_conn;
    image_transport::SubscriberFilter sub_colour;
    image_transport::SubscriberFilter sub_depth;

    std::mutex mutex;

    std::vector<float> distortion_param;

    dart::MirroredVector<DepthType> * _depthData;
    ColorType * _colorData;

    uint64_t _depthTime;
    uint64_t _colourTime;

    std::string camera_colour_frame;
    std::string camera_depth_frame;

    float threshold;    // distance in meter

    static const std::set<std::string> supported_transports;
};

template <typename DepthType, typename ColorType>
const std::set<std::string> RosDepthSource<DepthType,ColorType>::supported_transports = {"compressed", "compressedDepth"};

} // namespace dart

#endif // ROSDEPTHSOURCE_HPP
