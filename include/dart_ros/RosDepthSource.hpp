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
    }

    ~RosDepthSource() {
        img_sync.reset();
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
        if(!fetchCameraParameters(caminfo_topic)) {
            return false;
        }

        // allocate memory for depth image
#ifdef CUDA_BUILD
        _depthData = new dart::MirroredVector<DepthType>(this->_depthWidth*this->_depthHeight);
#else
        _depthData = new DepthType[this->_depthWidth*this->_depthHeight];
#endif // CUDA_BUILD
        _colorData = new ColorType[this->_colorWidth*this->_colorHeight];

        return true;
    }

    bool subscribe_images(const std::string depth_topic, const std::string colour_topic) {
        sub_colour.subscribe(it, colour_topic, 1, image_transport::TransportHints("raw", ros::TransportHints(), ros::NodeHandle("~/colour")));
        sub_depth.subscribe(it, depth_topic, 1, image_transport::TransportHints("raw", ros::TransportHints(), ros::NodeHandle("~/depth")));

        std::cout << "colour transport: " << sub_colour.getTransport() << std::endl;
        std::cout << "depth transport: " << sub_depth.getTransport() << std::endl;

        img_sync = std::make_shared<message_filters::Synchronizer<ApproximateTimePolicy>>(ApproximateTimePolicy(5), sub_colour, sub_depth);
        img_sync->registerCallback(boost::bind(&RosDepthSource::setImageData, this, _1, _2));
    }

private:
    bool fetchCameraParameters(const std::string &topic) {
        ros::Subscriber sub = n.subscribe(topic, 1, &RosDepthSource::setCameraParameter, this);

        while(this->_depthWidth==0 && this->_depthHeight==0) {
            ros::spinOnce();
        }
        sub.shutdown();
        std::cout << "set camera parameter" << std::endl;
        return true;
    }

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

    void setImageData(const sensor_msgs::ImageConstPtr& img_colour, const sensor_msgs::ImageConstPtr& img_depth) {
        cv::Mat img_depth_cv = cv_bridge::toCvShare(img_depth)->image;
        if(img_depth_cv.type()==CV_16UC1) {
            // convert from 16bit uint millimeter to 32bit float meter
            img_depth_cv.convertTo(img_depth_cv, CV_32F);
            img_depth_cv = img_depth_cv/1000.0;
        }

        cv::Mat img_colour_cv = cv_bridge::toCvShare(img_colour)->image;
        if(img_colour->encoding=="bgr8") {
            cv::cvtColor(img_colour_cv, img_colour_cv, cv::COLOR_BGR2RGB);
        }

        mutex.lock();
        this->_frame++;
        _depthTime = img_depth->header.stamp.sec*1e6 + img_depth->header.stamp.nsec/1e3;
        _colourTime = img_colour->header.stamp.sec*1e6 + img_colour->header.stamp.nsec/1e3;
        std::memcpy(_colorData, img_colour_cv.data, sizeof(ColorType)*this->_colorWidth*this->_colorHeight);
#ifdef CUDA_BUILD
        std::memcpy(_depthData->hostPtr(), img_depth_cv.data, sizeof(DepthType)*_depthData->length());
        _depthData->syncHostToDevice();
#else
        _depthData = (DepthType*)img_depth_cv.data;
#endif // CUDA_BUILD
        mutex.unlock();
    }

    ros::NodeHandle n;
    image_transport::ImageTransport it;
    std::shared_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> img_sync;
    image_transport::SubscriberFilter sub_colour;
    image_transport::SubscriberFilter sub_depth;

    std::mutex mutex;

    std::vector<float> distortion_param;

    dart::MirroredVector<DepthType> * _depthData;
    ColorType * _colorData;

    uint64_t _depthTime;
    uint64_t _colourTime;
};

} // namespace dart

#endif // ROSDEPTHSOURCE_HPP
