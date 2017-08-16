#ifndef ROSDEPTHSOURCE_HPP
#define ROSDEPTHSOURCE_HPP

#include <dart/depth_sources/depth_source.h>

namespace dart {

template <typename DepthType, typename ColorType>
class RosDepthSource : public dart::DepthSource<DepthType,ColorType> {
public:
    RosDepthSource() {
        this->_isLive = true;
    }

    void setFrame(const uint frame) {
        //
    }

    void advance() {
        //
    }

    bool hasRadialDistortionParams() const {
        // TODO
        return false;
    }

    const DepthType * getDepth() const {
        //
    }

    const DepthType * getDeviceDepth() const {
        //
    }

    const ColorType * getColor() const { return 0; }
};

} // namespace dart

#endif // ROSDEPTHSOURCE_HPP
