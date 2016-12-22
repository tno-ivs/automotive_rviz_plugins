#ifndef MOBILEYE_LANE_INFO_DISPLAY_H
#define MOBILEYE_LANE_INFO_DISPLAY_H

#ifndef Q_MOC_RUN
//#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.h>
#include <automotive_sensor_msgs/Lines.h>
#endif

#include "line_visual.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace automotive_rviz_plugins
{

class Line {
public:
    Line() = default;
    ~Line() = default;
    Line(const Line&) = default;

    LineVisual visual;
    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* width_property_;
};

class LineInfoDisplay : public rviz::MessageFilterDisplay<automotive_sensor_msgs::Lines>
{
Q_OBJECT
public:
    LineInfoDisplay();
    virtual ~LineInfoDisplay();

protected:
    virtual void onInitialize();

    virtual void reset();


private:
    void processMessage(const automotive_sensor_msgs::LinesConstPtr& msg );
    rviz::ColorProperty* colorFromLineType(int type);

    bool initialized_;
    std::vector<Line> lines_;
    rviz::ColorProperty* color_dashed;
    rviz::ColorProperty* color_solid;
    rviz::ColorProperty* color_undecided;
    rviz::ColorProperty* color_roadedge;
    rviz::ColorProperty* color_invalid;


    rviz::FloatProperty* width_property_;
    rviz::FloatProperty* stepsize_property_;
    rviz::FloatProperty* max_length_property_;

    void getLineFromSegment(Line& line, const Ogre::Quaternion &frame_orientation, const Ogre::Vector3 &frame_position,
                            const automotive_sensor_msgs::LineSegment_<std::allocator<void>> &segment);

    void ensureCapacity(unsigned int size);
};

} // end namespace tno_automotive_rviz_plugins

#endif // RADAR_OBJECT_DISPLAY_H
