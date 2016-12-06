#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

//Can de ifndef be omitted?
#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif

#include "line_visual.h"
#include "line_info_display.h"

namespace rviz_plugins {

LineInfoDisplay::LineInfoDisplay() {
    initialized_ = false;
}

void LineInfoDisplay::onInitialize() {
    color_dashed = new rviz::ColorProperty("Color - Dashed", QColor(255, 0, 0), "", this);

    width_property_ = new rviz::FloatProperty("Lane Width", 0.3, "", this);
    dx_property_ = new rviz::FloatProperty("dx", 0.1, "", this);
    x_start_property_ = new rviz::FloatProperty("x_start", 0, "", this);

    MFDClass::onInitialize();
    initialized_ = true;
}

LineInfoDisplay::~LineInfoDisplay() {

    delete color_dashed;
    delete width_property_;
    delete dx_property_;
    delete x_start_property_;
}

// Clear the visuals by deleting their objects.
void LineInfoDisplay::reset() {
    MFDClass::reset();
}

void LineInfoDisplay::processMessage(const automotive_sensor_msgs::LinesConstPtr &msg) {
    if (!initialized_)
        return;

    Ogre::Quaternion frame_orientation;
    Ogre::Vector3 frame_position;
    if (!context_->getFrameManager()->getTransform(msg->header.frame_id,
                                                   msg->header.stamp,
                                                   frame_position, frame_orientation)) {
        ROS_ERROR("Error transforming from frame '%s' to frame '%s'",
                  msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
        return;
    }

    lines_.clear();
    for (const auto &segment : msg->segments) {
        lines_.push_back(getLineFromSegment(frame_orientation, frame_position, segment));
    }
}

Line LineInfoDisplay::getLineFromSegment(const Ogre::Quaternion &frame_orientation, const Ogre::Vector3 &frame_position,
                                         const automotive_sensor_msgs::LineSegment_<std::allocator<void>> &segment) {
    auto lineInstance = Line();

    // Modify
    lineInstance.visual.initialize(scene_manager_, scene_node_);
    lineInstance.width_property_ = width_property_;
    lineInstance.color_property_ = colorFromLineType(segment.type);

    Ogre::Vector3 line_position(segment.pose.position.x, segment.pose.position.y, segment.pose.position.z);
    Ogre::Quaternion line_orientation(segment.pose.orientation.w, segment.pose.orientation.x,
                                      segment.pose.orientation.y, segment.pose.orientation.z);

    double opacity = (segment.confidence > 1) ? 1.0 : 0.1;
    lineInstance.visual.setFramePosition(frame_position + line_position);
    lineInstance.visual.setFrameOrientation(frame_orientation + line_orientation);
    lineInstance.visual.setLineParameters(
                -segment.c2,
                -segment.c3,
                x_start_property_->getFloat(),
                dx_property_->getFloat(),
                lineInstance.width_property_->getFloat(),
                lineInstance.color_property_->getOgreColor(),
                opacity);
    return lineInstance;
}

rviz::ColorProperty *LineInfoDisplay::colorFromLineType(int type) {
    return color_dashed;
}

}
// Export lib
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::LineInfoDisplay, rviz::Display )
