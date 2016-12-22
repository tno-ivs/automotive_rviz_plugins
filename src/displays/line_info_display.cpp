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

namespace automotive_rviz_plugins {

LineInfoDisplay::LineInfoDisplay() {
    initialized_ = false;
}

void LineInfoDisplay::onInitialize() {
    color_dashed = new rviz::ColorProperty("Color - Dashed", QColor(255, 255, 0), "", this);
    color_solid = new rviz::ColorProperty("Color - Solid", QColor(255, 0, 0), "", this);
    color_undecided = new rviz::ColorProperty("Color - Undecided", QColor(255, 0, 255), "", this);
    color_roadedge = new rviz::ColorProperty("Color - Road edge", QColor(0, 136, 0), "", this);
    color_invalid = new rviz::ColorProperty("Color - Invalid", QColor(0, 0, 0), "", this);

    width_property_ = new rviz::FloatProperty("Line Width", 0.3, "", this);
    width_property_->setMin(0);

    stepsize_property_ = new rviz::FloatProperty("Step size", 1, "", this);
    stepsize_property_->setMin(0.8);

    max_length_property_ = new rviz::FloatProperty("Max length", 100, "", this);
    max_length_property_->setMin(0);
    max_length_property_->setMax(200);

    MFDClass::onInitialize();
    initialized_ = true;
}

LineInfoDisplay::~LineInfoDisplay() {
    for(auto& line : lines_) {
        line.visual.destroy();
    }

    delete color_dashed;
    delete color_solid;
    delete color_undecided;
    delete color_roadedge;
    delete color_invalid;

    delete width_property_;
    delete stepsize_property_;
    delete max_length_property_;
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

    ensureCapacity(msg->segments.size());
    int count = 0;
    for (const auto &segment : msg->segments) {
        getLineFromSegment(lines_.at(count++),frame_orientation, frame_position, segment);
    }
    //Reset the old storage
    for(auto i = count; i < lines_.size(); i++) {
        lines_.at(i).visual.setOpacity(0);//invisible
    }
}

void LineInfoDisplay::getLineFromSegment(Line& lineInstance, const Ogre::Quaternion &frame_orientation, const Ogre::Vector3 &frame_position,
                                         const automotive_sensor_msgs::LineSegment_<std::allocator<void>> &segment) {

    lineInstance.width_property_ = width_property_;
    lineInstance.color_property_ = colorFromLineType(segment.type);

    Ogre::Vector3 line_position(segment.pose.position.x, segment.pose.position.y, segment.pose.position.z);
    Ogre::Quaternion line_orientation(segment.pose.orientation.w, segment.pose.orientation.x,
                                      segment.pose.orientation.y, segment.pose.orientation.z);

    double opacity = (segment.confidence >= 0.5) ? 1.0 : 0.1;
    lineInstance.visual.setFramePosition(frame_position + line_position);
    lineInstance.visual.setFrameOrientation(frame_orientation * line_orientation);
    lineInstance.visual.setLineParameters(
                segment.c2,
                segment.c3,
                0,
                std::min(segment.length, max_length_property_->getFloat()),
                lineInstance.width_property_->getFloat(),
                lineInstance.color_property_->getOgreColor(),
                opacity,
                stepsize_property_->getFloat());
}

rviz::ColorProperty* LineInfoDisplay::colorFromLineType(int type) {
    switch(type){
        case automotive_sensor_msgs::LineSegment::TYPE_DASHED:
            return color_dashed;
        case automotive_sensor_msgs::LineSegment::TYPE_SOLID:
            return color_solid;
        case automotive_sensor_msgs::LineSegment::TYPE_UNDECIDED:
            return color_undecided;
        case automotive_sensor_msgs::LineSegment::TYPE_ROAD_EDGE:
            return color_roadedge;
    }
    return color_invalid;
}

void LineInfoDisplay::ensureCapacity(unsigned int size) {
    for(unsigned int i = lines_.size(); i<size; i++) {
        lines_.push_back(Line());
        lines_.at(lines_.size()-1).visual.initialize(scene_manager_, scene_node_);
    }
}

}
// Export lib
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(automotive_rviz_plugins::LineInfoDisplay, rviz::Display )
