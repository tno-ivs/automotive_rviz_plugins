#include "line_visual.h"

namespace automotive_rviz_plugins
{

LineVisual::LineVisual()
{
    scene_manager_ = nullptr;
    parent_node_ = nullptr;
    frame_node_ = nullptr;
    line_ = nullptr;
}

void LineVisual::initialize(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
    scene_manager_ = scene_manager;
    parent_node_ = parent_node;
    frame_node_ = parent_node->createChildSceneNode();

    // The visual
    line_ = new rviz::BillboardLine( scene_manager_, frame_node_ );
}

LineVisual::~LineVisual()
{
    //This is called but not expected..
    //destroy();
}

void LineVisual::destroy() const {
    if(line_ != nullptr)
        delete line_;
    if(frame_node_ != nullptr)
        scene_manager_->destroySceneNode(frame_node_);
}

void LineVisual::setFramePosition( const Ogre::Vector3& position )
{
    frame_node_->setPosition( position );
}

void LineVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
    frame_node_->setOrientation( orientation );
}

void LineVisual::setLineParameters(double c2, double c3, double x_start, double max_length, double width, Ogre::ColourValue color, double opacity, double stepsize)
{
    line_->clear();

    // Width
    line_->setLineWidth(width);
    color_ = color;
    line_->setColor(color.r, color.g, color.b, opacity);

    for (double step = stepsize; step < std::min(max_length, 100.0); step += stepsize)
    {
        double x = x_start + step;
        double y = c3 * x * x * x + c2 * x * x;

        line_->addPoint(Ogre::Vector3(x, y, 0));
    }
}

void LineVisual::initialize(const LineVisual &other) {
    initialize(other.scene_manager_, other.parent_node_);
}

void LineVisual::setOpacity(double opacity) {
    line_->setColor(color_.r, color_.g, color_.b, opacity);
}

} // end namespace automotive_rviz_plugins

