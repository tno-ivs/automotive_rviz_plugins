#include "line_visual.h"

namespace rviz_plugins
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
    if(line_ != nullptr)
        delete line_;//SIGSEV
    if(frame_node_ != nullptr)
        scene_manager_->destroySceneNode( frame_node_ );
}

void LineVisual::setFramePosition( const Ogre::Vector3& position )
{
    frame_node_->setPosition( position );
}

void LineVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
    frame_node_->setOrientation( orientation );
}

void LineVisual::setLineParameters(double c2, double c3, double x_start, double dx, double width, Ogre::ColourValue color, double opacity)
{
    line_->clear();

    double c0 = 0;
    double c1 = 1;

    // Width
    line_->setLineWidth(width);
    line_->setColor(color.r, color.g, color.b, opacity);

    for (unsigned int i = 0; i < 100; ++i)
    {
        double x = x_start + i * dx;
        double y = c3 * x * x * x + c2 * x * x + c1 * x + c0;

        line_->addPoint(Ogre::Vector3(x, y, 0));
    }
}

void LineVisual::initialize(const LineVisual &other) {
    initialize(other.scene_manager_, other.parent_node_);
}

} // end namespace rviz_plugins

