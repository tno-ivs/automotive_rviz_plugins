#ifndef LANE_VISUAL_H
#define LANE_VISUAL_H

#include <rviz/ogre_helpers/billboard_line.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz_plugins
{

class LineVisual
{
public:
    LineVisual();
    ~LineVisual();


    void initialize(const LineVisual& other);
    void initialize(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

    void setFramePosition( const Ogre::Vector3& position );
    void setFrameOrientation( const Ogre::Quaternion& orientation );
    void setLineParameters(double c2, double c3, double x_start, double dx, double width, Ogre::ColourValue color, double opacity);

private:
    // The actual visual
    rviz::BillboardLine* line_;

    Ogre::SceneNode* frame_node_;

    Ogre::SceneNode* parent_node_;

    Ogre::SceneManager* scene_manager_;


};

} // end namespace rviz_plugins

#endif
