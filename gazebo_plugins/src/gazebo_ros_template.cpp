/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include <gazebo_plugins/gazebo_ros_template.h>

namespace gazebo
{


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTemplate::GazeboRosTemplate()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTemplate::~GazeboRosTemplate()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosTemplate::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTemplate::UpdateChild()
{
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosTemplate);

}
