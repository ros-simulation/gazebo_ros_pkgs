/***************************************************************************
 *   Copyright (C) 2014 by Markus Bader                                    *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/



#ifndef GAZEBO_ROS_SENSOR_UTIL
#define GAZEBO_ROS_SENSOR_UTIL


#include <boost/algorithm/string.hpp>
#include <ros/ros.h>

namespace gazebo
{

/**
 * @brief accessing model name like suggested by nkoenig at http://answers.gazebosim.org/question/4878/multiple-robots-with-ros-plugins-sensor-plugin-vs/
 * @Author: Markus Bader
 * @return accessing model name
 **/
inline std::string GetModelName(const sensors::SensorPtr &parent) {
    std::string modelName;
    std::vector<std::string> values;
    std::string scopedName = parent->GetScopedName();
    boost::replace_all(scopedName, "::", ",");
    boost::split(values, scopedName, boost::is_any_of(","));
    if(values.size() < 2) {
        modelName = "";
    } else {
        modelName = values[1];
    }
    return modelName;
}

inline std::string GetRobotNamespace(const sensors::SensorPtr &parent, const sdf::ElementPtr &sdf, const char *pInfo = NULL) {
    std::string name_space;
    std::stringstream ss;
    if (sdf->HasElement("robotNamespace")) {
        name_space = sdf->Get<std::string>("robotNamespace");
        if(name_space.empty()) {
            ss << "the 'robotNamespace' param was empty";
            name_space = GetModelName(parent);
        } else {
            ss << "Using the 'robotNamespace' param: '" <<  name_space << "'";
        }
    } else {
        ss << "the 'robotNamespace' param did not exit";
    }
    if(pInfo != NULL) {
        ROS_INFO ( "%s Plugin (robotNamespace = %s), Info: %s" , pInfo, name_space.c_str(), ss.str().c_str() );
    }
    return name_space;
}
}

#endif  //GAZEBO_ROS_SENSOR_UTIL
