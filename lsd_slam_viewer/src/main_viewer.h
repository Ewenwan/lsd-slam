/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/
#include "ros/ros.h"
#include "boost/thread.hpp"
#include "settings.h"
#include "PointCloudViewer.h"
#include "KeyFrameDisplay.h"

#include <dynamic_reconfigure/server.h>
#include "lsd_slam_viewer/LSDSLAMViewerParamsConfig.h"
#include <qapplication.h>


#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"


#include "boost/foreach.hpp"
#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include <sys/time.h>


extern PointCloudViewer* viewer;

