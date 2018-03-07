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
#include "main_viewer.h"
#include "../cfg/cpp/lsd_slam_viewer/LSDSLAMViewerParamsConfig.h"
#include "../../../../../../../usr/include/c++/4.8/iomanip"
#include "../../../../../../../usr/include/c++/4.8/cstdio"
#include "sophus/sim3.hpp"
#include "../../../../../../../usr/include/c++/4.8/cmath"
#include "robot/robot.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

PointCloudViewer* viewer = 0;
three_omni_wheel_robot* robotChasis=0;
RobotViewer*      robot  = 0;

void dynConfCb(lsd_slam_viewer::LSDSLAMViewerParamsConfig &config, uint32_t level)
{

	pointTesselation = config.pointTesselation;
	lineTesselation = config.lineTesselation;

	keepInMemory = config.keepInMemory;
	showKFCameras = config.showKFCameras;
	showKFPointclouds = config.showKFPointclouds;
	showConstraints = config.showConstraints;
	showCurrentCamera = config.showCurrentCamera;
	showCurrentPointcloud = config.showCurrentPointcloud;


	scaledDepthVarTH = exp10( config.scaledDepthVarTH );
	absDepthVarTH = exp10( config.absDepthVarTH );
	minNearSupport = config.minNearSupport;
	sparsifyFactor = config.sparsifyFactor;
	cutFirstNKf = config.cutFirstNKf;

	saveAllVideo = config.saveAllVideo;

}

void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{

	if(msg->time > lastFrameTime) return;

	if(viewer != 0)
		viewer->addFrameMsg(msg);

    //cout << setprecision(8) << "Now This Robot in\n\n"<< viewer->robot_pose << "\n\n";


    Sophus::Vector2f robot_direction ;

    robot_direction << viewer->robot_direction_global[0] - viewer->robot_pose[0],
                    viewer->robot_direction_global[2] - viewer->robot_pose[2];

    double sintheta = robot_direction[0]/sqrt(robot_direction[0]*robot_direction[0]
                                        +  robot_direction[1]*robot_direction[1]);

    double theta =  asin(sintheta);

    cout << setprecision(8) << "theta =" << theta << "\n" ;


    double kp1 = 1000;
    double vset = 1.414*kp1*fabs(viewer->robot_pose[0]);
    double vx = 0;
    double vz = 0;

    if(viewer->robot_pose[0] < 0)    {

        vx = sin(0.785-theta)*vset;
        vz = cos(0.785-theta)*vset;

    } else {

        vx = -sin(0.785+theta)*vset;
        vz = cos(0.785+theta)*vset;
    }

    double kp2 = 100;

    double omega = -kp2*theta;


    cout << setprecision(8) << "vx =" << vx << "\n" ;
    cout << setprecision(8) << "vz =" << vz << "\n" ;
    cout << setprecision(8) << "omega =" << omega << "\n" ;

    
    robotChasis->robot_velocity << vx,vz+100,0;
    robotChasis->wheel_velocity_decode();
    robotChasis->robot_move();


}
void graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	if(viewer != 0)
		viewer->addGraphMsg(msg);
}



void rosThreadLoop( int argc, char** argv )
{
	printf("Started ROS thread\n");

	//glutInit(&argc, argv);

	ros::init(argc, argv, "viewer");
	ROS_INFO("lsd_slam_viewer started");

	dynamic_reconfigure::Server<lsd_slam_viewer::LSDSLAMViewerParamsConfig> srv;
	srv.setCallback(dynConfCb);


	ros::NodeHandle nh;

	ros::Subscriber liveFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/liveframes"),1, frameCb);
	ros::Subscriber keyFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/keyframes"),20, frameCb);
	ros::Subscriber graph_sub       = nh.subscribe(nh.resolveName("lsd_slam/graph"),10, graphCb);

	ros::spin();

	ros::shutdown();

	printf("Exiting ROS thread\n");

	exit(1);
}


void rosFileLoop( int argc, char** argv )
{
	ros::init(argc, argv, "viewer");
	dynamic_reconfigure::Server<lsd_slam_viewer::LSDSLAMViewerParamsConfig> srv;
	srv.setCallback(dynConfCb);

	rosbag::Bag bag;
	bag.open(argv[1], rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(std::string("/lsd_slam/liveframes"));
	topics.push_back(std::string("/lsd_slam/keyframes"));
	topics.push_back(std::string("/lsd_slam/graph"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	 //for(rosbag::MessageInstance const m = view.begin(); m < view.end(); ++m)
	 BOOST_FOREACH(rosbag::MessageInstance const m, view)
	 {

		 if(m.getTopic() == "/lsd_slam/liveframes" || m.getTopic() == "/lsd_slam/keyframes")
			 frameCb(m.instantiate<lsd_slam_viewer::keyframeMsg>());


		 if(m.getTopic() == "/lsd_slam/graph")
			 graphCb(m.instantiate<lsd_slam_viewer::keyframeGraphMsg>());
	 }

	ros::spin();

	ros::shutdown();

	printf("Exiting ROS thread\n");

	exit(1);
}

void hello(){
    while(true) {
        struct  timeval  time_start;
        gettimeofday(&time_start,NULL);
        double cost_time  = (time_start.tv_sec)+(time_start.tv_usec)/1000000.0;
        //cout <<"Thread time " <<  cost_time << endl;
    }

}


int main( int argc, char** argv )
{

    glutInit(&argc, argv);
	printf("Started QApplication thread\n");
	// Read command lines arguments.
	QApplication application(argc,argv);//QApplication 类管理图形用户界面应用程序的控制流和主要设置

	robot =  new RobotViewer;
	robot->setWindowTitle("RobotMap");
	robot->show();

	// Instantiate the viewer.
	viewer = new PointCloudViewer();
	viewer->setWindowTitle("VSLAM PointCloud");
// Make the viewer window visible on screen.
	viewer->show();//show the "VSLAM Robot PointCloud Viewer" window

    Eigen::Vector3d  set_velocity(0,0,0);
    robotChasis = new three_omni_wheel_robot(set_velocity,"UART");

	boost::thread rosThread;
    boost::thread t(hello);

	if(argc > 1)
	{
		rosThread = boost::thread(rosFileLoop, argc, argv);
	}
	else
	{
		// start ROS thread
		rosThread = boost::thread(rosThreadLoop, argc, argv);
	}


	application.exec();

	printf("Shutting down... \n");
	ros::shutdown();
	rosThread.join();
	t.join();
	printf("Done. \n");

}
