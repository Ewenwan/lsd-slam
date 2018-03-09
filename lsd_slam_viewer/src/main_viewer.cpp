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
float kp_omega;

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

#define STOP_STATE  0
#define RIGHT_STATE 1
#define LEFT_STATE  2

float end_point_z = 0.1;
float end_point_x = 0;
char  state       = RIGHT_STATE;
float v_set = 125, theta_set = 0 ; 

void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{

	if(msg->time > lastFrameTime) return;

	if(viewer != 0)
		viewer->addFrameMsg(msg);

    Sophus::Vector4f originCar ;
    Sophus::Vector4f zCar ;
    Sophus::Vector4f xCar ;

    originCar << 0,0,0,1;//use this to set grid point coordinate
    zCar << 0,0,1,1;
    xCar << 1,0,0,1;

    Sophus::Matrix4f POSE = viewer->currentCamDisplay->camToWorld.matrix();//.inverse();//maybe data crash

    originCar   = POSE*originCar;
    zCar        = POSE*zCar - originCar;
    xCar        = POSE*xCar - originCar;

    Sophus::Vector4f end_point ;

    if(originCar[0] > end_point_x  -0.1 )
        state = LEFT_STATE;
    if(originCar[0] < -end_point_x +0.1 )
        state = RIGHT_STATE;

    if(state == LEFT_STATE)
        end_point << -end_point_x,0,end_point_z,0;
    else if (state == RIGHT_STATE)
        end_point << end_point_x,0,end_point_z,0;

    originCar[1] = originCar[3] = 0;
    zCar[1]      = zCar[3] = 0;
    xCar[1]      = xCar[3] = 0;

    cout <<"originCar\n" << originCar <<endl;
    cout <<"zCar\n" << zCar <<endl;
    cout <<"xCar\n" << xCar <<endl;

    Sophus::Vector4f  velocityerr_direction_car =  end_point - originCar;

    cout <<"velocityerr_direction_car\n" << velocityerr_direction_car <<endl;


    float vlat  = xCar.dot(velocityerr_direction_car)/xCar.norm();
    float vlong = zCar.dot(velocityerr_direction_car)/zCar.norm();

    cout <<"vlat\n"  << vlat  <<endl;
    cout <<"vlong\n" << vlong <<endl;

    //omega_error
    /*
    * theta  is Zcar  turn to Zglobal,Anti-clock is +
    * omega is Anti-clock is +
    *   zglobal
    *   /|\ __ zcar
    *    |   /|
    *    |  /
    *    | /
    *    |/ theta > 0
    * */
    float theta =  asin(zCar[0]/zCar.norm());

    if(zCar[0] >= 0 && zCar[2] < 0)
    {
        theta = (float)3.14159 - theta;
    }
    else if(zCar[0] < 0 && zCar[2] < 0)
    {
        theta = -(float)3.14159 - theta;
    }

//    cout << setprecision(8) << "velocityset_direction_car = \n" << velocityset_direction_car << "\n" ;
//    cout << setprecision(8) << "velocityerr_direction_car = \n" << velocityerr_direction_car << "\n" ;
    cout << setprecision(8) << "theta =" << theta << "\n" ;

    float omega = -kp_omega*(theta_set-theta);



    if( vlat > 200 )
        vlat =  200;
    else if(vlat < -200 )
        vlat = -200;

    if( vlong > 200 )
        vlong =  200;
    else if(vlong < -200 )
        vlong = -200;

    if( omega  > 200 )
        omega  =  200;
    else if(omega < -200 )
        omega = -200;
    cout << setprecision(8) << "lat = \n" << vlat  << "\n" ;
    cout << setprecision(8) << "long = \n" << vlong << "\n" ;
    cout << setprecision(8) << "omega = \n" << omega << "\n" ;

    Sophus::Vector3d  velocity_give;

    velocity_give << (double)vlat ,(double)vlong ,0;

    velocity_give = v_set*velocity_give/velocity_give.norm();
    velocity_give[2] = omega;

    cout <<"velocity_give   \n" << velocity_give   <<endl;

    robotChasis->robot_velocity = velocity_give;
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
  
  ros::NodeHandle prnode("~");
  prnode.param<float>("kp_omega",kp_omega,200);
  prnode.param<float>("end_point_z",end_point_z,0.0);
  prnode.param<float>("end_point_x",end_point_x,0.0);
  prnode.param<float>("v_set",v_set,125);
  prnode.param<float>("theta_set",theta_set,0.0);

  cout<<"omega="<<kp_omega<<endl;
  cout<<"end_point_z="<<end_point_z<<endl;
	cout<<"end_point_x="<<end_point_x<<endl;

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
