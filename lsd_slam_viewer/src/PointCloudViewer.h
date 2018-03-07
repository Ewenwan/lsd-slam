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

#pragma once
//#define GL_GLEXT_PROTOTYPES 1
//#define GL3_PROTOTYPES 1
//#include <GL/glew.h>

#include "QGLViewer/qglviewer.h"
#include <vector>
#include "boost/thread.hpp"
#include "qevent.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "sophus/sim3.hpp"
#include "QGLViewer/keyFrameInterpolator.h"
#include <GL/glut.h>
#include <map>
#include <set>


class QApplication;

class KeyFrameGraphDisplay;
class CameraDisplay;
class KeyFrameDisplay;
class RobotViewer;
class PointCloudViewer;

#include "settings.h"
#include "../msg_gen/cpp/include/lsd_slam_viewer/keyframeGraphMsg.h"
#include "../msg_gen/cpp/include/lsd_slam_viewer/keyframeMsg.h"
#include "../../../../../../../usr/include/c++/4.8/iostream"

#define gridUnit 0.02
using namespace std;

struct node
{
	int x;
	int z;
	bool operator < (const node &compNode) const
	{
		return (this->x < compNode.x || (this->x == compNode.x && this->z < compNode.z));
	}
	bool operator == (const node &compNode) const
	{
		return (this->x == compNode.x &&  this->z == compNode.z);
	}
	bool operator != (const node &compNode) const
	{
		return (this->x != compNode.x &&  this->z != compNode.z);
	}
};

struct globalVertex
{
	float x;
	float y;
	float z;
	bool operator < (const globalVertex &compVertex) const
	{
		return (this->x <  compVertex.x
				|| (this->x == compVertex.x && this->y <  compVertex.y)
				|| (this->x == compVertex.x && this->y == compVertex.y && this->z < compVertex.z )
		);
	}
};

class MAP
{
	//friend ostream &operator<< (ostream&,const node&);
public:
	MAP()
	{
		error = -1;
	}
	void gridInsertNode(int x, int z)
	{
		node temp_n;
		temp_n.x = x;
		temp_n.z = z;

		if(g_map.find(temp_n) == g_map.end())//no this node
		{
			g_map[temp_n] = 1;//add node
		}
		else//already have this node
		{
			g_map[temp_n]++;//point cloud num ++
		}

	}


	map<node, int> g_map;
	set<globalVertex> vertex_map;

	int error;

	int  &operator ()(int x, int z)
	{
		node temp_n;
		temp_n.x = x;
		temp_n.z = z;
		if (g_map.find(temp_n) == g_map.end())//no this node
		{
			//cout << "no this node"<<endl;
			return error;
		}

		//cout << "x=" << x << " y=" << y << " num=" << r_map[temp_n] << endl;
		return g_map[temp_n];
	}
	void PrintMap()
	{
		map<node, int>::iterator itr;
		itr = g_map.begin();
		int i = 0;
		while(itr != g_map.end())
		{
			cout << "node " << i  <<", x= "<< itr->first.x
				 << ", z= " << itr->first.z
				 << ", point cloud =" << itr->second
				 << endl;
			itr++;
			i++;
		}

	}
};

class Astar {

public:
	Astar(){

		startPosition = new node;
		destination   = new node;
	}
	~Astar();

	node* startPosition;
	node* destination;

};

//ostream &operator<< (ostream &out,const node &n)
//{
//    out << "x=" << n.x << " y=" <<n.y ;
//    return  out;
//}

class AnimationObject
{
public:
	double time;
	double duration;

	// settings
	float scaledTH;
	float absTH;
	int neighb;
	int sparsity;
	bool showLoopClosures;
	bool showKeyframes;
	bool showCurrentCam;

	// frame
	qglviewer::Frame frame;

	// whether is a KF or only settings change
	bool isSettings;

	bool isFix;

	AnimationObject(bool isSettings, double time, double duration, qglviewer::Frame f = qglviewer::Frame())
	{
		this->time = time;
		this->duration = duration;

		scaledTH = scaledDepthVarTH;
		absTH = absDepthVarTH;
		neighb = minNearSupport;
		showKeyframes = showKFCameras;
		showLoopClosures = showConstraints;
		showCurrentCam = showCurrentCamera;
		sparsity = sparsifyFactor;

		this->isSettings = isSettings;

		frame = f;

		isFix = false;
	}

	AnimationObject(std::string s)
	{
		int isSettings_i;
		int showLoopClosures_i;
		int showKeyframes_i;
		int showCurrentCam_i;
		int isFix_i;


		qglviewer::Quaternion orient;


		float x,y,z;

		if(17 != sscanf(s.c_str(),"Animation: %d at %lf (dur %lf) S: %f %f %d %d %d %d %d Frame: %lf %lf %lf %lf %f %f %f %d\n",
						&isSettings_i, &time, &duration,
						&scaledTH, &absTH, &showLoopClosures_i, &showKeyframes_i, &showCurrentCam_i, &sparsity, &neighb,
						&(orient[0]),&(orient[1]),&(orient[2]),&(orient[3]),
						&x, &y, &z, &isFix_i))
			printf("error parsing: %s\n", s.c_str());

		isSettings = isSettings_i;
		showLoopClosures = showLoopClosures_i;
		showKeyframes = showKeyframes_i;
		showCurrentCam = showCurrentCam_i;
		isFix = isFix_i;


		frame = qglviewer::Frame(qglviewer::Vec(0,0,0),orient);
		frame.setPosition(x,y,z);

		printf("read: %s\n",toString().c_str());
	}

	bool operator < (const AnimationObject& other) const
	{
		return (time < other.time);
	}

	std::string toString()
	{
		char buf[1000];

		int isSettings_i = isSettings;
		int showLoopClosures_i = showLoopClosures;
		int showKeyframes_i = showKeyframes;
		int showCurrentCam_i = showCurrentCam;
		int isFix_i = isFix;

		float x,y,z;
		frame.getPosition(x,y,z);

		snprintf(buf, 1000, "Animation: %d at %lf (dur %lf) S: %f %f %d %d %d %d %d Frame: %lf %lf %lf %lf %f %f %f %d",
				 isSettings_i, time, duration,
				 scaledTH, absTH, showLoopClosures_i, showKeyframes_i, showCurrentCam_i, sparsity, neighb,
				 frame.orientation()[0],frame.orientation()[1],frame.orientation()[2],frame.orientation()[3],
				 x,y,z, isFix_i);

		return buf;
	}
};



class PointCloudViewer : public QGLViewer
{

	friend RobotViewer;
public:
	PointCloudViewer();
	~PointCloudViewer();

	void reset();

	void addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);

	MAP* robot_map;

    Sophus::Vector4f robot_pose;
    Sophus::Vector4f robot_origin;
    Sophus::Vector4f robot_direction_global;
    Sophus::Vector4f robot_direction;

    Sophus::Matrix4f POSE;

protected :
	virtual void draw();
	virtual void init();
	virtual void keyPressEvent(QKeyEvent *e);
	virtual void keyReleaseEvent(QKeyEvent *e);
	virtual QString helpString() const;

//	virtual void drawText(int x, int y, const QString & text, const QFont & fnt) {printf(text.toStdString().c_str());};


private:


	int last_frame_id;
//    int key_frame_size;
//    int last_key_frame_size;



	// displays kf-graph
	KeyFrameGraphDisplay* graphDisplay;

	// displays only current keyframe (which is not yet in the graph).
	KeyFrameDisplay* currentCamDisplay;



	// meddle mutex
	boost::mutex meddleMutex;


	void setToVideoSize();
	bool resetRequested;

	// for saving stuff
	std::string save_folder;
	double localMsBetweenSaves;
	double simMsBetweenSaves;
	double lastSaveTime;
	double lastCamTime;
	int lastCamID;


	double lastLocalSaveTime;
	double lastRealSaveTime;


	// for keyframe interpolation
	int KFLastPCSeq;
	int KFcurrent;
	double KFautoPlayIdx[10];
	bool KFexists[10];
	double lastAutoplayCheckedSaveTime;

	// for display settings autoplay
	std::vector<AnimationObject> animationList;
	qglviewer::KeyFrameInterpolator* kfInt;
	bool customAnimationEnabled;

	bool animationPlaybackEnabled;
	double animationPlaybackTime;
	int animationPlaybackID;



	double lastAnimTime;


	void remakeAnimation();


};


class RobotViewer : public QGLViewer {

public:
	RobotViewer(){
		pathFinder = new Astar;
	}

protected:
	virtual void draw();
	virtual void init();
	virtual void animate();
	virtual void keyPressEvent(QKeyEvent *e);
	//virtual void keyReleaseEvent(QKeyEvent *e);
	virtual QString helpString() const;

private:

	Sophus::Matrix4f POSE;
	std::vector<Sophus::Vector4f> robot_pose;
	int last_frame_id = 0;
	Sophus::Vector4f Robot_pose;

	//Astar
	Astar* pathFinder;





};

class Viewer : public QGLViewer {
public:
	Viewer() : wireframe_(false), flatShading_(false){};

protected:
	virtual void draw();
	virtual void init();
	virtual void keyPressEvent(QKeyEvent *e);
	virtual void mousePressEvent(QMouseEvent *e);

	virtual QString helpString() const;

private:
	bool wireframe_, flatShading_;
};
