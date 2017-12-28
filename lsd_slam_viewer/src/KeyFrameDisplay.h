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

#undef Success
#include <Eigen/Core>

#include "QGLViewer/qglviewer.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "sophus/sim3.hpp"
#include "../msg_gen/cpp/include/lsd_slam_viewer/keyframeMsg.h"
#include "../thirdparty/Sophus/sophus/sim3.hpp"
#include "../../../../../../../usr/include/c++/4.8/set"
#include "../../../../../../../usr/include/c++/4.8/iostream"
#include "../../../../../../../usr/include/c++/4.8/ostream"
#include "../../../../../../../usr/include/c++/4.8/map"
#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <set>
#include <opencv2/core/types_c.h>

using namespace std;


#define gridUnit 0.02

struct node
{
    int x;
    int y;
    bool operator < (const node &compNode) const
    {
        return (this->x < compNode.x || (this->x == compNode.x && this->y < compNode.y));
    }
};


struct MyVertex
{
    float point[3];
    uchar color[4];
    bool operator < (const MyVertex &compVertex) const
    {
        return (         this->point[0] <  compVertex.point[0]
                     || (this->point[0] == compVertex.point[0] && this->point[1] <  compVertex.point[1])
                     || (this->point[0] == compVertex.point[0]
                         && this->point[1] == compVertex.point[1] && this->point[2] < compVertex.point[2] )
        );
    }
};

struct InputPointDense
{
    float idepth;
    float idepth_var;
    uchar color[4];
};

class MAP
{
    //friend ostream &operator<< (ostream&,const node&);
public:
    MAP()
    {
        error = 10000;
    }
    void gridInsertNode(int x, int y )
    {
        node temp_n;
        temp_n.x = x;
        temp_n.y = y;
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
    set<MyVertex> vertex_map;

    int error;

    int  &operator ()(int x, int y)
    {
        node temp_n;
        temp_n.x = x;
        temp_n.y = y;

        if (g_map.find(temp_n) == g_map.end())//no this node
        {
            cout << "no this node"<<endl;
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
            cout << "node "<< i  <<", x= "<< itr->first.x
                    << ", y= " << itr->first.y
                    << ", point cloud =" << itr->second
                    << endl;
            itr++;
            i++;
        }

    }
};

//ostream &operator<< (ostream &out,const node &n)
//{
//    out << "x=" << n.x << " y=" <<n.y ;
//    return  out;
//}



// stores a pointcloud associated to a Keyframe.
class KeyFrameDisplay
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	KeyFrameDisplay();
	~KeyFrameDisplay();


	void setFrom(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void drawCam(float lineWidth = 1, float* color = 0);
	void drawPC(float pointSize = 1, float alpha = 1);
	void refreshPC();

	int flushPC(std::ofstream* f);



	int id;
	double time;

	int totalPoints, displayedPoints;


	// camera pose
	// may be updated by kf-graph.
	Sophus::Sim3f camToWorld;
  
    

private:
	// camera parameter
	// fixed.
	float fx,fy,cx,cy;
    int width, height;
	float fxi,fyi,cxi,cyi;
	

	float my_scaledTH, my_absTH, my_scale;
	int my_minNearSupport;
	int my_sparsifyFactor;


	// pointcloud data & respective buffer
	InputPointDense* originalInput;


	// buffer & how many
	GLuint vertexBufferId;
	int vertexBufferNumPoints;


	bool vertexBufferIdValid;	// true if the vertixBufferID is valid (doesnt mean the data in there is still valid)
	bool glBuffersValid;		// true if the vertexBufferID contains valid data

    MAP* robot_map;
};

