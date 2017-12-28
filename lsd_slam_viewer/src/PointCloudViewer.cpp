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

#define GL_GLEXT_PROTOTYPES 1
#include "PointCloudViewer.h"
#include "qfiledialog.h"
#include "qcoreapplication.h"
#include <stdio.h>
#include "settings.h"
#include "ros/package.h"
#include <vector>
#include <zlib.h>
#include <iostream>


#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "QGLViewer/manipulatedCameraFrame.h"

#include "KeyFrameDisplay.h"
#include "KeyFrameGraphDisplay.h"

#include <iostream>
#include <fstream>

#include <math.h>
#include <stdlib.h> // RAND_MAX
#include "main_viewer.h"
#include "../../../../../../../usr/include/GL/gl.h"


using namespace qglviewer;
using namespace std;


PointCloudViewer::PointCloudViewer()
{
	setPathKey(Qt::Key_0,0);
	setPathKey(Qt::Key_1,1);
	setPathKey(Qt::Key_2,2);
	setPathKey(Qt::Key_3,3);
	setPathKey(Qt::Key_4,4);
	setPathKey(Qt::Key_5,5);
	setPathKey(Qt::Key_6,6);
	setPathKey(Qt::Key_7,7);
	setPathKey(Qt::Key_8,8);
	setPathKey(Qt::Key_9,9);

    last_frame_id = 0;
//    key_frame_size = 0;
//    last_key_frame_size = 0;


	currentCamDisplay = 0;
	graphDisplay = 0;


	for(int i=0;i<10;i++)
	{
		KFexists[i] = 0;
		KFautoPlayIdx[i] = -1;
	}

	kfInt = new qglviewer::KeyFrameInterpolator(new qglviewer::Frame());
	customAnimationEnabled = false;

	setSnapshotFormat(QString("PNG"));

	reset();//重置
}


PointCloudViewer::~PointCloudViewer()
{
	delete currentCamDisplay;
	delete graphDisplay;
}


void PointCloudViewer::reset()
{
	if(currentCamDisplay != 0)
		delete currentCamDisplay;
	if(graphDisplay != 0)
		delete graphDisplay;

	currentCamDisplay = new KeyFrameDisplay();
	graphDisplay = new KeyFrameGraphDisplay();

	KFcurrent = 0;
	KFLastPCSeq = -1;

	resetRequested=false;

	save_folder = ros::package::getPath("lsd_slam_viewer")+"/save/";
	localMsBetweenSaves = 1;
	simMsBetweenSaves = 1;
	lastCamID = -1;
	lastAnimTime = lastCamTime = lastSaveTime = 0;
	char buf[500];
	snprintf(buf,500,"rm -rf %s",save_folder.c_str());
	int k = system(buf);
	snprintf(buf,500,"mkdir %s",save_folder.c_str());
	k += system(buf);


	assert(k != -42);

	setSceneRadius(80);
	setTextIsEnabled(false);
	lastAutoplayCheckedSaveTime = -1;

	animationPlaybackEnabled = false;
}

void PointCloudViewer::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	meddleMutex.lock();
    
//  displays kf-graph
// 	KeyFrameGraphDisplay* graphDisplay;
// 
// 	displays only current keyframe (which is not yet in the graph).
// 	KeyFrameDisplay* currentCamDisplay;

	if(!msg->isKeyframe)//不是关键帧
	{
		if(currentCamDisplay->id > msg->id)
		{
			printf("detected backward-jump in id (%d to %d), resetting!\n", currentCamDisplay->id, msg->id);
			resetRequested = true;
		}
		currentCamDisplay->setFrom(msg);
		lastAnimTime = lastCamTime = msg->time;
		lastCamID = msg->id;
	}
	else//是关键帧
    {
        graphDisplay->addMsg(msg);//添加关键真的KeyFrameDisplay信息
        //key_frame_size++;
    }


	meddleMutex.unlock();
}

void PointCloudViewer::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	meddleMutex.lock();

	graphDisplay->addGraphMsg(msg);//添加关键真之间的约束

	meddleMutex.unlock();
}




void PointCloudViewer::init()
{
    robot_pose << 0,0,0,1;
    robot_origin << 0,0,0,1;
    
    cout.setf(ios::fixed);
	setAnimationPeriod(30);//设置频率，这个很重要，后期需要调整
	startAnimation();
}

QString PointCloudViewer::helpString() const
{
	return QString("");
}

void PointCloudViewer::draw()
{
	meddleMutex.lock();


	if(resetRequested)
	{
		reset();
		resetRequested = false;
	}


	glPushMatrix();


	if(animationPlaybackEnabled)
	{
		double tm = ros::Time::now().toSec() - animationPlaybackTime;

		if(tm > kfInt->lastTime())
		{
			animationPlaybackEnabled = false;
			tm = kfInt->lastTime();
		}

		if(tm < kfInt->firstTime())
			tm = kfInt->firstTime();

		printf("anim at %.2f (%.2f to %.2f)\n", tm, kfInt->firstTime(), kfInt->lastTime());


		kfInt->interpolateAtTime(tm);
		camera()->frame()->setFromMatrix(kfInt->frame()-> matrix());



		double accTime = 0;
		for(unsigned int i=0;i<animationList.size();i++)
		{
			if(tm >= accTime && tm < accTime+animationList[i].duration && animationList[i].isFix)
			{
				camera()->frame()->setFromMatrix(animationList[i].frame.matrix());

				printf("fixFrameto %d at %.2f (%.2f to %.2f)\n", i, tm, kfInt->firstTime(), kfInt->lastTime());
			}

			accTime += animationList[i].duration;
		}


		accTime = 0;
		AnimationObject* lastAnimObj = 0;
		for(unsigned int i=0;i<animationList.size();i++)
		{
			accTime += animationList[i].duration;
			if(animationList[i].isSettings && accTime <= tm)
				lastAnimObj = &(animationList[i]);
		}
		if(lastAnimObj != 0)
		{
			absDepthVarTH = lastAnimObj->absTH;
			scaledDepthVarTH = lastAnimObj->scaledTH;
			minNearSupport = lastAnimObj->neighb;
			sparsifyFactor = lastAnimObj->sparsity;
			showKFCameras = lastAnimObj->showKeyframes;
			showConstraints = lastAnimObj->showLoopClosures;
		}
	}



	if(showCurrentCamera)
    {
        //printf("This PC frame is %d\n",currentCamDisplay->id);
        if(currentCamDisplay->id > last_frame_id)
        {
            printf("This PC frame is %d\n",currentCamDisplay->id);
            Sophus::Matrix4f POSE = currentCamDisplay->camToWorld.matrix();

            //std::cout << "POSE is \n\n"<<  POSE.matrix() << "\n\n";
            /*
            Sophus::Matrix3f Rotation = POSE.rotationMatrix();
            //cout << "Rotation is \n\n"<< Rotation << "\n\n";
            
            Sophus::Vector3f Translation = POSE.translation();
            //cout << "Translation is \n\n"<< Translation << "\n\n";
            
            pose_pi *= Rotation.transpose();
            
            pose_sigma += pose_pi*Translation;*/
            
            robot_pose = POSE*robot_origin;
            
            cout << setprecision(8) << "Now This Robot in\n\n"<< robot_pose << "\n\n";
        }
        
        
        
		currentCamDisplay->drawCam(2*lineTesselation, 0);//currentCam画小电视
        last_frame_id = currentCamDisplay->id;
    }

	if(showCurrentPointcloud)
		currentCamDisplay->drawPC(pointTesselation, 1);//


    //if(key_frame_size > last_key_frame_size )
    //{
    graphDisplay->draw();//画关键帧的点云
        //last_key_frame_size = key_frame_size;
    //}



	glPopMatrix();

	meddleMutex.unlock();




	if(saveAllVideo)
	{
		double span = ros::Time::now().toSec() - lastRealSaveTime;
		if(span > 0.4)
		{
			setSnapshotQuality(100);

			printf("saved (img %d @ time %lf, saveHZ %f)!\n", lastCamID, lastAnimTime, 1.0/localMsBetweenSaves);

			char buf[500];
			snprintf(buf,500,"%s%lf.png",save_folder.c_str(),  ros::Time::now().toSec());
			saveSnapshot(QString(buf));
			lastRealSaveTime = ros::Time::now().toSec();
		}


	}
}

void PointCloudViewer::keyReleaseEvent(QKeyEvent *e)
  {

  }


void PointCloudViewer::setToVideoSize()
{
	this->setFixedSize(1600,900);
}


void PointCloudViewer::remakeAnimation()
{
	delete kfInt;
	kfInt = new qglviewer::KeyFrameInterpolator(new qglviewer::Frame());
	std::sort(animationList.begin(), animationList.end());

	float tm=0;
	for(unsigned int i=0;i<animationList.size();i++)
	{
	  if(!animationList[i].isSettings)
	  {
		  kfInt->addKeyFrame(&animationList[i].frame, tm);
		  tm += animationList[i].duration;
	  }
	}

	printf("made animation with %d keyframes, spanning %f s!\n", kfInt->numberOfKeyFrames(), tm);
}

void PointCloudViewer::keyPressEvent(QKeyEvent *e)
  {
    switch (e->key())
    {
      case Qt::Key_S :
    	    setToVideoSize();
    	  break;

      case Qt::Key_R :
    	    resetRequested = true;

    	  break;

      case Qt::Key_T:	// add settings item
    	  meddleMutex.lock();
    	  animationList.push_back(AnimationObject(true, lastAnimTime, 0));
    	  meddleMutex.unlock();
    	  printf("added St: %s\n", animationList.back().toString().c_str());

    	  break;

      case Qt::Key_K:	// add keyframe item
    	  meddleMutex.lock();


    	  float x,y,z;
    	  camera()->frame()->getPosition(x,y,z);
    	  animationList.push_back(AnimationObject(false, lastAnimTime, 2, qglviewer::Frame(qglviewer::Vec(0,0,0), camera()->frame()->orientation())));
    	  animationList.back().frame.setPosition(x,y,z);
    	  meddleMutex.unlock();
    	  printf("added KF: %s\n", animationList.back().toString().c_str());



    	  remakeAnimation();

    	  break;

      case Qt::Key_I :	// reset animation list
			meddleMutex.lock();
			animationList.clear();
			meddleMutex.unlock();
			printf("resetted animation list!\n");

			remakeAnimation();

    	  break;


      case Qt::Key_F :	// save list
      {
			meddleMutex.lock();
			std::ofstream myfile;
			myfile.open ("animationPath.txt");
			for(unsigned int i=0;i<animationList.size();i++)
			{
				myfile << animationList[i].toString() << "\n";
			}
			myfile.close();
			meddleMutex.unlock();

			printf("saved animation list (%d items)!\n", (int)animationList.size());
      }
    	  break;


      case Qt::Key_L :	// load list
      {
			meddleMutex.lock();
			animationList.clear();

			std::ifstream myfile;
			std::string line;
			myfile.open ("animationPath.txt");

			if (myfile.is_open())
			{
				while ( getline (myfile,line) )
				{
					if(!(line[0] == '#'))
						animationList.push_back(AnimationObject(line));
				}
				myfile.close();
			}
			else
				std::cout << "Unable to open file";
			myfile.close();
			meddleMutex.unlock();

			printf("loaded animation list! (%d items)!\n", (int)animationList.size());
			remakeAnimation();
      }
    	  break;


      case Qt::Key_A:
    	  if(customAnimationEnabled)
    		  printf("DISABLE custom animation!\n)");
    	  else
    		  printf("ENABLE custom animation!\n");
    	  customAnimationEnabled = !customAnimationEnabled;
    	  break;

      case Qt::Key_O:
    	  if(animationPlaybackEnabled)
    	  {
    		  animationPlaybackEnabled=false;
    	  }
    	  else
    	  {
    		  animationPlaybackEnabled = true;
    		  animationPlaybackTime = ros::Time::now().toSec();
    	  }
      	  break;


      case Qt::Key_P:
    	  graphDisplay->flushPointcloud = true;
    	  break;

      case Qt::Key_W:
    	  graphDisplay->printNumbers = true;
    	  break;

      default:
    	  QGLViewer::keyPressEvent(e);
    	  break;
    }
  }
  
///////////////////////RobotViewer  ///////////////////////
void RobotViewer::init() 
{
    restoreStateFromFile();
    glDisable(GL_LIGHTING);
    //setAxisIsDrawn();
    //nbPart_ = 2000;
    //particle_ = new Particle[nbPart_];
    glPointSize(8.0);
    //setGridIsDrawn();
    //drawGrid(2,10);
    //help();
    //setAnimationPeriod(30);//设置频率，这个很重要，后期需要调整
    startAnimation();
}

// 将立方体的八个顶点保存到一个数组里面

static const float vertex_list[][3] =
{   //upper 4 point
    0, 0,gridUnit,
    gridUnit, 0, gridUnit,
    gridUnit, 0, 0,
    0, 0, 0,

        //down 4 point
    0, 0.1,gridUnit,
    gridUnit, 0.1, gridUnit,
    gridUnit, 0.1, 0,
    0, 0.1, 0,

};

// 将要使用的顶点的序号保存到一个数组里面

static const GLint index_list[][4] =
{
    {4,5,6,7},//upper
    {0,1,2,3},//down
    {0,1,5,4},//front
    {3,2,6,7},//back
    {0,3,7,4},//left
    {1,2,6,5},//right
};

void RobotViewer::draw() 
{


	glColor3ub(0 ,255 ,127);//green
	//glRectf(0,0,1,1);

    int i=0,j=0;

    glBegin(GL_QUADS);
        for(i=0; i<6; ++i) // 6个面
        {
            glColor3ub((i+1)*30 ,0 ,0);
            for(j=0; j<4; ++j) // 每个面 4个顶点
            {
                glVertex3fv(vertex_list[index_list[i][j]]);
            }
        }
    glEnd();

    glPushMatrix();


        glTranslatef (0.1, 0.1, 0.1);
        glRotatef ((GLfloat) 0, 0.0, 0.0, 0.0);

        glColor3f(0.8,0.4,0.5);


        glScalef (1, 1, 1);
        glutSolidCube(0.2);



    glPopMatrix();


    if(viewer->currentCamDisplay->id > last_frame_id )
    {

        robot_pose.push_back(viewer->robot_pose);
        Robot_pose = viewer->robot_pose;

        //cout << "This Robot in\n\n"<< robot_pose << "\n\n";
        cout << "robot_pose.size() = \n\n"<< robot_pose.size() << "\n\n";


        glColor3f(1,0,0);
        glPointSize(4.0);
        //glLineWidth(4);
        glBegin(GL_POINTS);//GL_POINTS  GL_LINES
            for(unsigned int i = 0 ; i < robot_pose.size() ; i++ )
            {
                //Sophus::Vector4f rp = *robot_pose[i];
                glVertex3f(robot_pose[i][0],robot_pose[i][1],robot_pose[i][2]);
            }
            glVertex3f(Robot_pose[0],Robot_pose[1],Robot_pose[2]);
        glEnd();
        glFlush();

        last_frame_id = viewer->currentCamDisplay->id;
    }

    
}

void RobotViewer::animate() 
{
    //printf("animate OK!\n");
//     for (int i = 0; i < nbPart_; i++)
//         particle_[i].animate();
}

QString RobotViewer::helpString() const 
{
    QString text("<h2>A n i m a t i o n</h2>");
    text += "Use the <i>animate()</i> function to implement the animation part "
            "of your ";
    text += "application. Once the animation is started, <i>animate()</i> and "
            "<i>draw()</i> ";
    text += "are called in an infinite loop, at a frequency that can be "
            "fixed.<br><br>";
    text += "Press <b>Return</b> to start/stop the animation.";
    return text;
}

///////////////////////   P a r t i c l e   ///////////////////////////////

Particle::Particle() { init(); }

void Particle::animate() 
{
    speed_.z -= 0.05f;
    pos_ += 0.1f * speed_;

    if (pos_.z < 0.0) {
        speed_.z = -0.8 * speed_.z;
        pos_.z = 0.0;
    }

    if (++age_ == ageMax_)
        init();
}

void Particle::draw() 
{
    glColor3f(age_ / (float)ageMax_, age_ / (float)ageMax_, 1.0);
    glVertex3fv(pos_);
}

void Particle::init() 
{
    pos_ = Vec(0.0, 0.0, 0.0);
    float angle = 2.0 * M_PI * rand() / RAND_MAX;
    float norm = 0.04 * rand() / RAND_MAX;
    speed_ = Vec(norm * cos(angle), norm * sin(angle),
                rand() / static_cast<float>(RAND_MAX));
    age_ = 0;
    ageMax_ = 50 + static_cast<int>(100.0 * rand() / RAND_MAX);
}

