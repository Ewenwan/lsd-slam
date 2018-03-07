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
#include "../../../../../../../usr/include/c++/4.8/cstdio"
#include "../../../../../../../usr/include/GL/glut.h"
#include "../../../../../../../usr/include/c++/4.8/map"
#include "../../../../../../../usr/include/x86_64-linux-gnu/sys/time.h"
#include "../../../../../../../usr/include/x86_64-linux-gnu/bits/time.h"


#include <sys/time.h>


#include <QCursor>
#include <QKeyEvent>
#include <QMap>
#include <QMenu>
#include <QMouseEvent>

#include <math.h>


using namespace qglviewer;
using namespace std;

struct  timeval  time_PointCloudViewer;

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

	robot_map = new MAP;

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
    robot_direction << 0,0,1,1;

	restoreStateFromFile();
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

	gettimeofday(&time_PointCloudViewer,NULL);
	double cost_time  = (time_PointCloudViewer.tv_sec)+(time_PointCloudViewer.tv_usec)/1000000.0;

    //cout <<"cost time " <<  cost_time << endl;

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
			POSE = currentCamDisplay->camToWorld.matrix();
            //POSE = currentCamDisplay->camToWorld.inverse();
			robot_pose = POSE*robot_origin;
            robot_direction_global =   POSE*robot_direction;

			//cout << setprecision(8) << "Now This Robot in\n\n"<< robot_pose << "\n\n";
			//robot_map->PrintMap();
		}



		currentCamDisplay->drawCam(2*lineTesselation, 0);//currentCam画小电视
		last_frame_id = currentCamDisplay->id;
	}

	if(showCurrentPointcloud)
		currentCamDisplay->drawPC(pointTesselation, 1);//实际没多大用



	graphDisplay->draw();//画关键帧的点云

	//if( currentCamDisplay->id > 5000 )




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
	setAnimationPeriod(1);//设置周期（ms），这个很重要，后期需要调整
	startAnimation();
}


void RobotViewer::draw()
{
	//display trajecty
	if(viewer->currentCamDisplay->id > last_frame_id )
	{

		robot_pose.push_back(viewer->robot_pose);
		Robot_pose = viewer->robot_pose;
		POSE = viewer->POSE;

		//cout << "This Robot in\n\n"<< robot_pose << "\n\n";
		cout << "robot_pose.size() = \n\n"<< robot_pose.size() << "\n\n";


	//display track
		glColor3f(1,0,0);
		glPointSize(4.0);
		//glLineWidth(4);
		glBegin(GL_POINTS);//GL_POINTS  GL_LINES
		for(unsigned int i = 0 ; i < robot_pose.size() ; i++ )
		{
			//Sophus::Vector4f rp = *robot_pose[i];
			glVertex3f(robot_pose[i][0],robot_pose[i][1]-1,robot_pose[i][2]);
		}
		glVertex3f(Robot_pose[0],Robot_pose[1],Robot_pose[2]);
		glEnd();
		last_frame_id = viewer->currentCamDisplay->id;
	}

	//glColor3ub(30, 144, 255);//green
	//display map
	map<node, int>::iterator itr;
	itr = viewer->robot_map->g_map.begin();

	while(itr != viewer->robot_map->g_map.end())
	{
		glPushMatrix();
		glTranslatef(gridUnit/2+gridUnit*itr->first.x, gridUnit, gridUnit/2+gridUnit*itr->first.z);
		//glRotatef((GLfloat) 0, 0.0, 0.0, 0.0);
		if( itr->second > 100 )
			glColor3ub(30, 144, 255);//green
		else
			glColor3ub(0,238,118);
		glScalef(1,2,1);
		glutSolidCube(gridUnit);
		glPopMatrix();
		itr++;
	}


	//display camera
	//gridUnit
	glPushMatrix();
	glMultMatrixf((GLfloat*)POSE.data());
	glTranslatef(0.1, -0.3, 0.2);
	//glRotatef((GLfloat) 0, 0.0, 0.0, 0.0);
	glColor3f(0.8,0.4,0.5);
	glScalef (1, 1, 2);
	glutSolidCube(0.2);
	glPopMatrix();


}

void RobotViewer::keyPressEvent(QKeyEvent *e)
{
	switch (e->key())
	{
		case Qt::Key_Left:


			break;

		case Qt::Key_Up :

			break;

		case Qt::Key_Right:	// add settings item

			break;

		case Qt::Key_Down:	// add keyframe item
			break;

		case Qt::Key_S :	// set start point
			break;


		case Qt::Key_E :	// set end point

			break;


		case Qt::Key_R :	// set rigth,begin path planning
			break;

		case Qt::Key_D :	// Display cube map
			break;


		default:
			QGLViewer::keyPressEvent(e);
			break;
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

// Draws a spiral
void Viewer::draw() {
	const float nbSteps = 80.0;

	glBegin(GL_QUAD_STRIP);
	for (float i = 0; i < nbSteps; ++i) {
		float ratio = i / nbSteps;
		float angle = 21.0 * ratio;
		float c = cos(angle);
		float s = sin(angle);
		float r1 = 1.0 - 0.8 * ratio;
		float r2 = 0.8 - 0.8 * ratio;
		float alt = ratio - 0.5;
		const float nor = .5;
		const float up = sqrt(1.0 - nor * nor);
		glColor3f(fabs(c), 0.2f, fabs(s));
		glNormal3f(nor * c, up, nor * s);
		glVertex3f(r1 * c, alt, r1 * s);
		glVertex3f(r2 * c, alt + 0.05, r2 * s);
	}
	glEnd();
}

void Viewer::init() {
	// Restore previous viewer state.
	restoreStateFromFile();

	/////////////////////////////////////////////////////
	//       Keyboard shortcut customization           //
	//      Changes standard action key bindings       //
	/////////////////////////////////////////////////////

	// Define 'Control+Q' as the new exit shortcut (default was 'Escape')
	setShortcut(EXIT_VIEWER, Qt::CTRL + Qt::Key_Q);

	// Set 'Control+F' as the FPS toggle state key.
	setShortcut(DISPLAY_FPS, Qt::CTRL + Qt::Key_F);

	// Disable draw grid toggle shortcut (default was 'G')
	setShortcut(DRAW_GRID, 0);

	// Add custom key description (see keyPressEvent).
	setKeyDescription(Qt::Key_W, "Toggles wire frame display");
	setKeyDescription(Qt::Key_F, "Toggles flat shading display");

	/////////////////////////////////////////////////////
	//         Mouse bindings customization            //
	//     Changes standard action mouse bindings      //
	/////////////////////////////////////////////////////

	// Left and right buttons together make a camera zoom : emulates a mouse third
	// button if needed.

//    setMouseBinding(Qt::Key_Z, Qt::NoModifier, Qt::LeftButton, CAMERA, ZOOM);
//
//    // Disable previous TRANSLATE mouse binding (and remove it from help mouse
//    // tab).
//    setMouseBinding(Qt::NoModifier, Qt::RightButton, NO_CLICK_ACTION);
//
//    setMouseBinding(Qt::ControlModifier | Qt::ShiftModifier, Qt::RightButton,
//                    SELECT);
//    setWheelBinding(Qt::AltModifier, CAMERA, MOVE_FORWARD);
//    setMouseBinding(Qt::AltModifier, Qt::LeftButton, CAMERA, TRANSLATE);
//
//    // Add custom mouse bindings description (see mousePressEvent())
//    setMouseBindingDescription(Qt::NoModifier, Qt::RightButton,
//                               "Opens a camera path context menu");

	// Display the help window. The help window tabs are automatically updated
	// when you define new standard key or mouse bindings (as is done above).
	// Custom bindings descriptions are added using setKeyDescription() and
	// setMouseBindingDescription().
	help();
}

///////////////////////////////////////////////
//      Define new key bindings : F & W      //
///////////////////////////////////////////////

void Viewer::keyPressEvent(QKeyEvent *e) {
	// Get event modifiers key
	const Qt::KeyboardModifiers modifiers = e->modifiers();

	// A simple switch on e->key() is not sufficient if we want to take state key
	// into account. With a switch, it would have been impossible to separate 'F'
	// from 'CTRL+F'. That's why we use imbricated if...else and a "handled"
	// boolean.
	bool handled = false;
	if ((e->key() == Qt::Key_W) && (modifiers == Qt::NoButton)) {
		wireframe_ = !wireframe_;
		if (wireframe_)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		handled = true;
		update();
	} else if ((e->key() == Qt::Key_F) && (modifiers == Qt::NoButton)) {
		flatShading_ = !flatShading_;
		if (flatShading_)
			glShadeModel(GL_FLAT);
		else
			glShadeModel(GL_SMOOTH);
		handled = true;
		update();
	}
	// ... and so on with other else/if blocks.

	if (!handled)
		QGLViewer::keyPressEvent(e);
}

///////////////////////////////////////////////////////////
//             Define new mouse bindings                 //
//   A camera viewpoint menu binded on right button      //
///////////////////////////////////////////////////////////

void Viewer::mousePressEvent(QMouseEvent *e) {
	if ((e->button() == Qt::RightButton) && (e->modifiers() == Qt::NoButton)) {
		QMenu menu(this);
		menu.addAction("Camera positions");
		menu.addSeparator();
		QMap<QAction *, int> menuMap;

		bool atLeastOne = false;
		// We only test the 20 first indexes. This is a limitation.
		for (unsigned short i = 0; i < 20; ++i)
			if (camera()->keyFrameInterpolator(i)) {
				atLeastOne = true;
				QString text;
				if (camera()->keyFrameInterpolator(i)->numberOfKeyFrames() == 1)
					text = "Position " + QString::number(i);
				else
					text = "Path " + QString::number(i);

				menuMap[menu.addAction(text)] = i;
			}

		if (!atLeastOne) {
			menu.addAction("No position defined");
			menu.addAction("Use to Alt+Fx to define one");
		}

		QAction *action = menu.exec(e->globalPos());

		if (atLeastOne && action)
			camera()->playPath(menuMap[action]);
	} else
		QGLViewer::mousePressEvent(e);
}

QString Viewer::helpString() const {
	QString text("<h2>K e y b o a r d A n d M o u s e</h2>");
	text += "This example illustrates the mouse and key bindings "
			"customization.<br><br>";
	text += "Use <code>setShortcut()</code> to change standard action key "
			"bindings (display of axis, grid or fps, exit shortcut...).<br><br>";
	text += "Use <code>setMouseBinding()</code> and "
			"<code>setWheelBinding()</code> to change standard action mouse "
			"bindings ";
	text += "(camera rotation, translation, object selection...).<br><br>";
	text += "If you want to define <b>new</b> key or mouse actions, overload "
			"<code>keyPressEvent()</code> and/or ";
	text += "<code>mouse(Press|Move|Release)Event()</code> to define and bind "
			"your own new actions. ";
	text += "Use <code>setKeyDescription()</code> and "
			"<code>setMouseBindingDescription()</code> to add a description of "
			"your bindings in the help window.<br><br>";
	text += "In this example, we defined the <b>F</b> and <b>W</b> keys and the "
			"right mouse button opens a popup menu. ";
	text += "See the keyboard and mouse tabs in this help window for the "
			"complete bindings description.<br><br>";
	text += "By the way, exit shortcut has been binded to <b>Ctrl+Q</b>.";
	return text;
}
