//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   2.0.0 $Rev: 265 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "timers/CPrecisionClock.h"
#include <iostream>
#include <fstream>
using namespace std;
// HDAL
#include <hdl/hdl.h>
#include <hdlu/hdlu.h>
#include <string>

#include <cstdlib>


//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
#include <windows.h>
//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

double force[2][3];
double last_force[2][3];
// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W         = 600;
const int WINDOW_SIZE_H         = 600;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

// maximum number of haptic devices supported in this demo
const int MAX_DEVICES           = 8;
const double EndTime			= 3600;
const double StartTime			= 1;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------
// Handle to device
HDLDeviceHandle deviceHandle[MAX_DEVICES];

// Handle to Contact Callback
HDLServoOpExitCode servoOp;

cPrecisionClock* clock;

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the virtual scene
cLight *light;

// a little "chai3d" bitmap logo at the bottom of the screen
cBitmap* logo;

// width and height of the current window display
int displayW  = 0;
int displayH  = 0;

double Kp = 140.0; // [N/m] 600
double Kd = 1.0; // 10
double Ki = 3;


const int MAX_FREQ_NUM = 1;
double Freq[MAX_FREQ_NUM];
int Freq_count = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a table containing pointers to all haptic devices detected on this computer
cGenericHapticDevice* hapticDevices[MAX_DEVICES];

// a table containing pointers to label which display the position of
// each haptic device
cLabel* labels[MAX_DEVICES];
cGenericObject* rootLabels;

// number of haptic devices detected
int numHapticDevices = 0;

// table containing a list of 3D cursors for each haptic device
cShapeSphere* cursors[MAX_DEVICES];

// table containing a list of lines to display velocity
cShapeLine* velocityVectors[MAX_DEVICES];

// material properties used to render the color of the cursors
cMaterial matCursorButtonON;
cMaterial matCursorButtonOFF;

// status of the main simulation haptics loop
bool simulationRunning = false;

// root resource path
string resourceRoot;

// force field mode ON/OFF
bool useForceField = false;

// has exited haptics simulation thread
bool simulationFinished = false;

bool EnableHaptics = true;


struct HapticDevice
{
    HDLDeviceHandle handle;
    double workspaceDims[6];
    cVector3d pos;
	cVector3d vel;
	double time;
	cVector3d error; //Distance from desired trajectory
    double transformMat[16];
    cVector3d force;
    bool   button;
	char* devicename;
};

HapticDevice hd[2];

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// function called before exiting the application
void close(void);

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);

//===========================================================================
/*
    DEMO:    device.cpp

    This application illustrates the use of the haptic device handler
    "cHapticDevicehandler" to access all of the haptic devices
    "cGenericHapticDevice" connected to the computer.

    In this example the application opens an OpenGL window and displays a
    3D cursor for each device. Each cursor (sphere + reference frame)
    represents the position and orientation of its respective device.
    If the operator presses the device user button (if available), the color
    of the cursor changes accordingly.

    In the main haptics loop function  "updateHaptics()" , the position,
    orientation and user switch status of each device are retrieved at
    each simulation iteration. The information is then used to update the
    position, orientation and color of the cursor. A force is then commanded
    to the haptic device to attract the end-effector towards the device origin.
*/
//===========================================================================

int main(int argc, char* argv[])
{

    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    printf ("\n");
    printf ("-----------------------------------\n");
    printf ("CHAI 3D\n");
    printf ("Demo: 01-devices\n");
    printf ("Copyright 2003-2009\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[1] - Render attraction force\n");
    printf ("[2] - Render viscous environment\n");
    printf ("[x] - Exit application\n");
    printf ("\n\n");

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

	clock = new cPrecisionClock();


    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    world->setBackgroundColor(0.0, 0.0, 0.0);

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set( cVector3d (0.5, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // create a light source and attach it to the camera
    light = new cLight(world);
    camera->addChild(light);                   // attach light to camera
    light->setEnabled(true);                   // enable light source
    light->setPos(cVector3d( 2.0, 0.5, 1.0));  // position the light source
    light->setDir(cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam


    //-----------------------------------------------------------------------
    // 2D - WIDGETS
    //-----------------------------------------------------------------------

    // create a 2D bitmap logo
    logo = new cBitmap();

    // add logo to the front plane
    camera->m_front_2Dscene.addChild(logo);

    // load a "chai3d" bitmap image file
    bool fileload;
    fileload = logo->m_image.loadFromFile(RESOURCE_PATH("resources/images/chai3d.bmp"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = logo->m_image.loadFromFile("../../../bin/resources/images/chai3d.bmp");
        #endif
    }

    // position the logo at the bottom left of the screen (pixel coordinates)
    logo->setPos(10, 10, 0);

    // scale the logo along its horizontal and vertical axis
    logo->setZoomHV(0.4, 0.4);

    // here we replace all black pixels (0,0,0) of the logo bitmap
    // with transparent black pixels (0, 0, 0, 0). This allows us to make
    // the background of the logo look transparent.
    logo->m_image.replace(
                          cColorb(0, 0, 0),      // original RGB color
                          cColorb(0, 0, 0, 0)    // new RGBA color
                          );

    // enable transparency
    logo->enableTransparency(true);


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    //handler = new cHapticDeviceHandler();

    // read the number of haptic devices currently connected to the computer
    //numHapticDevices = handler->getNumDevices();
	numHapticDevices = hdlCountDevices();

    // limit the number of devices to MAX_DEVICES
    numHapticDevices = cMin(numHapticDevices, MAX_DEVICES);

	hd[0].devicename = "FALCON_1";
	hd[1].devicename = "FALCON_2";

    // create a node on which we will attach small labels that display the
    // position of each haptic device
    rootLabels = new cGenericObject();
    camera->m_front_2Dscene.addChild(rootLabels);

    // create a small label as title
    cLabel* titleLabel = new cLabel();
    rootLabels->addChild(titleLabel);

    // define its position, color and string message
    titleLabel->setPos(0, 30, 0);
    titleLabel->m_fontColor.set(1.0, 1.0, 1.0);
    titleLabel->m_string = "Haptic Device Pos [mm]:";

    // for each available haptic device, create a 3D cursor
    // and a small line to show velocity
    int i = 0;
	std::cout << "Number of devices: " <<numHapticDevices <<std::endl;

	double Freqmin;
	double Freqmax;

	cout<<"Enter the input frequency range"<<endl;
	cout<<"Min: ";
	cin>>Freqmin;
	cout<<"Max: ";
	cin>>Freqmax;

	Freq[0] = Freqmin;
	for(int i = 1; i < MAX_FREQ_NUM; i++)
	{
		Freq[i] = Freq[i-1] + (Freqmax - Freqmin)/(MAX_FREQ_NUM-1);

	}
    while (i < numHapticDevices)
    {
        // get a handle to the next haptic device
        //cGenericHapticDevice* newHapticDevice;
        //handler->getDevice(newHapticDevice, i);


		/*switch (i) {
			case 0:
				std::cout << "HDAL: hdlInitDevice 1" << std::endl;
				deviceHandle[i] = hdlInitNamedDevice("FALCON_1");
				break;
			case 1:
				std::cout << "HDAL: hdlInitDevice 2" << std::endl;
				deviceHandle[i] = hdlInitNamedDevice("FALCON_2");
				break;
		}*/
		//hd[i].handle = hdlInitNamedDevice(hd[i].devicename);
		hd[i].handle = hdlInitIndexedDevice(i);

		// Init device data
		hd[i].pos.zero();
		hd[i].vel.zero();
		hd[i].error.zero();
		hd[i].time = 0;

		if (hd[i].handle == HDL_INVALID_HANDLE)
		{
			std::cout << "Could not open device: HDL_INVALID_HANDLE" << std::endl;
			exit(1);
		}

        // open connection to haptic device
        //newHapticDevice->open();

		// initialize haptic device
		//newHapticDevice->initialize();

        // store the handle in the haptic device table
        //hapticDevices[i] = newHapticDevice;

        // retrieve information about the current haptic device
        //cHapticDeviceInfo info = newHapticDevice->getSpecifications();

        // create a cursor by setting its radius
        cShapeSphere* newCursor = new cShapeSphere(0.000000001);

        // add cursor to the world
        world->addChild(newCursor);

        // add cursor to the cursor table
        cursors[i] = newCursor;

        // create a small line to illustrate velocity
        cShapeLine* newLine = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
        velocityVectors[i] = newLine;

        // add line to the world
        world->addChild(newLine);

        // create a string that concatenates the device number and model name.
        string strID;
        cStr(strID, i);
        string strDevice = "#" + strID + " - ";

        // attach a small label next to the cursor to indicate device information
        cLabel* newLabel = new cLabel();
        newCursor->addChild(newLabel);
        newLabel->m_string = strDevice;
        newLabel->setPos(0.00, 0.02, 0.00);
        newLabel->m_fontColor.set(1.0, 1.0, 1.0);

        // if the device provided orientation sensing (stylus), a reference
        // frame is displayed
        /*if (info.m_sensedRotation == true)
        {
            // display a reference frame
            newCursor->setShowFrame(true);

            // set the size of the reference frame
            newCursor->setFrameSize(0.05, 0.05);
        }*/

        // crate a small label to indicate the position of the device
        cLabel* newPosLabel = new cLabel();
        rootLabels->addChild(newPosLabel);
        newPosLabel->setPos(0, -20 * i, 0);
        newPosLabel->m_fontColor.set(0.6, 0.6, 0.6);
        labels[i] = newPosLabel;


        // increment counter
        i++;
    }

	// starts servo and all haptic devices.
	std::cout << "HDAL: hdlStart" << std::endl;
	hdlStart();

	// sets callback for the nonblocking servo loop
	//std::cout << "HDAL: hdlCreateServoOp" << std::endl;
	//hdlCreateServoOp(NonBlockingServoOpCallback, NULL, bNonBlocking);

	// make a specific haptic device current

    // here we define the material properties of the cursor when the
    // user button of the device end-effector is engaged (ON) or released (OFF)

    // a light orange material color
    matCursorButtonOFF.m_ambient.set(0.5, 0.2, 0.0);
    matCursorButtonOFF.m_diffuse.set(1.0, 0.5, 0.0);
    matCursorButtonOFF.m_specular.set(1.0, 1.0, 1.0);

    // a blue material color
    matCursorButtonON.m_ambient.set(0.1, 0.1, 0.4);
    matCursorButtonON.m_diffuse.set(0.3, 0.3, 0.8);
    matCursorButtonON.m_specular.set(1.0, 1.0, 1.0);





    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve the resolution of the computer display and estimate the position
    // of the GLUT window so that it is located at the center of the screen
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
    int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI 3D");

    // create a mouse menu (right button)
    glutCreateMenu(menuSelect);
    glutAddMenuEntry("full screen", OPTION_FULLSCREEN);
    glutAddMenuEntry("window display", OPTION_WINDOWDISPLAY);
    glutAttachMenu(GLUT_RIGHT_BUTTON);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // simulation in now running
    simulationRunning = true;
	clock->start(true);

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

    // start the main graphics rendering loop
    glutMainLoop();

    // close everything
    close();

    // exit
    return (0);
}

//---------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    // update the size of the viewport
    displayW = w;
    displayH = h;
    glViewport(0, 0, displayW, displayH);

    // update position of labels
    rootLabels->setPos(10, displayH-70, 0);
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // escape key
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
    }

    // option 1:
    if (key == '1')
    {
        useForceField = !useForceField;
        if (useForceField)
        {
            printf ("- Enable force field\n");
        }
        else
        {
            printf ("- Disable force field\n");
        }
    }

	if (key == '[')
		Kp -= 10;
	if (key == ']')
		Kp += 10;
	if (key == 'l')
		Ki -= .1;
	if (key == ';')
		Ki += .1;
	if (key == ',')
		Kd -= 1;
	if (key == '.')
		Kd += 1;
	if (key == 'e')
		EnableHaptics = !EnableHaptics;

	if (key == 'q')
		last_force[1][0] -= 0.5;
	if (key == 'w')
		last_force[1][0] += 0.5;

	if (key == 'a')
		last_force[1][1] -= 0.5;
	if (key == 's')
		last_force[1][1] += 0.5;

	if (key == 'c')
		last_force[1][2] -= 0.5;
	if (key == 'v')
		last_force[1][2] += 0.5;
}

//---------------------------------------------------------------------------

void menuSelect(int value)
{
    switch (value)
    {
        // enable full screen display
        case OPTION_FULLSCREEN:
            glutFullScreen();
            break;

        // reshape window to original size
        case OPTION_WINDOWDISPLAY:
            glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
            break;
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close all haptic devices
    int i=0;
    while (i < numHapticDevices)
    {
        //hd[i]->close();
		//hdlDestroyServoOp();
		hdlStop();
		hdlUninitDevice(hd[i].handle);
        i++;
    }
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{

    // update content of position label
	double newTime = clock->getCurrentTimeSeconds();
    for (int i=0; i<numHapticDevices; i++)
    {
        // read position of device an convert into millimeters
		//hdlMakeCurrent(deviceHandle[i]);
		cVector3d pos;
		double positionServo[3];

		//hdlToolPosition(positionServo);
		pos = hd[i].force;

        //hapticDevices[i]->getPosition(pos);
        //pos.mul(1000);


        // create a string that concatenates the device number and its position.
        string strID;
        cStr(strID, i);
        string strLabel = "#" + strID + "  x: ";
        cStr(strLabel, pos.x, 5);
        strLabel = strLabel + "   y: ";
        cStr(strLabel, pos.y, 5);
        strLabel = strLabel + "  z: ";
        cStr(strLabel, pos.z, 5);
		strLabel = strLabel + "  t: ";
		cStr(strLabel, newTime, 2);

		strLabel = strLabel + "  Kp: ";
		cStr(strLabel, Kp, 2);

		strLabel = strLabel + "  Ki: ";
		cStr(strLabel, Ki, 2);

		strLabel = strLabel + "  Kd: ";
		cStr(strLabel, Kd, 2);

        labels[i]->m_string = strLabel;

		string strLabel2 = "";
		cStr(strLabel2, pos.x, 5);
		strLabel2 += " ";
		cStr(strLabel2, pos.y, 5);
		strLabel2 += " ";
		cStr(strLabel2, pos.z, 5);
		strLabel2 += " ";
		cStr(strLabel2, clock->getCurrentTimeSeconds(), 5);
		strLabel2 += "\n";

    }

    // render world
    camera->renderView(displayW, displayH);

    // Swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
	if (newTime>=EndTime)
	{

		clock->reset();

		Freq_count++;
		if (Freq_count<MAX_FREQ_NUM)
		{
			for (int i = 0; i<numHapticDevices; i++)
			{
				hd[i].error.zero();
			}
		}
		else
		{
			simulationRunning = false;
		}

	}

    // inform the GLUT window to call updateGraphics again (next frame)
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // main haptic simulation loop
    while(simulationRunning)
    {
			Sleep(8);
        // for each device
        int i=0;
		double newTime = clock->getCurrentTimeSeconds();
        while (i < numHapticDevices)
        {
			hdlMakeCurrent(hd[i].handle);

            // read position of haptic device
            cVector3d newPosition;
			cVector3d errorPosition;
            //hapticDevices[i]->getPosition(newPosition);
			double positionServo[3];
			//double force[3];
			hdlToolPosition(positionServo);
			newPosition.x = positionServo[2];
			newPosition.y = positionServo[0];
			newPosition.z = positionServo[1];

			

            // update position and orientation of cursor
            cursors[i]->setPos(newPosition);
            //cursors[i]->setRot(newRotation);

            // read linear velocity from device
            cVector3d linearVelocity;
			cVector3d errorVelocity;
			newPosition.subr(hd[i].pos, linearVelocity);
			double interval = newTime - hd[i].time;
			//cout<<interval<<endl;
			if (interval>0)
				linearVelocity.div(interval);
			else
				linearVelocity.zero();
            //hapticDevices[i]->getLinearVelocity(linearVelocity);

			
            // update arrow
            velocityVectors[i]->m_pointA = newPosition;
            velocityVectors[i]->m_pointB = cAdd(newPosition, linearVelocity);


            // compute a reaction force
            cVector3d newForce (0,0,0);
			double calc_force[3];
            // apply force field
            if (useForceField)
            {
				if (newTime<StartTime)
				{
					newPosition.add(0, 0, 0);
					calc_force[0] = -Kp*newPosition.y - 2*Kd*linearVelocity.y;
					calc_force[1] = -Kp*newPosition.z - 2*Kd*linearVelocity.z;
					calc_force[2] = -Kp*newPosition.x - 2*Kd*linearVelocity.x;
				}
				else if(newTime<EndTime)
				{


					/*errorPosition = newPosition;
					errorVelocity = linearVelocity;
					//cout<<errorVelocity.y<<endl;
					errorPosition.add(0, -0.04*cSinRad(2*3.14/T*(newTime-1)), -0.04*cCosRad(2*3.14/T*(newTime-1)));	
					errorVelocity.add(0, -0.04*2*3.14/T*cCosRad(2*3.14/T*(newTime-1)),0.04*2*3.14/T*cSinRad(2*3.14/T*(newTime-1)));
					hd[i].error +=errorPosition;
					linearVelocity.add(0, 0, 0);
					force[0] = -Kp*errorPosition.y - Kd*errorVelocity.y - Ki*hd[i].error.y;
					force[1] = -Kp*errorPosition.z - Kd*errorVelocity.z - Ki*hd[i].error.z;
					force[2] = -Kp*errorPosition.x - Kd*errorVelocity.x - Ki*hd[i].error.x;*/

errorPosition = newPosition - hd[1-i].pos;
					//errorPosition.add(0, -hd[1-i].pos.y, 0);
					errorVelocity = linearVelocity - hd[1-i].vel;
					//errorVelocity.add(0, -hd[1-i].vel.y, 0);
					hd[i].error +=errorPosition;

					calc_force[0] = -Kp*errorPosition.y - Kd*errorVelocity.y - Ki*hd[i].error.y;
					calc_force[1] = -Kp*errorPosition.z - Kd*errorVelocity.z - Ki*hd[i].error.z;
					calc_force[2] = -Kp*errorPosition.x - Kd*errorVelocity.x - Ki*hd[i].error.x;


				}
				if (i==1 && !EnableHaptics)
				{
					double f = Freq[Freq_count]; // in Hz
					double T = 1/f;
					//force[0] += 5*cSinRad(2*3.14/T*(newTime-StartTime));
					force[i][0] = 0;
					force[i][1] = 0;
					force[i][2] = 0;

				}

				force[i][0]= calc_force[0];
				force[i][1]= calc_force[1];
				force[i][2]= calc_force[2];


				//czy uzyc stalej sily do testow - zmiana wart sil - q,w, a,s, z,x 
				int const_force = false;

				//ktory falcon 
				int which_falcon = 1;
				if(const_force){
					force[i][0] = last_force[i][0];
					force[i][1] = last_force[i][1];
					force[i][2] = last_force[i][2];
					if(i%2 == 0){

					force[i][0] +=0.1;
					force[i][1] +=0.1;
					force[i][2] +=0.1;
					} else{

					//force[i][0] -=0.1;
					//force[i][1] -=0.1;
					//force[i][2] -=0.1;
					}
				}

			printf("pos %d %lf %lf %lf %lf %lf\n", i, newPosition.x, newPosition.y, newPosition.z, errorPosition.length(), errorVelocity.length());

				if(errorPosition.length() > 0.008 && errorVelocity.length() > 0.001){

					hdlSetToolForce(force[i]);
					//Sleep(1);

				}
				hd[i].pos = newPosition;
				hd[i].vel = linearVelocity;
				hd[i].time = newTime;
				hd[i].force.x = force[i][2];
				hd[i].force.y = force[i][0];
				hd[i].force.z = force[i][1];

			}

            // increment counter
            i++;


        }

    }
    
    // exit haptics thread



    simulationFinished = true;
}

//---------------------------------------------------------------------------
