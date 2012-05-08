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

// HDAL
#include <hdl/hdl.h>
#include <hdlu/hdlu.h>


//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

//w miejscach gdzie mamy wartosci w tablicach indeksowanych 0,1,2 odpowiadaja osiom x,y,z
double force[3];
// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W         = 600;
const int WINDOW_SIZE_H         = 600;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

// maximum number of haptic devices supported in this demo
const int MAX_DEVICES           = 8;

//ustawiamy maksymalny czas, po ktorym nie generujemy sil na falcony - dla bezpieczenstwa!
const double end_time			= 300;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

//potrzebujemy zegar, uplyw czasu i zmiana polozen pozwala nam okreslic predkosc - uzywane do obliczania wspolczynnikow PID
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

//startowe ustawienia dla regulatora PID (w odpowiednich jednostkach:)
double Kp = 140.0;
double Kd = 1.0;
double Ki = 3;

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

//zmienna (modyfikowana przez 'e') - nie generujemy sil na jednym falconie, brak spzezenia od sily - ale drugi nasladuje jego ruch
bool enable_haptic = true;


/*
Opis struktury
handle - 'wskaznik' na dane urzadzenie, otrzymany hdlInitIndexedDevice() (mozliwe jest po nazwie, ale nie dzialalo prawidlowo
position - odczytana pozycja
velocity - obliczona predkosc - na podstawie zmiany polozenia i czasu
time - czas jaki uplynal od uruchomienia symulacji do ostatniego sprawdzenia stanu danego falcona, majac ten czas mozemy na podstawie aktualnego czasu okreslic roznice czasu, a majac tez roznice polozen - obliczymy predkosc :)
error - wektor x,y,z - roznica polozen obu urzadzen - 'aktualny' - 'ten drugi' na kazdej osi :)
force - wektor sil, w sumie nie uzywany
*/
struct HapticDevice
{
    HDLDeviceHandle handle;
    cVector3d position;
	cVector3d velocity;
	double time;
	cVector3d error;
    cVector3d force;
};

HapticDevice haptic[2];

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
	printf("Liczba podlaczonych hapticow %d\n", numHapticDevices);
if(numHapticDevices != 2){
	exit(1);
}
    while (i < numHapticDevices)
    {

		//inicjujemy device na podstawie indeksu (mozliwe jest inicjowanie po nazwie, zapisanej w 
		//hdal.ini - domyslna sciezka C:\Program Files\Novint\Falcon\HDAL\config w tym przykladzie po nazwie nie udalo sie zainicjowac..
		haptic[i].handle = hdlInitIndexedDevice(i);

		//przypisujemy startowe - zerowe - wartosci kazdemu urzadzeniu
		haptic[i].position.zero();
		haptic[i].velocity.zero();
		haptic[i].error.zero();
		haptic[i].time = 0;

		if (haptic[i].handle == HDL_INVALID_HANDLE)
		{
			printf("Blad przy podlaczeniu z falconem %d: HDL_INVALID_HANDLE",i);
			exit(1);
		}

        // create a cursor by setting its radius
        cShapeSphere* newCursor = new cShapeSphere(0.01);

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
		//string strDevice = "#" + strID + " - " +info.m_modelName;

        // attach a small label next to the cursor to indicate device information
        cLabel* newLabel = new cLabel();
        newCursor->addChild(newLabel);
        newLabel->m_string = strDevice;
        newLabel->setPos(0.00, 0.02, 0.00);
        newLabel->m_fontColor.set(1.0, 1.0, 1.0);

        // crate a small label to indicate the position of the device
        cLabel* newPosLabel = new cLabel();
        rootLabels->addChild(newPosLabel);
        newPosLabel->setPos(0, -20 * i, 0);
        newPosLabel->m_fontColor.set(0.6, 0.6, 0.6);
        labels[i] = newPosLabel;

        // increment counter
        i++;
    }

	//Start servo and all haptic devices, uzywajac Haptic Device Abstraction Layer
	printf("HDAL: hdlStart()");
	hdlStart();

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
    if ((key == 27) || (key == 'x') || (key == 'X'))
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

	if (key == '8')
		Kp -= 10;
	if (key == '9')
		Kp += 10;
	if (key == '5')
		Ki -= .1;
	if (key == '6')
		Ki += .1;
	if (key == '2')
		Kd -= 1;
	if (key == '3')
		Kd += 1;
	if (key == 'e')
		enable_haptic = !enable_haptic;
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
		//stopujemy falcony za pomoca funkcji hdal
		hdlStop();
		hdlUninitDevice(haptic[i].handle);
        i++;
    }
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    // update content of position label
	double actual_time = clock->getCurrentTimeSeconds();
    for (int i=0; i<numHapticDevices; i++)
    {
		//tymczasowa pozycja do wyswietlenia na ekranie
		cVector3d tmp_position;

		tmp_position = haptic[i].position;

		//konwertujemy z metrow na mm
		tmp_position.mul(1000);

        // create a string that concatenates the device number and its position.
        string strID;
        cStr(strID, i);
        string strLabel = "#" + strID + "  x: ";
        cStr(strLabel, tmp_position.x, 5);
        strLabel = strLabel + "   y: ";
        cStr(strLabel, tmp_position.y, 5);
        strLabel = strLabel + "  z: ";
        cStr(strLabel, tmp_position.z, 5);
		strLabel = strLabel + "  t: ";
		cStr(strLabel, actual_time, 2);

		strLabel = strLabel + "  Kp: ";
		cStr(strLabel, Kp, 2);

		strLabel = strLabel + "  Ki: ";
		cStr(strLabel, Ki, 2);

		strLabel = strLabel + "  Kd: ";
		cStr(strLabel, Kd, 2);

        labels[i]->m_string = strLabel;

		string strLabel2 = "";
		cStr(strLabel2, tmp_position.x, 5);
		strLabel2 += " ";
		cStr(strLabel2, tmp_position.y, 5);
		strLabel2 += " ";
		cStr(strLabel2, tmp_position.z, 5);
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
			cSleepMs(9);
        // for each device
        int i=0;
		double actual_time = clock->getCurrentTimeSeconds();
        while (i < numHapticDevices)
        {
			//czynimy aktywnym urzadzeniem ten o numerze 'i'
			hdlMakeCurrent(haptic[i].handle);

            //wektory na pozycje i roznice pozycji
            cVector3d actual_position;
			cVector3d error_position;

			//biblioteka hdal odzekuje wektora indeksowanego [3]
			double tool_position[3];

			hdlToolPosition(tool_position);

			actual_position.x = tool_position[0];
			actual_position.y = tool_position[1];
			actual_position.z = tool_position[2];

			

            // update position and orientation of cursor
            cursors[i]->setPos(actual_position);

            //obliczamy predkosc
            cVector3d actual_velocity;
			cVector3d error_velocity;

			//subr - roznica miedzy aktualnym (actual_position) a haptic[i].position, zapisana do actual_velocity
			actual_position.subr(haptic[i].position, actual_velocity);

			//roznica miedzy ostatnim zapisanym czasem a aktualnym
			double interval = actual_time - haptic[i].time;

			//obliczenie predkosci na podstawie roznicy polozen w czasie interval
			if (interval>0){

				actual_velocity.div(interval);
			}
			else{
				actual_velocity.zero();
			}
			
            // update arrow
            velocityVectors[i]->m_pointA = actual_position;
            velocityVectors[i]->m_pointB = cAdd(actual_position, actual_velocity);


            // compute a reaction force
            cVector3d newForce (0,0,0);

            // apply force field
            if (useForceField)
            {
				if(actual_time < end_time)
				{
					//algorytm PID w C zaczerpniety z
					//http://www.embeddedheaven.com/pid-control-algorithm-c-language.htm

					//obliczamy roznice polozen i predkosci dla proporcjonalnego i rozniczkujacego
					error_position = haptic[1-i].position - actual_position;
					error_velocity = haptic[1-i].velocity - actual_velocity;

					//obliczamy wartosc dla czlonu calkujacego
					haptic[i].error +=error_position;

					//obliczamy sile jaka nalezy wygenerowac
					force[0] = Kp*error_position.x + Kd*error_velocity.x + Ki*haptic[i].error.x;
					force[1] = Kp*error_position.y + Kd*error_velocity.y + Ki*haptic[i].error.y;
					force[2] = Kp*error_position.z + Kd*error_velocity.z + Ki*haptic[i].error.z;


				}
				if (i==1 && !enable_haptic)
				{
					//brak sprzezenia od sily - falconowi 1 dajemy sily = 0, mozemy nim tylko zadawac ruch, kopiowany na drugim falconie
					force[0] = 0;
					force[1] = 0;
					force[2] = 0;

				}


				if(error_position.length() > 0.002 && error_velocity.length() > 0.001){

					//aplikujemy sily tylko gdy roznica polozen jest wieksza od >x< i roznica predkoscy od >y<
					hdlSetToolForce(force);

					//wyswietlamy roznice polozen - printf tutaj i w else doskonale wplywa na 'stabilnosc' falconow :)
					printf("%lf\t%lf\t%lf\t, %lf\t%lf\t%lf\t\n", error_position.x,error_position.y,error_position.z, error_velocity.x,error_velocity.y,error_velocity.z);

				} else{

					printf("roznica polozen i predkosci mala!\n");
				}


				//aktualizujemy dane
				haptic[i].position = actual_position;
				haptic[i].velocity = actual_velocity;
				haptic[i].time = actual_time;
				haptic[i].force.x = force[0];
				haptic[i].force.y = force[1];
				haptic[i].force.z = force[2];

			}

            // increment counter
            i++;
        }
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------
