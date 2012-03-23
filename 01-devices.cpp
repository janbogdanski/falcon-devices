
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

const int MAX_DEVICES           = 8;
int numHapticDevices = 0;
HDLDeviceHandle deviceHandle[MAX_DEVICES];

// Handle to Contact Callback 
HDLServoOpExitCode servoOp;

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

//HapticDevice hd[2];
int main(){

	numHapticDevices = hdlCountDevices();
	hd[0].devicename = "falcon1";
	hd[1].devicename = "falcon2";

	for(int i = 0; i<numHapticDevices; i++){
		hd[i].handle = hdlInitIndexedDevice(i);
		//hd[i].handle = hdlInitNamedDevice(hd[i].devicename);
		// Init device data
		hd[i].pos.zero();
		hd[i].vel.zero();
		hd[i].error.zero();
		hd[i].time = 0;

		if (hd[i].handle == HDL_INVALID_HANDLE)
		{
			//std::cout << "Could not open device: HDL_INVALID_HANDLE" << std::endl;
			printf("num %d, %s\n", numHapticDevices,hd[i].devicename);
			//exit(1);
		} else{
			printf("ok %d, %s %d\n", numHapticDevices,hd[i].devicename, hd[i].handle);
		}

	}


	while(1){

	
	//for(int i = 0; i< MAX_DEVICES; i++){
	//	servoOp
	//}
	

	//printf("num devices: %d %s\n", numHapticDevices, hd[0].devicename);
	
	

	

	hdlStart();

for(int i = 0; i<numHapticDevices; i++){
		hdlMakeCurrent(hd[i].handle);
		
		cVector3d newPosition;
			cVector3d errorPosition;
            //hapticDevices[i]->getPosition(newPosition);
			double positionServo[3];
			double force[3];
			int buttons;

			hdlToolPosition(positionServo);
			hdlToolButtons(&buttons);
			newPosition.x = 0; //positionServo[2];
			newPosition.y = 0; //positionServo[0];
			newPosition.z = 0; //positionServo[1];
			force[0] = 0;
			force[1] = 0;
			force[2] = 7;
			hdlSetToolForce(force);
			printf("%lf %lf %lf \n", positionServo[2], newPosition.y, newPosition.z);
			//printf("%d %d\n", buttons,HDL_BUTTON_1);

	


			

	}
for(int zz = 0; zz<10000; zz++){}

	}
	
	return 1;
	
}
