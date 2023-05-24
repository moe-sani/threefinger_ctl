#ifndef THREEFINGER_CTL__EPOS_DRIVER_HPP_
#define THREEFINGER_CTL__EPOS_DRIVER_HPP_

#include "threefinger_ctl/Definitions.h"
#include "threefinger_ctl/visibility_control.h"

namespace threefinger_ctl
{

class eposdriver
{
	typedef int BOOL;
	typedef unsigned int DWORD;
private:
	void *deviceHandle;
	unsigned int nodeId;
	unsigned int errCode;
public:
	eposdriver();
    eposdriver(const char[], unsigned int);
    ~eposdriver();
	const char* port;
	void* deviceOpen(const char *port, unsigned int nodeId);
	void nodeReset(void *deviceHandle, unsigned int nodeId);
	void deviceClose(void *deviceHandle, const char *port, unsigned int nodeId);
	void subDeviceCloseAll(void *deviceHandle);
	void nodeEnable(void *deviceHandle, unsigned int nodeId);
	void nodeDisable(void *deviceHandle, unsigned int nodeId);
	void nodeOpMode(void *deviceHandle, unsigned int nodeId);
	void nodeMotion(void *deviceHandle, unsigned int nodeId, long desiredAngle);
	void nodeReadAngle(void *deviceHandle, unsigned int nodeId, int &encoderAngle);
	void nodeStopMotion(void *deviceHandle, unsigned int nodeId);
	//void invGeometricModel(double p[2], double phi, double v, double theta[6]);
	// void invGeometricModel(double b1B[4], double b2B[4], double b3B[4],int tog, double theta[6]);
	// void transMatrixMultiply(double T1[4][4], double T2[4][4], double Tout[4][4]);
	// void vectorMatrixMultiply(double T1[4][4], double a[4], double b[4]);
	// void exoToolMapping(double e1B[4], double e2B[4], double e3B[4], double b1B[4], double b2B[4], double b3B[4]);
	// void daVinciIGM(double T[4][4], double w, double theta[4]);
};

}  // namespace threefinger_ctl

#endif  // THREEFINGER_CTL__EPOS_DRIVER_HPP_
