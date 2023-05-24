#include "threefinger_ctl/epos_driver.hpp"
#include <ostream>
#include <iostream>

namespace threefinger_ctl
{

eposdriver::eposdriver()
{
    port = "USB0";
	errCode = 0x00;
	nodeId = 64;
}

eposdriver::eposdriver(const char* portString, unsigned int input_nodeId)
{
	errCode = 0x00;
	port = portString;
	nodeId = input_nodeId;
}

eposdriver::~eposdriver()
{
}


void* eposdriver::deviceOpen(const char *port, unsigned int nodeId)
{

	char deviceName[] = "EPOS2";
	char protocolStackName[] = "MAXON SERIAL V2";
	char interfaceName[] = "USB";
	DWORD errCode = 0x00;
	unsigned long timeOut = 500;
	unsigned long baudRate = 1000000;
	void *deviceHandle;

	// Open Communication with ePos
	deviceHandle = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, (char*)port, &errCode);

	// Check if communication opened successfully
	if (deviceHandle == 0)
	{
		std::cout << "Device failed to open, error code = " << errCode << std::endl;
		exit(0);
	}
	else
	{
		std::cout << "Device opened successfully." << std::endl;
	}

	if (!VCS_SetProtocolStackSettings(deviceHandle, baudRate, timeOut, &errCode))
	{
		std::cout << "Protocol stack settings failed, error code =" << errCode << std::endl;
	}

	return deviceHandle;
}

void eposdriver::nodeReset(void *deviceHandle, unsigned int nodeId)
{

	DWORD errCode = 0x00;
	
	if (!VCS_ResetDevice(deviceHandle, nodeId, &errCode))
	{
		std::cout << "Node " << nodeId << "could not be reset, error code" << &errCode << std::endl;
	}

}


void eposdriver::deviceClose(void *deviceHandle, const char *port, unsigned int nodeId)
{

	DWORD errCode = 0x00;

	if (deviceHandle != 0)
	{
		VCS_CloseDevice(deviceHandle, &errCode);
	}
}

void eposdriver::subDeviceCloseAll(void *deviceHandle)
{

	DWORD errCode = 0x00;

	if (deviceHandle != 0)
	{
		VCS_CloseAllSubDevices(deviceHandle, &errCode);
	}
}

void eposdriver::nodeEnable(void *deviceHandle, unsigned int nodeId)
{

	DWORD errCode = 0x00;
	int fault = 0;

	if (VCS_GetFaultState(deviceHandle, nodeId, &fault, &errCode))
	{
		if (fault && !VCS_ClearFault(deviceHandle, nodeId, &errCode))
		{
			std::cout << "Clear fault failed, error code =" << errCode << std::endl;
			return;
		}

		int isEnabled = 0;
		if (VCS_GetEnableState(deviceHandle, nodeId, &isEnabled, &errCode))
		{
			if (!isEnabled && !VCS_SetEnableState(deviceHandle, nodeId, &errCode))
			{
				std::cout << "Failed to enable node " << nodeId << ", error code = " << errCode << std::endl;
			}
			else {
				std::cout << "Node " << nodeId << " is enabled." << std::endl;;
			}
		}
	}
	else
	{
		std::cout << "Could not get fault state, error code" << errCode << std::endl;
	}

}

void eposdriver::nodeDisable(void *deviceHandle, unsigned int nodeId)
{

	DWORD errCode = 0x00;
	int fault = 0;

	if (VCS_GetFaultState(deviceHandle, nodeId, &fault, &errCode))
	{
		if (fault && !VCS_ClearFault(deviceHandle, nodeId, &errCode))
		{
			std::cout << "Clear fault failed, error code =" << errCode << std::endl;
			return;
		}

		int isEnabled = 0;
		if (VCS_GetEnableState(deviceHandle, nodeId, &isEnabled, &errCode))
		{
			if (!isEnabled && !VCS_SetDisableState(deviceHandle, nodeId, &errCode))
			{
				std::cout << "Failed to disable node " << nodeId << ", error code = " << errCode << std::endl;
			}
			else {
				VCS_SetDisableState(deviceHandle, nodeId, &errCode);
				std::cout << "Node " << nodeId << " is disabled." << std::endl;
			}
		}
	}
	else
	{
		std::cout << "Could not get fault state, error code" << errCode << std::endl;
	}
}

void eposdriver::nodeOpMode(void *deviceHandle, unsigned int nodeId)
{
	DWORD errCode = 0x00;
	int nodeMode = -1; //position mode
	//const char* nodeMode = "OMD_POSITION_MODE";

	if (!VCS_SetOperationMode(deviceHandle, nodeId, nodeMode, &errCode))
	//if (!VCS_ActivatePositionMode(deviceHandle, nodeId, &errCode))
	{
		std::cout << "Could not set operating mode of node " << nodeId << std::endl;
	}

}

void eposdriver::nodeMotion(void *deviceHandle, unsigned int nodeId, long desiredAngle)
{

	DWORD errCode = 0x00;

	if (!VCS_SetPositionMust(deviceHandle, nodeId, desiredAngle, &errCode))
	{
		std::cout << "Could not set position of node " << nodeId << std::endl;
	}

}

void eposdriver::nodeReadAngle(void *deviceHandle, unsigned int nodeId, int &encoderAngle)
{
	DWORD errCode = 0x00;

	if (!VCS_GetPositionIs(deviceHandle, nodeId, &encoderAngle, &errCode))
	{
		std::cout << "Could not fetch position of node " << nodeId << std::endl;
	}
}

void eposdriver::nodeStopMotion(void *deviceHandle, unsigned int nodeId)
{
	DWORD errCode = 0x00;

	if (!VCS_SetQuickStopState(deviceHandle, nodeId, &errCode))
	{
		std::cout << "Could not stop node " << nodeId << std::endl;
	}
}

}  // namespace threefinger_ctl
