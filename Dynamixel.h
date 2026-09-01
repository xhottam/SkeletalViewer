/*
 * SerialPort.cpp
 *
 *  Created on: Jun 20, 2011
 *      Author: jose
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

typedef unsigned char byte;

#include "SerialPort.h"

#include <fstream> 


struct vectorR {
	float v [3];
};

struct matriz {
	float v [3][3];
};

class Dynamixel {

	private:
		static const int BufferSize=1024;
		static const int waitTime=0;

		byte buffer[BufferSize];
		byte bufferIn[BufferSize];

		void cleanBuffers();
		void toHexHLConversion(short pos, byte *hexH, byte *hexL);
		short fromHexHLConversion(byte hexH, byte hexL);
		byte checkSumatory(byte  data[], int length);

		int getReadAX12LoadCommand(byte id);
		int getSetAX12PositionCommand(byte id, short goal);

		int getReadAX12PositionCommand(byte id);
		int getReadAX12TemperatureCommand(byte id);

		int getSetLedCommand(byte id, bool onOff);
		int getReadLedCommand(byte id);

		int getSetSyncWrite(int spa_angle, int sa_angle, int e_angle);
		int getSetSyncWriteDerecha(int spa_angle, int sa_angle, int e_angle);

		int getSetSyncWrite(byte spa2[], byte spa1[],byte sa2[],byte sa1[],byte e2[],byte e1[]);
		int getSetSyncWrite(byte spa2, byte spa1,byte sa2,byte sa1,byte e2,byte e1);

		int getSetAX12PositionCommandRegWrite(byte id, short goal);
	    int getSetAX12LedRegWrite(byte id,bool accion);
		
		int getSetAX12LedWrite(byte id,bool accion);
		int getAction();

		int getReadAX12ControlTable(byte id);
		int getReadPosition0x20(byte id);

	
		

	public:
		Dynamixel();
		~Dynamixel();
		
	


		int getPosition(SerialPort *serialPort, int idAX12);
		int setPosition(SerialPort *serialPort, int idAX12, int position);		
		int sentTossModeCommand(SerialPort *serialPort);
        
		int sentTossModeCommandTest(SerialPort *serialPort);

		int setPositionSyn(SerialPort *serialPort,int spa_angle, int sa_angle, int e_angle);
		int setPositionSynDerecha(SerialPort *serialPort,int R_spa_angle, int R_sa_angle, int R_e_angle);
		
		int setPositionSyn(SerialPort *serialPort,byte spa2[], byte spa1[],byte sa2[],byte sa1[],byte e2[],byte e1[]);
		int setPositionSyn(SerialPort *serialPort,byte spa2, byte spa1,byte sa2,byte sa1,byte e2,byte e1);

		int setRegWrite (SerialPort *serialPort, int idAX12, int position);
		int setLedRegWrite (SerialPort *serialPort, int idAX12,bool accion);

		int setLedWrite (SerialPort *serialPort, int idAX12,bool accion);

		int setAction(SerialPort *serialPort);

		int getPositionControlTable(SerialPort *serialPort, int idAX12);
		
		int getPosition0x20(SerialPort *serialPort, int idAX12);

		void bf (byte *buffer, int n);

		float modulo_vector(float x1, float y1, float z1);
		float dot(float x1, float y1, float z1,float x2, float y2, float z2 );
		vectorR cross (float x1, float y1, float z1,float x2, float y2, float z2 );	

		float cons_angle (float x1, float y1, float z1, vectorR v2);
};

#endif