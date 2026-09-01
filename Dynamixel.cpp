/*
 * SerialPort.cpp
 *
 *  Created on: Jun 20, 2011
 *      Author: jose
 */

#include "stdafx.h"
#include "stdio.h"
#include "Dynamixel.h"





Dynamixel::Dynamixel()
{
	cleanBuffers();
}

Dynamixel::~Dynamixel()
{
	
}


void Dynamixel::cleanBuffers()
{
	memset(buffer,0,BufferSize);
	memset(bufferIn,0,BufferSize);
}


void Dynamixel::toHexHLConversion(short pos, byte *hexH, byte *hexL)
{    
    *hexH = (byte)(pos >> 8);
    *hexL = (byte)pos;
}

short Dynamixel::fromHexHLConversion(byte hexH, byte hexL)
{
    return (short)((hexL << 8) + hexH);
}

byte Dynamixel::checkSumatory(byte  data[], int length)
{
    int cs = 0;
    for (int i = 2; i < length; i++)
    {
        cs += data[i];
    }            
    return (byte)~cs;
}

int Dynamixel::getReadAX12LoadCommand(byte id)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 …PARAMETER N CHECK SUM
    int pos = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // length = 4
    buffer[pos++] = 4; //placeholder

    //the instruction, read => 2
    buffer[pos++] = 2;

    // pos registers 36 and 37
    buffer[pos++] = 40;

    //bytes to read
    buffer[pos++] = 2;

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    return pos;
}

int Dynamixel::getReadAX12PositionCommand(byte id)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 …PARAMETER N CHECK SUM
    int pos = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // length = 4
    buffer[pos++] = 4; //placeholder

    //the instruction, read => 2
    buffer[pos++] = 2;

    // pos registers 36 and 37
    buffer[pos++] = 36;

    //bytes to read
    buffer[pos++] = 2;

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    return pos;
}



int  Dynamixel::getSetSyncWrite(int spa_angle, int sa_angle, int e_angle)
{
	int pos = 0;
    byte numberOfParameters = 0;

	buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = 0XFE;

    // bodyLength
    buffer[pos++] = 0; //place holder

    //sync_wrete
    buffer[pos++] = 0X83;

	// goal registers 30 and 31
    buffer[pos++] = 0x1E;// 30

	// longuitud parametros por id
    buffer[pos++] = 0x04;// 30
	//buffer[pos++] = 0x01;// 30
	byte hexH = 0;
    byte hexL = 0;
    toHexHLConversion(spa_angle, &hexH, &hexL);

	// id
    buffer[pos++] = 0x00;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = hexL;
    numberOfParameters++;
    buffer[pos++] = hexH;
    numberOfParameters++;
	buffer[pos++] = 0X32;
    numberOfParameters++;
    buffer[pos++] = 0x03;
    numberOfParameters++;

	hexH = 0;
    hexL = 0;
    toHexHLConversion(sa_angle, &hexH, &hexL);
		// id
    buffer[pos++] = 0x01;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = hexL;
    numberOfParameters++;
    buffer[pos++] = hexH;
    numberOfParameters++;
	buffer[pos++] = 0X32;
    numberOfParameters++;
    buffer[pos++] = 0x03;
    numberOfParameters++;

	hexH = 0;
    hexL = 0;
    toHexHLConversion(e_angle, &hexH, &hexL);

		// id
    buffer[pos++] = 0x02;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = hexL;
    numberOfParameters++;
	buffer[pos++] = hexH;
    numberOfParameters++;
	buffer[pos++] = 0X32;
    numberOfParameters++;
    buffer[pos++] = 0x03;
    numberOfParameters++;

	//hexH = 0;
 //   hexL = 0;
 //   toHexHLConversion(R_spa_angle, &hexH, &hexL);

	//	// id
 //   buffer[pos++] = 0x03;
	////bytes to write
 //   //buffer[pos++] = 0xff;
	//buffer[pos++] = hexL;
 //   numberOfParameters++;
	//buffer[pos++] = hexH;
 //   numberOfParameters++;
	//buffer[pos++] = 0X32;
 //   numberOfParameters++;
 //   buffer[pos++] = 0x03;
 //   numberOfParameters++;

	//hexH = 0;
 //   hexL = 0;
 //   toHexHLConversion(R_sa_angle, &hexH, &hexL);

	//	// id
 //   buffer[pos++] = 0x04;
	////bytes to write
 //   //buffer[pos++] = 0xff;
	//buffer[pos++] = hexL;
 //   numberOfParameters++;
	//buffer[pos++] = hexH;
 //   numberOfParameters++;
	//buffer[pos++] = 0X32;
 //   numberOfParameters++;
 //   buffer[pos++] = 0x03;
 //   numberOfParameters++;

	//hexH = 0;
 //   hexL = 0;
 //   toHexHLConversion(R_e_angle, &hexH, &hexL);

	//	// id
 //   buffer[pos++] = 0x05;
	////bytes to write
 //   //buffer[pos++] = 0xff;
	//buffer[pos++] = hexL;
 //   numberOfParameters++;
	//buffer[pos++] = hexH;
 //   numberOfParameters++;
	//buffer[pos++] = 0X32;
 //   numberOfParameters++;
 //   buffer[pos++] = 0x03;
 //   numberOfParameters++;



	 // bodyLength
    buffer[3] = (byte)(19);
	byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

	return pos;
}
int  Dynamixel::getSetSyncWriteDerecha(int R_spa_angle, int R_sa_angle, int R_e_angle)
{
	int pos = 0;
    byte numberOfParameters = 0;

	buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = 0XFE;

    // bodyLength
    buffer[pos++] = 0; //place holder

    //sync_wrete
    buffer[pos++] = 0X83;

	// goal registers 30 and 31
    buffer[pos++] = 0x1E;// 30

	// longuitud parametros por id
    buffer[pos++] = 0x04;// 30
	//buffer[pos++] = 0x01;// 30
	byte hexH = 0;
    byte hexL = 0;
    toHexHLConversion(R_spa_angle, &hexH, &hexL);

	// id
    buffer[pos++] = 0x03;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = hexL;
    numberOfParameters++;
    buffer[pos++] = hexH;
    numberOfParameters++;
	buffer[pos++] = 0X32;
    numberOfParameters++;
    buffer[pos++] = 0x03;
    numberOfParameters++;

	hexH = 0;
    hexL = 0;
    toHexHLConversion(R_sa_angle, &hexH, &hexL);
		// id
    buffer[pos++] = 0x04;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = hexL;
    numberOfParameters++;
    buffer[pos++] = hexH;
    numberOfParameters++;
	buffer[pos++] = 0X32;
    numberOfParameters++;
    buffer[pos++] = 0x03;
    numberOfParameters++;

	hexH = 0;
    hexL = 0;
    toHexHLConversion(R_e_angle, &hexH, &hexL);

		// id
    buffer[pos++] = 0x05;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = hexL;
    numberOfParameters++;
	buffer[pos++] = hexH;
    numberOfParameters++;
	buffer[pos++] = 0X32;
    numberOfParameters++;
    buffer[pos++] = 0x03;
    numberOfParameters++;

	//hexH = 0;
 //   hexL = 0;
 //   toHexHLConversion(R_spa_angle, &hexH, &hexL);

	//	// id
 //   buffer[pos++] = 0x03;
	////bytes to write
 //   //buffer[pos++] = 0xff;
	//buffer[pos++] = hexL;
 //   numberOfParameters++;
	//buffer[pos++] = hexH;
 //   numberOfParameters++;
	//buffer[pos++] = 0X32;
 //   numberOfParameters++;
 //   buffer[pos++] = 0x03;
 //   numberOfParameters++;

	//hexH = 0;
 //   hexL = 0;
 //   toHexHLConversion(R_sa_angle, &hexH, &hexL);

	//	// id
 //   buffer[pos++] = 0x04;
	////bytes to write
 //   //buffer[pos++] = 0xff;
	//buffer[pos++] = hexL;
 //   numberOfParameters++;
	//buffer[pos++] = hexH;
 //   numberOfParameters++;
	//buffer[pos++] = 0X32;
 //   numberOfParameters++;
 //   buffer[pos++] = 0x03;
 //   numberOfParameters++;

	//hexH = 0;
 //   hexL = 0;
 //   toHexHLConversion(R_e_angle, &hexH, &hexL);

	//	// id
 //   buffer[pos++] = 0x05;
	////bytes to write
 //   //buffer[pos++] = 0xff;
	//buffer[pos++] = hexL;
 //   numberOfParameters++;
	//buffer[pos++] = hexH;
 //   numberOfParameters++;
	//buffer[pos++] = 0X32;
 //   numberOfParameters++;
 //   buffer[pos++] = 0x03;
 //   numberOfParameters++;



	 // bodyLength
    buffer[3] = (byte)(19);
	byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

	return pos;
}
int  Dynamixel::getSetSyncWrite(byte spa2[], byte spa1[],byte sa2[],byte sa1[],byte e2[],byte e1[])
{
	int pos = 0;
    byte numberOfParameters = 0;

	buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = 0XFE;

    // bodyLength
    buffer[pos++] = 0; //place holder

    //sync_wrete
    buffer[pos++] = 0X83;

	// goal registers 30 and 31
    buffer[pos++] = 0x1E;// 30

	// longuitud parametros por id
    buffer[pos++] = 0x04;// 30
	//buffer[pos++] = 0x01;// 30

	// id
    buffer[pos++] = 0x00;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = 0x00;
    numberOfParameters++;
    buffer[pos++] = 0x00;
    numberOfParameters++;
	buffer[pos++] = 0X00;
    numberOfParameters++;
    buffer[pos++] = 0x08;
    numberOfParameters++;

		// id
    buffer[pos++] = 0x01;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = 0x00;
    numberOfParameters++;
    buffer[pos++] = 0x00;
    numberOfParameters++;
	buffer[pos++] = 0X00;
    numberOfParameters++;
    buffer[pos++] = 0x08;
    numberOfParameters++;

		// id
    buffer[pos++] = 0x02;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = 0x00;
    numberOfParameters++;
	buffer[pos++] = 0x03;
    numberOfParameters++;
	buffer[pos++] = 0X00;
    numberOfParameters++;
    buffer[pos++] = 0x08;
    numberOfParameters++;



	 // bodyLength
    buffer[3] = (byte)(19);
	byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

	return pos;
}
int  Dynamixel::getSetSyncWrite(byte spa2, byte spa1,byte sa2,byte sa1,byte e2,byte e1)
{
	int pos = 0;
    byte numberOfParameters = 0;

	buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = 0XFE;

    // bodyLength
    buffer[pos++] = 0; //place holder

    //sync_wrete
    buffer[pos++] = 0X83;

	// goal registers 30 and 31
    buffer[pos++] = 0x1E;// 30

	// longuitud parametros por id
    buffer[pos++] = 0x04;// 30
	//buffer[pos++] = 0x01;// 30

	// id
    buffer[pos++] = 0x00;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = spa2;
    numberOfParameters++;
    buffer[pos++] = spa1;
    numberOfParameters++;
	buffer[pos++] = 0X00;
    numberOfParameters++;
    buffer[pos++] = 0x08;
    numberOfParameters++;

		// id
    buffer[pos++] = 0x01;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = sa2;
    numberOfParameters++;
    buffer[pos++] = sa1;
    numberOfParameters++;
	buffer[pos++] = 0X00;
    numberOfParameters++;
    buffer[pos++] = 0x08;
    numberOfParameters++;

		// id
    buffer[pos++] = 0x02;
	//bytes to write
    //buffer[pos++] = 0xff;
	buffer[pos++] = e2;
    numberOfParameters++;
	buffer[pos++] = e1;
    numberOfParameters++;
	buffer[pos++] = 0X00;
    numberOfParameters++;
    buffer[pos++] = 0x08;
    numberOfParameters++;



	 // bodyLength
    buffer[3] = (byte)(19);
	byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

	return pos;
}
int Dynamixel::getSetAX12PositionCommand(byte id, short goal)
{
    int pos = 0;
    byte numberOfParameters = 0;
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 …PARAMETER N CHECK SUM

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // bodyLength
    buffer[pos++] = 0; //place holder

    //the instruction, query => 3
    buffer[pos++] = 3;

    // goal registers 30 and 31
    buffer[pos++] = 0x1E;// 30;

    //bytes to write
    byte hexH = 0;
    byte hexL = 0;
    toHexHLConversion(goal, &hexH, &hexL);
    buffer[pos++] = hexL;
    numberOfParameters++;
    buffer[pos++] = hexH;
    numberOfParameters++;

    // bodyLength
    buffer[3] = (byte)(numberOfParameters + 3);

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    return pos;
}
int Dynamixel::getAction()
{
	 int pos = 0;
    byte numberOfParameters = 0;
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 …PARAMETER N CHECK SUM

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = 0XFE;

    // bodyLength
    buffer[pos++] = 2; //place holder

	//the instruction, 
    buffer[pos++] = 5;

	byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    return pos;
}
int Dynamixel::getSetAX12PositionCommandRegWrite(byte id, short goal)
{
    int pos = 0;
    byte numberOfParameters = 0;
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 …PARAMETER N CHECK SUM

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // bodyLength
    buffer[pos++] = 0; //place holder

    //the instruction, query => 3
    buffer[pos++] = 4;

    // goal registers 30 and 31
    buffer[pos++] = 0x1E;// 30;

    //bytes to write
    byte hexH = 0;
    byte hexL = 0;
    toHexHLConversion(goal, &hexH, &hexL);
    buffer[pos++] = hexL;
    numberOfParameters++;
    buffer[pos++] = hexH;
    numberOfParameters++;

    // bodyLength
    buffer[3] = (byte)(numberOfParameters + 3);

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    return pos;
}

int Dynamixel::getReadAX12TemperatureCommand(byte id)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 …PARAMETER N CHECK SUM
    int pos = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // length = 4
    buffer[pos++] = 4;

    //the instruction, read => 2
    buffer[pos++] = 2;

    // pos registers 36 and 37
    buffer[pos++] = 0x2b;

    //bytes to read
    buffer[pos++] = 1;

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    return pos;
}

int Dynamixel::getSetLedCommand(byte id, bool onOff)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 …PARAMETER N CHECK SUM
    int pos = 0;
    byte numberOfParameters = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // length = (Numbers of parameters) + 3
    buffer[pos++] = 0; //placeholder

    //the instruction, send => 3
    buffer[pos++] = 3;

    // led register
    buffer[pos++] = 25;

    byte ledOnOff = 0;
    if (onOff)
        ledOnOff = 1;

    byte hexHPos=0, hexLPos=0;
    toHexHLConversion(ledOnOff, &hexHPos, &hexLPos);
    buffer[pos++] = hexLPos;
    numberOfParameters++;
    buffer[pos++] = hexHPos;
    numberOfParameters++;
//            buffer[pos++] = torqueOnOff;
    buffer[3] = (byte)(numberOfParameters + 3);
    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    //string command = Util.byteArrayTostring(buffer, pos);
    return pos;
}

int Dynamixel::getReadLedCommand(byte id)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 …PARAMETER N CHECK SUM
    int pos = 0;
    byte numberOfParameters = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

	// length
    buffer[pos++] = 3;

    //the instruction, read => 2
    buffer[pos++] = 2;

    // led register
    buffer[pos++] = 25;

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    //string command = Util.byteArrayTostring(buffer, pos);
    return pos;
}

int Dynamixel::getPosition(SerialPort *serialPort, int idAX12) 
{
	int ret=0;

	int n=getReadAX12PositionCommand(idAX12);
	long l=serialPort->sendArray(buffer,n);
	Sleep(waitTime);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 8);

	short pos = -1;
	if (n>7)
	{
		pos = fromHexHLConversion(bufferIn[5], bufferIn[6]);				
	}

	printf("\nid=<%i> pos=<%i> length=<%i>\n", idAX12, pos, n);
	if (pos<0 || pos > 1023)
		ret=-2;
	else
		ret=pos;

	return ret;
}
int Dynamixel::getReadAX12ControlTable(byte id)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 …PARAMETER N CHECK SUM
    int pos = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // length = 4
    buffer[pos++] = 2; //placeholder

    //the instruction, read => 2
    buffer[pos++] = 2;

    // pos registers 36 and 37
    buffer[pos++] = 0x00;

    //bytes to read
    buffer[pos++] = 49;

    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    return pos;
}
int Dynamixel::getPositionControlTable(SerialPort *serialPort, int idAX12) 
{
	int ret=0;

	int n=getReadAX12ControlTable(idAX12);
	long l=serialPort->sendArray(buffer,n);
	Sleep(waitTime);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 49);

	short pos = -1;
	if (n>5)
	{
		//pos = fromHexHLConversion(bufferIn[5], bufferIn[6]);
		pos = (short)(bufferIn[4]);
	}

	printf("\nid=<%i> pos=<%i> length=<%i>\n", idAX12, pos, n);
	if (pos<0 || pos > 1023)
		ret=-2;
	else
		ret=pos;

	return ret;
}

int Dynamixel::setPosition(SerialPort *serialPort, int idAX12, int position) 
{
	int error=0;

	int n=getSetAX12PositionCommand(idAX12, position);
	long l=serialPort->sendArray(buffer,n);
	Sleep(waitTime);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 8);

	if (n>4 && bufferIn[4] == 0)
		printf("\nid=<%i> set at pos=<%i>\n", idAX12, position);
	else {
		error=-1;
		printf("\nid=<%i> error: <%i>\n", idAX12, bufferIn[4]);
		bf(bufferIn, n);
	}

	return error;
}

int Dynamixel::setAction(SerialPort *serialPort)
{

		int error=0;

	int n=getAction();
	long l=serialPort->sendArray(buffer,n);
	Sleep(waitTime);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 8);

	//if (n>4 && bufferIn[4] == 0)
	//	printf("\nid=<%i> set at pos=<%i>\n");
	//else {
	//	error=-1;
	//	printf("\nid=<%i> error: <%i>\n");
	//	bf(bufferIn, n);
	//}

	return error;

}

int Dynamixel::setRegWrite(SerialPort *serialPort, int idAX12, int position) 
{
	int error=0;

	int n=getSetAX12PositionCommandRegWrite(idAX12, position);
	long l=serialPort->sendArray(buffer,n);
	Sleep(waitTime);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 8);

	if (n>4 && bufferIn[4] == 0)
		printf("\nid=<%i> set at pos=<%i>\n", idAX12, position);
	else {
		error=-1;
		printf("\nid=<%i> error: <%i>\n", idAX12, bufferIn[4]);
		bf(bufferIn, n);
	}

	return error;
}

int Dynamixel::getSetAX12LedRegWrite(byte id, bool accion)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 …PARAMETER N CHECK SUM
    int pos = 0;
    byte numberOfParameters = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // length = (Numbers of parameters) + 3
    buffer[pos++] = 0; //placeholder

    //the instruction, send => 3
    buffer[pos++] = 4;

    // led register
    buffer[pos++] = 25;

    byte ledOnOff = 0;
    if (accion)
        ledOnOff = 1;

    byte hexHPos=0, hexLPos=0;
    toHexHLConversion(ledOnOff, &hexHPos, &hexLPos);
    buffer[pos++] = hexLPos;
    numberOfParameters++;
    buffer[pos++] = hexHPos;
    numberOfParameters++;
	//buffer[pos++] = ledOnOff;
 //   numberOfParameters++;

//            buffer[pos++] = torqueOnOff;
    buffer[3] = (byte)(numberOfParameters + 3);
    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    //string command = Util.byteArrayTostring(buffer, pos);
    return pos;
}

int Dynamixel::setLedRegWrite(SerialPort *serialPort, int idAX12,bool accion) 
{
	int error=0;

	int n=getSetAX12LedRegWrite(idAX12,accion);
	long l=serialPort->sendArray(buffer,n);
	Sleep(waitTime);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 8);

	if (n>4 && bufferIn[4] == 0)
		printf("\nid=<%i> set at pos=<%i>\n", idAX12);
	else {
		error=-1;
		printf("\nid=<%i> error: <%i>\n", idAX12, bufferIn[4]);
		bf(bufferIn, n);
	}

	return error;
}

int Dynamixel::getSetAX12LedWrite(byte id, bool accion)
{
    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 …PARAMETER N CHECK SUM
    int pos = 0;
    byte numberOfParameters = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = id;

    // length = (Numbers of parameters) + 3
    buffer[pos++] = 0; //placeholder

    //the instruction, send => 3
    buffer[pos++] = 3;

    // led register
    buffer[pos++] = 25;

    byte ledOnOff = 0;
    if (accion)
        ledOnOff = 1;

    byte hexHPos=0, hexLPos=0;
    toHexHLConversion(ledOnOff, &hexHPos, &hexLPos);
    buffer[pos++] = hexLPos;
    numberOfParameters++;
    buffer[pos++] = hexHPos;
    numberOfParameters++;
	/*buffer[pos++] = ledOnOff;
    numberOfParameters++;*/
//            buffer[pos++] = torqueOnOff;
    buffer[3] = (byte)(numberOfParameters + 3);
    byte checksum = checkSumatory(buffer, pos);
    buffer[pos++] = checksum;

    //string command = Util.byteArrayTostring(buffer, pos);
    return pos;
}

int Dynamixel::setLedWrite(SerialPort *serialPort, int idAX12,bool accion) 
{
	int error=0;

	int n=getSetAX12LedRegWrite(idAX12,accion);
	long l=serialPort->sendArray(buffer,n);
	Sleep(waitTime);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 8);

	if (n>4 && bufferIn[4] == 0)
		printf("\nid=<%i> set at pos=<%i>\n", idAX12);
	else {
		error=-1;
		printf("\nid=<%i> error: <%i>\n", idAX12, bufferIn[4]);
		bf(bufferIn, n);
	}

	return error;
}

int Dynamixel::setPositionSyn(SerialPort *serialPort,int spa_angle, int sa_angle, int e_angle) 
{
	int error=0;

	//int n=getSetAX12PositionCommand(idAX12, position);
	int n = getSetSyncWrite(spa_angle,sa_angle,e_angle);
	long l=serialPort->sendArray(buffer,n);
	Sleep(waitTime);

	memset(bufferIn,0,BufferSize);
		  std::ofstream fs("prueba.txt",std::ios::app);
	                fs << "conecto" << std::endl;
					fs.close();
	//n=serialPort->getArray(bufferIn, 8);

	//if (n>4 && bufferIn[4] == 0)
	//	//printf("\nid=<%i> set at pos=<%i>\n", idAX12, position);
	//	printf("\nid=<%i> set at pos=<%i>\n");
	//else {
	//	error=-1;
	//	//printf("\nid=<%i> error: <%i>\n", idAX12, bufferIn[4]);
	//	printf("\nid=<%i> error: <%i>\n");
	//	bf(bufferIn, n);
	//}
	
	return error;
}
int Dynamixel::setPositionSynDerecha(SerialPort *serialPort,int R_spa_angle, int R_sa_angle, int R_e_angle) 
{
	int error=0;

	//int n=getSetAX12PositionCommand(idAX12, position);
	int n = getSetSyncWriteDerecha(R_spa_angle,R_sa_angle,R_e_angle);
	long l=serialPort->sendArray(buffer,n);
	Sleep(waitTime);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 8);

	//if (n>4 && bufferIn[4] == 0)
	//	//printf("\nid=<%i> set at pos=<%i>\n", idAX12, position);
	//	printf("\nid=<%i> set at pos=<%i>\n");
	//else {
	//	error=-1;
	//	//printf("\nid=<%i> error: <%i>\n", idAX12, bufferIn[4]);
	//	printf("\nid=<%i> error: <%i>\n");
	//	bf(bufferIn, n);
	//}

	return error;
}

int Dynamixel::setPositionSyn(SerialPort *serialPort,byte spa2[], byte spa1[],byte sa2[],byte sa1[],byte e2[],byte e1[]) 
{
	int error=0;

	//int n=getSetAX12PositionCommand(idAX12, position);
	int n = getSetSyncWrite(spa2,spa1,sa2,sa1,e2,e1);
	long l=serialPort->sendArray(buffer,n);
	Sleep(waitTime);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 8);

	//if (n>4 && bufferIn[4] == 0)
	//	//printf("\nid=<%i> set at pos=<%i>\n", idAX12, position);
	//	printf("\nid=<%i> set at pos=<%i>\n");
	//else {
	//	error=-1;
	//	//printf("\nid=<%i> error: <%i>\n", idAX12, bufferIn[4]);
	//	printf("\nid=<%i> error: <%i>\n");
	//	bf(bufferIn, n);
	//}

	return error;
}
int Dynamixel::setPositionSyn(SerialPort *serialPort,byte spa2, byte spa1,byte sa2,byte sa1,byte e2,byte e1) 
{
	int error=0;

	//int n=getSetAX12PositionCommand(idAX12, position);
	int n = getSetSyncWrite(spa2,spa1,sa2,sa1,e2,e1);
	long l=serialPort->sendArray(buffer,n);
	Sleep(waitTime);

	memset(bufferIn,0,BufferSize);
	n=serialPort->getArray(bufferIn, 8);

	//if (n>4 && bufferIn[4] == 0)
	//	//printf("\nid=<%i> set at pos=<%i>\n", idAX12, position);
	//	printf("\nid=<%i> set at pos=<%i>\n");
	//else {
	//	error=-1;
	//	//printf("\nid=<%i> error: <%i>\n", idAX12, bufferIn[4]);
	//	printf("\nid=<%i> error: <%i>\n");
	//	bf(bufferIn, n);
	//}

	return error;
}


int Dynamixel::sentTossModeCommand(SerialPort *serialPort)
{
	byte tossModeCommandBuffer[15];
	tossModeCommandBuffer[0]='t';
	tossModeCommandBuffer[1]='\n';
	tossModeCommandBuffer[2]=0;

	int n=serialPort->sendArray(tossModeCommandBuffer, 2);
	Sleep(200);	

	serialPort->clear();
	Sleep(5);

	return n;
}
int Dynamixel::sentTossModeCommandTest(SerialPort *serialPort)
{
	byte tossModeCommandBuffer[4];
	tossModeCommandBuffer[0]='1';
	tossModeCommandBuffer[1]=' ';
	tossModeCommandBuffer[2]= 100;

	int n=serialPort->sendArray(tossModeCommandBuffer, 3);
	Sleep(200);	

	serialPort->clear();
	Sleep(5);

	return n;
}

void Dynamixel::bf (byte *buffer, int n)
{
	printf ("Response (length <%i>)\n",n);
	for (int i=0;i<n;i++)
	{
		//printf("%i [%c]", buffer[i], buffer[i]);
		printf("%i", buffer[i]);
		if (i<(n-1))
		{
			printf(",", buffer[i]);
		}
	}
	printf("\n");
}
