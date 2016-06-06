/*
 * MySCServo.cpp
 * Series Control Servo for SCS
 * Created on: 2015.6.12
 * Author: Tony tan
 */
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#define printf(args) (Serial2.write(args))
#define flush() (Serial2.flush())

#else
#include "WProgram.h"
#define printf(args) (Serial.print(args,uint8_t))
#define flush() (Serial.flush())
#endif

#include "MySCServo.h"

MySCServo::MySCServo()
{
	ReturnLevel = 2;
}

void MySCServo::InitForSC(void)
{
	CpuEnd = 1;
	AdcNegBit = 10;
	MoveVaddr = P_GOAL_TIME_L;
	//enableRX();
}

void MySCServo::InitForSM(void)
{
	CpuEnd = 0;
	AdcNegBit = 15;
	MoveVaddr = P_GOAL_SPEED_L;
}

void MySCServo::Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data)
{
	if(!CpuEnd){
		*DataL = (Data&0xff);
		*DataH = (Data>>8);
	}else{
		*DataL = (Data>>8);
		*DataH = (Data&0xff);			
	}
}

int MySCServo::SCS2Host(uint8_t DataL, uint8_t DataH)
{
	int Data;
	if(!CpuEnd){
		Data = DataH;
		Data<<=8;
		Data |= DataL;	
	}else{
		Data = DataL;
		Data<<=8;
		Data |= DataH;			
	}
	return Data;
}

int MySCServo::EnableTorque(uint8_t ID, uint8_t Enable)
{
	int messageLength = 4;

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_TORQUE_ENABLE);
	printf(Enable);
	printf((~(ID + messageLength + INST_WRITE + Enable + P_TORQUE_ENABLE))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}

int MySCServo::WritePos(uint8_t ID, int position, int velocity)
{
	int messageLength = 7;
	uint8_t posL,posH,velL,velH;
	Host2SCS(&posL, &posH, position);
	Host2SCS(&velL, &velH, velocity);

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_GOAL_POSITION_L);
	printf(posL);
	printf(posH);
	printf(velL);
	printf(velH);
	printf((~(ID + messageLength + INST_WRITE + P_GOAL_POSITION_L + posL + posH + velL + velH))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}

int MySCServo::RegWritePos(uint8_t ID, int position, int velocity)
{
	int messageLength = 7;
	uint8_t posL,posH,velL,velH;
	Host2SCS(&posL, &posH, position);
	Host2SCS(&velL, &velH, velocity);

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_REG_WRITE);
	printf(P_GOAL_POSITION_L);
	printf(posL);
	printf(posH);
	printf(velL);
	printf(velH);
	printf((~(ID + messageLength + INST_REG_WRITE + P_GOAL_POSITION_L + posL + posH + velL + velH))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}

void MySCServo::RegWriteAction()
{
	int messageLength = 2;
	uint8_t ID = 0xfe;

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_ACTION);
	printf((~(ID + messageLength + INST_ACTION))&0xFF);
	flush();
	enableRX();
}

int MySCServo::ReadBuf(uint8_t len, uint8_t *buf)
{
	if(buf) return Serial2.readBytes(buf,len);
	else
	{
		uint8_t buftmp[8];
		return Serial2.readBytes(buftmp,len);
	}
}

int MySCServo::ReadPos(uint8_t ID)
{	
	uint8_t buf[8];
	int size;
	int pos;
	memset(buf,0,sizeof(buf));

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(4);
	printf(INST_READ);
	printf(P_PRESENT_POSITION_L);
	printf(2);
	printf((~(ID + 4 + INST_READ + P_PRESENT_POSITION_L + 2))&0xFF);
	flush();
	enableRX();

	size = ReadBuf(8, buf);
	if(size<8)
		return -1;
	
	pos = SCS2Host(buf[5], buf[6]);
	return pos;
}

void MySCServo::SyncWritePos(uint8_t ID[], uint8_t IDN, int position[], int velocity)
{
	int messageLength = 5*IDN+4;
	uint8_t Sum = 0;
	uint8_t posL,posH,velL,velH;
	Host2SCS(&velL, &velH, velocity);

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(0xfe);
	printf(messageLength);
	printf(INST_SYNC_WRITE);
	printf(P_GOAL_POSITION_L);
	printf(4);
	
	Sum = 0xfe + messageLength + INST_SYNC_WRITE + P_GOAL_POSITION_L + 4;
	int i;
	for(i=0; i<IDN; i++){
		Host2SCS(&posL, &posH, position[i]);
		printf(ID[i]);
		printf(posL);
		printf(posH);
		printf(velL);
		printf(velH);
		Sum += ID[i] + posL + posH + velL + velH;
	}
	printf((~Sum)&0xFF);
	flush();
	enableRX();
}

int MySCServo::WriteID(uint8_t oldID, uint8_t newID)
{
	int messageLength = 4;

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(oldID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_ID);
	printf(newID);
	printf((~(oldID + messageLength + INST_WRITE + newID + P_ID))&0xFF);
	flush();
	enableRX();	

	if(oldID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}

int MySCServo::WriteLimitAngle(uint8_t ID, int MinAngel, int MaxAngle)
{
	int messageLength = 7;
	uint8_t MinAL,MinAH,MaxAL,MaxAH;
	Host2SCS(&MinAL, &MinAH, MinAngel);
	Host2SCS(&MaxAL, &MaxAH, MaxAngle);

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_MIN_ANGLE_LIMIT_L);
	printf(MinAL);
	printf(MinAH);
	printf(MaxAL);
	printf(MaxAH);
	printf((~(ID + messageLength + INST_WRITE + P_MIN_ANGLE_LIMIT_L + MinAL + MinAH + MaxAL + MaxAH))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}

int MySCServo::WriteLimitTroque(uint8_t ID, int MaxTroque)
{
	int messageLength = 5;
	uint8_t MaxTL,MaxTH;
	Host2SCS(&MaxTL, &MaxTH, MaxTroque);
	
	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_MAX_TORQUE_L);
	printf(MaxTL);
	printf(MaxTH);

	printf((~(ID + messageLength + INST_WRITE + P_MAX_TORQUE_L + MaxTL + MaxTH))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}

int MySCServo::WritePunch(uint8_t ID, int Punch)
{
	int messageLength = 5;
	uint8_t PunchL,PunchH;
	Host2SCS(&PunchL, &PunchH, Punch);
	
	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_PUNCH_L);
	printf(PunchL);
	printf(PunchH);

	printf((~(ID + messageLength + INST_WRITE + P_PUNCH_L + PunchL + PunchH))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}

int MySCServo::WriteBaund(uint8_t ID, uint8_t Baund)
{
	int messageLength = 4;

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_BAUD_RATE);
	printf(Baund);

	printf((~(ID + messageLength + INST_WRITE + P_BAUD_RATE + Baund))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}

int MySCServo::WriteDeadBand(uint8_t ID, uint8_t CWDB, uint8_t CCWDB)
{
	int messageLength = 5;

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_CW_DEAD);
	printf(CWDB);
	printf(CCWDB);

	printf((~(ID + messageLength + INST_WRITE + P_CW_DEAD + CWDB + CCWDB))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}

int MySCServo::WriteIMax(uint8_t ID, int IMax)
{
	int messageLength = 5;
	uint8_t IMaxL,IMaxH;
	Host2SCS(&IMaxL, &IMaxH, IMax);
	
	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_IMAX_L);
	printf(IMaxL);
	printf(IMaxH);
	printf((~(ID + messageLength + INST_WRITE + P_IMAX_L + IMaxL + IMaxH))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}


int MySCServo::LockEprom(uint8_t ID, uint8_t Enable)
{
	int messageLength = 4;

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_LOCK);
	printf(Enable);

	printf((~(ID + messageLength + INST_WRITE + P_LOCK + Enable))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}

int MySCServo::WritePID(uint8_t ID, uint8_t P, uint8_t I, uint8_t D)
{
	int messageLength = 6;

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(P_COMPLIANCE_P);
	printf(P);
	printf(D);
	printf(I);

	printf((~(ID + messageLength + INST_WRITE + P_COMPLIANCE_P + P + D + I))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}

int MySCServo::WriteSpe(uint8_t ID, int velocity)
{
	int messageLength = 5;	

	int vel = velocity;
	if(velocity<0){
		vel = -velocity;
		vel |= (1<<AdcNegBit);
	}

	uint8_t velL,velH;
	Host2SCS(&velL, &velH, vel);
	
	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(messageLength);
	printf(INST_WRITE);
	printf(MoveVaddr);
	printf(velL);
	printf(velH);
	printf((~(ID + messageLength + INST_WRITE + MoveVaddr + velL + velH))&0xFF);
	flush();
	enableRX();

	if(ID != 0xfe && ReturnLevel==2)
		return ReadBuf(6);
	return 0;
}


int MySCServo::ReadVoltage(uint8_t ID)
{	
	uint8_t buf[7];
	uint8_t size;
	int vol;
	memset(buf,0,sizeof(buf));

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(4);
	printf(INST_READ);
	printf(P_PRESENT_VOLTAGE);
	printf(1);
	printf((~(ID + 4 + INST_READ + P_PRESENT_VOLTAGE + 1))&0xFF);
	flush();
	enableRX();

	size = ReadBuf(7, buf);
	if(size<7)
		return -1;
	vol = buf[5];
	return vol;
}

int MySCServo::ReadTemper(uint8_t ID)
{
	uint8_t buf[7];
	uint8_t size;
	int temper;
	memset(buf,0,sizeof(buf));

	disableRX();
	printf(startByte);
	printf(startByte);
	printf(ID);
	printf(4);
	printf(INST_READ);
	printf(P_PRESENT_TEMPERATURE);
	printf(1);
	printf((~(ID + 4 + INST_READ + P_PRESENT_TEMPERATURE + 1))&0xFF);
	flush();
	enableRX();
	
	size = ReadBuf(7, buf);
	if(size<7)
		return -1;
	temper = buf[5];
	return temper;
}
