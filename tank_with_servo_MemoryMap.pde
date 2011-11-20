// Andy MemoryMapped protocol version
//
// Program coded by Yasuhiro ISHII
//
// The Memory-map protocol is originally by JS Robotics.
//
// 2011/11/16 : Version 0.01	First release

#include <MsTimer2.h>
#include <Servo.h>
#include "MemoryMap.h"

#define  ANDY_SHIELD	1		// Andy Shield rev.1


void jobMotorDrive(unsigned char ,unsigned char ,unsigned char );
void jobNop(unsigned char ,unsigned char addr,unsigned char data);
void controlMotorPower(int motortype,int motoraction,int power);

#ifdef ANDY_SHIELD
#define PIN_MOTOR_L_VREF      6
#define PIN_MOTOR_R_VREF      5
#define PIN_MOTOR_L_CONTROL1  A0
#define PIN_MOTOR_L_CONTROL2  A1
#define PIN_MOTOR_R_CONTROL1  A3
#define PIN_MOTOR_R_CONTROL2  A2
#else
#define PIN_MOTOR_R_VREF      5
#define PIN_MOTOR_L_VREF      6
#define PIN_MOTOR_R_CONTROL1  8
#define PIN_MOTOR_R_CONTROL2  9
#define PIN_MOTOR_L_CONTROL1  10
#define PIN_MOTOR_L_CONTROL2  11
#endif

Job JobTable[] = {
    jobNop,
    jobMotorDrive
};

Var dummyBuffer;
ReceivedMessage packet;

Command MemoryMapTable[] = {
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x00
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x01
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x02
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x03
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x04
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x05
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x06
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x07
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x08
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x09
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x0a
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x0b
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x0c
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x0d
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x0e
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x0f
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x10
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x11
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x12
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x13
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x14
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x15
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x16
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x17
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x18
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x19
    { OPERATION_NOP					, (void*)&dummyBuffer	}, // 0x1a
    { OPERATION_WRITE | OPERATION_JOB			, (void*)JOBID_MOTOR	}, // 0x1b
    { OPERATION_WRITE | OPERATION_JOB			, (void*)JOBID_MOTOR	}, // 0x1c
};

#define MOTORTYPE_MOTOR_R    0
#define MOTORTYPE_MOTOR_L    1

#define MOTORACTION_MOTOR_STOP		0
#define MOTORACTION_MOTOR_FORWARD	1
#define MOTORACTION_MOTOR_REVERSE	2
#define MOTORACTION_MOTOR_BRAKE		3

#define SERIAL_COMMUNICATION_BAUD_RATE		9600

int processWriteCommand(ReceivedMessage );
int writeMemory(unsigned char address,unsigned char value);

////////////////////////////////////////////////////////////////////////////////
// register operation jobs
////////////////////////////////////////////////////////////////////////////////

void jobMotorDrive(unsigned char rwop,unsigned char addr,unsigned char data)
{
    // debug_printf("%s Address = %Xh,data = %Xh\n",__func__,addr,data);

    // for Intel x86 bitorder
    struct S_MOTOR_BIT_FIELD {
	unsigned char power :6;
	unsigned char cont :2;
    };

    //#define MOTORCONT_FREE	0x00
    //#define MOTORCONT_CW	0x01
    //#define MOTORCONT_CCW	0x02
    //#define MOTORCONT_BRAKE	0x03

    unsigned char cont,power;

    cont  = ((struct S_MOTOR_BIT_FIELD*)&data)->cont;
    power = ((struct S_MOTOR_BIT_FIELD*)&data)->power;

    controlMotorPower(addr == MEMMAP_ADR_MMPK_Motor_0 ? MOTORTYPE_MOTOR_R : MOTORTYPE_MOTOR_L,cont,power << 2);
}

void jobNop(unsigned char rwop,unsigned char addr,unsigned char data)
{
    // nothing to do for NOP
    // debug_printf("%s Address = %Xh,data = %Xh\n",__func__,addr,data);
}

int checkPacketHeader(ReceivedMessage msg)
{
    int ret = false;

    if((msg.Field.Header[0] == MEMMAP_HEADER1) &&
       (msg.Field.Header[1] == MEMMAP_HEADER2) &&
       (msg.Field.Length <= MEMMAP_MAX_DATA_LENGTH) &&
       (msg.Field.Cmd >= 1) &&
       (msg.Field.Cmd <= 4)){
	ret = true;
    }

    return ret;
}

int checkPacketSum(ReceivedMessage msg)
{
    int ret = false;
    int len;
    unsigned char sum;
    int i;

    len  = msg.Field.Length;
    sum  = 0;
    sum  = msg.Field.Cmd;
    sum ^= msg.Field.Address;
    sum ^= msg.Field.Length;

    for(i=0;i<len;i++){
	sum ^= msg.Field.data[i];
    }

    if(sum == msg.Field.Sum){
	ret = true;
    }

    return(ret);
}

int processCommand(ReceivedMessage msg)
{
    int ret = false;

    switch(msg.Field.Cmd){
    case MEMMAP_CMD_READ:
	break;
    case MEMMAP_CMD_RETURN:
	break;
    case MEMMAP_CMD_WRITE:
	processWriteCommand(msg);
	break;
    case MEMMAP_CMD_ACK:
	break;
    default:
	break;
    }

    return(ret);
}

int processWriteCommand(ReceivedMessage msg)
{
    int counter;
    unsigned char address;
    unsigned char value;

    // debug_printf("msg.Field.address = %02Xh\n",msg.Field.Address);

    for(counter=0;counter<msg.Field.Length;counter++){
	address = msg.Field.Address + counter;
	value = msg.Field.data[counter];
	// debug_printf("Address %02Xh <= %02Xh\n",address,value);
	writeMemory(address,value);
    }

    return true;
}

int writeMemory(unsigned char address,unsigned char value)
{
    // debug_printf("%s(%02Xh,%02Xh)\n",__func__,address,value);
    unsigned char RWOP;
    int ret = false;

    RWOP = MemoryMapTable[address].RWOP;

    // debug_printf("RWOP = %02Xh\n",RWOP);
    if(RWOP & OPERATION_WRITE){
	if(RWOP & OPERATION_JOB){

	    (*(JobTable[*((int*)&(MemoryMapTable[address].ptr))]))(OPERATION_WRITE,address,value);

	} else {

	    // debug_printf(" +++ ??? \n");

	}
    }
    return(true);
}

int readMemory(unsigned char address,unsigned char* value)
{

}

void setup()
{
    /////////////////////////////////////////////////
    // GPIO SETUP
    /////////////////////////////////////////////////

    // hardware bug fix
//#if ANDY_SHIELD==1
    // only rev.1 has the mis-port assignment
    pinMode(4,INPUT);
    digitalWrite(4,LOW);
//#endif

    // pin direction setup
    pinMode(PIN_MOTOR_R_VREF,OUTPUT);
    pinMode(PIN_MOTOR_L_VREF,OUTPUT);
    pinMode(PIN_MOTOR_R_CONTROL1,OUTPUT);
    pinMode(PIN_MOTOR_R_CONTROL2,OUTPUT);
    pinMode(PIN_MOTOR_L_CONTROL1,OUTPUT);
    pinMode(PIN_MOTOR_L_CONTROL2,OUTPUT);

    // pin output value setup
    digitalWrite(PIN_MOTOR_R_CONTROL1,LOW);
    digitalWrite(PIN_MOTOR_R_CONTROL2,HIGH);
    digitalWrite(PIN_MOTOR_L_CONTROL1,LOW);
    digitalWrite(PIN_MOTOR_L_CONTROL2,HIGH);
    analogWrite(PIN_MOTOR_R_VREF,100);
    analogWrite(PIN_MOTOR_L_VREF,100);

    /////////////////////////////////////////////////
    // MISC SETUP
    /////////////////////////////////////////////////

    // initialize peripherals
    analogWrite(PIN_MOTOR_R_VREF,0);
    analogWrite(PIN_MOTOR_L_VREF,0);

    // initialize servo motor

    // start UART
    Serial.begin(SERIAL_COMMUNICATION_BAUD_RATE);
    Serial.flush();
}

void loop()
{
    static SerialReceiveState serial_receive_state = SRECV_IDLE;
    static unsigned long startReceiveTime = 0;
    
    int counter;

#if 0
    if(DETECT_TIMEOUT(startReceiveTime) == true){
	serial_receive_state = SRECV_IDLE;
    }
#endif
    
    switch(serial_receive_state){
    case SRECV_IDLE:
	// debug_printf("SRECV_IDLE\n");
	if(Serial.available() != 0){
	    serial_receive_state = SRECV_RECEIVING_HEADER;
	    startReceiveTime = millis();
	}
	break;
	
    case SRECV_RECEIVING_HEADER:
	// debug_printf("SRECV_RECEIVING_HEADER\n");

	if(Serial.available() >= 5){
	    for(counter=0;counter<5;counter++){
		packet.Raw[counter] = Serial.read();
	    }
	    if(checkPacketHeader(packet) == true){
		serial_receive_state = SRECV_RECEIVING;
	    } else {
		// data that received might be broken -> flush
		Serial.flush();
		serial_receive_state = SRECV_IDLE;
	    }
	}
	break;
	
    case SRECV_RECEIVING:
	// debug_printf("SRECV_RECEIVING\n");
	
	if(Serial.available() >= (packet.Field.Length+1)){
	    for(counter=0;counter<(packet.Field.Length+1);counter++){
		if(counter != (packet.Field.Length)) {
		    packet.Field.data[counter] = Serial.read();
		} else {
		    packet.Field.Sum = Serial.read();
		}
	    }
	    if(checkPacketSum(packet) == true){
		// debug_printf("Sum check => OK\n");
		processCommand(packet);
	    } else {
		// data that received might be broke -> flush
		Serial.flush();
	    }
	    serial_receive_state = SRECV_IDLE;
	}
	
    default:
	break;
    }
}

void controlMotorPower(int motortype,int motoraction,int power)
{
    int pinno_1;
    int pinno_2;
    int pinno_pwm;

    const unsigned char cont[4][2] = {
	{ LOW	,	LOW	},	// STOP
	{ LOW	,	HIGH	},	// REVERSE
	{ HIGH	,	LOW	},	// FORWARD
	{ HIGH	,	HIGH	}	// BRAKE
    };
    
    if(motortype == MOTORTYPE_MOTOR_R){
	pinno_1 = PIN_MOTOR_R_CONTROL1;
	pinno_2 = PIN_MOTOR_R_CONTROL2;
	pinno_pwm = PIN_MOTOR_R_VREF;
    } else {
	pinno_1 = PIN_MOTOR_L_CONTROL1;
	pinno_2 = PIN_MOTOR_L_CONTROL2;
	pinno_pwm = PIN_MOTOR_L_VREF;
    }
    
    digitalWrite(pinno_1,cont[motoraction][0]);
    digitalWrite(pinno_2,cont[motoraction][1]);
    analogWrite(pinno_pwm,power);
    delay(10);
}

