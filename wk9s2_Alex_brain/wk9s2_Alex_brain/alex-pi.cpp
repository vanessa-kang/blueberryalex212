#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"
#include <ncurses.h>

#define PORT_NAME			"/dev/ttyACM0"
//define PORT_NAME			"/dev/ttyACM1"
#define BAUD_RATE			B9600

int exitFlag=0;
int recvOk = 0;
sem_t _xmitSema;

void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			printw("ERROR: Bad Magic Number\n");
			break;

		case PACKET_CHECKSUM_BAD:
			printw("ERROR: Bad checksum\n");
			break;

		default:
			printw("ERROR: UNKNOWN ERROR\n");
	}
}

void handleStatus(TPacket *packet)
{
	printw("\n ------- ALEX STATUS REPORT ------- \n\n");
	printw("Left Ticks:\t\t%d\n", packet->params[0]);
	printw("Right Ticks:\t\t%d\n", packet->params[1]);
	printw("\n---------------------------------------\n\n");
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			printw("Command OK\n");
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;
		
		case RESP_FINISH:
			printw("Command Finished\n");
			recvOk = 1;
		break;

		default:
			printw("Arduino is confused\n");
	}
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)
	{
		case RESP_BAD_PACKET:
			printw("Arduino received bad magic number\n");
		break;

		case RESP_BAD_CHECKSUM:
			printw("Arduino received bad checksum\n");
		break;

		case RESP_BAD_COMMAND:
			printw("Arduino received bad command\n");
		break;

		case RESP_BAD_RESPONSE:
			printw("Arduino received unexpected response\n");
		break;

		default:
			printw("Arduino reports a weird error\n");
	}
}

void handleMessage(TPacket *packet)
{
	printw("Message from Alex: %s\n", packet->data);
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
	}
}

void sendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
}

void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printw("PACKET ERROR\n");
					handleError(result);
				}
		}
	}
}

//void flushInput()
//{
	//char c;

	//while((c = getchar()) != '\n' && c != EOF);
//}

//void getParams(TPacket *commandPacket, char ch)
//{
	//if(ch == 'w' || ch == 's') {
		//commandPacket->params[0] = 5;
		//commandPacket->params[1] = 55;
	//}
	//else {
		//commandPacket->params[0] = 15;
		//commandPacket->params[1] = 50;
	//}
//}

void sendCommand(char command)
{
	TPacket commandPacket;

	commandPacket.packetType = PACKET_TYPE_COMMAND;

	switch(command)
	{
		case 'w':
		case 'W':
			printw("Forward\n");
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			printw("Reverse\n");
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;

		case 'a':
		case 'A':
			printw("Left\n");
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;

		case 'd':
		case 'D':
			printw("Right\n");
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;

		case 'x':
		case 'X':
			printw("Stop\n");
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			sendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			sendPacket(&commandPacket);
			break;

		case 'q':
		case 'Q':
			exitFlag=1;
			break;

		default:
			printw("Bad command\n");

	}
}

int main()
{
	char ch;
	
	//Ncurses initialization
	initscr();	//Initialize the structures required
	cbreak();	//No input buffering
	noecho();	//Does not echo the entered chracter
	scrollok(stdscr, TRUE);	//Automatically scrolls the window if exceed bottom
	
	// Connect to the Arduino
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

	// Sleep for two seconds
	printw("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
	sleep(2);
	printw("DONE\n");

	// Spawn receiver thread
	pthread_t recv;

	pthread_create(&recv, NULL, receiveThread, NULL);

	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);

	while(!exitFlag)
	{
		recvOk = 0;
		ch = getch();
		sendCommand(ch);
	}

	printw("Closing connection to Arduino.\n");
	endSerial();
	
	//Close Ncurses window, return to normal termianl output
	refresh();
	endwin();
}
