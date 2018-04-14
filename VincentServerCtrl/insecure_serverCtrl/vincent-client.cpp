#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <arpa/inet.h>
#include "netconstants.h"
#include "constants.h"

// Socket
static int sockfd;

// Tells us that the network is running.
static volatile int networkActive=0;

char ch; 

void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			printf("Command / Status OK\n");
			break;

		case RESP_BAD_PACKET:
			printf("BAD MAGIC NUMBER FROM ARDUINO\n");
			break;

		case RESP_BAD_CHECKSUM:
			printf("BAD CHECKSUM FROM ARDUINO\n");
			break;

		case RESP_BAD_COMMAND:
			printf("PI SENT BAD COMMAND TO ARDUINO\n");
			break;

		case RESP_BAD_RESPONSE:
			printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
			break;

		default:
			printf("PI IS CONFUSED!\n");
	}
}

void handleStatus(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));

	printf("\n ------- VINCENT STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", data[0]);
	printf("Right Forward Ticks:\t\t%d\n", data[1]);
	printf("Left Reverse Ticks:\t\t%d\n", data[2]);
	printf("Right Reverse Ticks:\t\t%d\n", data[3]);
	printf("Left Forward Ticks Turns:\t%d\n", data[4]);
	printf("Right Forward Ticks Turns:\t%d\n", data[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", data[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", data[7]);
	printf("Forward Distance:\t\t%d\n", data[8]);
	printf("Reverse Distance:\t\t%d\n", data[9]);
	printf("\n---------------------------------------\n\n");
}

void handleMessage(const char *buffer)
{
	printf("MESSAGE FROM VINCENT: %s\n", &buffer[1]);
}

void handleCommand(const char *buffer)
{
	// We don't do anything because we issue commands
	// but we don't get them. Put this here
	// for future expansion
}

void handleNetwork(const char *buffer, int len)
{
	// The first byte is the packet type
	int type = buffer[0];

	switch(type)
	{
		case NET_ERROR_PACKET:
		handleError(buffer);
		break;

		case NET_STATUS_PACKET:
		handleStatus(buffer);
		break;

		case NET_MESSAGE_PACKET:
		handleMessage(buffer);
		break;

		case NET_COMMAND_PACKET:
		handleCommand(buffer);
		break;
	}
}

void sendData(const char *buffer, int len)
{
	int c;
	printf("\nSENDING %d BYTES DATA\n\n", len);
	if(networkActive)
	{
		c = write(sockfd, buffer, len);
		networkActive = (c > 0);
	}
}

void *receiveThread(void *p)
{
	char buffer[128];
	int len;

	while(networkActive)
	{
		len = read(sockfd, buffer, sizeof(buffer));
		networkActive = (len > 0);

		if(networkActive)
			handleNetwork(buffer, len);
	}

	printf("Exiting network listener thread\n");
	pthread_exit(NULL);
}

void connectToServer(const char *serverName, int portNum)
{
	struct sockaddr_in serv_addr;
	memset(&serv_addr, 0, sizeof(serv_addr));

	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if(sockfd<0)
	{
		perror("Cannot create socket: ");
		exit(-1);
	}

	char hostIP[32];
	struct hostent *he;

	he=gethostbyname(serverName);

	if(he==NULL)
	{
		herror("Unable to get IP address: ");
		exit(-1);
	}

	struct in_addr **addr_list = (struct in_addr **) he->h_addr_list;
	strncpy(hostIP, inet_ntoa(*addr_list[0]), sizeof(hostIP));
	printf("Host %s IP address %s\n", serverName, hostIP);

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(portNum);
	inet_pton(AF_INET, hostIP, &serv_addr.sin_addr);

	if(connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{
		perror("Error connecting: ");
		exit(-1);
	}

	networkActive=1;
	// Spawn receive thread
	pthread_t _recv;
	pthread_create(&_recv, NULL, receiveThread, NULL);

	void *result;
	pthread_join(_recv, &result);
	
	printf("Connection closed. Exiting\n");
	close(sockfd);
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(int32_t *params)
{
	//printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	//printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");

	if (ch == 'I' || ch == 'i') 
	{
		char dir; 
		scanf("%c %d", &dir, &params[1]); 
		params[0] = (int32_t) dir; 
	} 
	else scanf("%d %d", &params[0], &params[1]);
	flushInput();
}

void *kbThread(void *p)
{
	int quit=0;

	while(!quit)
	{
		printf("**Command_Manual**\n > F/B/R/L (for moving in four directions)\n  (dist/angle) (speed)\n > S = stop\n > M = mark\n > P = popFromStack\n > C = clearStats | G = getStats\n > I = InsertIntoStack\n   (direction) (distance/angle) [fixed speed]\n > q = exit (Ctrl+C before exiting server)\n");
		scanf("%c", &ch);

		// Purge extraneous characters from input stream
		flushInput();

		char buffer[10];
		int32_t params[2];

		buffer[0] = NET_COMMAND_PACKET;
		switch(ch)
		{
			case 'f':
			case 'F':
			case 'b':
			case 'B':
			case 'l':
			case 'L':
			case 'r':
			case 'R':
			case 'I':
			case 'i': 
				getParams(params);
				buffer[1] = ch;
				memcpy(&buffer[2], params, sizeof(params));
				sendData(buffer, sizeof(buffer));
				break;
			case 's':
			case 'S':
			case 'c':
			case 'C':
			case 'g':
			case 'G':
			case 'M': 
			case 'm': 
			case 'E': 
			case 'e': 
			case 'p': 
			case 'P': 
				params[0]=0;
				params[1]=0;
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = ch;
				sendData(buffer, sizeof(buffer));
				break;
			case 'q':
			case 'Q':
				quit=1;
				break;
			default:
				printf("BAD COMMAND\n");
		}
	}

	printf("Exiting keyboard thread\n");
	pthread_exit(NULL);
}

int main(int ac, char **av)
{
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

	// Spawn the keyboard thread
	pthread_t _kb;

	// Start the client
	pthread_create(&_kb, NULL, kbThread, NULL);
	connectToServer(av[1], atoi(av[2]));

	void *result;
	pthread_join(_kb, &result);

	printf("\nMAIN exiting\n\n");
}
