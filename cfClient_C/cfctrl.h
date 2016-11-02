#include <stdio.h>
#include <string.h>
#include <zmq.h>
#include <assert.h>
#include <stdlib.h>

#define DRONE_CNT 10

static char recv[7000];

int channelSeq[DRONE_CNT];
void *cmdSocket[DRONE_CNT];
void *ctrlSocket[DRONE_CNT];

int connCnt = 0;

int findChanidx(int channel)
{
	int i = 0;
	for(i = 0; i<DRONE_CNT; i++){
		if(channelSeq[i] == channel)
			return i;
	}
	return -1;
}

int seqEmptyChk()
{
	int i = 0;
	for(i = 0;i<DRONE_CNT;i++){
		if(channelSeq[i] == 0)
			return i;
	}
}

int responseChk(char *source)
{
	int i = 0;
	char *str = "status";
	char ans;
	printf("%s\n", source);
	while(1)
	{
		if(source[i] == str[0] && source[i+1] == str[1] && source[i+2] == str[2]
				&& source[i+3] == str[3] && source[i+4] == str[4] && source[i+5]
				== str[5])
		{
			ans = source[i+8];
			break;
		}
		i++;
	}
	if(ans == '0')
		return 1;
	else
		return -1;
}

void connDrone(int channel)
{
	if(connCnt >= DRONE_CNT){
		printf("Full connection!!!\n");
		return;
	}
	else if(findChanidx(channel) != -1)
	{
		printf("Channel : %d exist Link!!!\n", channel);
		return;
	}

	char json[100];

	sprintf(json, "{ \"version\" : 1, \"cmd\" : \"connect\", \"uri\" : \"radio://0/%d/250K\"}", channel);

	//Check empty space of channelSeq array
	int idx = seqEmptyChk();
	printf("idx = %d\n", idx);

	void *context = zmq_ctx_new();
	cmdSocket[idx] = zmq_socket(context, ZMQ_REQ);

	char addr[20];
	sprintf(addr, "tcp://127.0.0.1:20%d0", idx);

	int rc = zmq_connect(cmdSocket[idx], addr);
	assert(rc == 0);

	zmq_send(cmdSocket[idx], json, strlen(json), 0);
	zmq_recv(cmdSocket[idx], recv, 7000, 0);

	if(responseChk(recv)==-1){
		printf("Connect Fail!!!\n");
		return;
	}
	else
		printf("Channel : %d success connection\n", channel);

	//void *context2 = zmq_ctx_new();
	ctrlSocket[idx] = zmq_socket(context, ZMQ_PUSH);
	sprintf(addr, "tcp://127.0.0.1:20%d4", idx);
	rc = zmq_connect(ctrlSocket[idx], addr);
	assert(rc == 0);
	printf("Channel : %d ctrlSocket connect complete\n", channel);

	channelSeq[idx] = channel;
	connCnt++;
}

void disconnDrone(int channel){
	int idx = 0;
	if((idx = findChanidx(channel)) == -1){
		printf("channel : %d not exist Link!!!\n", channel);
		return;
	}
	char json[50];
	char recv[50];

	sprintf(json, "{ \"version\" : 1, \"cmd\" : \"disconnect\", \"uri\" : \"radio://0/%d/250K\"}", channel);

	zmq_send(cmdSocket[idx], json, strlen(json), 0);
	zmq_recv(cmdSocket[idx], recv, 50, 0);

	if(responseChk(recv)==-1){
		printf("Connect Fail!!!\n");
		return;
	}
	else
		printf("Channel : %d Disconnect complete.\n", channel);

	zmq_close(cmdSocket[idx]);
	cmdSocket[idx] = NULL;
	zmq_close(ctrlSocket[idx]);
	ctrlSocket[idx] = NULL;
	channelSeq[idx] = 0;

	connCnt--;
}

void setControl(int channel, float roll, float pitch, float yaw, unsigned short thrust)
{
	int idx = 0;

	if((idx = findChanidx(channel)) == -1)
	{
		printf("Channel : %d Drone not connect\n", channel);
		return;
	}

	char ctrl_Json[200]; 
	sprintf(ctrl_Json, "{ \"version\" : 1, \"roll\" : %f, \"pitch\" : %f, \"yaw\" : %f, \"thrust\" : %hu }", roll, pitch, yaw, thrust);

	zmq_send(ctrlSocket[idx], ctrl_Json, strlen(ctrl_Json), 0);

}

void setLog(int channel)
{
	int idx = 0;
	if((idx = findChanidx(channel)) == -1)
	{
		printf("Channel : %d Drone not connect\n", channel);
		return;
	}

	char recv[50] = {0,};
	char recv2[50] = {0,};
	char *log_create = "{ \"version\" : 1, \"cmd\" : \"log\", \"action\" : \"create\", \"name\" : \"Test log block\", \"period\" : 30, \"variables\" : [ \"stabilizer.roll\", \"stabilizer.pitch\", \"stabilizer.yaw\", \"stabilizer.thrust\"] }";

	zmq_send(cmdSocket[idx], log_create, strlen(log_create), 0);
	zmq_recv(cmdSocket[idx], recv, 50, 0);

	if(responseChk(recv)==-1){
		printf("Connect Fail!!!\n");
		return;
	}
	else
		printf("log create ! %s\n", recv);

	char *log_start = "{ \"version\" : 1, \"cmd\" : \"log\", \"action\" : \"start\", \"name\" : \"Test log block\" }";

	printf("start logging\n");
	zmq_send(cmdSocket[idx], log_start, strlen(log_start), 0);
	zmq_recv(cmdSocket[idx], recv2, 50, 0);

	if(responseChk(recv2)==-1){
		printf("Connect Fail!!!\n");
		return;
	}
	else
		printf("log start!\n");
}
