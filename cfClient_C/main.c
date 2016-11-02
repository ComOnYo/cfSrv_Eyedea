#include <stdio.h>
#include <unistd.h>
#include "cfctrl.h"
#include <pthread.h>

typedef struct CtrlData{
	unsigned short thrust;
	float pitch;
}CtrlData;

CtrlData ctrldata;

void thread();

int main()
{
	int sel = 0;
	int count = 0;
	//unsigned short thrust = 0;
	unsigned short thrust_step = 100;

	pthread_t id;
	while(1){
		printf("input :");
		scanf("%d", &sel);
		switch(sel){
			case 0:
				connDrone(55);
				break;
				//	setControl(71,0.0,0.0,0.0,20000.0);
			case 1:
				connDrone(72);
				break;
			case 2:
				disconnDrone(71);
				break;
			case 3:
				disconnDrone(72);
				break;
			case 4:
				setLog(55);

				pthread_create(&id, NULL, (void *)thread, &ctrldata);
				while(1)
				{
					printf("thrust input : ");
					scanf("%hu", &(ctrldata.thrust));
					printf("pitch input : ");
					scanf("%f", &(ctrldata.pitch));
					//printf("%hu", thrust);
					//thrust+=thrust_step;
					//if(thrust>=30000 || thrust <= 20000)
					//	thrust_step*=-1;
					//usleep(10000);
				}

			default:
				printf("fault\n");
				/*
				while(1)
				{
					setControl(71,0.0,0.0,0.0, thrust);
				}
				*/
		}
	}
	return 0;
}

void thread(struct CtrlData* ctrldata)
{
	while(1)
	{
		setControl(55,0.0,ctrldata->pitch,0.0, ctrldata->thrust);
		usleep(10000);
		printf("%f %hu\n", ctrldata->pitch, ctrldata->thrust);
	}
}
