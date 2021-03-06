/*
 ============================================================================
 Name        : ARDrone.c
 Author      : Bruno De Kelper
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <semaphore.h>
#include <math.h>
#include <poll.h>
#include <string.h>

#include "Sensor.h"
#include "Motor.h"
#include "Control.h"
#include "Mavlink.h"

#define PERIODE_uS 	5000
#define PERIODE_nS 	5000000L

#define MAX_PERIOD	1000000000L
#define MAIN_PERIOD	20


SensorRawData	RawData[NUM_SENSOR][DATABUFSIZE];
SensorData	 	NavData[NUM_SENSOR][DATABUFSIZE];
AttitudeData	AttitudeDesire, AttitudeMesure;

MotorStruct		Motor;
MavlinkStruct	Mavlink;
SensorParam	 	ParamData[NUM_SENSOR]   = { ACCEL_PARAM, GYRO_PARAM, SONAR_PARAM, BAROM_PARAM, MAGNETO_PARAM };
SensorStruct	SensorTab[NUM_SENSOR]   = { ACCEL_INIT, GYRO_INIT, SONAR_INIT, BAROM_INIT, MAGNETO_INIT };
AttitudeStruct	AttitudeTab[NUM_SENSOR] = { ACCEL_ATT_INIT, GYRO_ATT_INIT, SONAR_ATT_INIT, BAROM_ATT_INIT, MAGNETO_ATT_INIT };
ControlStruct	Control = CONTROL_INIT;


sem_t 	MotorTimerSem;
int		MotorActivated = 0;
sem_t	MavlinkReceiveTimerSem;
sem_t	MavlinkStatusTimerSem;
int		MavlinkActivated = 0;
sem_t 	ControlTimerSem;
int		ControlActivated = 0;

sem_t 	MainTimerSem;


struct itimerspec	NewTimer, OldTimer;
timer_t				TimerID;

struct sigaction  TimerSig, old_TimerSig;           /* definition of signal action */

void SigTimerHandler (int signo) {
	static uint32_t  Period = 0;

	if (MavlinkActivated) {
		if ((Period % MAVLINK_RECEIVE_PERIOD) == 0)
			sem_post(&MavlinkReceiveTimerSem);
		if ((Period % MAVLINK_STATUS_PERIOD) == 0)
			sem_post(&MavlinkStatusTimerSem);
	}
	if(MotorActivated){
		if ((Period % MOTOR_PERIOD) == 0){
			sem_post(&MotorTimerSem);
		}
	}
	if(ControlActivated){
			if ((Period % CONTROL_PERIOD) == 0){
				sem_post(&ControlTimerSem);
			}
		}
	if ((Period % MAIN_PERIOD) == 0){
		sem_post (&MainTimerSem);
	}
	Period = (Period + 1) % MAX_PERIOD;
}


int StartTimer (void) {
	struct sigevent	 Sig;
	int				 retval = 0;

	memset (&TimerSig, 0, sizeof(struct sigaction));
	TimerSig.sa_handler = SigTimerHandler;
	if ((retval = sigaction(SIGRTMIN, &TimerSig, &old_TimerSig)) != 0) {
		printf("%s : Problème avec sigaction : retval = %d\n", __FUNCTION__, retval);
		return retval;
	}
	Sig.sigev_signo  = SIGRTMIN;
	Sig.sigev_notify = SIGEV_SIGNAL;
	timer_create(CLOCK_MONOTONIC, &Sig, &TimerID);
	NewTimer.it_value.tv_sec     = PERIODE_nS / 1000000000L;
	NewTimer.it_value.tv_nsec	 = PERIODE_nS % 1000000000L;
	NewTimer.it_interval.tv_sec  = PERIODE_nS / 1000000000L;
	NewTimer.it_interval.tv_nsec = PERIODE_nS % 1000000000L;
	timer_settime(TimerID, 0, &NewTimer, &OldTimer);

	return retval;
}


int StopTimer (void) {
	int	 retval = 0;

	timer_settime(TimerID, 0, &OldTimer, NULL);
	timer_delete(TimerID);
	sigaction(SIGRTMIN, &old_TimerSig, NULL);

	return retval;
}


#include <ctype.h>    /* For tolower() function */
int getchar_nonblock(void) {
	struct termios oldt, newt;
	int    ch=-1;

	tcgetattr( STDIN_FILENO, &oldt );
	memcpy ((void *) &newt, (void *) &oldt, sizeof(struct termios));
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK | O_NDELAY);
	ch = getchar();
   	fcntl(STDIN_FILENO, F_SETFL, 0);
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

	return ch;
}




/*  MAIN
 *
 *  Init des spinlock et semaphores globaux
 *  Start des differentes tâches
 *  Start timer
 *  Boucle while :  touche 'q' = arrête le programme
 *
 */
int main(int argc, char *argv[]) {
	struct sched_param	param;
	int		minprio, maxprio;
	char	IPAddress[20] = {TARGET_IP};
	int		i, j, retval = 0;
	int		ch = 0;

	printf("Usage :\n");
	printf("	DroneFirmware <Option>\n");
	printf("	   Option :	Pas d'option -> Par défaut (Pas de Log, IP = 192.168.1.2)\n");
	printf("	   		    LogAccel     -> Log de l'Accéléromètre\n");
	printf("	   		    LogGyro      -> Log du Gyroscope\n");
	printf("	   		    LogMagneto   -> Log du Magnétomètre\n");
	printf("	   		    LogSonar     -> Log du Sonar\n");
	printf("	   		    LogBarom     -> Log du Baromètre\n");
	printf("	   		    LogAll       -> Log de tous les capteurs\n");
	printf("	   		    IP=<Adresse>  (ex. : IP=192.168.1.3)\n");
	printf("	Note : Les options peuvent être cumulées et dans n'importe quel ordre.\n");
	printf("		   La syntaxe des options doit être strictement respectée.\n");

	if (argc > 1) {
		for (i = 1; i < argc; i++) {
			if (strcmp(argv[i],"LogAccel") == 0)
				SensorTab[ACCELEROMETRE].DoLog = 1;
			if (strcmp(argv[i],"LogGyro") == 0)
				SensorTab[GYROSCOPE].DoLog = 1;
			if (strcmp(argv[i],"LogSonar") == 0)
				SensorTab[SONAR].DoLog = 1;
			if (strcmp(argv[i],"LogBarom") == 0)
				SensorTab[BAROMETRE].DoLog = 1;
			if (strcmp(argv[i],"LogMagneto") == 0)
				SensorTab[MAGNETOMETRE].DoLog = 1;
			if (strcmp(argv[i],"LogAll") == 0)
				for (j = ACCELEROMETRE; j <= MAGNETOMETRE; j++)
					SensorTab[j].DoLog = 1;
			if (strncmp ( argv[i], "IP=", 3) == 0)
				strcpy( IPAddress, &(argv[i][3]));
		}
	}
	printf("%s : IP = %s\n", __FUNCTION__, IPAddress);
	printf("%s ça démarre !!!\n", __FUNCTION__);

	param.sched_priority = sched_get_priority_min(POLICY);
    pthread_setschedparam(pthread_self(), POLICY, &param);

	sem_init(&MainTimerSem, 0, 0);
	sem_init(&ControlTimerSem, 0, 0);

	if ((retval = pthread_spin_init(&(AttitudeDesire.AttitudeLock), 1)) < 0) {
		printf("%s : Impossible d'initialiser le spinlock (AttitudeDesiree.AttitudeLock): retval = %d\n", __FUNCTION__, retval);
		return -1; /* exit thread */
	}
	if ((retval = pthread_spin_init(&(AttitudeMesure.AttitudeLock), 1)) < 0) {
		printf("%s : Impossible d'initialiser le spinlock (AttitudeDesiree.AttitudeLock): retval = %d\n", __FUNCTION__, retval);
		return -1; /* exit thread */
	}
	if ((retval = pthread_spin_init(&(Motor.MotorLock), 1)) < 0) {
		printf("%s : Impossible d'initialiser le spinlock (Motor.MotorLock): retval = %d\n", __FUNCTION__, retval);
				return -1; /* exit thread */
	}
	for(i = 0; i < NUM_SENSOR; i++){
		if ((retval = pthread_spin_init(&(SensorTab[i].DataLock), 1)) < 0){
			printf("%s : Impossible d'initialiser le spinlock (SensorTab[%d].Datalock): retval = %d\n", __FUNCTION__,i, retval);
							return -1; /* exit thread */
		}
	}

	if ((retval = MotorInit(&Motor)) < 0)
		return EXIT_FAILURE;
	if ((retval = SensorsLogsInit(SensorTab)) < 0)
		return EXIT_FAILURE;
	if ((retval = SensorsInit(SensorTab)) < 0)
		return EXIT_FAILURE;
	if ((retval = AttitudeInit(AttitudeTab)) < 0)
		return EXIT_FAILURE;
	if ((retval = MavlinkInit(&Mavlink, &AttitudeDesire, &AttitudeMesure, IPAddress)) < 0)
		return EXIT_FAILURE;
	if ((retval = ControlInit(&Control)) < 0)
		return EXIT_FAILURE;

	printf("%s Tout initialisé\n", __FUNCTION__);

	StartTimer();

	MotorStart();
	SensorsStart();
	AttitudeStart();

	SensorsLogsStart();

	MavlinkStart();
	ControlStart();

	printf("%s Tout démarré\n", __FUNCTION__);

	ch = 0;
	while (ch != 'q') {
		sem_wait(&MainTimerSem);

		//printf("%s : Main period \n", __FUNCTION__);
		ch = tolower(getchar_nonblock());

		if (ch =='z'){ //Speed Up
			if(Motor.pwm[0]<250){
				Motor.pwm[0]+= 0x05;
				Motor.pwm[1]+= 0x05;
				Motor.pwm[2]+= 0x05;
				Motor.pwm[3]+= 0x05;
			}

		}
		else if (ch =='s'){ //Speed Down
			if(Motor.pwm[0]>0){
				Motor.pwm[0]-=0x05;
				Motor.pwm[1]-=0x05;
				Motor.pwm[2]-=0x05;
				Motor.pwm[3]-=0x05;
			}
		}
		else if(ch == 'g'){
			Motor.led[0] = MOTOR_LEDGREEN;
			Motor.led[1] = MOTOR_LEDGREEN;
			Motor.led[2] = MOTOR_LEDGREEN;
			Motor.led[3] = MOTOR_LEDGREEN;
		}
		else if(ch == 'o'){
			Motor.led[0] = MOTOR_LEDORANGE;
			Motor.led[1] = MOTOR_LEDORANGE;
			Motor.led[2] = MOTOR_LEDORANGE;
			Motor.led[3] = MOTOR_LEDORANGE;
		}
		else if(ch == 'r'){
			Motor.led[0] = MOTOR_LEDRED;
			Motor.led[1] = MOTOR_LEDRED;
			Motor.led[2] = MOTOR_LEDRED;
			Motor.led[3] = MOTOR_LEDRED;
		}
		else if(ch == 'l'){
			Motor.led[0] = MOTOR_LEDOFF;
			Motor.led[1] = MOTOR_LEDOFF;
			Motor.led[2] = MOTOR_LEDOFF;
			Motor.led[3] = MOTOR_LEDOFF;
		}
	}

	MavlinkStop(&Mavlink);
	pthread_spin_destroy(&(AttitudeDesire.AttitudeLock));
	pthread_spin_destroy(&(AttitudeMesure.AttitudeLock));

	ControlStop(&Control);

	MotorStop(&Motor);
	pthread_spin_destroy(&(Motor.MotorLock));
	SensorsLogsStop(SensorTab);
	SensorsStop(SensorTab);
	for(i = 0; i < NUM_SENSOR; i++){
		pthread_spin_destroy(&(SensorTab[i].DataLock));
	}
	AttitudeStop(AttitudeTab);

	StopTimer();

	printf("%s Tout arrêté\n", __FUNCTION__);

	sem_destroy(&MainTimerSem);
	sem_destroy(&MotorTimerSem);
	sem_destroy(&ControlTimerSem);

	return EXIT_SUCCESS;
}
