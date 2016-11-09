/*
 * Motor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */


#include "Motor.h"

#define TimeDelay 50000

extern sem_t	MotorTimerSem;
extern int		MotorActivated;

pthread_barrier_t 	MotorStartBarrier;


int gpio_set (int nr, int val)  {
	char cmdline[200];

	if (val < 0)
		sprintf(cmdline, "/usr/sbin/gpio %d -d i", nr);
	else if (val > 0)
		sprintf(cmdline, "/usr/sbin/gpio %d -d ho 1", nr);
	else
		sprintf(cmdline, "/usr/sbin/gpio %d -d ho 0", nr);

	return system(cmdline);
}


int motor_open(void) {
	struct termios config;
	int uart = open(MOTOR_UART, O_RDWR | O_NOCTTY | O_NDELAY);

	if (uart < 0) {
		printf("motor_open : impossible d'ouvrir le uart du moteur\n");
		return uart;
	}

	fcntl(uart, F_SETFL, 0); //read calls are non blocking

	//set port config
	tcgetattr(uart, &config);
	cfsetspeed(&config, B115200);
	config.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
	config.c_iflag = 0; //clear input config
	config.c_lflag = 0; //clear local config
	config.c_oflag &= ~OPOST; //clear output config (raw output)
	cfmakeraw(&config);
	tcsetattr(uart, TCSANOW, &config);
	return uart;
}

int motor_cmd(int file, uint8_t cmd, uint8_t *reply, int replylen) {
	int size;

	write(file, &cmd, 1);
	fsync(file);
	usleep(TimeDelay);
	size = read(file, reply, replylen);

	return size;
}

int MotorPortInit(MotorStruct *Motor) {
	uint8_t reply[256];
	int		i;

	//open motor port
	Motor->file = motor_open();
	if (Motor->file < 0) {
		printf("motor_open: Impossible d'ouvrir le UART\n");
		return Motor->file;
	}
	//reset IRQ flipflop - this code resets GPIO_ERROR_READ to 0
	gpio_set(GPIO_ERROR_RESET, 0);
	usleep(2*TimeDelay);

	//all select lines inactive
	gpio_set(GPIO_M1, 0);
	gpio_set(GPIO_M2, 0);
	gpio_set(GPIO_M3, 0);
	gpio_set(GPIO_M4, 0);
	usleep(2*TimeDelay);

	//configure motors
	for (i = 0; i < 4; ++i) {
		gpio_set(GPIO_M1 + i, -1);
		usleep(2*TimeDelay);
		motor_cmd(Motor->file, 0xE0, reply, 2);
		motor_cmd(Motor->file, 0x91, reply, 121);
		motor_cmd(Motor->file, 0xA1, reply, 2);
		motor_cmd(Motor->file, i + 1, reply, 1);
		motor_cmd(Motor->file, 0x40, reply, 2);
		gpio_set(GPIO_M1 + i ,0);
		usleep(2*TimeDelay);
	}

	//all select lines active
	gpio_set(GPIO_M1, -1);
	gpio_set(GPIO_M2, -1);
	gpio_set(GPIO_M3, -1);
	gpio_set(GPIO_M4, -1);
	usleep(2*TimeDelay);

	gpio_set(GPIO_ERROR_READ, -1);
	usleep(2*TimeDelay);

	return 0;
}


void motor_send(MotorStruct *Motor, int SendMode) {
/* Fonction utilitaire pour simplifier les transmissions aux moteurs */


	uint8_t cmdMotor[5]; //Trame 39 bits commande moteur
	uint16_t cmdLed; //Trame 16 bits Led

	switch (SendMode) {
	case MOTOR_NONE :
		break;

	case MOTOR_PWM_ONLY :	/* A faire! */
		//Voir guide Prog page 10
		pthread_spin_lock(&(Motor->MotorLock));
		cmdMotor[0] = 0x20 | (((Motor->pwm[0])&0x1ff)>>4);
		cmdMotor[1] = (((Motor->pwm[0])&0x1ff)<<4) | (((Motor->pwm[1])&0x1ff)>>5);
		cmdMotor[2] = (((Motor->pwm[1])&0x1ff)<<3) | (((Motor->pwm[2])&0x1ff)>>6);
		cmdMotor[3] = (((Motor->pwm[2])&0x1ff)<<2) | (((Motor->pwm[3])&0x1ff)>>7);
		cmdMotor[4] = (((Motor->pwm[3])&0x1ff)<<1);
		pthread_spin_unlock(&(Motor->MotorLock));
		write(Motor->file, cmdMotor, 5);

		break;
	case MOTOR_LED_ONLY :	/* A faire! */
							break;
	case MOTOR_PWM_LED :	/* A faire! */
							break;
	}




}


void *MotorTask ( void *ptr ) {
/* A faire! */
/* Tache qui transmet les nouvelles valeurs de vitesse */
/* à chaque moteur à interval régulier (5 ms).         */
	MotorStruct	*Motor = (MotorStruct *) ptr;

	printf("%s : Motor prêt à démarrer\n", __FUNCTION__);
	pthread_barrier_wait(&(MotorStartBarrier));
	printf("%s : Motor démarré \n", __FUNCTION__);

	while (MotorActivated) {

		sem_wait(&MotorTimerSem);
		printf("%s : Moteur period \n", __FUNCTION__);
		if (MotorActivated == 0){
				break;
		}
		//Envoie des vitesses aux moteurs
		printf("%s : Envoie des commandes aux moteurs \n", __FUNCTION__);
		motor_send(Motor, MOTOR_PWM_ONLY);
	}
	printf("%s : Motor arrêté \n", __FUNCTION__);

	pthread_exit(0); /* exit thread */
}


int MotorInit (MotorStruct *Motor) {
/* A faire! */
/* Ici, vous devriez faire l'initialisation des moteurs.   */
/* C'est-à-dire initialiser le Port des moteurs avec la    */
/* fonction MotorPortInit() et créer la Tâche MotorTask()  */
/* qui va s'occuper des mises à jours des moteurs en cours */ 
/* d'exécution.                                            */
	pthread_attr_t		attr;
	struct sched_param	param;
	int					minprio, maxprio;
	int					i;
    int 				retval;

	if(MotorPortInit(Motor)!=0){
		return -1;
	}

	pthread_barrier_init(&MotorStartBarrier, NULL, 2);


	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	minprio = sched_get_priority_min(POLICY);
	maxprio = sched_get_priority_max(POLICY);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = minprio + (maxprio - minprio)/6;
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);
	sem_init(&MotorTimerSem, 0, 0);

	//Init Motor Struct :
	Motor->pwm[0]=0x00;
	Motor->pwm[1]=0x00;
	Motor->pwm[2]=0x00;
	Motor->pwm[3]=0x00;
	Motor->led[0]=0x00;
	Motor->led[1]=0x00;
	Motor->led[2]=0x00;
	Motor->led[3]=0x00;



	retval = pthread_create(&(Motor->MotorThread), &attr, MotorTask, Motor);
	if (retval) {
			printf("pthread_create : Impossible de créer le thread MotorTask\n");
			return retval;
	}

	pthread_attr_destroy(&attr);

	return 0;
}



int MotorStart (void) {
/* A faire! */
/* Ici, vous devriez démarrer la mise à jour des moteurs (MotorTask).    */ 
/* Tout le système devrait être prêt à faire leur travail et il ne reste */
/* plus qu'à tout démarrer.                                              */
	MotorActivated = 1;
	pthread_barrier_wait(&(MotorStartBarrier));
	pthread_barrier_destroy(&MotorStartBarrier);
	printf("%s Moteur démarré\n", __FUNCTION__);
	return 0;
}



int MotorStop (MotorStruct *Motor) {
/* A faire! */
/* Ici, vous devriez arrêter les moteurs et fermer le Port des moteurs. */ 
	int i, err = 0;
	MotorActivated = 0;
	sem_post(&MotorTimerSem);

	err = pthread_join(Motor->MotorThread,NULL);
	if (err){
		printf("pthread_join(MotorTask) : Erreur\n");
	}
	close(Motor->file);

	return 0;
}
