/*
 * Sensor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#include "Sensor.h"

#define ABS(x) (((x) < 0.0) ? -(x) : (x))


#define MAX_TOT_SAMPLE 1000


extern SensorStruct	SensorTab[NUM_SENSOR];

pthread_barrier_t   SensorStartBarrier;
pthread_barrier_t   LogStartBarrier;
pthread_mutex_t 	  Log_Mutex;

uint8_t  SensorsActivated 	= 0;
uint8_t  LogActivated  	  	= 0;
uint8_t  numLogOutput 	  	= 0;

/*  SensorTask
 *  One task for each sensor : ACCELEROMETRE, GYROSCOPE, SONAR, BAROMETRE, MAGNETOMETRE
 */
void *SensorTask ( void *ptr ) {

	int i=0;
	int flagGyroBiais =0, flagGyroCount=0;
	double gyroBetaTmp[3] = {0,0,0};
	SensorStruct	*Sensor = (SensorStruct *) ptr;
	SensorRawData   sensorRawSample; //sensor actual raw sample received
	SensorRawData   sensorOldRawSample; //sensor precedent sample to calculate timeDelay
	SensorData		sensorSample; //sensor data and timedelay after conversion

	sensorSample.TimeDelay = 0;
	sensorRawSample.ech_num = 0;
	sensorRawSample.timestamp_n = 0;
	sensorRawSample.timestamp_s = 0;
	sensorOldRawSample.ech_num = DATABUFSIZE;
	sensorOldRawSample.timestamp_n = 0;
	sensorOldRawSample.timestamp_s = 0;



	printf("%s : %s prêt à démarrer\n", __FUNCTION__, Sensor->Name);
	pthread_barrier_wait(&SensorStartBarrier);
	printf("%s : %s Démarrer\n", __FUNCTION__, Sensor->Name);

	while (SensorsActivated) {

		//Read, Dort jusqu'à la réception d'un échantillon, donnee brute placee dans sensorRawSample
		if (read(Sensor->File, &sensorRawSample, sizeof(SensorRawData)) == sizeof(SensorRawData)) {

			if(sensorRawSample.status==NEW_SAMPLE){
				//Mise à jour de l'index
				Sensor->DataIdx++;
				if(Sensor->DataIdx>=DATABUFSIZE){
					Sensor->DataIdx=0;
				}

				//Correction de la donnée, decalage de centerVal + multiplication par le facteur de conversion
				for(i=0;i<3;i++){
					sensorRawSample.data[i] -= Sensor->Param->centerVal;
					sensorSample.Data[i]=sensorRawSample.data[i]*(Sensor->Param->Conversion);

					//Calibration d'erreur (Gain, orthogonalité des axes + biais)
					sensorSample.Data[i] = sensorSample.Data[i]*Sensor->Param->alpha[i][0] + sensorSample.Data[i]*Sensor->Param->alpha[i][1] + sensorSample.Data[i]*Sensor->Param->alpha[i][2] + Sensor->Param->beta[i];

					//Détection d'erreur (Bornes statiques)

				}

				//Mesure du biais du gyroscope que 100 echantillons au démarrage de la tâche :
				if (Sensor->type==GYROSCOPE && flagGyroBiais==0){
					if(flagGyroCount<150){
						if(flagGyroCount>0){ //pas de moyenne avec 0 au premier echantillon
							gyroBetaTmp[0] = (gyroBetaTmp[0] + sensorSample.Data[0])/2;
							gyroBetaTmp[1] = (gyroBetaTmp[1] + sensorSample.Data[1])/2;
							gyroBetaTmp[2] = (gyroBetaTmp[2] + sensorSample.Data[2])/2;
						}
						else{
							gyroBetaTmp[0] = sensorSample.Data[0];
							gyroBetaTmp[1] = sensorSample.Data[1];
							gyroBetaTmp[2] = sensorSample.Data[2];
						}
						flagGyroCount ++;
						printf("Gyroscope mesure du biais a l'init : Beta[0]=%d  Beta[1]=%d  Beta[2]=%d",gyroBetaTmp[0],gyroBetaTmp[1],gyroBetaTmp[2])
					}
					else{ //count > 150 on enregistre la moyenne
						Sensor->Param->beta[0]=gyroBetaTmp[0];
						Sensor->Param->beta[1]=gyroBetaTmp[1];
						Sensor->Param->beta[2]=gyroBetaTmp[2];
						flagGyroBiais =1;
					}
				}

				// DEBUG AFFICHAGE SENSOR
				//if(sensorRawSample.type==MAGNETOMETRE){ //donnees brutes magnetometre
				//	printf("[  Valeur Raw Magnetometre : EchNum = %d  DATA 0 = %d || DATA 1 = %d || DATA 2 = %d  ]",sensorRawSample.ech_num,sensorRawSample.data[0],sensorRawSample.data[1],sensorRawSample.data[2]);
				//}

				if(sensorRawSample.ech_num!=sensorOldRawSample.ech_num){ //echantillon différente de l'ancien stocké en local

					//  calculer le délais par rapport au OldRawSample (echantillon brute précédent)
					if(sensorOldRawSample.timestamp_s < sensorRawSample.timestamp_s){
						sensorSample.TimeDelay = (sensorRawSample.timestamp_s - sensorOldRawSample.timestamp_s)*(uint32_t)1000000000;
						if(sensorOldRawSample.timestamp_n < sensorRawSample.timestamp_n){
							sensorSample.TimeDelay += sensorRawSample.timestamp_n - sensorOldRawSample.timestamp_n;
						}
						else{
							sensorSample.TimeDelay += sensorOldRawSample.timestamp_n - sensorRawSample.timestamp_n;
						}
					}
					else{
						if(sensorOldRawSample.timestamp_n < sensorRawSample.timestamp_n){
							sensorSample.TimeDelay = sensorRawSample.timestamp_n - sensorOldRawSample.timestamp_n;
						}
					}




					//Placer rawData et Data dans la structure globale SensorStruct
					pthread_spin_lock(&Sensor->DataLock);
					memcpy((void *) &(Sensor->Data[Sensor->DataIdx]),(void *) &(sensorSample),sizeof(SensorData));
					memcpy((void *) &(Sensor->RawData[Sensor->DataIdx]),(void *) &(sensorRawSample),sizeof(SensorRawData));
					pthread_spin_unlock(&Sensor->DataLock);

					//Sauvegarde de l'enchantillon pour la prochaine comparaison (TimeDelay)
					memcpy((void *) &(sensorOldRawSample),(void *) &(sensorRawSample),sizeof(SensorRawData));

					// - Avertir qu'un nouvel échantillon est arrivé (Broadcast)
					pthread_mutex_lock(&(Sensor->DataSampleMutex));
					pthread_cond_broadcast(&(Sensor->DataNewSampleCondVar));
					pthread_mutex_unlock(&(Sensor->DataSampleMutex));
				}

			}
		} else {
			//La structure n'a pas été copiée en entier
			printf("La structure n'a pas été copiée en entier\n");
		}
	}
	printf("%s : %s Arrêté \n", __FUNCTION__,Sensor->Name);
	pthread_exit(0); /* exit thread */
}



/*  SensorsInit
 *  Init all sensors : ACCELEROMETRE, GYROSCOPE, SONAR, BAROMETRE, MAGNETOMETRE
 */
int SensorsInit (SensorStruct SensorTab[NUM_SENSOR]) {

	pthread_attr_t		attr;
	struct sched_param	param;
	int					minprio, maxprio;
	int					i;

	pthread_barrier_init(&SensorStartBarrier, NULL, NUM_SENSOR+1);

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	minprio = sched_get_priority_min(POLICY);
	maxprio = sched_get_priority_max(POLICY);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = minprio + (maxprio - minprio)/2;
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	for (i = 0; i < NUM_SENSOR ; i++){
		pthread_mutex_init(&(SensorTab[i].DataSampleMutex),NULL);
		pthread_cond_init(&(SensorTab[i].DataNewSampleCondVar),NULL);
	}

	//Open sensor device virtual files
	for (i = 0; i < NUM_SENSOR; i++) {
		SensorTab[i].File = open(SensorTab[i].DevName, O_RDONLY);
		if(SensorTab[i].File < 0 ){
			printf("%s : Error, %s file not open\n", __FUNCTION__, SensorTab[i].DevName);
			return SensorTab[i].File;
		}
	}

	//Create a task for each sensor
	for (i = 0; i < NUM_SENSOR; i++) {
		pthread_create(&(SensorTab[i].SensorThread), &attr, SensorTask, (void *) &(SensorTab[i]));
	}

	pthread_attr_destroy(&attr);

	return 0;
};

/*  SensorsStart
 *
 *  Place SensorsActivated à 1 -> Permet le démarrage des tâche sensor
 */
int SensorsStart (void) {

	SensorsActivated = 1;
	pthread_barrier_wait(&(SensorStartBarrier));
	pthread_barrier_destroy(&SensorStartBarrier);
	printf("%s Sensor démarré\n", __FUNCTION__);

	return 0;
}

/*  SensorsStop
 *
 *  Place SensorsActivated à 0 -> Arrêt des tâches sensor
 *  Fermeture des fichiers virtuels des sensors
 */
int SensorsStop (SensorStruct SensorTab[NUM_SENSOR]) {

	int i, retval;

	SensorsActivated = 0;

	//Close sensor device virtual files
	for (i = 0; i < NUM_SENSOR; i++) {
		retval = close(SensorTab[i].File);
		if(retval){
			printf("%s : Error, %s File not closed \n", __FUNCTION__, SensorTab[i].DevName);
			return retval;
		}
		pthread_join(SensorTab[i].SensorThread, NULL); //Wait the end of the sensor task
		pthread_cond_destroy(&(SensorTab[i].DataNewSampleCondVar));
		pthread_mutex_destroy(&(SensorTab[i].DataSampleMutex));
	}
	return 0;
}



/* Le code ci-dessous est un CADEAU !!!	*/
/* Ce code permet d'afficher dans la console les valeurs reçues des capteurs.               */
/* Évidemment, celà suppose que les acquisitions sur les capteurs fonctionnent correctement. */
/* Donc, dans un premier temps, ce code peut vous servir d'exemple ou de source d'idées.     */
/* Et dans un deuxième temps, peut vous servir pour valider ou vérifier vos acquisitions.    */
/*                                                                                           */
/* NOTE : Ce code suppose que les échantillons des capteurs sont placés dans un tampon       */
/*        circulaire de taille DATABUFSIZE, tant pour les données brutes (RawData) que       */
/*        les données converties (NavData) (voir ci-dessous)                                 */
void *SensorLogTask ( void *ptr ) {
	SensorStruct	*Sensor    = (SensorStruct *) ptr;
	uint16_t		*Idx       = &(Sensor->DataIdx);
	uint16_t		LocalIdx   = DATABUFSIZE;
	SensorData	 	*NavData   = NULL;
	SensorRawData	*RawData   = NULL;
	SensorRawData   tpRaw;
	SensorData 	    tpNav;
	double			norm;

	printf("%s : Log de %s prêt à démarrer\n", __FUNCTION__, Sensor->Name);
	pthread_barrier_wait(&(LogStartBarrier));

	while (LogActivated) {
		pthread_mutex_lock(&(Sensor->DataSampleMutex));
		while (LocalIdx == *Idx)
			pthread_cond_wait(&(Sensor->DataNewSampleCondVar), &(Sensor->DataSampleMutex));
	    pthread_mutex_unlock(&(Sensor->DataSampleMutex));

	   	pthread_spin_lock(&(Sensor->DataLock));
    	NavData   = &(Sensor->Data[LocalIdx]);
    	RawData   = &(Sensor->RawData[LocalIdx]);
		memcpy((void *) &tpRaw, (void *) RawData, sizeof(SensorRawData));
		memcpy((void *) &tpNav, (void *) NavData, sizeof(SensorData));
	   	pthread_spin_unlock(&(Sensor->DataLock));

	   	pthread_mutex_lock(&Log_Mutex);
		if (numLogOutput == 0)
			printf("Sensor  :     TimeStamp      SampleDelay  Status  SampleNum   Raw Sample Data  =>        Converted Sample Data               Norme\n");
		else switch (tpRaw.type) {
				case ACCELEROMETRE :	norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Accel   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
				case GYROSCOPE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Gyro    : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
				case SONAR :			printf("Sonar   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], tpNav.Data[0]);
										break;
				case BAROMETRE :		printf("Barom   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], tpNav.Data[0]);
										break;
				case MAGNETOMETRE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Magneto : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
			 }
		LocalIdx = *Idx;
		numLogOutput++;
		if (numLogOutput > 20)
			numLogOutput = 0;
		pthread_mutex_unlock(&Log_Mutex);
	}

	printf("%s : %s Terminé\n", __FUNCTION__, Sensor->Name);

	pthread_exit(0); /* exit thread */
}


int InitSensorLog (SensorStruct *Sensor) {
	pthread_attr_t		attr;
	struct sched_param	param;
	int					retval;

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_PROCESS);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = sched_get_priority_min(POLICY);
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	printf("Creating Log thread : %s\n", Sensor->Name);
	if ((retval = pthread_create(&(Sensor->LogThread), &attr, SensorLogTask, (void *) Sensor)) != 0)
		printf("%s : Impossible de créer Tâche Log de %s => retval = %d\n", __FUNCTION__, Sensor->Name, retval);


	pthread_attr_destroy(&attr);

	return 0;
}


int SensorsLogsInit (SensorStruct SensorTab[]) {
	int16_t	  i, numLog = 0;
	int16_t	  retval = 0;

	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			if ((retval = InitSensorLog(&SensorTab[i])) < 0) {
				printf("%s : Impossible d'initialiser log de %s => retval = %d\n", __FUNCTION__, SensorTab[i].Name, retval);
				return -1;
			}
			numLog++;
		}
	}
	pthread_barrier_init(&LogStartBarrier, NULL, numLog+1);
	pthread_mutex_init(&Log_Mutex, NULL);

	return 0;
};


int SensorsLogsStart (void) {
	LogActivated = 1;
	pthread_barrier_wait(&(LogStartBarrier));
	pthread_barrier_destroy(&LogStartBarrier);
	printf("%s NavLog démarré\n", __FUNCTION__);

	return 0;
};


int SensorsLogsStop (SensorStruct SensorTab[]) {
	int16_t	i;

	LogActivated = 0;
	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			pthread_join(SensorTab[i].LogThread, NULL);
			SensorTab[i].DoLog = 0;
		}
	}
	pthread_mutex_destroy(&Log_Mutex);

	return 0;
};

