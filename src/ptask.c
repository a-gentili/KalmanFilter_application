#include <time.h>
#include <pthread.h>
#include <sched.h>
#include "ptask.h"


//---------------------------------------------------------------------------
// Variables definitions
//---------------------------------------------------------------------------
task_par tp[N_THREADS];	// task parameters for thread creation


//---------------------------------------------------------------------------
// Support library for pthread library
//---------------------------------------------------------------------------
int 	task_create (
		void *(*task)(void *),
		int idx,
		int period,
		int drel,
		int prio) {

pthread_attr_t myatt;
struct sched_param mypar;
int tret;

	tp[idx].arg 		= idx;
	tp[idx].period 		= period;
	tp[idx].deadline 	= drel;
	tp[idx].priority 	= prio;
	tp[idx].dmiss 		= 0;

	pthread_attr_init(&myatt);
	pthread_attr_setdetachstate(&myatt, PTHREAD_CREATE_DETACHED);
	pthread_attr_setinheritsched(&myatt, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&myatt, SCHED_FIFO);
	mypar.sched_priority = tp[idx].priority;
	pthread_attr_setschedparam(&myatt, &mypar);
	tret = pthread_create(&tp[idx].tid, &myatt, task, (void*)(&tp[idx]));
	return tret;
}



int 	get_task_index(void * arg) {

	struct task_par *tp;
	tp = (struct task_par *)arg;
	return tp->arg;
}


int 	set_activation(int i) {

	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	time_copy(&(tp[i].at), t);
	time_copy(&(tp[i].dl), t);
	time_add_ms(&(tp[i].at), tp[i].period);
	time_add_ms(&(tp[i].dl), tp[i].deadline);
}


int 	deadline_miss(int i) {

	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	if(time_cmp(now, tp[i].dl) > 0) {
		tp[i].dmiss++;
		return 1;
	}
	return 0;
}


void 	wait_activation(int i) {

	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp[i].at), NULL);
	time_add_ms(&(tp[i].at), tp[i].period);
	time_add_ms(&(tp[i].dl), tp[i].period);
}


// Copies a source time variable ts in a destination variable pointed by td
void	time_copy(struct timespec *td, struct timespec ts) {

	td->tv_sec  = ts.tv_sec;
	td->tv_nsec = ts.tv_nsec;
}


// Adds a value ms expressed in milliseconds to the time variable pointed by t
void	time_add_ms(struct timespec *t, int ms) {

	t->tv_sec  += ms/1000;
	t->tv_nsec += (ms%1000)*1000000;

	if (t->tv_nsec > 1000000000)
	{
	t->tv_nsec -= 1000000000;
	t->tv_sec   += 1;
	}
}


// Compare time variables
int	time_cmp(struct timespec t1, struct timespec t2) {

	if (t1.tv_sec > t2.tv_sec)   return 1;
	if (t1.tv_sec < t2.tv_sec)   return -1;
	if (t1.tv_nsec > t2.tv_nsec) return 1;
	if (t1.tv_nsec < t2.tv_nsec) return -1;
	return 0;
}