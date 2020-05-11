#ifndef PTASK_H_
#define PTASK_H_


//---------------------------------------------------------------------------
// Define
//---------------------------------------------------------------------------
#define N_THREADS	10


//---------------------------------------------------------------------------
// Typedef variables definition
//---------------------------------------------------------------------------
typedef struct task_par {

	int arg;			// task argument
	pthread_t tid;		// thread id
	long period;		// in milliseconds (ms)
	int deadline;		// relative (ms)
	int priority;		// in [0,99]
	int dmiss;			// number of deadline misses
	struct timespec at;	// next acrtiv.time
	struct timespec dl;	// abs. deadline
} task_par;


//---------------------------------------------------------------------------
// Global variables declarations
//---------------------------------------------------------------------------
extern task_par tp[N_THREADS];


//---------------------------------------------------------------------------
// Function declarations
//---------------------------------------------------------------------------
int		task_create(void *(*task)(void *), int i, int period, int drel, int prio);
int 	get_task_index(void * arg);
int 	set_activation(int i);
int 	deadline_miss(int i);
void 	wait_activation(int i);
void	time_copy(struct timespec *td, struct timespec ts);
void	time_add_ms(struct timespec *t, int ms);
int		time_cmp(struct timespec t1, struct timespec t2);

#endif /* PTASK_H_ */
