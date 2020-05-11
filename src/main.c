#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <allegro.h>

#include "matrix.h"
#include "task.h"
#include "ptask.h"


int main() {

	int exit = 1;

	init_graphics();
	init_struct();
	printf("\ninitialization ...\n");

	if (task_create(starter_task, 5, 1, 1, 99) == 0)
		printf("\nstarter task created ...\n");

	while (exit) {
		if (key[KEY_ESC]) {
			printf("\n\nshutting down ...\n");
			exit = 0;

			if(pthread_cancel(tp[0].tid) == 0)
				printf("\nuser task canceled ...\n");

			if(pthread_cancel(tp[1].tid) == 0)
				printf("\nsensor task canceled ...\n");

			if(pthread_cancel(tp[2].tid) == 0)
				printf("\nkalman task canceled ...\n");

			if(pthread_cancel(tp[3].tid) == 0)
				printf("\ngraphic task canceled ...\n");

			if(pthread_cancel(tp[4].tid) == 0)
				printf("\nball task canceled ...\n");

			pthread_mutex_destroy(&mux_fade);
			pthread_mutex_destroy(&mux_noise);
			pthread_mutex_destroy(&mux_state);
			pthread_mutex_destroy(&mux_matrix);
			pthread_mutex_destroy(&mux_points_pred);
			pthread_mutex_destroy(&mux_prediction);
			pthread_mutex_destroy(&mux_p_to_predict);
			pthread_mutex_destroy(&mux_ball_btn);
			pthread_mutex_destroy(&mux_screen);
			pthread_mutex_destroy(&mux_un_dec);

			printf("\nshutdown completed\n\n");
		}
	}
}
END_OF_MAIN();