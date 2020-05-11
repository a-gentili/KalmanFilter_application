#ifndef TASK_H_
#define TASK_H_

//---------------------------------------------------------------------------
// Define
//---------------------------------------------------------------------------
#define CEIL_COUNT			750


//---------------------------------------------------------------------------
// Global variables declarations
//---------------------------------------------------------------------------
extern 	pthread_mutexattr_t muxatt;
extern 	pthread_mutex_t 	mux_points, mux_matrix, mux_noise, mux_state, mux_fade, mux_points_pred, mux_prediction, mux_mouse, mux_p_to_predict, mux_ball_btn, mux_screen, mux_un_dec;

extern int prediction;
extern int p_to_predict;
extern int ball_btn;
extern int fadeP[4];
extern int fadeS[4];
extern int fadeN[4];
extern int fadeT[4];
extern int fadeUnit[4];
extern int fadeDeci[4];

extern float units; 
extern float decimals;

//---------------------------------------------------------------------------
// Function declarations
//---------------------------------------------------------------------------
void 	init_graphics(void);
void 	*starter_task(void *arg);

#endif /* TASK_H_ */