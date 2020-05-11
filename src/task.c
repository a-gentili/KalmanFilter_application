#include <allegro.h>
#include <math.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>

#include "matrix.h"
#include "ptask.h"
#include "task.h"
#include "graphic.h"


//---------------------------------------------------------------------------
// Variables definitions
//---------------------------------------------------------------------------
const double	pi = 3.14159265359;

pthread_mutexattr_t muxatt;
pthread_mutex_t 	mux_matrix, mux_noise, mux_state, mux_fade, mux_points_pred, mux_prediction, mux_mouse, mux_p_to_predict, mux_ball_btn, mux_screen, mux_un_dec;

int fadeP[4] = {107, 278, 117, 294};// needed initialization coords fot the sliders North, South, T and P
int fadeS[4] = {126, 358, 136, 374};
int fadeN[4] = {126, 318, 136, 334};
int fadeT[4] = {40, 398, 50, 414};
int fadeUnit[4] = {26, 438, 36, 454};
int fadeDeci[4] = {26, 478, 36, 494};

int 	prediction =		0;
int 	p_to_predict = 		50;
int 	btn_state = 		0;
int 	btn_old = 			0;
int 	btn_state_ball = 	0;
int 	btn_old_ball = 		0;
int 	ball_btn = 			0;
int 	ball_btn_old = 		0;
int 	count = 			1;
float	var_noise[2]; 		// noise variance on x and y
float	units, decimals;	// support variables for change matrix numbers


//---------------------------------------------------------------------------
// Simulate the behaviour of a sensor takin as input the noise variance.
// It adds to the mouse position a gaussian noise then save into state
// the output of the sensor.
//---------------------------------------------------------------------------
void	sensor(states *state, float *var_noise) {

	poll_mouse();

	int x, y;
	float	xs_new, ys_new, Vxs_new, Vys_new;
	float 	gx_norm, gy_norm, gx_unif, gy_unif, gx, gy; 

	pthread_mutex_lock(&mux_ball_btn);
		int ball = ball_btn;
	pthread_mutex_unlock(&mux_ball_btn);

	pthread_mutex_lock(&mux_noise);
		float noise_x = var_noise[0];
		float noise_y = var_noise[1];
	pthread_mutex_unlock(&mux_noise);
	
	pthread_mutex_lock(&mux_matrix);
		float period_A = sys.A.m[2];
	pthread_mutex_unlock(&mux_matrix);

	pthread_mutex_lock(&mux_state);
		float xs_old = state->X_s.m[0];
		float ys_old = state->X_s.m[1];
	pthread_mutex_unlock(&mux_state);

	if (ball) {
		pthread_mutex_lock(&mux_state);
			x = state->X_ball.m[0];
			y = state->X_ball.m[1];
		pthread_mutex_unlock(&mux_state);
	} else {
		pthread_mutex_lock(&mux_mouse);
			x = mouse_x;
			y = mouse_y;
		pthread_mutex_unlock(&mux_mouse);
	}

	gx_unif = rand() / (float)RAND_MAX;						// realization of a uniformly distribuited n aleatory variable
	gy_unif = rand() / (float)RAND_MAX;
	gx_norm = sqrt(-2*log(gx_unif)) * cos(2*pi*gy_unif);	// realization of a gaussian distribuited aleatory variable
	gy_norm = sqrt(-2*log(gx_unif)) * sin(2*pi*gy_unif);
	gx 		= gx_norm * sqrt(noise_x);						// gaussian noise on x
	gy 		= gy_norm * sqrt(noise_y);						// gaussian noise on y

	if(x > FRAME_W)
		xs_new = FRAME_W + gx;
	else if(x < 0)
		xs_new = 0 + gx;
	else
		xs_new = (float)x + gx;

	if(y > FRAME_H)
		ys_new = FRAME_H + gy;
	else if((int)(y + gy) < 2)
		ys_new = 3;
	else
		ys_new = (float)y + gy;

	Vxs_new = (xs_new - xs_old)/(period_A);
	Vys_new = (ys_new - ys_old)/(period_A);
	
	pthread_mutex_lock(&mux_state);
		state->X_s.m[0] = xs_new;	// x
		state->X_s.m[1] = ys_new;	// y
		state->X_s.m[2] = Vxs_new; 	// x velocity
		state->X_s.m[3] = Vys_new;	// y velocity
	pthread_mutex_unlock(&mux_state);
	return;
}


//---------------------------------------------------------------------------
// Calculate the dynamic of the ball if the ball button is on
//---------------------------------------------------------------------------
void	ball_dynamic(systems sys, states *state) {

	if (ball_btn_old) {
		pthread_mutex_lock(&mux_state);
			float xball_old = state->X_ball.m[0];
			float yball_old = state->X_ball.m[1];
		pthread_mutex_unlock(&mux_state);

		scare_mouse();
		pthread_mutex_lock(&mux_screen);
			circlefill(screen, xball_old, yball_old, R_BALL, CANCEL);
		pthread_mutex_unlock(&mux_screen);
		unscare_mouse();
	}

	pthread_mutex_lock(&mux_ball_btn);
		int ball = ball_btn;
	pthread_mutex_unlock(&mux_ball_btn);
	if (ball) {
		float x_ball[DIM][1], t1[DIM][1], t2[DIM][1];
		matrice X_ball 	 = {DIM, 1, (float *)x_ball};
		matrice T1		 = {DIM, 1, (float *)t1};
		matrice	T2		 = {DIM, 1, (float *)t2};

		pthread_mutex_lock(&mux_matrix);
			float c_loc[DIM][DIM];
			matrice C_loc = {DIM, DIM, (float *)c_loc};
			C_loc = Copia(sys.C, &C_loc);
		pthread_mutex_unlock(&mux_matrix);

		pthread_mutex_lock(&mux_state);
			float x_ball_old[DIM][1], u_loc[DIM][1];
			matrice X_ball_old	= {DIM, 1, (float *)x_ball_old};
			matrice U_loc		= {DIM, 1, (float *)u_loc};
			X_ball_old	= Copia(state->X_ball, &X_ball_old);
			U_loc 		= Copia(U, &U_loc);
		pthread_mutex_unlock(&mux_state);

		if (count == CEIL_COUNT) {
			U_loc.m[1] = rand()%(2*F_X+1) -F_X;
			U_loc.m[0] = rand()%(2*F_Y+1) -F_Y;
			count = 1;
		} else count++;
		pthread_mutex_lock(&mux_state);
			U = Copia(U_loc, &U);
		pthread_mutex_unlock(&mux_state);

		X_ball = Somma(Prodotto(A_b, X_ball_old, &T1), Prodotto(B_b, U_loc, &T2), &X_ball, 1);
		if (X_ball.m[0] < R_BALL) {
			X_ball.m[0] = R_BALL;
			X_ball.m[2] = -X_ball.m[2];
		} else if (X_ball.m[0] > (FRAME_W - R_BALL)) {
			X_ball.m[0] = FRAME_W - R_BALL;
			X_ball.m[2] = -X_ball.m[2];
		}
		if (X_ball.m[1] < R_BALL) {
			X_ball.m[1] = R_BALL;
			X_ball.m[3] = -X_ball.m[3];
		} else if (X_ball.m[1] > (FRAME_H - R_BALL)) {
			X_ball.m[1] = FRAME_H - R_BALL;
			X_ball.m[3] = -X_ball.m[3];
		}

		pthread_mutex_lock(&mux_state);
			state->X_ball = Copia(X_ball, &(state->X_ball));
		pthread_mutex_unlock(&mux_state);
		scare_mouse();
		pthread_mutex_lock(&mux_screen);
		circlefill(screen, X_ball.m[0], X_ball.m[1], R_BALL, COLOR_WHITE);
		pthread_mutex_unlock(&mux_screen);
		unscare_mouse();
	}
	ball_btn_old = ball;
}


//---------------------------------------------------------------------------
// calculate the Kalman gain K and the covariance P
//---------------------------------------------------------------------------
void	kalman_loop(systems *sys) {

	// auxiliar matrixes
	float	t1[DIM][DIM], t2[DIM][DIM], t3[DIM][DIM], pl[DIM][DIM];
	matrice T1  = {DIM, DIM, (float *)t1};
	matrice	T2  = {DIM, DIM, (float *)t2};
	matrice T3  = {DIM, DIM, (float *)t3};
	matrice Pl	= {DIM, DIM, (float *)pl};

	// matrix update
	pthread_mutex_lock(&mux_matrix);
		float a_loc[DIM][DIM], c_loc[DIM][DIM], q_loc[DIM][DIM], r_loc[DIM][DIM];
		matrice A_loc = {DIM, DIM, (float * )a_loc};
		matrice C_loc = {DIM, DIM, (float * )c_loc};
		matrice Q_loc = {DIM, DIM, (float * )q_loc};
		matrice R_loc = {DIM, DIM, (float * )r_loc};
		A_loc = Copia(sys->A, &A_loc);
		C_loc = Copia(sys->C, &C_loc);
		Q_loc = Copia(sys->Q, &Q_loc);
		R_loc = Copia(sys->R, &R_loc);
	pthread_mutex_unlock(&mux_matrix);

	Pl 		= Somma(Prodotto(A_loc, Prodotto(sys->P, Trasposta(A_loc, &T1), &T2), &T1), Q_loc, &Pl, 0);		// P = A*P*A' + Q
	T1 		= Somma(Prodotto(C_loc, Prodotto(Pl, Trasposta(C_loc, &T2), &T3), &T2), R_loc, &T1, 0);			// T1 = C*P*C' + R (con T3 = P*C')
	sys->K 	= Prodotto(T3, Inversa(T1, &T2), &(sys->K));													// K = P*C'*T1^-1	out1
	sys->P 	= Prodotto(Somma(Identita(&T1, 1), Prodotto(sys->K, C_loc, &T2), &T3, -1), Pl, &(sys->P));		// P = (I - K*C)*P	out2
	return;
}


//---------------------------------------------------------------------------
// Calculate the estimation and prediction of Kalman Filter
//---------------------------------------------------------------------------
void	kalman_est(systems sys, states *state) {

	int i;
	float	t1[DIM][1], t2[DIM][1];
	matrice T1 	= {DIM, 1, (float *)t1};
	matrice	T2 	= {DIM, 1, (float *)t2};

	pthread_mutex_lock(&mux_matrix);
		float a_loc[DIM][DIM], c_loc[DIM][DIM], b_loc[DIM][DIM];
		matrice A_loc = {DIM, DIM, (float * )a_loc};
		matrice C_loc = {DIM, DIM, (float * )c_loc};
		matrice B_loc = {DIM, DIM, (float * )b_loc};
		A_loc = Copia(sys.A, &A_loc);
		C_loc = Copia(sys.C, &C_loc);
		B_loc = Copia(sys.B, &B_loc);
	pthread_mutex_unlock(&mux_matrix);

	pthread_mutex_lock(&mux_prediction);
		int pred = prediction;
	pthread_mutex_unlock(&mux_prediction);

	pthread_mutex_lock(&mux_ball_btn);
		int ball = ball_btn;
	pthread_mutex_unlock(&mux_ball_btn);

	pthread_mutex_lock(&mux_state);
		if (ball) {
			state->X_pred	= Somma(Prodotto(A_loc, state->X_k, &T1), Prodotto(B_loc, U, &T2), &(state->X_pred), 1);
		} else {
			state->X_pred	= Prodotto(A_loc, state->X_k, &(state->X_pred));	// X_pred = A*X_k	// X_k = X_pred + K*(X_s - C*X_pred)
		}
		state->X_k		= Somma(state->X_pred, Prodotto(sys.K, Somma(state->X_s, Prodotto(C_loc, state->X_pred, &T1), &T2, -1), &T1), &(state->X_k), 0);
	pthread_mutex_unlock(&mux_state);

	if (pred) {
		pthread_mutex_lock(&mux_state);
			float x_pred_loc[DIM][1], u_loc[DIM][1];
			matrice X_pred_loc	= {DIM, 1, (float *)x_pred_loc};
			matrice U_loc		= {DIM, 1, (float *)u_loc};
			X_pred_loc	= Copia(state->X_k, &X_pred_loc);
			U_loc 		= Copia(U, &U_loc);
		pthread_mutex_unlock(&mux_state);
		pthread_mutex_lock(&mux_points_pred);
			if (ball) {
				T2 = Prodotto(B_loc, U_loc, &T2);
				for (i = 0; i < DIM_POINTS_PRED/2; i++) {
					T1 = Somma(Prodotto(A_loc, X_pred_loc, &T1), T2, &T1, 1);
					X_pred_loc = Copia(T1, &(X_pred_loc));
					points_pred[2*i] 	= X_pred_loc.m[0];
					points_pred[2*i+1] 	= X_pred_loc.m[1];
				}
			} else {
				for (i = 0; i < DIM_POINTS_PRED/2; i++) {
					T1 = Prodotto(A_loc, X_pred_loc, &T1);
					X_pred_loc = Copia(T1, &(X_pred_loc));
					points_pred[2*i] 	= X_pred_loc.m[0];
					points_pred[2*i+1] 	= X_pred_loc.m[1];
				}
			}
		pthread_mutex_unlock(&mux_points_pred);
	}
	return;
}


//---------------------------------------------------------------------------
// Function that generates the graphic part using double buffering technique with allegro lib
//---------------------------------------------------------------------------
void	graphics(float *points, states state) {

	scare_mouse();

	draw_trail();
	draw_right_toolbar();
	draw_console();
	draw_sliders();
	draw_btns();
	draw_bottom_toolbar();
	draw_matrix_C();
	draw_matrix_Q();
	draw_matrix_R();
	draw_matrix_A();

	//copy on screen
	pthread_mutex_lock(&mux_screen);
		blit(bottom_toolbar, screen, 0, 0, 0, FRAME_H, B_TB_W, B_TB_H);
		blit(right_toolbar, screen, 0, 0, FRAME_W, 0, R_TB_W, FRAME_H+B_TB_H);
	pthread_mutex_unlock(&mux_screen);

	unscare_mouse();
}


//---------------------------------------------------------------------------
// Initialize allegro lib and create the screen 1024x768
//---------------------------------------------------------------------------
void	init_graphics() {

	srand(time(NULL));
	install_allegro(SYSTEM_AUTODETECT, &errno, atexit);
	install_keyboard();
	install_mouse();
	show_os_cursor(MOUSE_CURSOR_ARROW);
	poll_mouse();
	set_color_depth(8);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED,B_TB_W,FRAME_H+B_TB_H,0,0);
	clear_to_color(screen, CANCEL);
	set_window_title("Venturini Gentili Kalman Filter");
	right_toolbar = create_bitmap(R_TB_W, R_TB_H);
	bottom_toolbar = create_bitmap(B_TB_W, B_TB_H);
}


//---------------------------------------------------------------------------
// Moves the slider right and left if user click on it and moves the mouse
//---------------------------------------------------------------------------
void	mouse_actions() {

	pthread_mutex_lock(&mux_p_to_predict);
		int n_prediction = p_to_predict;
	pthread_mutex_unlock(&mux_p_to_predict);

	pthread_mutex_lock(&mux_mouse);
		int x = mouse_x;
		int y = mouse_y;
	pthread_mutex_unlock(&mux_mouse);

	pthread_mutex_lock(&mux_fade);
		// move the slider of number of point to predict
		if (mouse_b & 1 && ((30+FRAME_W) < x) && (x < (194+FRAME_W)) && (y > fadeP[1]) && (y < fadeP[3])) {
			if (x > 830 && x < 994) {
				fadeP[0] = (x-805);
				fadeP[2] = (x-795);
				n_prediction = (int)(((fadeP[0]-26)*(((float)DIM_POINTS_PRED/2)/164))+2);
			}
		}
		// move the slider of noise x
		if (mouse_b & 1 && ((30+FRAME_W) < x) && (x < (194+FRAME_W)) && (y > fadeN[1]) && (y < fadeN[3])) {
			if (x > 830 && x < 994) {
				fadeN[0] = (x-805);
				fadeN[2] = (x-795);
				pthread_mutex_lock(&mux_noise);
				var_noise[0] = ((fadeN[0]-26)*2);
				pthread_mutex_unlock(&mux_noise);
			}
		}
		// moves the slide of noise y
		if (mouse_b & 1 && ((30+FRAME_W) < x) && (x < (194+FRAME_W)) && (y > fadeS[1]) && (y < fadeS[3])) {
			if (x > 830 && x < 994) {
				fadeS[0] = (x-805);
				fadeS[2] = (x-795);
				pthread_mutex_lock(&mux_noise);
				var_noise[1] = ((fadeS[0]-26)*2);
				pthread_mutex_unlock(&mux_noise);
			}
		}
		// moves the slide of the system velocity
		if (mouse_b & 1 && ((30+FRAME_W) < x) && (x < (194+FRAME_W)) && (y > fadeT[1]) && (y < fadeT[3])) {
			if (x > 830 && x < 994) {
				fadeT[0] = (x-805);
				fadeT[2] = (x-795);
				pthread_mutex_lock(&mux_matrix);
					float DT = (((float)fadeT[0]-26)/1000) + 0.0001;
					sys.A.m[2] = DT;
					sys.A.m[7] = DT;
					sys.B.m[0] = (DT*DT)/(2*M);
					sys.B.m[2] = (DT*DT)/(2*M);
					sys.B.m[5] = (DT*DT)/(2*M);
					sys.B.m[7] = (DT*DT)/(2*M);
					sys.B.m[8] = DT/M;
					sys.B.m[10] = DT/M;
					sys.B.m[13] = DT/M;
					sys.B.m[15] = DT/M;
				pthread_mutex_unlock(&mux_matrix);
			}
		}
		// moves the slide of unit value of the matrixies
		if (mouse_b & 1 && ((30+FRAME_W) < x) && (x < (194+FRAME_W)) && (y > fadeUnit[1]) && (y < fadeUnit[3])) {
			if (x > 830 && x < 994) {
				fadeUnit[0] = (x-805);
				fadeUnit[2] = (x-795);
				pthread_mutex_lock(&mux_un_dec);
					units = ((fadeUnit[0]-26)*1);
				pthread_mutex_unlock(&mux_un_dec);
			}
		}
		// moves the slide of decimals value of the matrixies
		if (mouse_b & 1 && ((30+FRAME_W) < x) && (x < (194+FRAME_W)) && (y > fadeDeci[1]) && (y < fadeDeci[3])) {
			if (x > 830 && x < 994) {
				fadeDeci[0] = (x-805);
				fadeDeci[2] = (x-795);
				pthread_mutex_lock(&mux_un_dec);
					decimals = ((fadeDeci[0]-26)*0.006);
				pthread_mutex_unlock(&mux_un_dec);
			}
		}
	pthread_mutex_unlock(&mux_fade);

	pthread_mutex_lock(&mux_p_to_predict);
		p_to_predict = n_prediction;
	pthread_mutex_unlock(&mux_p_to_predict);

	// change prediction button state
	if ( (x > 915) && (x < 935) && (y > 555) && (y < 570) ) {
		if (mouse_b & 1)
			btn_state = 1; 
		if (btn_state == 1 && btn_old == 0) {
		pthread_mutex_lock(&mux_prediction);
			if (prediction)
				prediction = 0;
			else
				prediction = 1;
		pthread_mutex_unlock(&mux_prediction);
		}
		btn_old = btn_state;
		btn_state = 0;
	}

	// change ball button state
	if ( (x > 915) && (x < 935) && (y > 577) && (y < 587) ) {
		if (mouse_b & 1)
			btn_state_ball = 1; 
		if (btn_state_ball == 1 && btn_old_ball == 0) {
		pthread_mutex_lock(&mux_ball_btn);
			if (ball_btn)
				ball_btn = 0;
			else {
				ball_btn = 1;
				state.X_ball.m[0] = rand()%801;
				state.X_ball.m[1] = rand()%601;
				state.X_ball.m[2] = 0;
				state.X_ball.m[3] = 0;
			}
		pthread_mutex_unlock(&mux_ball_btn);
		}
		btn_old_ball = btn_state_ball;
		btn_state_ball = 0;
	}
}


//---------------------------------------------------------------------------
// Matrix customization function
//---------------------------------------------------------------------------
void	matrix_actions() {

	int i, j;
	char num;
	pthread_mutex_lock(&mux_mouse);
		int x = mouse_x;
		int y = mouse_y;
	pthread_mutex_unlock(&mux_mouse);
	pthread_mutex_lock(&mux_un_dec);
		float units_l = units;
		float decimals_l = decimals;
	pthread_mutex_unlock(&mux_un_dec);

	// customize matrix C
	if (mouse_b & 1 && x > (UL_X) && x < (UL_X+4*CELL_W) && y > (FRAME_H+UL_Y) && y < (FRAME_H+UL_Y+4*CELL_H)) {
		j = ((x-UL_X)/CELL_W);
		i = ((y-(FRAME_H+UL_Y))/CELL_H);
		pthread_mutex_lock(&mux_matrix);
			sys.C.m[i*4+j] = units_l+decimals_l;
		pthread_mutex_unlock(&mux_matrix);
	}
	//customize matrix Q
	if (mouse_b & 1 && x > (MATRIX_GAP+UL_X) && x < (MATRIX_GAP+UL_X+4*CELL_W) && y > (FRAME_H+UL_Y) && y < (FRAME_H+UL_Y+4*CELL_H)) {
		j = ((x-(MATRIX_GAP+UL_X))/CELL_W);
		i = ((y-(FRAME_H+UL_Y))/CELL_H);
		pthread_mutex_lock(&mux_matrix);
			sys.Q.m[i*4+j] = units_l+decimals_l;//((float)(num - '0'))/10;
		pthread_mutex_unlock(&mux_matrix);
	}
	// customize matrix R
	if (mouse_b & 1 && x > (2*MATRIX_GAP+UL_X) && x < (2*MATRIX_GAP+UL_X+4*CELL_W) && y > (FRAME_H+UL_Y) && y < (FRAME_H+UL_Y+4*CELL_H)) {
		j = ((x-(2*MATRIX_GAP+UL_X))/CELL_W);
		i = ((y-(FRAME_H+UL_Y))/CELL_H);
		pthread_mutex_lock(&mux_matrix);
			sys.R.m[i*4+j] = units_l+decimals_l;//((float)(num - '0'))/10;
		pthread_mutex_unlock(&mux_matrix);
	}
	// customize matrix A
	if (mouse_b & 1 && x > (3*MATRIX_GAP+UL_X) && x < (3*MATRIX_GAP+UL_X+4*CELL_W) && y > (FRAME_H+UL_Y) && y < (FRAME_H+UL_Y+4*CELL_H)) {
		j = ((x-(3*MATRIX_GAP+UL_X))/CELL_W);
		i = ((y-(FRAME_H+UL_Y))/CELL_H);
		pthread_mutex_lock(&mux_matrix);
			sys.A.m[i*4+j] = units_l+decimals_l;//((float)(num - '0'))/10;
		pthread_mutex_unlock(&mux_matrix);
	}
}


//---------------------------------------------------------------------------
// Defines how users can interact with the graphic part of the
// real time program and keep listening during the execution
//---------------------------------------------------------------------------
void	user_loop() {

	mouse_actions();
	matrix_actions();
}


//---------------------------------------------------------------------------
// graphic routine task
//---------------------------------------------------------------------------
void	*graphic_task(void *arg) {

	int ti;

	ti = get_task_index(arg);
	set_activation(ti);

	while (1) {
		graphics(points, state);
		if (deadline_miss(ti))
			printf("deadline miss graphic task\n");
		wait_activation(ti);
	}
}


//---------------------------------------------------------------------------
// Task that simulate the sensor and draw the readings
//---------------------------------------------------------------------------
void	*sensor_task(void *arg) {

	int ti;

	ti = get_task_index(arg);
	set_activation(ti);

	while (1) {
		scare_mouse();
		pthread_mutex_lock(&mux_state);
			float x_s_loc[DIM][1];
			matrice X_s_loc	= {DIM, 1, (float *)x_s_loc};
			X_s_loc	= Copia(state.X_s, &X_s_loc);
		pthread_mutex_unlock(&mux_state);
		if ((int)X_s_loc.m[0] < FRAME_W && (int)X_s_loc.m[1] < FRAME_H && (int)X_s_loc.m[0] > 0 && (int)X_s_loc.m[1] > 0) {
			pthread_mutex_lock(&mux_screen);
				circlefill(screen, (int)X_s_loc.m[0], (int)X_s_loc.m[1], 2, CANCEL);
			pthread_mutex_unlock(&mux_screen);
		}
		sensor(&state, var_noise);
		pthread_mutex_lock(&mux_state);
			X_s_loc	= Copia(state.X_s, &X_s_loc);
		pthread_mutex_unlock(&mux_state);
		if ((int)X_s_loc.m[0] < FRAME_W && (int)X_s_loc.m[1] < FRAME_H && (int)X_s_loc.m[0] > 0 && (int)X_s_loc.m[1] > 0) {
			pthread_mutex_lock(&mux_screen);
				circlefill(screen, (int)X_s_loc.m[0], (int)X_s_loc.m[1], 2, 2);
			pthread_mutex_unlock(&mux_screen);
		}
		unscare_mouse();
		if (deadline_miss(ti))
			printf("deadline miss sensor task\n");
		wait_activation(ti);
	}
}


//---------------------------------------------------------------------------
// Kalman routine task
//---------------------------------------------------------------------------
void	*kalman_task(void *arg) {

	int ti;

	ti = get_task_index(arg);
	set_activation(ti);

	while (1) {
		kalman_loop(&sys);
		kalman_est(sys, &state);
		if (deadline_miss(ti))
			printf("deadline miss kalman task\n");
		wait_activation(ti);
	}
}


//---------------------------------------------------------------------------
// User routine task
//---------------------------------------------------------------------------
void	*user_task(void *arg) {

	int ti;

	ti = get_task_index(arg);
	set_activation(ti);

	while (1) {
		user_loop();
		if (deadline_miss(ti))
			printf("deadline miss user task\n");
		wait_activation(ti);
	}
}


//---------------------------------------------------------------------------
// Ball routine task
//---------------------------------------------------------------------------
void	*ball_task(void *arg) {

	int ti;

	ti = get_task_index(arg);
	set_activation(ti);

	while (1) {
		ball_dynamic(sys, &state);
		if (deadline_miss(ti))
			printf("deadline miss ball task\n");
		wait_activation(ti);
	}
}


//---------------------------------------------------------------------------
// Task that initialize other task for close activation time
//---------------------------------------------------------------------------
void	*starter_task(void *arg) {

	pthread_mutexattr_init(&muxatt);
	pthread_mutexattr_setprotocol(&muxatt, PTHREAD_PRIO_INHERIT); //PTHREAD_PRIO_INHERIT

	pthread_mutex_init(&mux_matrix, &muxatt);
	pthread_mutex_init(&mux_noise, &muxatt);
	pthread_mutex_init(&mux_fade, &muxatt);
	pthread_mutex_init(&mux_state, &muxatt);
	pthread_mutex_init(&mux_points_pred, &muxatt);
	pthread_mutex_init(&mux_prediction, &muxatt);
	pthread_mutex_init(&mux_mouse, &muxatt);
	pthread_mutex_init(&mux_p_to_predict, &muxatt);
	pthread_mutex_init(&mux_ball_btn, &muxatt);
	pthread_mutex_init(&mux_screen, &muxatt);
	pthread_mutex_init(&mux_un_dec, &muxatt);

	pthread_mutexattr_destroy(&muxatt);


	if (task_create(ball_task, 4, (S*1000), (S*1000), 30) == 0)
		printf("kalman task created ... \n\n");

	if (task_create(sensor_task, 1, (T*1000), (T*1000), 25) == 0)
		printf("\nsensor task created ...\n\n");

	if (task_create(kalman_task, 2, (T*1000), (T*1000), 20) == 0)
		printf("kalman task created ... \n\n");

	if (task_create(user_task, 0, 10, 10, 15) == 0)
		printf("user task created ...\n\n");

	if (task_create(graphic_task, 3, 33, 33, 10) == 0)
		printf("graphic task created ... \n\n");
}