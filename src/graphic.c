#include <stdio.h>
#include <allegro.h>
#include <pthread.h>

#include "matrix.h"
#include "ptask.h"
#include "task.h"
#include "graphic.h"


//---------------------------------------------------------------------------
// Variables definitions
//---------------------------------------------------------------------------
static int prediction_old = 0;

BITMAP *right_toolbar;
BITMAP *bottom_toolbar;


//---------------------------------------------------------------------------
// Function that draw the estimations computed by the kalman filter
//---------------------------------------------------------------------------
void	draw_trail() {

	pthread_mutex_lock(&mux_p_to_predict);
		int n_prediction = p_to_predict;
	pthread_mutex_unlock(&mux_p_to_predict);

	pthread_mutex_lock(&mux_prediction);
		int pred = prediction;
	pthread_mutex_unlock(&mux_prediction);

	if(prediction_old){
		int i;
		pthread_mutex_lock(&mux_screen);
			line(screen, (int)points[DIM_POINTS-2], (int)points[DIM_POINTS-1], (int)points_pred_old[0], (int)points_pred_old[1], CANCEL);
		pthread_mutex_unlock(&mux_screen);
		for (i = 1; i < (DIM_POINTS_PRED/2)-1; i++) {
			pthread_mutex_lock(&mux_screen);
				line(screen, (int)points_pred_old[2*i], (int)points_pred_old[2*i+1], (int)points_pred_old[2*i+2], (int)points_pred_old[2*i+3], CANCEL);
			pthread_mutex_unlock(&mux_screen);
		}
	}

	pthread_mutex_lock(&mux_screen);
		line(screen, (int)points[0], (int)points[1], (int)points[2], (int)points[3], CANCEL);
	pthread_mutex_unlock(&mux_screen);
	aggiorna_points(points, state);
	pthread_mutex_lock(&mux_screen);
		line(screen, (int)points[DIM_POINTS-4], (int)points[DIM_POINTS-3], (int)points[DIM_POINTS-2], (int)points[DIM_POINTS-1], COLOR_YELLOW);
	pthread_mutex_unlock(&mux_screen);

	if (pred) {
		int i;
		pthread_mutex_lock(&mux_points_pred);
			float p_pred[DIM_POINTS_PRED];
			for (i = 0; i < DIM_POINTS_PRED; i++) {
				p_pred[i] = points_pred[i];
			}
		pthread_mutex_unlock(&mux_points_pred);

		pthread_mutex_lock(&mux_screen);
			line(screen, (int)points[DIM_POINTS-2], (int)points[DIM_POINTS-1], (int)p_pred[0], (int)p_pred[1], COLOR_RED);
		pthread_mutex_unlock(&mux_screen);
		for (i = 1; i < (n_prediction/2)-1; i++) {
			pthread_mutex_lock(&mux_screen);
				line(screen, (int)p_pred[2*i], (int)p_pred[2*i+1], (int)p_pred[2*i+2], (int)p_pred[2*i+3], COLOR_RED);
			pthread_mutex_unlock(&mux_screen);
		}
		for (i = 0; i < DIM_POINTS_PRED; i++) {
			points_pred_old[i] = p_pred[i];
		}
	}
	prediction_old = pred;
}


//---------------------------------------------------------------------------
// Graphic functions
//---------------------------------------------------------------------------
void	draw_right_toolbar() {

	clear_to_color(right_toolbar, COLOR_BLACK);
	vline(right_toolbar, 0, 0, FRAME_H, COLOR_PINK);
}


//---------------------------------------------------------------------------
void	draw_console() {

	rect(right_toolbar, 10, 5, 213, 255, COLOR_PINK);
	textout_ex(right_toolbar, font, "Real-Time Project", 40, 15, COLOR_PINK, COLOR_BLACK);
	hline(right_toolbar, 10, 30, 213, COLOR_PINK);
	textout_ex(right_toolbar, font, " - Deadline misses -", 20, 40, COLOR_PINK, COLOR_BLACK);
	textprintf_ex(right_toolbar, font, 20, 110, COLOR_PINK, COLOR_BLACK, "sensor loop: %d", tp[1].dmiss);
	textprintf_ex(right_toolbar, font, 20, 80, COLOR_PINK, COLOR_BLACK, "kalman loop: %d", tp[2].dmiss);
	textprintf_ex(right_toolbar, font, 20, 95, COLOR_PINK, COLOR_BLACK, "graphic loop: %d", tp[3].dmiss);
	textprintf_ex(right_toolbar, font, 20, 65, COLOR_PINK, COLOR_BLACK, "user loop: %d", tp[0].dmiss);
	textprintf_ex(right_toolbar, font, 20, 125, COLOR_PINK, COLOR_BLACK, "ball loop: %d", tp[4].dmiss);

	textout_ex(right_toolbar, font, "State vector:", 20, 220, COLOR_PINK, COLOR_BLACK);
	textout_ex(right_toolbar, font, "X = [x, y, Vx, Vy]'", 20, 235, COLOR_PINK, COLOR_BLACK);
}


//---------------------------------------------------------------------------
void	draw_btns() {

	pthread_mutex_lock(&mux_prediction);
		int pred = prediction;
	pthread_mutex_unlock(&mux_prediction);

	textout_ex(right_toolbar, font, "Prediction:", 20, 560, COLOR_PINK, COLOR_BLACK);
	if (pred) {
		rectfill(right_toolbar, 120, 557, 130, 567, COLOR_PINK);
		textout_ex(right_toolbar, font, "On", 150, 560, COLOR_PINK, COLOR_BLACK);
	}
	else {
		rect(right_toolbar, 120, 557, 130, 567, COLOR_PINK);
		textout_ex(right_toolbar, font, "Off", 150, 560, COLOR_PINK, COLOR_BLACK);
	}

	pthread_mutex_lock(&mux_ball_btn);
		int ball = ball_btn;
	pthread_mutex_unlock(&mux_ball_btn);

	textout_ex(right_toolbar, font, "Ball:", 20, 580, COLOR_PINK, COLOR_BLACK);
	if (ball) {
		rectfill(right_toolbar, 120, 577, 130, 587, COLOR_PINK);
		textout_ex(right_toolbar, font, "On", 150, 580, COLOR_PINK, COLOR_BLACK);
	}
	else {
		rect(right_toolbar, 120, 577, 130, 587, COLOR_PINK);
		textout_ex(right_toolbar, font, "Off", 150, 580, COLOR_PINK, COLOR_BLACK);
	}
}


//---------------------------------------------------------------------------
void	draw_sliders() {


	pthread_mutex_lock(&mux_noise);
		int noise_x = (int)var_noise[0];
		int noise_y = (int)var_noise[1];
	pthread_mutex_unlock(&mux_noise);

	pthread_mutex_lock(&mux_matrix);
		float step = sys.A.m[2];
	pthread_mutex_unlock(&mux_matrix);

	pthread_mutex_lock(&mux_un_dec);
		float units_l = units;
		float decimals_l = decimals;
	pthread_mutex_unlock(&mux_un_dec);

	textprintf_ex(right_toolbar, font, 20, 264, COLOR_PINK, COLOR_BLACK, "Point to predict: %d", p_to_predict);
	rectfill(right_toolbar, 30, 284, 194, 288, COLOR_PINK); // base slider prediction
	textprintf_ex(right_toolbar, font, 20, 304, COLOR_PINK, COLOR_BLACK, "Noise x: %d", noise_x);
	rectfill(right_toolbar, 30, 324, 194, 328, COLOR_PINK); // base slider x
	textprintf_ex(right_toolbar, font, 20 , 344 , COLOR_PINK, COLOR_BLACK, "Noise y: %d", noise_y);
	rectfill(right_toolbar, 30, 364, 194, 368, COLOR_PINK); // base slider y
	textprintf_ex(right_toolbar, font, 20 , 384 , COLOR_PINK, COLOR_BLACK, "Period: %.3f", step);
	rectfill(right_toolbar, 30, 404, 194, 408, COLOR_PINK); // base slider t
	textprintf_ex(right_toolbar, font, 20 , 424 , COLOR_PINK, COLOR_BLACK, "Units: %.0f", units_l);
	rectfill(right_toolbar, 30, 444, 194, 448, COLOR_PINK); // base slider Units
	textprintf_ex(right_toolbar, font, 20 , 464 , COLOR_PINK, COLOR_BLACK, "Decimals: %.3f", decimals_l);
	rectfill(right_toolbar, 30, 484, 194, 488, COLOR_PINK); // base slider Decimals

	// mutex slider to change
	pthread_mutex_lock(&mux_fade);
		rectfill(right_toolbar, fadeP[0], fadeP[1], fadeP[2], fadeP[3], COLOR_PINK); 				// slider point predicted
		rectfill(right_toolbar, fadeS[0], fadeS[1], fadeS[2], fadeS[3], COLOR_PINK); 				// slider y noise
		rectfill(right_toolbar, fadeN[0], fadeN[1], fadeN[2], fadeN[3], COLOR_PINK); 				// slider x noise
		rectfill(right_toolbar, fadeT[0], fadeT[1], fadeT[2], fadeT[3], COLOR_PINK); 				// slider period T
		rectfill(right_toolbar, fadeUnit[0], fadeUnit[1], fadeUnit[2], fadeUnit[3], COLOR_PINK); 	// slider units
		rectfill(right_toolbar, fadeDeci[0], fadeDeci[1], fadeDeci[2], fadeDeci[3], COLOR_PINK); 	// slider decimals
	pthread_mutex_unlock(&mux_fade);
}


//---------------------------------------------------------------------------
void	draw_bottom_toolbar() {

	clear_to_color(bottom_toolbar, COLOR_BLACK);
	hline(bottom_toolbar, 0, 0, FRAME_W, COLOR_PINK);
	textout_ex(bottom_toolbar, font, "To customize matrices select the units and decimals with the sliders and click on the desired cell:", 10, 8, COLOR_PINK, COLOR_BLACK);
}


//---------------------------------------------------------------------------
void	draw_matrix_C() {

	// mutex matrix
	textout_ex(bottom_toolbar, font, "C:", MAT_ID_X, MAT_ID_Y, COLOR_PINK, COLOR_BLACK);
	textout_ex(bottom_toolbar, font, "Output", UL_X, MAT_DES_Y, COLOR_PINK, COLOR_BLACK);
	vline(bottom_toolbar, UL_X, 		 UL_Y, 			UL_Y+4*CELL_H, COLOR_PINK);
	vline(bottom_toolbar, UL_X+CELL_W, 	 UL_Y, 			UL_Y+4*CELL_H, COLOR_PINK);
	vline(bottom_toolbar, UL_X+2*CELL_W, UL_Y, 			UL_Y+4*CELL_H, COLOR_PINK);
	vline(bottom_toolbar, UL_X+3*CELL_W, UL_Y, 			UL_Y+4*CELL_H, COLOR_PINK);
	vline(bottom_toolbar, UL_X+4*CELL_W, UL_Y, 			UL_Y+4*CELL_H, COLOR_PINK);
	hline(bottom_toolbar, UL_X, 		 UL_Y, 			UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, UL_X, 		 UL_Y+CELL_H, 	UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, UL_X, 		 UL_Y+2*CELL_H, UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, UL_X, 		 UL_Y+3*CELL_H, UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, UL_X, 		 UL_Y+4*CELL_H, UL_X+4*CELL_W, COLOR_PINK);

	pthread_mutex_lock(&mux_matrix);
		int i;
		float C_local[DIM*DIM];
		for (i = 0; i < DIM*DIM; i++) {
			C_local[i] = sys.C.m[i];
		}
	pthread_mutex_unlock(&mux_matrix);

	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X, 		 UL_Y+UL_CELL_GAP_Y, 		  COLOR_PINK, COLOR_BLACK, "%.1f", C_local[0]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+CELL_W, 	 UL_Y+UL_CELL_GAP_Y, 		  COLOR_PINK, COLOR_BLACK, "%.1f", C_local[1]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*CELL_W, UL_Y+UL_CELL_GAP_Y, 		  COLOR_PINK, COLOR_BLACK, "%.1f", C_local[2]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*CELL_W, UL_Y+UL_CELL_GAP_Y, 		  COLOR_PINK, COLOR_BLACK, "%.1f", C_local[3]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X, 		 UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", C_local[4]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+CELL_W, 	 UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", C_local[5]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*CELL_W, UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", C_local[6]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*CELL_W, UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", C_local[7]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X, 		 UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", C_local[8]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+CELL_W, 	 UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", C_local[9]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*CELL_W, UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", C_local[10]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*CELL_W, UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", C_local[11]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X, 		 UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", C_local[12]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+CELL_W, 	 UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", C_local[13]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*CELL_W, UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", C_local[14]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*CELL_W, UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", C_local[15]);
}


//---------------------------------------------------------------------------
void	draw_matrix_Q() {

	// mutex matrix
	textout_ex(bottom_toolbar, font, "Q:", MAT_ID_X+MATRIX_GAP, MAT_ID_Y, COLOR_PINK, COLOR_BLACK);
	textout_ex(bottom_toolbar, font, "System covariance", MATRIX_GAP+UL_X, MAT_DES_Y, COLOR_PINK, COLOR_BLACK);
	vline(bottom_toolbar, MATRIX_GAP+UL_X, 			UL_Y, 		   UL_Y+4*CELL_H, 			 COLOR_PINK);
	vline(bottom_toolbar, MATRIX_GAP+UL_X+CELL_W, 	UL_Y, 		   UL_Y+4*CELL_H, 			 COLOR_PINK);
	vline(bottom_toolbar, MATRIX_GAP+UL_X+2*CELL_W, UL_Y, 		   UL_Y+4*CELL_H, 			 COLOR_PINK);
	vline(bottom_toolbar, MATRIX_GAP+UL_X+3*CELL_W, UL_Y, 		   UL_Y+4*CELL_H, 			 COLOR_PINK);
	vline(bottom_toolbar, MATRIX_GAP+UL_X+4*CELL_W, UL_Y, 		   UL_Y+4*CELL_H, 			 COLOR_PINK);
	hline(bottom_toolbar, MATRIX_GAP+UL_X, 			UL_Y, 		   MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, MATRIX_GAP+UL_X, 			UL_Y+CELL_H,   MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, MATRIX_GAP+UL_X, 			UL_Y+2*CELL_H, MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, MATRIX_GAP+UL_X, 			UL_Y+3*CELL_H, MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, MATRIX_GAP+UL_X, 			UL_Y+4*CELL_H, MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);

	pthread_mutex_lock(&mux_matrix);
		int i;
		float Q_local[DIM*DIM];
		for (i = 0; i < DIM*DIM; i++) {
			Q_local[i] = sys.Q.m[i];
		}
	pthread_mutex_unlock(&mux_matrix);

	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP, 			UL_Y+UL_CELL_GAP_Y,			 COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[0]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+CELL_W, 	UL_Y+UL_CELL_GAP_Y,			 COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[1]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y,			 COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[2]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y,			 COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[3]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP, 			UL_Y+UL_CELL_GAP_Y+CELL_H,	 COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[4]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+CELL_W, 	UL_Y+UL_CELL_GAP_Y+CELL_H,	 COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[5]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y+CELL_H,	 COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[6]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y+CELL_H,	 COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[7]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP, 			UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[8]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+CELL_W, 	UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[9]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[10]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[11]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP, 			UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[12]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+CELL_W, 	UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[13]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[14]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", Q_local[15]);
}


//---------------------------------------------------------------------------
void	draw_matrix_R() {

	// mutex matrix
	textout_ex(bottom_toolbar, font, "R:", MAT_ID_X+2*MATRIX_GAP, MAT_ID_Y, COLOR_PINK, COLOR_BLACK);
	textout_ex(bottom_toolbar, font, "Sensor covariance", 2*MATRIX_GAP+UL_X, MAT_DES_Y, COLOR_PINK, COLOR_BLACK);
	vline(bottom_toolbar, 2*MATRIX_GAP+UL_X, 		  UL_Y,			 UL_Y+4*CELL_H,				 COLOR_PINK);
	vline(bottom_toolbar, 2*MATRIX_GAP+UL_X+CELL_W,   UL_Y,			 UL_Y+4*CELL_H,				 COLOR_PINK);
	vline(bottom_toolbar, 2*MATRIX_GAP+UL_X+2*CELL_W, UL_Y,			 UL_Y+4*CELL_H,				 COLOR_PINK);
	vline(bottom_toolbar, 2*MATRIX_GAP+UL_X+3*CELL_W, UL_Y,			 UL_Y+4*CELL_H,				 COLOR_PINK);
	vline(bottom_toolbar, 2*MATRIX_GAP+UL_X+4*CELL_W, UL_Y,			 UL_Y+4*CELL_H,				 COLOR_PINK);
	hline(bottom_toolbar, 2*MATRIX_GAP+UL_X, 		  UL_Y,			 2*MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, 2*MATRIX_GAP+UL_X, 		  UL_Y+CELL_H,	 2*MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, 2*MATRIX_GAP+UL_X, 		  UL_Y+2*CELL_H, 2*MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, 2*MATRIX_GAP+UL_X, 		  UL_Y+3*CELL_H, 2*MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, 2*MATRIX_GAP+UL_X, 		  UL_Y+4*CELL_H, 2*MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);

	pthread_mutex_lock(&mux_matrix);
		int i;
		float R_local[DIM*DIM];
		for (i = 0; i < DIM*DIM; i++) {
			R_local[i] = sys.R.m[i];
		}
	pthread_mutex_unlock(&mux_matrix);

	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP,		  UL_Y+UL_CELL_GAP_Y,		   COLOR_PINK, COLOR_BLACK, "%.1f", R_local[0]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+CELL_W,	  UL_Y+UL_CELL_GAP_Y,		   COLOR_PINK, COLOR_BLACK, "%.1f", R_local[1]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y,		   COLOR_PINK, COLOR_BLACK, "%.1f", R_local[2]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y,		   COLOR_PINK, COLOR_BLACK, "%.1f", R_local[3]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP, 		  UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", R_local[4]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+CELL_W,   UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", R_local[5]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", R_local[6]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", R_local[7]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP, 		  UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", R_local[8]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+CELL_W,   UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", R_local[9]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", R_local[10]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", R_local[11]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP, 		  UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", R_local[12]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+CELL_W,   UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", R_local[13]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", R_local[14]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+2*MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", R_local[15]);
	pthread_mutex_unlock(&mux_matrix);
}


//---------------------------------------------------------------------------
void	draw_matrix_A() {

	// mutex matrix
	textout_ex(bottom_toolbar, font, "A:", MAT_ID_X+3*MATRIX_GAP, MAT_ID_Y, COLOR_PINK, COLOR_BLACK);
	textout_ex(bottom_toolbar, font, "Dynamic", 3*MATRIX_GAP+UL_X, MAT_DES_Y, COLOR_PINK, COLOR_BLACK);
	vline(bottom_toolbar, 3*MATRIX_GAP+UL_X,		  UL_Y,			 UL_Y+4*CELL_H,				 COLOR_PINK);
	vline(bottom_toolbar, 3*MATRIX_GAP+UL_X+CELL_W,   UL_Y,			 UL_Y+4*CELL_H,				 COLOR_PINK);
	vline(bottom_toolbar, 3*MATRIX_GAP+UL_X+2*CELL_W, UL_Y,			 UL_Y+4*CELL_H,				 COLOR_PINK);
	vline(bottom_toolbar, 3*MATRIX_GAP+UL_X+3*CELL_W, UL_Y,			 UL_Y+4*CELL_H,				 COLOR_PINK);
	vline(bottom_toolbar, 3*MATRIX_GAP+UL_X+4*CELL_W, UL_Y,			 UL_Y+4*CELL_H,				 COLOR_PINK);
	hline(bottom_toolbar, 3*MATRIX_GAP+UL_X, 		  UL_Y,			 3*MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, 3*MATRIX_GAP+UL_X, 		  UL_Y+CELL_H,	 3*MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, 3*MATRIX_GAP+UL_X, 		  UL_Y+2*CELL_H, 3*MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, 3*MATRIX_GAP+UL_X, 		  UL_Y+3*CELL_H, 3*MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);
	hline(bottom_toolbar, 3*MATRIX_GAP+UL_X, 		  UL_Y+4*CELL_H, 3*MATRIX_GAP+UL_X+4*CELL_W, COLOR_PINK);

	pthread_mutex_lock(&mux_matrix);
		int i;
		float A_local[DIM*DIM];
		for (i = 0; i < DIM*DIM; i++) {
			A_local[i] = sys.A.m[i];
		}
	pthread_mutex_unlock(&mux_matrix);

	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP,		  UL_Y+UL_CELL_GAP_Y,		   COLOR_PINK, COLOR_BLACK, "%.1f", A_local[0]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+CELL_W,	  UL_Y+UL_CELL_GAP_Y,		   COLOR_PINK, COLOR_BLACK, "%.1f", A_local[1]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y,		   COLOR_PINK, COLOR_BLACK, "%.1f", A_local[2]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y,		   COLOR_PINK, COLOR_BLACK, "%.1f", A_local[3]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP,		  UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", A_local[4]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+CELL_W,   UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", A_local[5]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", A_local[6]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y+CELL_H,   COLOR_PINK, COLOR_BLACK, "%.1f", A_local[7]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP, 		  UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", A_local[8]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+CELL_W,   UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", A_local[9]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", A_local[10]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y+2*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", A_local[11]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP,		  UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", A_local[12]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+CELL_W,	  UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", A_local[13]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+2*CELL_W, UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", A_local[14]);
	textprintf_ex(bottom_toolbar, font, UL_X+UL_CELL_GAP_X+3*MATRIX_GAP+3*CELL_W, UL_Y+UL_CELL_GAP_Y+3*CELL_H, COLOR_PINK, COLOR_BLACK, "%.1f", A_local[15]);
}