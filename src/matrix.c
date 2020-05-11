#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>

#include "matrix.h"
#include "task.h"


//---------------------------------------------------------------------------
// Variables definitions
//---------------------------------------------------------------------------
systems sys;
states state;

float	var_noise[2] = {200, 200}; 			// {var_noise_x, var_noise_ y}
float 	points[DIM_POINTS];					// vector of points to be drawn
float 	points_pred[DIM_POINTS_PRED];		// vector of prediction points to be drawn
float 	points_pred_old[DIM_POINTS_PRED];	// vector of prediction points to be canceled

float 	a_b[DIM][DIM] = {{1,0,S,0},	// mouse and ball system dynamic matrix
						 {0,1,0,S},
						 {0,0,1,0},
						 {0,0,0,1}
						 };
float 	b_b[DIM][DIM] = {{(S*S)/(2*M),           0,(S*S)/(2*M),           0},	// ball input matrix (Fx, Fy, Fconstx, Fconsty)
						 {           0,(S*S)/(2*M),           0,(S*S)/(2*M)},
						 {        S/M,           0,         S/M,          0},
						 {           0,        S/M,           0,        S/M}
						 };
matrice	A_b = {DIM, DIM, (float *)a_b};
matrice	B_b = {DIM, DIM, (float *)b_b};

float 	a[DIM][DIM] = {	{1,0,T,0},	// mouse and ball system dynamic matrix
						{0,1,0,T},
						{0,0,1,0},
						{0,0,0,1}
						};
float 	b[DIM][DIM] = {	{(T*T)/(2*M),          0,(T*T)/(2*M),          0},	// ball input matrix (Fx, Fy, Fconstx, Fconsty)
						{          0,(T*T)/(2*M),          0,(T*T)/(2*M)},
						{        T/M,          0,        T/M,          0},
						{          0,        T/M,          0,        T/M}
						};
float 	c[DIM][DIM] = {	{1,0,0,0},	// output matrix
						{0,1,0,0},
						{0,0,0,0},
						{0,0,0,0}
						};
float 	q[DIM][DIM] = {	{0.1,  0,  0,  0},	// system covariance matrix
						{  0,0.1,  0,  0},
						{  0,  0,0.1,  0},
						{  0,  0,  0,0.1}
						};
float 	r[DIM][DIM] = {	{200,  0,  0,  0},	// sensor covariance matrix
						{  0,200,  0,  0},
						{  0,  0,0.1,  0},
						{  0,  0,  0,0.1}
						};
float 	p[DIM][DIM] = {	{0,0,0,0},	// evaluation error covariance matrix
						{0,0,0,0},
						{0,0,0,0},
						{0,0,0,0}
						};
float 	k[DIM][DIM] = {	{0,0,0,0},	// kalman gain
						{0,0,0,0},
						{0,0,0,0},
						{0,0,0,0}
						};
float 	*m[7] = {(float *)a, (float *)b, (float *)c, (float *)q, (float *)r, (float *)p, (float *)k};	// auxiliary vector for initialization of variable sys

float	x_k[DIM][1] 	= {{0},{0},{0},{0}};		// estimated state
float	x_pred[DIM][1] 	= {{0},{0},{0},{0}};		// predicted state
float	x_s[DIM][1] 	= {{0},{0},{0},{0}};		// sensor output
float	x_ball[DIM][1] 	= {{0},{0},{0},{0}};		// ball state
float	*s[4] = {(float *)x_k, (float *)x_pred, (float *)x_s, (float *)x_ball};	// auxiliary vector for initialization of variable state

float	u[DIM][1] = {{0},{0},{F_CONST_X},{F_CONST_Y}};
matrice U = {DIM, 1, (float *)u};					// vector of input for ball system

//---------------------------------------------------------------------------
// Library for matrix operations (the name is explicative and in italian)
//---------------------------------------------------------------------------
matrice Copia(matrice A, matrice *B) {

	int i, j;

	if (A.righe != B->righe || A.colonne != B->colonne) {
		printf("\nCopia: le matrici non hanno le stesse dimensioni\n");
		exit(1);
	}

	for (i = 0; i < A.righe; i++) {
		for (j = 0; j < A.colonne; j++) {
			(*B).m[i*A.colonne + j] = A.m[i*A.colonne + j];
		}
	}
	return *B;
}


//---------------------------------------------------------------------------
matrice	Identita(matrice *Mat, float cost) {

	int i, j;

	for (i = 0; i < Mat->righe; i++) {
		for (j = 0; j < Mat->righe; j++) {
			if (i==j) {
				(*Mat).m[i*Mat->righe +j] = cost;
			} else (*Mat).m[i*Mat->righe +j] = 0;
		}
	}
	return *Mat;
}


//---------------------------------------------------------------------------
float	Traccia(matrice Mat) {

	float tr = 0;
	int i;

	if (Mat.righe != Mat.colonne) {
		printf("\nTraccia: la matrice non è quadrata\n");
		return 0;
	}

	for (i = 0; i < Mat.righe; i++) {
		tr += Mat.m[i*Mat.colonne +i];
	}
	return tr;
}


//---------------------------------------------------------------------------
float	Determinante(matrice Mat) {

  	int r = Mat.righe;
  	int c = Mat.colonne;
	float det;

  	if (r != c) {
    	printf("\nDeterminante: matrice non quadrata\n");
    	return 0;
  	}
  	if (r > 4) {
    	printf("\nDeterminante: matrice troppo grande\n");
    	return 0;
  	}

  	if (r == 1) return Mat.m[0];
  	if (r == 2) {
    		det = Mat.m[0]*Mat.m[3] - Mat.m[1]*Mat.m[2];
    		return det;
  	}
  	if (r == 3) {
    		det = Mat.m[0]*Mat.m[4]*Mat.m[8] + Mat.m[1]*Mat.m[5]*Mat.m[6] + Mat.m[2]*Mat.m[3]*Mat.m[7]\
    			 -Mat.m[1]*Mat.m[3]*Mat.m[8] - Mat.m[0]*Mat.m[5]*Mat.m[7] - Mat.m[2]*Mat.m[4]*Mat.m[6];
    		return det;
  	}
  	if (r == 4) {	// determinante sull'ultima colonna dal basso in alto, 3x3 con Sarrus.
    		det = +Mat.m[15]*(Mat.m[0]*Mat.m[5]*Mat.m[10] + Mat.m[1]*Mat.m[6]*Mat.m[8] + Mat.m[2]*Mat.m[4]*Mat.m[9]\
    			   -Mat.m[1]*Mat.m[4]*Mat.m[10] - Mat.m[0]*Mat.m[6]*Mat.m[9] - Mat.m[2]*Mat.m[5]*Mat.m[8])\
    			  -Mat.m[11]*(Mat.m[0]*Mat.m[5]*Mat.m[14] + Mat.m[1]*Mat.m[6]*Mat.m[12] + Mat.m[2]*Mat.m[4]*Mat.m[13]\
    			   -Mat.m[1]*Mat.m[4]*Mat.m[14] - Mat.m[0]*Mat.m[6]*Mat.m[13] - Mat.m[2]*Mat.m[5]*Mat.m[12])\
    			  +Mat.m[7]*(Mat.m[0]*Mat.m[9]*Mat.m[14] + Mat.m[1]*Mat.m[10]*Mat.m[12] + Mat.m[2]*Mat.m[8]*Mat.m[13]\
    			   -Mat.m[1]*Mat.m[8]*Mat.m[14] - Mat.m[0]*Mat.m[10]*Mat.m[13] - Mat.m[2]*Mat.m[9]*Mat.m[12])\
    			  -Mat.m[3]*(Mat.m[4]*Mat.m[9]*Mat.m[14] + Mat.m[5]*Mat.m[10]*Mat.m[12] + Mat.m[6]*Mat.m[8]*Mat.m[13]\
    			   -Mat.m[5]*Mat.m[8]*Mat.m[14] - Mat.m[4]*Mat.m[10]*Mat.m[13] - Mat.m[6]*Mat.m[9]*Mat.m[12]);
    		return det;
  	}

}


//---------------------------------------------------------------------------
matrice	Trasposta(matrice A, matrice *B) {

	int i, j;

	if (A.righe != (*B).colonne || A.colonne != (*B).righe) {
		printf("\nTrasposta: Righe e colonne non corrispondono\n");
		exit(1);
	}

	for (i = 0; i < A.righe; i++) {
		for (j = 0; j < A.colonne; j++) {
			(*B).m[j*B->colonne + i] = A.m[i*A.colonne + j];
		}
	}
	return *B;
}


//---------------------------------------------------------------------------
matrice	Somma(matrice A, matrice B, matrice *C, int sgn) {

	int k = 1;
	int  i, j;

	if (A.righe != B.righe || A.colonne != B.colonne || A.righe != C->righe || A.colonne != C->colonne) {
		printf("\nSomma: Matrici incompatibili dimensionalmente\n");
		exit(1);
	}

	if (sgn == -1) k = -1;

	for (i = 0; i< A.righe; i++) {
		for (j = 0; j < A.colonne; j++) {
			*((*C).m + i*A.colonne + j) = *(A.m + i*A.colonne + j) + (float)k*(*(B.m + i*A.colonne + j));
		}
	}
	return *C;
}


//---------------------------------------------------------------------------
matrice	Prodotto(matrice A, matrice B, matrice *C) {

	int i, j, k;
	float C_temp, C_e;

	if (A.colonne != B.righe) {
		printf("\nProdotto: colonne e righe non corrispondenti\n");
		exit(1);
	}

	for (i = 0; i < A.righe; i++) {	// ciclo che scorre le righe di A
		for (j = 0; j < B.colonne; j++) {	// ciclo che scorre le colonne di B
			C_temp = 0;
			C_e = 0;
			for (k = 0; k < B.righe; k++) {	// ciclo che scorre gli elementi ed esegue
				C_temp = A.m[i*A.colonne +k] * B.m[k*B.colonne +j];	// prodotto scalare
				C_e += C_temp;
			}
			(*C).m[i*B.colonne +j] = C_e;
		}
	}
	return *C;
}


//---------------------------------------------------------------------------
matrice	Inversa(matrice A, matrice *B) {

	float det = Determinante(A);
	if (det <= 0.0001) {	// ci andrà <= di un numero basso
		printf("\nInversa: la matrice non è invertibile\n");
		exit(1);
	}

	// Algoritmo di Leverrier
	int i;
	int	n = A.righe;
	float t = pow(-1, n+1);
	float	p[n][n];
	float	ap[n][n];
	float	eye[n][n];
	matrice P  = {n, n, (float*)p};
	matrice AP = {n, n, (float*)ap};
	matrice I  = {n, n, (float*)eye};		// matrici di supporto
	P = Identita(&P, 1);					// inizializzo P e I a identità


	for (i = 1; i < n; i++) {	// algoritmo vero e proprio
		P = Somma(Identita(&I, (-Traccia(Prodotto(A, P, &AP)) / (float)i)), AP, &P, 0);
	}

	for (i = 0; i < n*n; i++) {	// P va divisa per det e cambiata di segno secondo la dimensione
		(*B).m[i] = t*P.m[i]/det;
	}
	return *B;
}


//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------
void	init_matrici(systems *sys, float *m[7]) {

	(sys->A).righe = DIM;
	(sys->A).colonne = DIM;
	(sys->A).m = m[0];
	(sys->B).righe = DIM;
	(sys->B).colonne = DIM;
	(sys->B).m = m[1];
	(sys->C).righe = DIM;
	(sys->C).colonne = DIM;
	(sys->C).m = m[2];
	(sys->Q).righe = DIM;
	(sys->Q).colonne = DIM;
	(sys->Q).m = m[3];
	(sys->R).righe = DIM;
	(sys->R).colonne = DIM;
	(sys->R).m = m[4];
	(sys->P).righe = DIM;
	(sys->P).colonne = DIM;
	(sys->P).m = m[5];
	(sys->K).righe = DIM;
	(sys->K).colonne = DIM;
	(sys->K).m = m[6];
}


//---------------------------------------------------------------------------
void	init_state(states *state, float *s[4]) {

	(state->X_k).righe = DIM;
	(state->X_k).colonne = 1;
	(state->X_k).m = s[0];
	(state->X_pred).righe = DIM;
	(state->X_pred).colonne = 1;
	(state->X_pred).m = s[1];
	(state->X_s).righe = DIM;
	(state->X_s).colonne = 1;
	(state->X_s).m = s[2];
	(state->X_ball).righe = DIM;
	(state->X_ball).colonne = 1;
	(state->X_ball).m = s[3];
}


//---------------------------------------------------------------------------
void	init_points(float *points) {

	int i;
	for (i = 0; i < DIM_POINTS; i++) {
		points[i] = 0;
	}
}


//---------------------------------------------------------------------------
void	init_points_pred(float *points_pred) {

	int i;
	for (i = 0; i < DIM_POINTS_PRED; i++) {
		points_pred[i] = 0;
	}
}


//---------------------------------------------------------------------------
void	aggiorna_points(float *points, states state) {

	int i;

	pthread_mutex_lock(&mux_state);
		float kalman_pos_x = state.X_k.m[0];
		float kalman_pos_y = state.X_k.m[1];
	pthread_mutex_unlock(&mux_state);

	for (i = 0; i < DIM_POINTS-2; i++) {
		points[i] = points[i+2];
	}

	if( kalman_pos_x > 800)
		kalman_pos_x = 800;
	if( kalman_pos_x < 0)
		kalman_pos_x = 0;
	if( kalman_pos_y > 600)
		kalman_pos_y = 600;
	if( kalman_pos_y < 0)
		kalman_pos_y = 0;
	points[DIM_POINTS-2] = kalman_pos_x;
	points[DIM_POINTS-1] = kalman_pos_y;
}


//---------------------------------------------------------------------------
void	init_struct(void) {

	init_matrici(&sys, m);
	init_state(&state, s);
	init_points(points);
	init_points_pred(points_pred);
}