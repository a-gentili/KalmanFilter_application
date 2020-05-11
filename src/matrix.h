#ifndef MATRIX_H_
#define MATRIX_H_

//---------------------------------------------------------------------------
// Define
//---------------------------------------------------------------------------
#define F_CONST_X			0		// constant force on x
#define F_CONST_Y			0		// constant force on y
#define F_X 				10		// magnitude ceil force x
#define F_Y 				10		// magnitude ceil force y
#define M					1 		// ball mass
#define T					0.015	// period of kalman task, sensor task and relative matrices
#define S					0.010	// period of ball task and relative matrices
#define CEIL_COUNT			750		// CEIL_COUNT*S: milliseconds acting time of the random force

#define	DIM_POINTS			40		// 20 points
#define	DIM_POINTS_PRED		200
#define DIM 				4
#define R_BALL				8



//---------------------------------------------------------------------------
// Typedef variables definition
//---------------------------------------------------------------------------
typedef struct matrice {

	int     righe;
 	int     colonne;
 	float   *m;	// pointer to first element of matrix
} matrice;

typedef struct systems {

	matrice A;
	matrice B;
	matrice C;
	matrice Q;
	matrice R;
	matrice P;
	matrice K;
} systems;

typedef	struct	states {

	matrice X_k;
	matrice	X_pred;
	matrice	X_s;
	matrice X_ball;
} states;


//---------------------------------------------------------------------------
// Global variables declarations
//---------------------------------------------------------------------------
extern 	float 	var_noise[2];
extern 	float 	points[DIM_POINTS];
extern 	float 	points_pred[DIM_POINTS_PRED];
extern 	float 	points_pred_old[DIM_POINTS_PRED];
extern  matrice U;
extern 	systems sys;
extern 	states 	state;
extern 	matrice	A_b;
extern 	matrice	B_b;


//---------------------------------------------------------------------------
// Function declarations
//---------------------------------------------------------------------------
matrice		Copia (matrice A, matrice *B);
matrice		Identita (matrice *Mat, float cost);
matrice		Trasposta (matrice A, matrice *B);
matrice		Somma (matrice A, matrice B, matrice *C, int sgn);
matrice		Prodotto (matrice A, matrice B, matrice *C);
matrice		Inversa (matrice A, matrice *B);

void 	aggiorna_points(float *points, states state);
void 	init_struct(void);

#endif /* MATRIX_H_ */