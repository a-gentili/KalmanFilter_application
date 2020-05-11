#ifndef GRAPHIC_H_
#define GRAPHIC_H_

//---------------------------------------------------------------------------
// Define
//---------------------------------------------------------------------------
#define		R_TB_W			224   // right toolbar width
#define 	R_TB_H  		600   // right toolbar height
#define 	B_TB_W  		1024  // bottom toolbar width
#define 	B_TB_H  		168   // bottom toolbar height
#define 	CELL_H			25	  // matrix cell high
#define 	CELL_W			50	  // matrix cell width
#define 	UL_X			30	  // upper left matrix point x
#define 	UL_Y			35    // upper left matrix point y
#define 	UL_CELL_GAP_X 	5    // 
#define 	UL_CELL_GAP_Y 	10
#define 	MATRIX_GAP		250	  // gap between matrices
#define 	MAT_ID_X		10	  // matrix index x coord 
#define 	MAT_ID_Y		82	  // matrix index y coord
#define 	MAT_DES_Y 		150   // y coord of the matrix description text
#define 	FRAME_W 		800   // kalman filter frame width
#define 	FRAME_H 		600   // kalman filter frame height
#define 	COLOR_PINK 		5
#define 	COLOR_RED 		4
#define 	COLOR_YELLOW	14 
#define 	COLOR_BLACK 	0
#define 	COLOR_WHITE		15
#define 	CANCEL 			0 
#define 	_GNU_SOURCE


//---------------------------------------------------------------------------
// Global variables declarations
//---------------------------------------------------------------------------
extern BITMAP *right_toolbar;
extern BITMAP *bottom_toolbar;

extern 	float 	points_pred_old[DIM_POINTS_PRED];


//---------------------------------------------------------------------------
// Function declarations
//---------------------------------------------------------------------------
void	draw_trail();
void	draw_right_toolbar();
void	draw_console();
void 	draw_btns();
void	draw_sliders();
void	draw_bottom_toolbar();
void	draw_matrix_C();
void	draw_matrix_Q();
void	draw_matrix_R();
void	draw_matrix_A();

#endif /* GRAPHIC_H_ */