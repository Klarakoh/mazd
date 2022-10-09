#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6315413778743344648) {
   out_6315413778743344648[0] = delta_x[0] + nom_x[0];
   out_6315413778743344648[1] = delta_x[1] + nom_x[1];
   out_6315413778743344648[2] = delta_x[2] + nom_x[2];
   out_6315413778743344648[3] = delta_x[3] + nom_x[3];
   out_6315413778743344648[4] = delta_x[4] + nom_x[4];
   out_6315413778743344648[5] = delta_x[5] + nom_x[5];
   out_6315413778743344648[6] = delta_x[6] + nom_x[6];
   out_6315413778743344648[7] = delta_x[7] + nom_x[7];
   out_6315413778743344648[8] = delta_x[8] + nom_x[8];
   out_6315413778743344648[9] = delta_x[9] + nom_x[9];
   out_6315413778743344648[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8827564806774451557) {
   out_8827564806774451557[0] = -nom_x[0] + true_x[0];
   out_8827564806774451557[1] = -nom_x[1] + true_x[1];
   out_8827564806774451557[2] = -nom_x[2] + true_x[2];
   out_8827564806774451557[3] = -nom_x[3] + true_x[3];
   out_8827564806774451557[4] = -nom_x[4] + true_x[4];
   out_8827564806774451557[5] = -nom_x[5] + true_x[5];
   out_8827564806774451557[6] = -nom_x[6] + true_x[6];
   out_8827564806774451557[7] = -nom_x[7] + true_x[7];
   out_8827564806774451557[8] = -nom_x[8] + true_x[8];
   out_8827564806774451557[9] = -nom_x[9] + true_x[9];
   out_8827564806774451557[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_6144059787071489127) {
   out_6144059787071489127[0] = 1.0;
   out_6144059787071489127[1] = 0;
   out_6144059787071489127[2] = 0;
   out_6144059787071489127[3] = 0;
   out_6144059787071489127[4] = 0;
   out_6144059787071489127[5] = 0;
   out_6144059787071489127[6] = 0;
   out_6144059787071489127[7] = 0;
   out_6144059787071489127[8] = 0;
   out_6144059787071489127[9] = 0;
   out_6144059787071489127[10] = 0;
   out_6144059787071489127[11] = 0;
   out_6144059787071489127[12] = 1.0;
   out_6144059787071489127[13] = 0;
   out_6144059787071489127[14] = 0;
   out_6144059787071489127[15] = 0;
   out_6144059787071489127[16] = 0;
   out_6144059787071489127[17] = 0;
   out_6144059787071489127[18] = 0;
   out_6144059787071489127[19] = 0;
   out_6144059787071489127[20] = 0;
   out_6144059787071489127[21] = 0;
   out_6144059787071489127[22] = 0;
   out_6144059787071489127[23] = 0;
   out_6144059787071489127[24] = 1.0;
   out_6144059787071489127[25] = 0;
   out_6144059787071489127[26] = 0;
   out_6144059787071489127[27] = 0;
   out_6144059787071489127[28] = 0;
   out_6144059787071489127[29] = 0;
   out_6144059787071489127[30] = 0;
   out_6144059787071489127[31] = 0;
   out_6144059787071489127[32] = 0;
   out_6144059787071489127[33] = 0;
   out_6144059787071489127[34] = 0;
   out_6144059787071489127[35] = 0;
   out_6144059787071489127[36] = 1.0;
   out_6144059787071489127[37] = 0;
   out_6144059787071489127[38] = 0;
   out_6144059787071489127[39] = 0;
   out_6144059787071489127[40] = 0;
   out_6144059787071489127[41] = 0;
   out_6144059787071489127[42] = 0;
   out_6144059787071489127[43] = 0;
   out_6144059787071489127[44] = 0;
   out_6144059787071489127[45] = 0;
   out_6144059787071489127[46] = 0;
   out_6144059787071489127[47] = 0;
   out_6144059787071489127[48] = 1.0;
   out_6144059787071489127[49] = 0;
   out_6144059787071489127[50] = 0;
   out_6144059787071489127[51] = 0;
   out_6144059787071489127[52] = 0;
   out_6144059787071489127[53] = 0;
   out_6144059787071489127[54] = 0;
   out_6144059787071489127[55] = 0;
   out_6144059787071489127[56] = 0;
   out_6144059787071489127[57] = 0;
   out_6144059787071489127[58] = 0;
   out_6144059787071489127[59] = 0;
   out_6144059787071489127[60] = 1.0;
   out_6144059787071489127[61] = 0;
   out_6144059787071489127[62] = 0;
   out_6144059787071489127[63] = 0;
   out_6144059787071489127[64] = 0;
   out_6144059787071489127[65] = 0;
   out_6144059787071489127[66] = 0;
   out_6144059787071489127[67] = 0;
   out_6144059787071489127[68] = 0;
   out_6144059787071489127[69] = 0;
   out_6144059787071489127[70] = 0;
   out_6144059787071489127[71] = 0;
   out_6144059787071489127[72] = 1.0;
   out_6144059787071489127[73] = 0;
   out_6144059787071489127[74] = 0;
   out_6144059787071489127[75] = 0;
   out_6144059787071489127[76] = 0;
   out_6144059787071489127[77] = 0;
   out_6144059787071489127[78] = 0;
   out_6144059787071489127[79] = 0;
   out_6144059787071489127[80] = 0;
   out_6144059787071489127[81] = 0;
   out_6144059787071489127[82] = 0;
   out_6144059787071489127[83] = 0;
   out_6144059787071489127[84] = 1.0;
   out_6144059787071489127[85] = 0;
   out_6144059787071489127[86] = 0;
   out_6144059787071489127[87] = 0;
   out_6144059787071489127[88] = 0;
   out_6144059787071489127[89] = 0;
   out_6144059787071489127[90] = 0;
   out_6144059787071489127[91] = 0;
   out_6144059787071489127[92] = 0;
   out_6144059787071489127[93] = 0;
   out_6144059787071489127[94] = 0;
   out_6144059787071489127[95] = 0;
   out_6144059787071489127[96] = 1.0;
   out_6144059787071489127[97] = 0;
   out_6144059787071489127[98] = 0;
   out_6144059787071489127[99] = 0;
   out_6144059787071489127[100] = 0;
   out_6144059787071489127[101] = 0;
   out_6144059787071489127[102] = 0;
   out_6144059787071489127[103] = 0;
   out_6144059787071489127[104] = 0;
   out_6144059787071489127[105] = 0;
   out_6144059787071489127[106] = 0;
   out_6144059787071489127[107] = 0;
   out_6144059787071489127[108] = 1.0;
   out_6144059787071489127[109] = 0;
   out_6144059787071489127[110] = 0;
   out_6144059787071489127[111] = 0;
   out_6144059787071489127[112] = 0;
   out_6144059787071489127[113] = 0;
   out_6144059787071489127[114] = 0;
   out_6144059787071489127[115] = 0;
   out_6144059787071489127[116] = 0;
   out_6144059787071489127[117] = 0;
   out_6144059787071489127[118] = 0;
   out_6144059787071489127[119] = 0;
   out_6144059787071489127[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2066965521722786900) {
   out_2066965521722786900[0] = dt*state[3] + state[0];
   out_2066965521722786900[1] = dt*state[4] + state[1];
   out_2066965521722786900[2] = dt*state[5] + state[2];
   out_2066965521722786900[3] = state[3];
   out_2066965521722786900[4] = state[4];
   out_2066965521722786900[5] = state[5];
   out_2066965521722786900[6] = dt*state[7] + state[6];
   out_2066965521722786900[7] = dt*state[8] + state[7];
   out_2066965521722786900[8] = state[8];
   out_2066965521722786900[9] = state[9];
   out_2066965521722786900[10] = state[10];
}
void F_fun(double *state, double dt, double *out_4120121182446732345) {
   out_4120121182446732345[0] = 1;
   out_4120121182446732345[1] = 0;
   out_4120121182446732345[2] = 0;
   out_4120121182446732345[3] = dt;
   out_4120121182446732345[4] = 0;
   out_4120121182446732345[5] = 0;
   out_4120121182446732345[6] = 0;
   out_4120121182446732345[7] = 0;
   out_4120121182446732345[8] = 0;
   out_4120121182446732345[9] = 0;
   out_4120121182446732345[10] = 0;
   out_4120121182446732345[11] = 0;
   out_4120121182446732345[12] = 1;
   out_4120121182446732345[13] = 0;
   out_4120121182446732345[14] = 0;
   out_4120121182446732345[15] = dt;
   out_4120121182446732345[16] = 0;
   out_4120121182446732345[17] = 0;
   out_4120121182446732345[18] = 0;
   out_4120121182446732345[19] = 0;
   out_4120121182446732345[20] = 0;
   out_4120121182446732345[21] = 0;
   out_4120121182446732345[22] = 0;
   out_4120121182446732345[23] = 0;
   out_4120121182446732345[24] = 1;
   out_4120121182446732345[25] = 0;
   out_4120121182446732345[26] = 0;
   out_4120121182446732345[27] = dt;
   out_4120121182446732345[28] = 0;
   out_4120121182446732345[29] = 0;
   out_4120121182446732345[30] = 0;
   out_4120121182446732345[31] = 0;
   out_4120121182446732345[32] = 0;
   out_4120121182446732345[33] = 0;
   out_4120121182446732345[34] = 0;
   out_4120121182446732345[35] = 0;
   out_4120121182446732345[36] = 1;
   out_4120121182446732345[37] = 0;
   out_4120121182446732345[38] = 0;
   out_4120121182446732345[39] = 0;
   out_4120121182446732345[40] = 0;
   out_4120121182446732345[41] = 0;
   out_4120121182446732345[42] = 0;
   out_4120121182446732345[43] = 0;
   out_4120121182446732345[44] = 0;
   out_4120121182446732345[45] = 0;
   out_4120121182446732345[46] = 0;
   out_4120121182446732345[47] = 0;
   out_4120121182446732345[48] = 1;
   out_4120121182446732345[49] = 0;
   out_4120121182446732345[50] = 0;
   out_4120121182446732345[51] = 0;
   out_4120121182446732345[52] = 0;
   out_4120121182446732345[53] = 0;
   out_4120121182446732345[54] = 0;
   out_4120121182446732345[55] = 0;
   out_4120121182446732345[56] = 0;
   out_4120121182446732345[57] = 0;
   out_4120121182446732345[58] = 0;
   out_4120121182446732345[59] = 0;
   out_4120121182446732345[60] = 1;
   out_4120121182446732345[61] = 0;
   out_4120121182446732345[62] = 0;
   out_4120121182446732345[63] = 0;
   out_4120121182446732345[64] = 0;
   out_4120121182446732345[65] = 0;
   out_4120121182446732345[66] = 0;
   out_4120121182446732345[67] = 0;
   out_4120121182446732345[68] = 0;
   out_4120121182446732345[69] = 0;
   out_4120121182446732345[70] = 0;
   out_4120121182446732345[71] = 0;
   out_4120121182446732345[72] = 1;
   out_4120121182446732345[73] = dt;
   out_4120121182446732345[74] = 0;
   out_4120121182446732345[75] = 0;
   out_4120121182446732345[76] = 0;
   out_4120121182446732345[77] = 0;
   out_4120121182446732345[78] = 0;
   out_4120121182446732345[79] = 0;
   out_4120121182446732345[80] = 0;
   out_4120121182446732345[81] = 0;
   out_4120121182446732345[82] = 0;
   out_4120121182446732345[83] = 0;
   out_4120121182446732345[84] = 1;
   out_4120121182446732345[85] = dt;
   out_4120121182446732345[86] = 0;
   out_4120121182446732345[87] = 0;
   out_4120121182446732345[88] = 0;
   out_4120121182446732345[89] = 0;
   out_4120121182446732345[90] = 0;
   out_4120121182446732345[91] = 0;
   out_4120121182446732345[92] = 0;
   out_4120121182446732345[93] = 0;
   out_4120121182446732345[94] = 0;
   out_4120121182446732345[95] = 0;
   out_4120121182446732345[96] = 1;
   out_4120121182446732345[97] = 0;
   out_4120121182446732345[98] = 0;
   out_4120121182446732345[99] = 0;
   out_4120121182446732345[100] = 0;
   out_4120121182446732345[101] = 0;
   out_4120121182446732345[102] = 0;
   out_4120121182446732345[103] = 0;
   out_4120121182446732345[104] = 0;
   out_4120121182446732345[105] = 0;
   out_4120121182446732345[106] = 0;
   out_4120121182446732345[107] = 0;
   out_4120121182446732345[108] = 1;
   out_4120121182446732345[109] = 0;
   out_4120121182446732345[110] = 0;
   out_4120121182446732345[111] = 0;
   out_4120121182446732345[112] = 0;
   out_4120121182446732345[113] = 0;
   out_4120121182446732345[114] = 0;
   out_4120121182446732345[115] = 0;
   out_4120121182446732345[116] = 0;
   out_4120121182446732345[117] = 0;
   out_4120121182446732345[118] = 0;
   out_4120121182446732345[119] = 0;
   out_4120121182446732345[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_6843245966466803220) {
   out_6843245966466803220[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_8758283351134468461) {
   out_8758283351134468461[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8758283351134468461[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8758283351134468461[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8758283351134468461[3] = 0;
   out_8758283351134468461[4] = 0;
   out_8758283351134468461[5] = 0;
   out_8758283351134468461[6] = 1;
   out_8758283351134468461[7] = 0;
   out_8758283351134468461[8] = 0;
   out_8758283351134468461[9] = 0;
   out_8758283351134468461[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_8951694425031887835) {
   out_8951694425031887835[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_7311235469948348987) {
   out_7311235469948348987[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7311235469948348987[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7311235469948348987[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7311235469948348987[3] = 0;
   out_7311235469948348987[4] = 0;
   out_7311235469948348987[5] = 0;
   out_7311235469948348987[6] = 1;
   out_7311235469948348987[7] = 0;
   out_7311235469948348987[8] = 0;
   out_7311235469948348987[9] = 1;
   out_7311235469948348987[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_4334617316991746036) {
   out_4334617316991746036[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_390293331376768139) {
   out_390293331376768139[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[6] = 0;
   out_390293331376768139[7] = 1;
   out_390293331376768139[8] = 0;
   out_390293331376768139[9] = 0;
   out_390293331376768139[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_4334617316991746036) {
   out_4334617316991746036[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_390293331376768139) {
   out_390293331376768139[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_390293331376768139[6] = 0;
   out_390293331376768139[7] = 1;
   out_390293331376768139[8] = 0;
   out_390293331376768139[9] = 0;
   out_390293331376768139[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6315413778743344648) {
  err_fun(nom_x, delta_x, out_6315413778743344648);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8827564806774451557) {
  inv_err_fun(nom_x, true_x, out_8827564806774451557);
}
void gnss_H_mod_fun(double *state, double *out_6144059787071489127) {
  H_mod_fun(state, out_6144059787071489127);
}
void gnss_f_fun(double *state, double dt, double *out_2066965521722786900) {
  f_fun(state,  dt, out_2066965521722786900);
}
void gnss_F_fun(double *state, double dt, double *out_4120121182446732345) {
  F_fun(state,  dt, out_4120121182446732345);
}
void gnss_h_6(double *state, double *sat_pos, double *out_6843245966466803220) {
  h_6(state, sat_pos, out_6843245966466803220);
}
void gnss_H_6(double *state, double *sat_pos, double *out_8758283351134468461) {
  H_6(state, sat_pos, out_8758283351134468461);
}
void gnss_h_20(double *state, double *sat_pos, double *out_8951694425031887835) {
  h_20(state, sat_pos, out_8951694425031887835);
}
void gnss_H_20(double *state, double *sat_pos, double *out_7311235469948348987) {
  H_20(state, sat_pos, out_7311235469948348987);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4334617316991746036) {
  h_7(state, sat_pos_vel, out_4334617316991746036);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_390293331376768139) {
  H_7(state, sat_pos_vel, out_390293331376768139);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4334617316991746036) {
  h_21(state, sat_pos_vel, out_4334617316991746036);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_390293331376768139) {
  H_21(state, sat_pos_vel, out_390293331376768139);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
