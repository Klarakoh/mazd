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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3650691405938361009) {
   out_3650691405938361009[0] = delta_x[0] + nom_x[0];
   out_3650691405938361009[1] = delta_x[1] + nom_x[1];
   out_3650691405938361009[2] = delta_x[2] + nom_x[2];
   out_3650691405938361009[3] = delta_x[3] + nom_x[3];
   out_3650691405938361009[4] = delta_x[4] + nom_x[4];
   out_3650691405938361009[5] = delta_x[5] + nom_x[5];
   out_3650691405938361009[6] = delta_x[6] + nom_x[6];
   out_3650691405938361009[7] = delta_x[7] + nom_x[7];
   out_3650691405938361009[8] = delta_x[8] + nom_x[8];
   out_3650691405938361009[9] = delta_x[9] + nom_x[9];
   out_3650691405938361009[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_257155659230545517) {
   out_257155659230545517[0] = -nom_x[0] + true_x[0];
   out_257155659230545517[1] = -nom_x[1] + true_x[1];
   out_257155659230545517[2] = -nom_x[2] + true_x[2];
   out_257155659230545517[3] = -nom_x[3] + true_x[3];
   out_257155659230545517[4] = -nom_x[4] + true_x[4];
   out_257155659230545517[5] = -nom_x[5] + true_x[5];
   out_257155659230545517[6] = -nom_x[6] + true_x[6];
   out_257155659230545517[7] = -nom_x[7] + true_x[7];
   out_257155659230545517[8] = -nom_x[8] + true_x[8];
   out_257155659230545517[9] = -nom_x[9] + true_x[9];
   out_257155659230545517[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_24761517867664939) {
   out_24761517867664939[0] = 1.0;
   out_24761517867664939[1] = 0;
   out_24761517867664939[2] = 0;
   out_24761517867664939[3] = 0;
   out_24761517867664939[4] = 0;
   out_24761517867664939[5] = 0;
   out_24761517867664939[6] = 0;
   out_24761517867664939[7] = 0;
   out_24761517867664939[8] = 0;
   out_24761517867664939[9] = 0;
   out_24761517867664939[10] = 0;
   out_24761517867664939[11] = 0;
   out_24761517867664939[12] = 1.0;
   out_24761517867664939[13] = 0;
   out_24761517867664939[14] = 0;
   out_24761517867664939[15] = 0;
   out_24761517867664939[16] = 0;
   out_24761517867664939[17] = 0;
   out_24761517867664939[18] = 0;
   out_24761517867664939[19] = 0;
   out_24761517867664939[20] = 0;
   out_24761517867664939[21] = 0;
   out_24761517867664939[22] = 0;
   out_24761517867664939[23] = 0;
   out_24761517867664939[24] = 1.0;
   out_24761517867664939[25] = 0;
   out_24761517867664939[26] = 0;
   out_24761517867664939[27] = 0;
   out_24761517867664939[28] = 0;
   out_24761517867664939[29] = 0;
   out_24761517867664939[30] = 0;
   out_24761517867664939[31] = 0;
   out_24761517867664939[32] = 0;
   out_24761517867664939[33] = 0;
   out_24761517867664939[34] = 0;
   out_24761517867664939[35] = 0;
   out_24761517867664939[36] = 1.0;
   out_24761517867664939[37] = 0;
   out_24761517867664939[38] = 0;
   out_24761517867664939[39] = 0;
   out_24761517867664939[40] = 0;
   out_24761517867664939[41] = 0;
   out_24761517867664939[42] = 0;
   out_24761517867664939[43] = 0;
   out_24761517867664939[44] = 0;
   out_24761517867664939[45] = 0;
   out_24761517867664939[46] = 0;
   out_24761517867664939[47] = 0;
   out_24761517867664939[48] = 1.0;
   out_24761517867664939[49] = 0;
   out_24761517867664939[50] = 0;
   out_24761517867664939[51] = 0;
   out_24761517867664939[52] = 0;
   out_24761517867664939[53] = 0;
   out_24761517867664939[54] = 0;
   out_24761517867664939[55] = 0;
   out_24761517867664939[56] = 0;
   out_24761517867664939[57] = 0;
   out_24761517867664939[58] = 0;
   out_24761517867664939[59] = 0;
   out_24761517867664939[60] = 1.0;
   out_24761517867664939[61] = 0;
   out_24761517867664939[62] = 0;
   out_24761517867664939[63] = 0;
   out_24761517867664939[64] = 0;
   out_24761517867664939[65] = 0;
   out_24761517867664939[66] = 0;
   out_24761517867664939[67] = 0;
   out_24761517867664939[68] = 0;
   out_24761517867664939[69] = 0;
   out_24761517867664939[70] = 0;
   out_24761517867664939[71] = 0;
   out_24761517867664939[72] = 1.0;
   out_24761517867664939[73] = 0;
   out_24761517867664939[74] = 0;
   out_24761517867664939[75] = 0;
   out_24761517867664939[76] = 0;
   out_24761517867664939[77] = 0;
   out_24761517867664939[78] = 0;
   out_24761517867664939[79] = 0;
   out_24761517867664939[80] = 0;
   out_24761517867664939[81] = 0;
   out_24761517867664939[82] = 0;
   out_24761517867664939[83] = 0;
   out_24761517867664939[84] = 1.0;
   out_24761517867664939[85] = 0;
   out_24761517867664939[86] = 0;
   out_24761517867664939[87] = 0;
   out_24761517867664939[88] = 0;
   out_24761517867664939[89] = 0;
   out_24761517867664939[90] = 0;
   out_24761517867664939[91] = 0;
   out_24761517867664939[92] = 0;
   out_24761517867664939[93] = 0;
   out_24761517867664939[94] = 0;
   out_24761517867664939[95] = 0;
   out_24761517867664939[96] = 1.0;
   out_24761517867664939[97] = 0;
   out_24761517867664939[98] = 0;
   out_24761517867664939[99] = 0;
   out_24761517867664939[100] = 0;
   out_24761517867664939[101] = 0;
   out_24761517867664939[102] = 0;
   out_24761517867664939[103] = 0;
   out_24761517867664939[104] = 0;
   out_24761517867664939[105] = 0;
   out_24761517867664939[106] = 0;
   out_24761517867664939[107] = 0;
   out_24761517867664939[108] = 1.0;
   out_24761517867664939[109] = 0;
   out_24761517867664939[110] = 0;
   out_24761517867664939[111] = 0;
   out_24761517867664939[112] = 0;
   out_24761517867664939[113] = 0;
   out_24761517867664939[114] = 0;
   out_24761517867664939[115] = 0;
   out_24761517867664939[116] = 0;
   out_24761517867664939[117] = 0;
   out_24761517867664939[118] = 0;
   out_24761517867664939[119] = 0;
   out_24761517867664939[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2953610155191611994) {
   out_2953610155191611994[0] = dt*state[3] + state[0];
   out_2953610155191611994[1] = dt*state[4] + state[1];
   out_2953610155191611994[2] = dt*state[5] + state[2];
   out_2953610155191611994[3] = state[3];
   out_2953610155191611994[4] = state[4];
   out_2953610155191611994[5] = state[5];
   out_2953610155191611994[6] = dt*state[7] + state[6];
   out_2953610155191611994[7] = dt*state[8] + state[7];
   out_2953610155191611994[8] = state[8];
   out_2953610155191611994[9] = state[9];
   out_2953610155191611994[10] = state[10];
}
void F_fun(double *state, double dt, double *out_7769726719115423260) {
   out_7769726719115423260[0] = 1;
   out_7769726719115423260[1] = 0;
   out_7769726719115423260[2] = 0;
   out_7769726719115423260[3] = dt;
   out_7769726719115423260[4] = 0;
   out_7769726719115423260[5] = 0;
   out_7769726719115423260[6] = 0;
   out_7769726719115423260[7] = 0;
   out_7769726719115423260[8] = 0;
   out_7769726719115423260[9] = 0;
   out_7769726719115423260[10] = 0;
   out_7769726719115423260[11] = 0;
   out_7769726719115423260[12] = 1;
   out_7769726719115423260[13] = 0;
   out_7769726719115423260[14] = 0;
   out_7769726719115423260[15] = dt;
   out_7769726719115423260[16] = 0;
   out_7769726719115423260[17] = 0;
   out_7769726719115423260[18] = 0;
   out_7769726719115423260[19] = 0;
   out_7769726719115423260[20] = 0;
   out_7769726719115423260[21] = 0;
   out_7769726719115423260[22] = 0;
   out_7769726719115423260[23] = 0;
   out_7769726719115423260[24] = 1;
   out_7769726719115423260[25] = 0;
   out_7769726719115423260[26] = 0;
   out_7769726719115423260[27] = dt;
   out_7769726719115423260[28] = 0;
   out_7769726719115423260[29] = 0;
   out_7769726719115423260[30] = 0;
   out_7769726719115423260[31] = 0;
   out_7769726719115423260[32] = 0;
   out_7769726719115423260[33] = 0;
   out_7769726719115423260[34] = 0;
   out_7769726719115423260[35] = 0;
   out_7769726719115423260[36] = 1;
   out_7769726719115423260[37] = 0;
   out_7769726719115423260[38] = 0;
   out_7769726719115423260[39] = 0;
   out_7769726719115423260[40] = 0;
   out_7769726719115423260[41] = 0;
   out_7769726719115423260[42] = 0;
   out_7769726719115423260[43] = 0;
   out_7769726719115423260[44] = 0;
   out_7769726719115423260[45] = 0;
   out_7769726719115423260[46] = 0;
   out_7769726719115423260[47] = 0;
   out_7769726719115423260[48] = 1;
   out_7769726719115423260[49] = 0;
   out_7769726719115423260[50] = 0;
   out_7769726719115423260[51] = 0;
   out_7769726719115423260[52] = 0;
   out_7769726719115423260[53] = 0;
   out_7769726719115423260[54] = 0;
   out_7769726719115423260[55] = 0;
   out_7769726719115423260[56] = 0;
   out_7769726719115423260[57] = 0;
   out_7769726719115423260[58] = 0;
   out_7769726719115423260[59] = 0;
   out_7769726719115423260[60] = 1;
   out_7769726719115423260[61] = 0;
   out_7769726719115423260[62] = 0;
   out_7769726719115423260[63] = 0;
   out_7769726719115423260[64] = 0;
   out_7769726719115423260[65] = 0;
   out_7769726719115423260[66] = 0;
   out_7769726719115423260[67] = 0;
   out_7769726719115423260[68] = 0;
   out_7769726719115423260[69] = 0;
   out_7769726719115423260[70] = 0;
   out_7769726719115423260[71] = 0;
   out_7769726719115423260[72] = 1;
   out_7769726719115423260[73] = dt;
   out_7769726719115423260[74] = 0;
   out_7769726719115423260[75] = 0;
   out_7769726719115423260[76] = 0;
   out_7769726719115423260[77] = 0;
   out_7769726719115423260[78] = 0;
   out_7769726719115423260[79] = 0;
   out_7769726719115423260[80] = 0;
   out_7769726719115423260[81] = 0;
   out_7769726719115423260[82] = 0;
   out_7769726719115423260[83] = 0;
   out_7769726719115423260[84] = 1;
   out_7769726719115423260[85] = dt;
   out_7769726719115423260[86] = 0;
   out_7769726719115423260[87] = 0;
   out_7769726719115423260[88] = 0;
   out_7769726719115423260[89] = 0;
   out_7769726719115423260[90] = 0;
   out_7769726719115423260[91] = 0;
   out_7769726719115423260[92] = 0;
   out_7769726719115423260[93] = 0;
   out_7769726719115423260[94] = 0;
   out_7769726719115423260[95] = 0;
   out_7769726719115423260[96] = 1;
   out_7769726719115423260[97] = 0;
   out_7769726719115423260[98] = 0;
   out_7769726719115423260[99] = 0;
   out_7769726719115423260[100] = 0;
   out_7769726719115423260[101] = 0;
   out_7769726719115423260[102] = 0;
   out_7769726719115423260[103] = 0;
   out_7769726719115423260[104] = 0;
   out_7769726719115423260[105] = 0;
   out_7769726719115423260[106] = 0;
   out_7769726719115423260[107] = 0;
   out_7769726719115423260[108] = 1;
   out_7769726719115423260[109] = 0;
   out_7769726719115423260[110] = 0;
   out_7769726719115423260[111] = 0;
   out_7769726719115423260[112] = 0;
   out_7769726719115423260[113] = 0;
   out_7769726719115423260[114] = 0;
   out_7769726719115423260[115] = 0;
   out_7769726719115423260[116] = 0;
   out_7769726719115423260[117] = 0;
   out_7769726719115423260[118] = 0;
   out_7769726719115423260[119] = 0;
   out_7769726719115423260[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_507136575089039816) {
   out_507136575089039816[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_1421338190566676548) {
   out_1421338190566676548[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1421338190566676548[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1421338190566676548[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1421338190566676548[3] = 0;
   out_1421338190566676548[4] = 0;
   out_1421338190566676548[5] = 0;
   out_1421338190566676548[6] = 1;
   out_1421338190566676548[7] = 0;
   out_1421338190566676548[8] = 0;
   out_1421338190566676548[9] = 0;
   out_1421338190566676548[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_2353567699693444253) {
   out_2353567699693444253[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5340928639051498153) {
   out_5340928639051498153[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5340928639051498153[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5340928639051498153[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5340928639051498153[3] = 0;
   out_5340928639051498153[4] = 0;
   out_5340928639051498153[5] = 0;
   out_5340928639051498153[6] = 1;
   out_5340928639051498153[7] = 0;
   out_5340928639051498153[8] = 0;
   out_5340928639051498153[9] = 1;
   out_5340928639051498153[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_8644793348608052699) {
   out_8644793348608052699[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_1931195383238368014) {
   out_1931195383238368014[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[6] = 0;
   out_1931195383238368014[7] = 1;
   out_1931195383238368014[8] = 0;
   out_1931195383238368014[9] = 0;
   out_1931195383238368014[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_8644793348608052699) {
   out_8644793348608052699[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_1931195383238368014) {
   out_1931195383238368014[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1931195383238368014[6] = 0;
   out_1931195383238368014[7] = 1;
   out_1931195383238368014[8] = 0;
   out_1931195383238368014[9] = 0;
   out_1931195383238368014[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3650691405938361009) {
  err_fun(nom_x, delta_x, out_3650691405938361009);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_257155659230545517) {
  inv_err_fun(nom_x, true_x, out_257155659230545517);
}
void gnss_H_mod_fun(double *state, double *out_24761517867664939) {
  H_mod_fun(state, out_24761517867664939);
}
void gnss_f_fun(double *state, double dt, double *out_2953610155191611994) {
  f_fun(state,  dt, out_2953610155191611994);
}
void gnss_F_fun(double *state, double dt, double *out_7769726719115423260) {
  F_fun(state,  dt, out_7769726719115423260);
}
void gnss_h_6(double *state, double *sat_pos, double *out_507136575089039816) {
  h_6(state, sat_pos, out_507136575089039816);
}
void gnss_H_6(double *state, double *sat_pos, double *out_1421338190566676548) {
  H_6(state, sat_pos, out_1421338190566676548);
}
void gnss_h_20(double *state, double *sat_pos, double *out_2353567699693444253) {
  h_20(state, sat_pos, out_2353567699693444253);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5340928639051498153) {
  H_20(state, sat_pos, out_5340928639051498153);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8644793348608052699) {
  h_7(state, sat_pos_vel, out_8644793348608052699);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1931195383238368014) {
  H_7(state, sat_pos_vel, out_1931195383238368014);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8644793348608052699) {
  h_21(state, sat_pos_vel, out_8644793348608052699);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1931195383238368014) {
  H_21(state, sat_pos_vel, out_1931195383238368014);
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
