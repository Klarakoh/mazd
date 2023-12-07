#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7958337668253159685) {
   out_7958337668253159685[0] = delta_x[0] + nom_x[0];
   out_7958337668253159685[1] = delta_x[1] + nom_x[1];
   out_7958337668253159685[2] = delta_x[2] + nom_x[2];
   out_7958337668253159685[3] = delta_x[3] + nom_x[3];
   out_7958337668253159685[4] = delta_x[4] + nom_x[4];
   out_7958337668253159685[5] = delta_x[5] + nom_x[5];
   out_7958337668253159685[6] = delta_x[6] + nom_x[6];
   out_7958337668253159685[7] = delta_x[7] + nom_x[7];
   out_7958337668253159685[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_759863855661418044) {
   out_759863855661418044[0] = -nom_x[0] + true_x[0];
   out_759863855661418044[1] = -nom_x[1] + true_x[1];
   out_759863855661418044[2] = -nom_x[2] + true_x[2];
   out_759863855661418044[3] = -nom_x[3] + true_x[3];
   out_759863855661418044[4] = -nom_x[4] + true_x[4];
   out_759863855661418044[5] = -nom_x[5] + true_x[5];
   out_759863855661418044[6] = -nom_x[6] + true_x[6];
   out_759863855661418044[7] = -nom_x[7] + true_x[7];
   out_759863855661418044[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2543812563470930545) {
   out_2543812563470930545[0] = 1.0;
   out_2543812563470930545[1] = 0;
   out_2543812563470930545[2] = 0;
   out_2543812563470930545[3] = 0;
   out_2543812563470930545[4] = 0;
   out_2543812563470930545[5] = 0;
   out_2543812563470930545[6] = 0;
   out_2543812563470930545[7] = 0;
   out_2543812563470930545[8] = 0;
   out_2543812563470930545[9] = 0;
   out_2543812563470930545[10] = 1.0;
   out_2543812563470930545[11] = 0;
   out_2543812563470930545[12] = 0;
   out_2543812563470930545[13] = 0;
   out_2543812563470930545[14] = 0;
   out_2543812563470930545[15] = 0;
   out_2543812563470930545[16] = 0;
   out_2543812563470930545[17] = 0;
   out_2543812563470930545[18] = 0;
   out_2543812563470930545[19] = 0;
   out_2543812563470930545[20] = 1.0;
   out_2543812563470930545[21] = 0;
   out_2543812563470930545[22] = 0;
   out_2543812563470930545[23] = 0;
   out_2543812563470930545[24] = 0;
   out_2543812563470930545[25] = 0;
   out_2543812563470930545[26] = 0;
   out_2543812563470930545[27] = 0;
   out_2543812563470930545[28] = 0;
   out_2543812563470930545[29] = 0;
   out_2543812563470930545[30] = 1.0;
   out_2543812563470930545[31] = 0;
   out_2543812563470930545[32] = 0;
   out_2543812563470930545[33] = 0;
   out_2543812563470930545[34] = 0;
   out_2543812563470930545[35] = 0;
   out_2543812563470930545[36] = 0;
   out_2543812563470930545[37] = 0;
   out_2543812563470930545[38] = 0;
   out_2543812563470930545[39] = 0;
   out_2543812563470930545[40] = 1.0;
   out_2543812563470930545[41] = 0;
   out_2543812563470930545[42] = 0;
   out_2543812563470930545[43] = 0;
   out_2543812563470930545[44] = 0;
   out_2543812563470930545[45] = 0;
   out_2543812563470930545[46] = 0;
   out_2543812563470930545[47] = 0;
   out_2543812563470930545[48] = 0;
   out_2543812563470930545[49] = 0;
   out_2543812563470930545[50] = 1.0;
   out_2543812563470930545[51] = 0;
   out_2543812563470930545[52] = 0;
   out_2543812563470930545[53] = 0;
   out_2543812563470930545[54] = 0;
   out_2543812563470930545[55] = 0;
   out_2543812563470930545[56] = 0;
   out_2543812563470930545[57] = 0;
   out_2543812563470930545[58] = 0;
   out_2543812563470930545[59] = 0;
   out_2543812563470930545[60] = 1.0;
   out_2543812563470930545[61] = 0;
   out_2543812563470930545[62] = 0;
   out_2543812563470930545[63] = 0;
   out_2543812563470930545[64] = 0;
   out_2543812563470930545[65] = 0;
   out_2543812563470930545[66] = 0;
   out_2543812563470930545[67] = 0;
   out_2543812563470930545[68] = 0;
   out_2543812563470930545[69] = 0;
   out_2543812563470930545[70] = 1.0;
   out_2543812563470930545[71] = 0;
   out_2543812563470930545[72] = 0;
   out_2543812563470930545[73] = 0;
   out_2543812563470930545[74] = 0;
   out_2543812563470930545[75] = 0;
   out_2543812563470930545[76] = 0;
   out_2543812563470930545[77] = 0;
   out_2543812563470930545[78] = 0;
   out_2543812563470930545[79] = 0;
   out_2543812563470930545[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2371139363969154978) {
   out_2371139363969154978[0] = state[0];
   out_2371139363969154978[1] = state[1];
   out_2371139363969154978[2] = state[2];
   out_2371139363969154978[3] = state[3];
   out_2371139363969154978[4] = state[4];
   out_2371139363969154978[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2371139363969154978[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2371139363969154978[7] = state[7];
   out_2371139363969154978[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5819561008987145557) {
   out_5819561008987145557[0] = 1;
   out_5819561008987145557[1] = 0;
   out_5819561008987145557[2] = 0;
   out_5819561008987145557[3] = 0;
   out_5819561008987145557[4] = 0;
   out_5819561008987145557[5] = 0;
   out_5819561008987145557[6] = 0;
   out_5819561008987145557[7] = 0;
   out_5819561008987145557[8] = 0;
   out_5819561008987145557[9] = 0;
   out_5819561008987145557[10] = 1;
   out_5819561008987145557[11] = 0;
   out_5819561008987145557[12] = 0;
   out_5819561008987145557[13] = 0;
   out_5819561008987145557[14] = 0;
   out_5819561008987145557[15] = 0;
   out_5819561008987145557[16] = 0;
   out_5819561008987145557[17] = 0;
   out_5819561008987145557[18] = 0;
   out_5819561008987145557[19] = 0;
   out_5819561008987145557[20] = 1;
   out_5819561008987145557[21] = 0;
   out_5819561008987145557[22] = 0;
   out_5819561008987145557[23] = 0;
   out_5819561008987145557[24] = 0;
   out_5819561008987145557[25] = 0;
   out_5819561008987145557[26] = 0;
   out_5819561008987145557[27] = 0;
   out_5819561008987145557[28] = 0;
   out_5819561008987145557[29] = 0;
   out_5819561008987145557[30] = 1;
   out_5819561008987145557[31] = 0;
   out_5819561008987145557[32] = 0;
   out_5819561008987145557[33] = 0;
   out_5819561008987145557[34] = 0;
   out_5819561008987145557[35] = 0;
   out_5819561008987145557[36] = 0;
   out_5819561008987145557[37] = 0;
   out_5819561008987145557[38] = 0;
   out_5819561008987145557[39] = 0;
   out_5819561008987145557[40] = 1;
   out_5819561008987145557[41] = 0;
   out_5819561008987145557[42] = 0;
   out_5819561008987145557[43] = 0;
   out_5819561008987145557[44] = 0;
   out_5819561008987145557[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5819561008987145557[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5819561008987145557[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5819561008987145557[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5819561008987145557[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5819561008987145557[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5819561008987145557[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5819561008987145557[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5819561008987145557[53] = -9.8000000000000007*dt;
   out_5819561008987145557[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5819561008987145557[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5819561008987145557[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5819561008987145557[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5819561008987145557[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5819561008987145557[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5819561008987145557[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5819561008987145557[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5819561008987145557[62] = 0;
   out_5819561008987145557[63] = 0;
   out_5819561008987145557[64] = 0;
   out_5819561008987145557[65] = 0;
   out_5819561008987145557[66] = 0;
   out_5819561008987145557[67] = 0;
   out_5819561008987145557[68] = 0;
   out_5819561008987145557[69] = 0;
   out_5819561008987145557[70] = 1;
   out_5819561008987145557[71] = 0;
   out_5819561008987145557[72] = 0;
   out_5819561008987145557[73] = 0;
   out_5819561008987145557[74] = 0;
   out_5819561008987145557[75] = 0;
   out_5819561008987145557[76] = 0;
   out_5819561008987145557[77] = 0;
   out_5819561008987145557[78] = 0;
   out_5819561008987145557[79] = 0;
   out_5819561008987145557[80] = 1;
}
void h_25(double *state, double *unused, double *out_9188288045972936111) {
   out_9188288045972936111[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4976393246024881055) {
   out_4976393246024881055[0] = 0;
   out_4976393246024881055[1] = 0;
   out_4976393246024881055[2] = 0;
   out_4976393246024881055[3] = 0;
   out_4976393246024881055[4] = 0;
   out_4976393246024881055[5] = 0;
   out_4976393246024881055[6] = 1;
   out_4976393246024881055[7] = 0;
   out_4976393246024881055[8] = 0;
}
void h_24(double *state, double *unused, double *out_915206111334749955) {
   out_915206111334749955[0] = state[4];
   out_915206111334749955[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7149042845030380621) {
   out_7149042845030380621[0] = 0;
   out_7149042845030380621[1] = 0;
   out_7149042845030380621[2] = 0;
   out_7149042845030380621[3] = 0;
   out_7149042845030380621[4] = 1;
   out_7149042845030380621[5] = 0;
   out_7149042845030380621[6] = 0;
   out_7149042845030380621[7] = 0;
   out_7149042845030380621[8] = 0;
   out_7149042845030380621[9] = 0;
   out_7149042845030380621[10] = 0;
   out_7149042845030380621[11] = 0;
   out_7149042845030380621[12] = 0;
   out_7149042845030380621[13] = 0;
   out_7149042845030380621[14] = 1;
   out_7149042845030380621[15] = 0;
   out_7149042845030380621[16] = 0;
   out_7149042845030380621[17] = 0;
}
void h_30(double *state, double *unused, double *out_548410569254394227) {
   out_548410569254394227[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2458060287517632428) {
   out_2458060287517632428[0] = 0;
   out_2458060287517632428[1] = 0;
   out_2458060287517632428[2] = 0;
   out_2458060287517632428[3] = 0;
   out_2458060287517632428[4] = 1;
   out_2458060287517632428[5] = 0;
   out_2458060287517632428[6] = 0;
   out_2458060287517632428[7] = 0;
   out_2458060287517632428[8] = 0;
}
void h_26(double *state, double *unused, double *out_4453262698406451418) {
   out_4453262698406451418[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8717896564898937279) {
   out_8717896564898937279[0] = 0;
   out_8717896564898937279[1] = 0;
   out_8717896564898937279[2] = 0;
   out_8717896564898937279[3] = 0;
   out_8717896564898937279[4] = 0;
   out_8717896564898937279[5] = 0;
   out_8717896564898937279[6] = 0;
   out_8717896564898937279[7] = 1;
   out_8717896564898937279[8] = 0;
}
void h_27(double *state, double *unused, double *out_6375735942703082728) {
   out_6375735942703082728[0] = state[3];
}
void H_27(double *state, double *unused, double *out_234466216333689211) {
   out_234466216333689211[0] = 0;
   out_234466216333689211[1] = 0;
   out_234466216333689211[2] = 0;
   out_234466216333689211[3] = 1;
   out_234466216333689211[4] = 0;
   out_234466216333689211[5] = 0;
   out_234466216333689211[6] = 0;
   out_234466216333689211[7] = 0;
   out_234466216333689211[8] = 0;
}
void h_29(double *state, double *unused, double *out_6650930004987588617) {
   out_6650930004987588617[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1947828943203240244) {
   out_1947828943203240244[0] = 0;
   out_1947828943203240244[1] = 1;
   out_1947828943203240244[2] = 0;
   out_1947828943203240244[3] = 0;
   out_1947828943203240244[4] = 0;
   out_1947828943203240244[5] = 0;
   out_1947828943203240244[6] = 0;
   out_1947828943203240244[7] = 0;
   out_1947828943203240244[8] = 0;
}
void h_28(double *state, double *unused, double *out_3771330621894385077) {
   out_3771330621894385077[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7030227960272770818) {
   out_7030227960272770818[0] = 1;
   out_7030227960272770818[1] = 0;
   out_7030227960272770818[2] = 0;
   out_7030227960272770818[3] = 0;
   out_7030227960272770818[4] = 0;
   out_7030227960272770818[5] = 0;
   out_7030227960272770818[6] = 0;
   out_7030227960272770818[7] = 0;
   out_7030227960272770818[8] = 0;
}
void h_31(double *state, double *unused, double *out_2487620801386264569) {
   out_2487620801386264569[0] = state[8];
}
void H_31(double *state, double *unused, double *out_9102639406577262861) {
   out_9102639406577262861[0] = 0;
   out_9102639406577262861[1] = 0;
   out_9102639406577262861[2] = 0;
   out_9102639406577262861[3] = 0;
   out_9102639406577262861[4] = 0;
   out_9102639406577262861[5] = 0;
   out_9102639406577262861[6] = 0;
   out_9102639406577262861[7] = 0;
   out_9102639406577262861[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7958337668253159685) {
  err_fun(nom_x, delta_x, out_7958337668253159685);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_759863855661418044) {
  inv_err_fun(nom_x, true_x, out_759863855661418044);
}
void car_H_mod_fun(double *state, double *out_2543812563470930545) {
  H_mod_fun(state, out_2543812563470930545);
}
void car_f_fun(double *state, double dt, double *out_2371139363969154978) {
  f_fun(state,  dt, out_2371139363969154978);
}
void car_F_fun(double *state, double dt, double *out_5819561008987145557) {
  F_fun(state,  dt, out_5819561008987145557);
}
void car_h_25(double *state, double *unused, double *out_9188288045972936111) {
  h_25(state, unused, out_9188288045972936111);
}
void car_H_25(double *state, double *unused, double *out_4976393246024881055) {
  H_25(state, unused, out_4976393246024881055);
}
void car_h_24(double *state, double *unused, double *out_915206111334749955) {
  h_24(state, unused, out_915206111334749955);
}
void car_H_24(double *state, double *unused, double *out_7149042845030380621) {
  H_24(state, unused, out_7149042845030380621);
}
void car_h_30(double *state, double *unused, double *out_548410569254394227) {
  h_30(state, unused, out_548410569254394227);
}
void car_H_30(double *state, double *unused, double *out_2458060287517632428) {
  H_30(state, unused, out_2458060287517632428);
}
void car_h_26(double *state, double *unused, double *out_4453262698406451418) {
  h_26(state, unused, out_4453262698406451418);
}
void car_H_26(double *state, double *unused, double *out_8717896564898937279) {
  H_26(state, unused, out_8717896564898937279);
}
void car_h_27(double *state, double *unused, double *out_6375735942703082728) {
  h_27(state, unused, out_6375735942703082728);
}
void car_H_27(double *state, double *unused, double *out_234466216333689211) {
  H_27(state, unused, out_234466216333689211);
}
void car_h_29(double *state, double *unused, double *out_6650930004987588617) {
  h_29(state, unused, out_6650930004987588617);
}
void car_H_29(double *state, double *unused, double *out_1947828943203240244) {
  H_29(state, unused, out_1947828943203240244);
}
void car_h_28(double *state, double *unused, double *out_3771330621894385077) {
  h_28(state, unused, out_3771330621894385077);
}
void car_H_28(double *state, double *unused, double *out_7030227960272770818) {
  H_28(state, unused, out_7030227960272770818);
}
void car_h_31(double *state, double *unused, double *out_2487620801386264569) {
  h_31(state, unused, out_2487620801386264569);
}
void car_H_31(double *state, double *unused, double *out_9102639406577262861) {
  H_31(state, unused, out_9102639406577262861);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
