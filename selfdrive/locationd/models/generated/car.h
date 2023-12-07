#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7958337668253159685);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_759863855661418044);
void car_H_mod_fun(double *state, double *out_2543812563470930545);
void car_f_fun(double *state, double dt, double *out_2371139363969154978);
void car_F_fun(double *state, double dt, double *out_5819561008987145557);
void car_h_25(double *state, double *unused, double *out_9188288045972936111);
void car_H_25(double *state, double *unused, double *out_4976393246024881055);
void car_h_24(double *state, double *unused, double *out_915206111334749955);
void car_H_24(double *state, double *unused, double *out_7149042845030380621);
void car_h_30(double *state, double *unused, double *out_548410569254394227);
void car_H_30(double *state, double *unused, double *out_2458060287517632428);
void car_h_26(double *state, double *unused, double *out_4453262698406451418);
void car_H_26(double *state, double *unused, double *out_8717896564898937279);
void car_h_27(double *state, double *unused, double *out_6375735942703082728);
void car_H_27(double *state, double *unused, double *out_234466216333689211);
void car_h_29(double *state, double *unused, double *out_6650930004987588617);
void car_H_29(double *state, double *unused, double *out_1947828943203240244);
void car_h_28(double *state, double *unused, double *out_3771330621894385077);
void car_H_28(double *state, double *unused, double *out_7030227960272770818);
void car_h_31(double *state, double *unused, double *out_2487620801386264569);
void car_H_31(double *state, double *unused, double *out_9102639406577262861);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}