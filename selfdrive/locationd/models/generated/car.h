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
void car_err_fun(double *nom_x, double *delta_x, double *out_6640615245471508365);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1043887668715740411);
void car_H_mod_fun(double *state, double *out_7111844645351811375);
void car_f_fun(double *state, double dt, double *out_8778630928766258795);
void car_F_fun(double *state, double dt, double *out_1942049354301175592);
void car_h_25(double *state, double *unused, double *out_4201921277942440821);
void car_H_25(double *state, double *unused, double *out_7518948963823819198);
void car_h_24(double *state, double *unused, double *out_5623951842834884862);
void car_H_24(double *state, double *unused, double *out_8363945572607514597);
void car_h_30(double *state, double *unused, double *out_6574399121308423629);
void car_H_30(double *state, double *unused, double *out_4011104768394115663);
void car_h_26(double *state, double *unused, double *out_5406655320232281892);
void car_H_26(double *state, double *unused, double *out_3777445644949762974);
void car_h_27(double *state, double *unused, double *out_6112066858775789643);
void car_H_27(double *state, double *unused, double *out_6185868080194540574);
void car_h_29(double *state, double *unused, double *out_4661435466146906476);
void car_H_29(double *state, double *unused, double *out_7899230807064091607);
void car_h_28(double *state, double *unused, double *out_8929155164122454466);
void car_H_28(double *state, double *unused, double *out_5465114249575929435);
void car_h_31(double *state, double *unused, double *out_2751289885313557654);
void car_H_31(double *state, double *unused, double *out_7549594925700779626);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}