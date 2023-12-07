#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3650691405938361009);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_257155659230545517);
void gnss_H_mod_fun(double *state, double *out_24761517867664939);
void gnss_f_fun(double *state, double dt, double *out_2953610155191611994);
void gnss_F_fun(double *state, double dt, double *out_7769726719115423260);
void gnss_h_6(double *state, double *sat_pos, double *out_507136575089039816);
void gnss_H_6(double *state, double *sat_pos, double *out_1421338190566676548);
void gnss_h_20(double *state, double *sat_pos, double *out_2353567699693444253);
void gnss_H_20(double *state, double *sat_pos, double *out_5340928639051498153);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8644793348608052699);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1931195383238368014);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8644793348608052699);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1931195383238368014);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}