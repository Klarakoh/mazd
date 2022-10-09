#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6315413778743344648);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8827564806774451557);
void gnss_H_mod_fun(double *state, double *out_6144059787071489127);
void gnss_f_fun(double *state, double dt, double *out_2066965521722786900);
void gnss_F_fun(double *state, double dt, double *out_4120121182446732345);
void gnss_h_6(double *state, double *sat_pos, double *out_6843245966466803220);
void gnss_H_6(double *state, double *sat_pos, double *out_8758283351134468461);
void gnss_h_20(double *state, double *sat_pos, double *out_8951694425031887835);
void gnss_H_20(double *state, double *sat_pos, double *out_7311235469948348987);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4334617316991746036);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_390293331376768139);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4334617316991746036);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_390293331376768139);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}