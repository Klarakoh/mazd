#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1237325442709970628);
void live_err_fun(double *nom_x, double *delta_x, double *out_3629510305670504186);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3500694904292406605);
void live_H_mod_fun(double *state, double *out_1821110240313906847);
void live_f_fun(double *state, double dt, double *out_1819509483554871615);
void live_F_fun(double *state, double dt, double *out_7480378043706369704);
void live_h_4(double *state, double *unused, double *out_904402081155690941);
void live_H_4(double *state, double *unused, double *out_6327695051802815796);
void live_h_9(double *state, double *unused, double *out_1582515772269223850);
void live_H_9(double *state, double *unused, double *out_6086505405173225151);
void live_h_10(double *state, double *unused, double *out_1110366687721286647);
void live_H_10(double *state, double *unused, double *out_2997648176330185213);
void live_h_12(double *state, double *unused, double *out_1890277588110140188);
void live_H_12(double *state, double *unused, double *out_5706596026755222129);
void live_h_35(double *state, double *unused, double *out_4328895334668431461);
void live_H_35(double *state, double *unused, double *out_2961032994430208420);
void live_h_32(double *state, double *unused, double *out_1659530923952545034);
void live_H_32(double *state, double *unused, double *out_3273654689317653448);
void live_h_13(double *state, double *unused, double *out_2840807863896650648);
void live_H_13(double *state, double *unused, double *out_6198575880178307593);
void live_h_14(double *state, double *unused, double *out_1582515772269223850);
void live_H_14(double *state, double *unused, double *out_6086505405173225151);
void live_h_33(double *state, double *unused, double *out_6773537615204010340);
void live_H_33(double *state, double *unused, double *out_189524010208649184);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}