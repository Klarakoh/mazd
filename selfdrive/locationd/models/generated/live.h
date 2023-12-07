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
void live_H(double *in_vec, double *out_214500349649226956);
void live_err_fun(double *nom_x, double *delta_x, double *out_4930751168403226085);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5324820040472701178);
void live_H_mod_fun(double *state, double *out_5188937138529325432);
void live_f_fun(double *state, double dt, double *out_7723766835351807537);
void live_F_fun(double *state, double dt, double *out_9205111709488920692);
void live_h_4(double *state, double *unused, double *out_2358197272576436694);
void live_H_4(double *state, double *unused, double *out_4947000358369127463);
void live_h_9(double *state, double *unused, double *out_385849584469780270);
void live_H_9(double *state, double *unused, double *out_6212524780075976683);
void live_h_10(double *state, double *unused, double *out_2567975715080056660);
void live_H_10(double *state, double *unused, double *out_571864675711830003);
void live_h_12(double *state, double *unused, double *out_8131953882863353040);
void live_H_12(double *state, double *unused, double *out_1434258018673605533);
void live_h_35(double *state, double *unused, double *out_2371272486677336243);
void live_H_35(double *state, double *unused, double *out_3087052369332959952);
void live_h_32(double *state, double *unused, double *out_8832656386972401992);
void live_H_32(double *state, double *unused, double *out_4094963178069825769);
void live_h_13(double *state, double *unused, double *out_3950732492360766849);
void live_H_13(double *state, double *unused, double *out_2366987762894106980);
void live_h_14(double *state, double *unused, double *out_385849584469780270);
void live_H_14(double *state, double *unused, double *out_6212524780075976683);
void live_h_33(double *state, double *unused, double *out_9045185635883086757);
void live_H_33(double *state, double *unused, double *out_63504635305897652);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}