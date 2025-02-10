/*
 * PI_REG1.c
 *
 *  Created on: 15 de ago de 2023
 *      Author: Ever_
 */

#include "PI_REG1.h"

/*
 * Calculates the PI controller with proportional
 * and integrative gains.
 */
void pi_reg1_calc(PIREG1 *v){
    // Compute the error
    v->Err = v->Ref - v->Fdb;
    // Compute the proportional output
    //v->Up = v->Kp *( v->Err - v->Err1); // v.1
    //v->Up = v->Kp *(v->Err);              // v.2, v.3
    v->Up = v->Kp *(v->Err - v->Err1);
    // Compute the integral output

    //v->Ui = 0.5*(v->Ts) * v->Ki * (v->Err + v->Err1) + v->Out; // v.1
    //v->Ui = 0.5*(v->Ts) * v->Ki * (v->Err + v->Err1) + v->Ui1; // v.2
    //v->Ui = 0.5*(v->Ts) * v->Ki * (v->Err + v->Err) + v->Ui1;    // v.3
    v->Ui = 0.5*(v->Ts) * v->Ki * (v->Err + v->Err1) + v->Out;    // v.4
    // Saturate the integral output
//    if(v->Ui > v->UiMax)
//        v->Ui = v->UiMax;
//    else if (v->Ui < v->UiMin)
//        v->Ui = v->UiMin;
    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui;
    //v->OutPreSat = v->Up + v->Ui

    // Saturate the output
    if (v->OutPreSat > v->OutMax)
        v->Out = v->OutMax;
    else if (v->OutPreSat < v->OutMin)
        v->Out = v->OutMin;
    else
        v->Out = v->OutPreSat;

    // Compute the saturate difference
    //v->SatErr = v->Out - v->OutPreSat;

    // Update the previous proportional output
    v->Up1 = v->Up;
    // Update the previous integral output
    v->Ui1 = v->Ui;
    // Update the previous error signal
    v->Err1 = v->Err;
}

/*
 * Calculates the PI controller as a difference equation
 *
 *              y[n] = Kp*x[n] + Ki*x[n-1] + y[n-1]
 *
 * in which     x[n] = Err = (Ref - Fdb)
 */
void pi_reg1_calc_eq_dif(PIREG1 *v){
    // Compute the error x[n]
    v->Err = v->Ref - v->Fdb;
    // Compute the proportional output Kp*x[n]
    v->Up = v->Kp * v->Err;
    // Compute the integral output Ki*x[n-1] + y[n-1]
    v->Ui = (v->Ki * v->Up1) + v->Ui;

    //integration threshold
    if (v->Ui > v->UiMax){
        v->Ui = v->UiMax;
    }else if(v->Ui < v->UiMin){
        v->Ui = v->UiMin;
    }

    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui;
    // Saturate the output
    if (v->OutPreSat > v->OutMax)
        v->Out = v->OutMax;
    else if (v->OutPreSat < v->OutMin)
        v->Out = v->OutMin;
    else{
        v->Out = v->OutPreSat; // control output without saturation
    }

    // Update the x[n-1]
    v->Up1 = v->Err;
}


void pi_reg1_reset(PIREG1 *v){
    v->Err = 0;
    v->Err1 = 0;
    v->Fdb = 0;
    v->OutPreSat = 0;
    v->Out = 0;
    v->Ui = 0;
    v->Up1 = 0;
    v->SatErr = 0;
}
