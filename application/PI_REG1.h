/*
 * PI_REG1.h
 *
 *  Created on: 15 de ago de 2023
 *      Author: Ever_
 */

#ifndef APPLICATION_PI_REG1_H_
#define APPLICATION_PI_REG1_H_

typedef struct {
    float Ts;                // Parameter:   Sample period
    float Ref;              // Input:       Reference input
    float Fdb;              // Input:       Feedback input
    float Err;              // Variable:    Error value
    float Err1;             // Variable:    Error value [n-1]
    float Kp;               // Parameter:   Proportional gain
    float Ki;               // Parameter:   integral gain
    float Kc;               // Parameter:   integral correction gain
    float Up;               // Variable:    Proportional output
    float Ui;               // Variable:    integral output
    float UiMax;            // Parameter:   Maximum integral threshold
    float UiMin;            // Parameter:   Minimum integral threshold
    float OutPreSat;        // Variable:    Pre-saturated output
    float OutMax;           // Parameter:   Maximum output
    float OutMin;           // Parameter:   Minimum output
    float Out;              // Output:      PI output
    float SatErr;           // Variable:    Saturated difference
    float Up1;              // history:     Previous proportional output
    float Ui1;              // history:     Previous integral output
    void (*calc)();         // Pointer to calculation function
    void (*calcEq)();         // Pointer to calculation function
    void (*reset)();        // Pointer to reset function
}PIREG1;

typedef PIREG1 *PIREG1_handle;

/*----------------------------------------------------------------------
 *  Default initializer for the PIREG1 object.
 */
#define PIREG1_DEFAULTS {0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         1.0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         (void (*)(Uint32))pi_reg1_calc,\
                         (void (*)(Uint32))pi_reg1_calc_eq_dif,\
                         (void (*)(Uint32))pi_reg1_reset, \
};

/*----------------------------------------------------------------------
*  Prototypes for the functions in PIREG1.C
*/
void pi_reg1_calc(PIREG1_handle);
void pi_reg1_calc_eq_dif(PIREG1_handle);
void pi_reg1_reset(PIREG1 *v);

#endif /* APPLICATION_PI_REG1_H_ */
