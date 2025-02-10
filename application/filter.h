/*
 * filter.h
 *
 *  Created on: 8 de ago de 2023
 *      Author: Ever_
 */

#ifndef APPLICATION_FILTER_H_
#define APPLICATION_FILTER_H_

typedef struct {    // filtro ressonante
    float a1;       // a - denominador
    float a2;       // b - numerador
    float a3;
    float b0;
    float b1;
    float b2;
    float b3;
    float x0;       // entrada atual
    float x1;       // entrada anterior
    float x2;
    float x3;
    float y0;       // saida atual
    float y1;       // saida anterior
    float y2;
    float y3;
    void (*calc)(); // pointer to calculation function
    void (*reset)();// pointer to reset function
} filter;

void filter_calc(filter *v);
void filter_reset(filter *v);
#define filter_DEFAULTS {0, \
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
                         0, \
                         0, \
                         0, \
                         (void (*)(Uint32))filter_calc,\
                         (void (*)(Uint32))filter_reset,\
};


#endif /* APPLICATION_FILTER_H_ */
