/*
 * Foo
 * 
 * This is a dummy module. It doesn't do much.
 * Feel free to use this as a boilerplate template.
 * 
 */

#include <math.h>
#include <stdlib.h>
#include "soundpipe.h"
#include "stm32746g_discovery.h"

#define ARM_MATH_CM7
#include "arm_math.h"


int sp_gain_create(sp_gain **p)
{
    *p = malloc(sizeof(sp_gain));
    return SP_OK;
}

int sp_gain_destroy(sp_gain **p)
{
    free(*p);
    return SP_OK;
}

int sp_gain_init(sp_data *sp, sp_gain *p)
{
  p->out1 = 0;
  p->out2 = 0;
  p->out3 = 0;
  p->out4 = 0;
  p->in1 = 0;
  p->in2 = 0;
  p->in3 = 0;
  p->in4 = 0;
  p->res = 1;
  p->freq = 0.1;
  return SP_OK;
}

int sp_gain_compute(sp_data *sp, sp_gain *p, SPFLOAT *in, SPFLOAT *out)
{
  
  SPFLOAT fc = p->freq;
  SPFLOAT res = p->res;
//
  SPFLOAT input = *in;
  if(isnan(input)) input = 0;
//   SPFLOAT f = fc * 1.16;
//   SPFLOAT fb = res * (1.0 - 0.15 * f * f);
//
//   input -= p->out4 * fb;
//   input *= 0.35013 * (f*f)*(f*f);
//   p->out1 = input + 0.3 * p->in1 + (1 - f) * p->out1; // Pole 1
//   p->in1 = input;
//   p->out2 = p->out1 + 0.3 * p->in2 + (1 - f) * p->out2; // Pole 2
//   p->in2 = p->out1;
//   p->out3 = p->out2 + 0.3 * p->in3 + (1 - f) * p->out3; // Pole 3
//   p->in3 = p->out2;
//   p->out4 = p->out3 + 0.3 * p->in4 + (1 - f) * p->out4; // Pole 4
//   p->in4 = p->out3;
//   //return p->out4;
//
//   *out = p->out4;
//   return SP_OK;
  
  // float w0 = 1000 * 0.00014247165533;
 //  float c, alpha;
 //
 //  //c = cos(w0);
 //  c = arm_cos_f32(w0);
 //  alpha = sin(w0);// / (2 * 1);
 //
 //  //arm_sin_cos_f32(w0, &alpha, &c);
 //  alpha /= (2 * 1);
 //
 //  float b2;
 //  float b0 = b2 = (1 - c) / 2;
 //  float b1 = 1 - c;
 //  float a0 = 1.0 / (1 + alpha);
 //  float a1 = -2 * c;
 //  float a2 = 1 - alpha;
 //
 //  float val = ( (b0 * input) + (b1 * p->in1) + (b2 * p->in2) - (a1 * p->out1) - (a2 * p->out2) ) * a0;
 //  p->out2 = p->out1;
 //  p->out1 = val;
 //  *out = *in;
 //  //self->data[i] = self->y1 = val;
 //  p->in2 = p->in1;
 //  p->in1 = input;
  
  SPFLOAT g = tan(3.1415 * fc / 48000);
  SPFLOAT k = 1/res;
  SPFLOAT a1 = 1/(1+g*(g+k));
  SPFLOAT a2 = g*a1;
  SPFLOAT a3 = g*a2;
  SPFLOAT m0 = 0;
  SPFLOAT m1 = 1;
  SPFLOAT m2 = 0;
  
  SPFLOAT v3 = input - p->in2;
  SPFLOAT v1 = a1 * p->in1 + a2 * v3;
  SPFLOAT v2 = p->in2 + a2 * p->in1 + a3 * v3;
  p->in1 = 2 * v1 - p->in1;
  p->in2 = 2 * v2 - p->in2;
  
  *out = m0 * input + m1 * v1 + m2 * v2;
  
  
  return SP_OK;

}
