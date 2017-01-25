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
int numStages=1;
int samples=512;

int sp_filts_create(sp_filts **p)
{
    *p = malloc(sizeof(sp_filts));
    return SP_OK;
}

int sp_filts_destroy(sp_filts **p)
{
    free(*p);
    return SP_OK;
}

int sp_filts_init(sp_data *sp, sp_filts *p)
{
  p->out1 = 0;
  p->out2 = 0;
  p->out3 = 0;
  p->out4 = 0;
  p->in1 = 0;
  p->in2 = 0;
  p->in3 = 0;
  p->in4 = 0;
  
  p->a1 = 0;
  p->a2 = 0;
  p->a3 = 0;
  p->m0 = 0;
  p->m1 = 0;
  p->m2 = 0;
  
  //p->res = 1;
  //p->freq = 1;
  return SP_OK;
}

int sp_filts_compute_coeffs(sp_data *sp, sp_filts *p)
{
  
  SPFLOAT fc = p->freq;
  SPFLOAT res = p->res;
  
  SPFLOAT g = tan(3.1415 * fc / 24000);
  SPFLOAT k = 1/res;
  p->a1 = 1/(1+g*(g+k));
  p->a2 = g*p->a1;
  p->a3 = g*p->a2;
  p->m0 = 0;
  p->m1 = 0;
  p->m2 = 1;
  return SP_OK;
}

int sp_filts_compute(sp_data *sp, sp_filts *p, SPFLOAT *in, SPFLOAT *out)
{
 //
//   SPFLOAT fc = p->freq;
//   SPFLOAT res = p->res;
// // //
  float32_t dotprod_tst_a[4];
  float32_t dotprod_tst_b[4];
  float32_t result;
  
     float input = *in;
     if(isnan(input)) input = 0;
     
     //float g = tanf(3.14f* p->freq/24000.0f);
     float Kvalue = 3.14f* p->freq/24000.0f;
     float g = tan(Kvalue);//sinf(Kvalue) / cosf(Kvalue);
     float k = 1.0 - 0.99 * p->res;
     // float a1 = 1.0f / (1.0f + g * (g + k));
     // float a2 = g * a1;
     // float a3 = g * a2;
     float ginv = g / (1.0f + g * (g + k));
     float g1 = ginv;
     float g2 = 2.0f * (g + k) * ginv;
     float g3 = g * ginv;
     float g4 = 2.0f * ginv;
     
     float v0 = input;
     float v1z = p->in1;
     float v2z = p->in2;
     float v3 = v0 + p->m0 - 2.0 * v2z;
     
     dotprod_tst_a[0] = g1;
     dotprod_tst_b[0] = v3;
     
     dotprod_tst_a[1] = -g2;
     dotprod_tst_b[1] = v1z;
     
     arm_dot_prod_f32(dotprod_tst_a, dotprod_tst_b,2,&result);   
     p->in1 += result;
     
     dotprod_tst_a[0] = g3;
     dotprod_tst_b[0] = v3;
     
     dotprod_tst_a[1] = g4;
     dotprod_tst_b[1] = v1z;   

     arm_dot_prod_f32(dotprod_tst_a, dotprod_tst_b,2,&result);   
     p->in2 += result;     
     
       
     //p->in1 += g1 * v3 - g2 * v1z;
     //p->in2 += g3 * v3 + g4 * v1z;
     p->m0 = v0;
     *out = p->in2;
     //*out = input;

     // float v3 = input - p->in2;
     // float v1 = a1 * p->in1 + a2 * v3;
     // float v2 = p->in2 + a2 * p->in1 + a3 * v3;
     // p->in1 = 2 * v1 - p->in1;
     // p->in2 = 2 * v2 - p->in2;
     //
     // *out = v2;
     //
     // *out = input;
//
//   int numStages=1;
//   int samples=512;
//
//   float32_t pCoeffs[8] = {0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
//   float32_t pState[8] = {0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
//   float32_t pSrc[512];
//   float32_t pDst[512];// =  malloc(sizeof(float32_t) * samples);
//
//   arm_biquad_casd_df1_inst_f32 S;
//   arm_biquad_cascade_df1_init_f32(&S, numStages, pCoeffs, pState);
//
//   arm_biquad_cascade_df1_f32(&S, pSrc, pDst, samples);
//
// //   SPFLOAT f = fc * 1.16;
// //   SPFLOAT fb = res * (1.0 - 0.15 * f * f);
// //
// //   input -= p->out4 * fb;
// //   input *= 0.35013 * (f*f)*(f*f);
// //   p->out1 = input + 0.3 * p->in1 + (1 - f) * p->out1; // Pole 1
// //   p->in1 = input;
// //   p->out2 = p->out1 + 0.3 * p->in2 + (1 - f) * p->out2; // Pole 2
// //   p->in2 = p->out1;
// //   p->out3 = p->out2 + 0.3 * p->in3 + (1 - f) * p->out3; // Pole 3
// //   p->in3 = p->out2;
// //   p->out4 = p->out3 + 0.3 * p->in4 + (1 - f) * p->out4; // Pole 4
// //   p->in4 = p->out3;
// //   //return p->out4;
// //
// //   *out = p->out4;
// //   return SP_OK;
//
//   // float w0 = 1000 * 0.00014247165533;
//  //  float c, alpha;
//  //
//  //  //c = cos(w0);
//  //  c = arm_cos_f32(w0);
//  //  alpha = sin(w0);// / (2 * 1);
//  //
//  //  //arm_sin_cos_f32(w0, &alpha, &c);
//  //  alpha /= (2 * 1);
//  //
//  //  float b2;
//  //  float b0 = b2 = (1 - c) / 2;
//  //  float b1 = 1 - c;
//  //  float a0 = 1.0 / (1 + alpha);
//  //  float a1 = -2 * c;
//  //  float a2 = 1 - alpha;
//  //
//  //  float val = ( (b0 * input) + (b1 * p->in1) + (b2 * p->in2) - (a1 * p->out1) - (a2 * p->out2) ) * a0;
//  //  p->out2 = p->out1;
//  //  p->out1 = val;
//  //  *out = *in;
//  //  //self->data[i] = self->y1 = val;
//  //  p->in2 = p->in1;
//  //  p->in1 = input;

  // SPFLOAT v3 = input - p->in2;
  // SPFLOAT v1 = p->a1 * p->in1 + p->a2 * v3;
  // SPFLOAT v2 = p->in2 + p->a2 * p->in1 + p->a3 * v3;
  // p->in1 = 2 * v1 - p->in1;
  // p->in2 = 2 * v2 - p->in2;
  //
  // *out = p->m0 * input + p->m1 * v1 + p->m2 * v2;
  
  
  // float x = input - 0.1 * p->in4;
 //  p->in1 = (x + 0.1) * 0.1 - 0.1 * p->in1;
 //  p->in2 = (p->in1 + 0.1) * 0.1 - 0.1 * p->in2;
 //  p->in3 = (p->in2 + 0.1) * 0.1 - 0.1 * p->in3;
  //p->in4 = (p->in3 + 0.1) * 0.1 - 0.1 * p->in4;
  //p->in4 -= (p->in4*p->in4*p->in4) * 0.16666666666666666;
  //self->oldX = x; self->oldY1 = self->y1; self->oldY2 = self->y2; self->oldY3 = self->y3;
  
  // float frequency = 0.01;
 //  float resonance = 4;
 //
 //  float q = 1.0f - frequency;
 //  float p2 = frequency + 0.8f * frequency * q;
 //  float f = p2 + p2 - 1.0f;
 //  q = resonance * (1.0f + 0.5f * q * (1.0f - q + 5.6f * q * q));
 //
 //  input -= q * p->in4;                          //feedback
 //    float t1 = p->in1; p->in1 = (input + p->m0) * p2 - p->in1 * f;
 //    float t2 = p->in2; p->in2 = (p->in1 + t1) * p2 - p->in2 * f;
 //    t1 = p->in3;  p->in3 = (p->in2 + t2) * p2 - p->in3 * f;
 //    p->in4 = (p->in3 + t1) * p2 - p->in4 * f;
 //    p->in4 = p->in4 - p->in4 * p->in4 * p->in4 * 0.166667f;    //clipping
 //    p->m0 = input;
 //
 // *out = p->in4;
 //*out = *in;
  
  // float k = 1;//p->out4; if (k > 1) {k = 1;}
//   float filtin = 1;//(-k * 0.1) + input;
//   p->out1 = ((-p->out1 + filtin) * 0.14) + p->out1;
//   p->out2 = ((-p->out2 + p->out1) * 0.14) + p->out2;
//   p->out3 = ((-p->out3 + p->out2) * 0.14) + p->out3;
//   p->out4 = ((-p->out4 + p->out3) * 0.14) + p->out4;
//   *out = 0;//p->out4;
  
  //c = pow(0.5, (128-cutoff) / 16.0);
  //r = pow(0.5, (resonance+24) / 16.0);

  //volatile double tst = ceil(p->freq);
  
  //if(isnan(tst)) tst = 0;
  //tst+=10;
  
  //volatile float c1 = 128-tst / 16.0;
  
  //tst=0;
  //float c = powf(0.5, 1);
  //float r = powf(0.5, 1);

  //Loop:

  //p->out1 = (1-r*c)*p->out1 - (c)*p->out2 + (c)*input;
  //p->out2 = (1-r*c)*p->out2 + (c)*p->out1;

  //*out = p->out2;

  
  //*out = *in;
  
  return SP_OK;

}
