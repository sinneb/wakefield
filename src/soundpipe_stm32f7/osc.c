/*
 * Osc
 *
 * This code has been extracted from the Csound opcode "oscili".
 * It has been modified to work as a Soundpipe module.
 *
 * Original Author(s): Barry Vercoe, John FFitch, Robin Whittle
 * Year: 1991
 * Location: OOps/ugens2.c
 *
 */

#include <stdlib.h>
#include <math.h>
#include "soundpipe.h"

#include "stm32746g_discovery.h"

#define ARM_MATH_CM7
#include "arm_math.h"

int sp_osc_create(sp_osc **osc)
{
    *osc = malloc(sizeof(sp_osc));
    return SP_OK;
}

int sp_osc_destroy(sp_osc **osc)
{
    free(*osc);
    return SP_NOT_OK;
}

int sp_osc_init(sp_data *sp, sp_osc *osc, sp_ftbl *ft, SPFLOAT iphs)
{
    osc->freq = 440.0;
    osc->amp = 0.2;
    osc->tbl = ft;
    osc->iphs = fabs(iphs);
    osc->inc = 0.0;
    if (osc->iphs >= 0){
        osc->lphs = ((int32_t)(osc->iphs * SP_FT_MAXLEN)) & SP_FT_PHMASK;
    }
    return SP_OK;
}

// sp_osc_compute(sp, osc, NULL, &tmp);
int sp_osc_compute(sp_data *sp, sp_osc *osc, SPFLOAT *in, SPFLOAT *out)
{
    sp_ftbl *datatable;
    SPFLOAT amp, freq, step, sr;
    
    datatable = osc->tbl;
    amp = osc->amp; 
    freq = osc->freq; // 401
    sr = sp->sr;
    
    //float arm_linear_interep_table[10]={0,2,4,6,8,10,12,14,16,18};
    // init linear interp instance
    // with table length 2048, startpoint 0, steps 1 and datatable ft/ftp
    arm_linear_interp_instance_f32 S = {600, 0, 1, (float32_t *)&datatable[0]};
    
    // 1 sine = 2048
    // freq = 440
    // sr = 48000
    
    // 48000 / 440 = 109.09 datapoints per sample
    // 0 -> 600 (datatable length)
    // 0 -> 109.1 
    // step = 600 / 109.1 = 18.77
    
    step = osc->inc + (600.0f/(sr/freq));
    if(step>600){step=step-600;}  
    *out = amp * (arm_linear_interp_f32(&S, step));
    osc->inc = step;

    return SP_OK;
}
