#include <stddef.h>
#include "motion_fx.h"
#include <assert.h>
#include <string.h>

MFX_output_t host_mfx_output;
uint32_t host_mfx_calls, host_mfx_initializations;
size_t host_mfx_size=100U;

size_t MotionFX_GetStateSize(void) { return host_mfx_size; }
void MotionFX_initialize(MFXState_t state) { (void)state; host_mfx_initializations++; }
void MotionFX_getKnobs(MFXState_t state, MFX_knobs_t *knobs)
{ (void)state; memset(knobs,0,sizeof(*knobs)); }
void MotionFX_setKnobs(MFXState_t state, MFX_knobs_t *knobs)
{
    (void)state;
    assert(!strcmp(knobs->acc_orientation,"nwu") && !strcmp(knobs->gyro_orientation,"nwu"));
    assert(knobs->output_type==MFX_ENGINE_OUTPUT_ENU && knobs->LMode==1U && knobs->modx==1U);
}
void MotionFX_enable_6X(MFXState_t state, MFX_engine_state_t enabled)
{ (void)state; assert(enabled==MFX_ENGINE_ENABLE); }
void MotionFX_enable_9X(MFXState_t state, MFX_engine_state_t enabled)
{ (void)state; assert(enabled==MFX_ENGINE_DISABLE); }
void MotionFX_propagate(MFXState_t state, MFX_output_t *out, MFX_input_t *in, float *dt)
{ (void)state; (void)out; (void)in; assert(*dt>0.0f && *dt<=0.1f); }
void MotionFX_update(MFXState_t state, MFX_output_t *out, MFX_input_t *in, float *dt, float *q)
{ (void)state; (void)in; (void)dt; assert(q==NULL); *out=host_mfx_output; host_mfx_calls++; }
