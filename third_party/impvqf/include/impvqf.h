#ifndef IMPVQF_H
#define IMPVQF_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMPVQF_ABI_VERSION UINT32_C(2)
#define IMPVQF_VERSION "0.2.0-float"
#define IMPVQF_CONTEXT_BYTES 1024
typedef float ImpVqfReal;
#define IMPVQF_OK INT32_C(0)
#define IMPVQF_ERROR_ARGUMENT INT32_C(-1)
#define IMPVQF_ERROR_VERSION INT32_C(-2)
#define IMPVQF_ERROR_CONFIG INT32_C(-3)
#define IMPVQF_ERROR_STATE INT32_C(-4)
#define IMPVQF_ERROR_NUMERIC INT32_C(-5)

#define IMPVQF_MOTION_BIAS UINT32_C(1)
#define IMPVQF_REST_BIAS UINT32_C(2)
#define IMPVQF_MAG_REJECTION UINT32_C(4)
#define IMPVQF_TRAPEZOID UINT32_C(8)
#define IMPVQF_MAG_CALIBRATION UINT32_C(16)

#define IMPVQF_REST_DETECTED UINT32_C(1)
#define IMPVQF_MAG_DISTURBED UINT32_C(2)
#define IMPVQF_MAG_REFERENCE_READY UINT32_C(4)
#define IMPVQF_MAG_STARTUP_COMPLETE UINT32_C(8)
#define IMPVQF_RECOVERY_ACTIVE UINT32_C(16)

typedef struct {
    unsigned char opaque[IMPVQF_CONTEXT_BYTES];
} ImpVqfContext;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t options;
    uint32_t reserved;
    ImpVqfReal gyr_ts;
    ImpVqfReal acc_ts;
    ImpVqfReal mag_ts;
    ImpVqfReal tau_acc;
    ImpVqfReal tau_mag;
    ImpVqfReal bias_sigma_init_dps;
    ImpVqfReal bias_forgetting_time;
    ImpVqfReal bias_clip_dps;
    ImpVqfReal bias_sigma_motion_dps;
    ImpVqfReal bias_vertical_forgetting_factor;
    ImpVqfReal bias_sigma_rest_dps;
    ImpVqfReal rest_min_t;
    ImpVqfReal rest_filter_tau;
    ImpVqfReal rest_th_gyr_dps;
    ImpVqfReal rest_th_acc;
    ImpVqfReal mag_current_tau;
    ImpVqfReal mag_ref_tau;
    ImpVqfReal mag_norm_th;
    ImpVqfReal mag_dip_th_deg;
    ImpVqfReal mag_new_time;
    ImpVqfReal mag_new_first_time;
    ImpVqfReal mag_min_undisturbed_time;
    ImpVqfReal mag_max_rejection_time;
    ImpVqfReal mag_rejection_factor;
    ImpVqfReal rest_gyro_max_dps;
    ImpVqfReal rest_direction_limit_deg;
    ImpVqfReal rest_return_limit_deg;
    ImpVqfReal motion_innovation_limit_deg;
    ImpVqfReal rest_direction_tau;
    ImpVqfReal recovery_tau;
    ImpVqfReal recovery_max_rate_dps;
    ImpVqfReal recovery_stop_error_deg;
    ImpVqfReal mag_hard_iron[3];
    ImpVqfReal mag_soft_iron[9];
} ImpVqfConfig;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t flags;
    uint32_t reserved;
    ImpVqfReal quat6d[4];
    ImpVqfReal quat9d[4];
    ImpVqfReal bias_rad_s[3];
    ImpVqfReal delta_rad;
    ImpVqfReal mag_ref_norm;
    ImpVqfReal mag_ref_dip_rad;
    ImpVqfReal mag_candidate_t;
    ImpVqfReal mag_reject_t;
    ImpVqfReal mag_undisturbed_t;
    ImpVqfReal mag_innovation_rad;
    ImpVqfReal mag_correction_rad_s;
    ImpVqfReal mag_startup_gain;
} ImpVqfOutput;

const char *impvqf_version(void);
size_t impvqf_context_size(void);

int32_t impvqf_config_default(ImpVqfConfig *config, size_t config_size);
int32_t impvqf_init(ImpVqfContext *context, const ImpVqfConfig *config);
int32_t impvqf_reset(ImpVqfContext *context);

int32_t impvqf_update(ImpVqfContext *context, const ImpVqfReal gyr[3],
                     const ImpVqfReal acc[3], const ImpVqfReal mag[3]);

int32_t impvqf_update_gyr(ImpVqfContext *context, const ImpVqfReal gyr[3]);
int32_t impvqf_update_acc(ImpVqfContext *context, const ImpVqfReal acc[3]);
int32_t impvqf_update_mag(ImpVqfContext *context, const ImpVqfReal mag[3]);
int32_t impvqf_get_output(const ImpVqfContext *context, ImpVqfOutput *output,
                         size_t output_size);

#ifdef __cplusplus
}
#endif
#endif
