#include "proslic_config.h"
#include "proslic_api.h"

#define FLASH_TIME_MIN      (100)     /* 100 ms */
#define FLASH_TIME_MAX      (700)    /* 700 ms */
#define DTMF_MUTE_PERIOD    (1000)    /* 1000 ms */

static struct fxs_config fxs_configure =
{
    FLASH_TIME_MIN, /* flash_time_min */
    FLASH_TIME_MAX, /* flash_time_max */
    DCFEED_48V_25MA, /* dcfeed */
    RING_F20_45VRMS_0VDC_LPR, /* ring */
    ZSYN_600_0_0_30_0, /* impedance */
    0, /* audio_gain_rx */
    0, /* audio_gain_tx */
    LIN16_PCM, /* pcm_config */
    1, /* pcm_timeslot_rx */
    1, /* pcm_timeslot_tx */
    DEFAULT_FSK, /* fsk */
    2500, /* fsk_onhook_wait */
    30, /* fsk_onhook_preamble */
    140, /* fsk_onhook_mark */
    DTMF_MUTE_PERIOD, /* dtmf_mute_period */
};

static struct fxo_config fxo_configure =
{
    COU_USA, /* country */
    RING_DET_40_5VRMS_49_5VRMS_VALIDATION, /* ring */
    0, /* audio_gain_rx */
    0, /* audio_gain_tx */
    DAA_PCM_LINEAR16, /* pcm_config */
    17, /* pcm_timeslot_rx */
    17, /* pcm_timeslot_tx */
    0, /* hybrid */
};

void proslic_config_init(struct fxs_config *fxs,
        struct fxo_config *fxo)
{
    fxs->flash_time_min = fxs_configure.flash_time_min;
    fxs->flash_time_max = fxs_configure.flash_time_max;
    fxs->dcfeed = fxs_configure.dcfeed;
    fxs->ring = fxs_configure.ring;
    fxs->impedance = fxs_configure.impedance;
    fxs->audio_gain_rx = fxs_configure.audio_gain_rx;
    fxs->audio_gain_tx = fxs_configure.audio_gain_tx;
    fxs->pcm_config = fxs_configure.pcm_config;
    fxs->pcm_timeslot_rx = fxs_configure.pcm_timeslot_rx;
    fxs->pcm_timeslot_tx = fxs_configure.pcm_timeslot_tx;
    fxs->fsk_onhook_wait = fxs_configure.fsk_onhook_wait;
    fxs->fsk_onhook_preamble = fxs_configure.fsk_onhook_preamble;
    fxs->fsk_onhook_mark = fxs_configure.fsk_onhook_mark;
    fxs->fsk = fxs_configure.fsk;
    fxs->dtmf_mute_period = fxs_configure.dtmf_mute_period;

    fxo->country = fxo_configure.country;
    fxo->ring = fxo_configure.ring;
    fxo->audio_gain_rx = fxo_configure.audio_gain_rx;
    fxo->audio_gain_tx = fxo_configure.audio_gain_tx;
    fxo->pcm_config = fxo_configure.pcm_config;
    fxo->pcm_timeslot_rx = fxo_configure.pcm_timeslot_rx;
    fxo->pcm_timeslot_tx = fxo_configure.pcm_timeslot_tx;
    fxo->hybrid = fxo_configure.hybrid;
}
