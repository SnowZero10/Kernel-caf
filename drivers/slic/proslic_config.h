#ifndef _PROSLIC_CONFIG_H
#define _PROSLIC_CONFIG_H

struct fxs_config {
    int flash_time_min;
    int flash_time_max;
    int dcfeed;
    int ring;
    int impedance;
    int audio_gain_rx;
    int audio_gain_tx;
    int pcm_config;
    int pcm_timeslot_rx;
    int pcm_timeslot_tx;
    int fsk;
    int fsk_onhook_wait;
    int fsk_onhook_preamble;
    int fsk_onhook_mark;
    int dtmf_mute_period;
};

struct fxo_config {
    int country;
    int ring;
    int audio_gain_rx;
    int audio_gain_tx;
    int pcm_config;
    int pcm_timeslot_rx;
    int pcm_timeslot_tx;
    int hybrid;
};

extern void proslic_config_init(struct fxs_config *fxs,
        struct fxo_config *fxo);
#endif