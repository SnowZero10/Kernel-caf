#ifndef _PROSLIC_TONE_H
#define _PROSLIC_TONE_H

#include "proslic_dev.h"

#define TONE_HW_CTL         (0)
#define TONE_SW_CTL         (1)

struct tone_list {
    int preset;
    int time;
};

struct tone_set {
    int ctl_type; /* hardware or software timer */
    int play_times; /* how many times the tone been played */
    int list_num; /* how many element in playlist */
    struct tone_list *playlist;
};

struct tone_config {
    int id;
    struct tone_set tones;
};

extern int proslic_find_toneset(struct tone_set **toneset, int type);
extern void proslic_play_tonelist(struct proslic_dev *si3217x,
                struct tone_list *playlist);
extern void proslic_play_cas_tone(struct proslic_dev *si3217x);
#endif