#include "proslic_tone.h"

static struct tone_list dail_tone[] = {
    { TONE_F440_F350, 1000 },/*hubo change  dail_tone from TONE_F450 to TONE_F440_F350 13db 20170222 */
};

static struct tone_list howler_tone[] = {
    { TONE_F425_ON200_OFF200, 0 },
};

static struct tone_list call_waiting_tone[] = {
    { TONE_F440, 200 },
    { TONE_NONE, 100 },
    { TONE_F440, 200 },
    { TONE_NONE, 3500 },
};

static struct tone_list alerting_tone[] = {
    { TONE_F932, 330 },
    { TONE_F1397, 330 },
    { TONE_F1760, 330 },
    { TONE_NONE, 1000 },
};

static struct tone_list success_tone[] = {
    { TONE_F425, 500 },
    { TONE_NONE, 1000 },
};

static struct tone_list fail_tone[] = {
    { TONE_F932, 330 },
    { TONE_F1397, 330 },
    { TONE_F1760, 330 },
    { TONE_NONE, 1000 },
};

static struct tone_list lower_power_tone[] = {
    { TONE_F2750, 60 },
    { TONE_NONE, 100 },
    { TONE_F2900, 60 },
    { TONE_NONE, 100 },
    { TONE_F2750, 60 },
    { TONE_NONE, 100 },
};

static struct tone_list voice_mail_tone[] = {
    { TONE_F440_F350_ON100_OFF100, 0 },
};

static struct tone_list congestion_tone[] = {
    { TONE_F450_ON700_OFF700, 0 },
};

/*hubo change ringback_tone from TONE_F450_ON1000_OFF4000 to TONE_F440_F480_ON2000_OFF4000 20170222*/
static struct tone_list ringback_tone[] = {
    { TONE_F440_F480_ON2000_OFF4000, 0 },
};
/*hubo change busy_tone from TONE_F450_ON350_OFF350 to TONE_F480_F620_ON500_OFF500 20170222*/
static struct tone_list busy_tone[] = {
    { TONE_F480_F620_ON500_OFF500, 0 },
};

static struct tone_list pin_tone[] = {
    { TONE_F480_F620_ON200_OFF200, 0 },
};

static struct tone_list puk_tone[] = {
    { TONE_F480_F620_ON300_OFF200, 0 },
};

static struct tone_list cas_tone[] = {
    { TONE_F2130_F2750, 0 },
};

static struct tone_config tone_configure[] = {
    {
        SLIC_TONE_DIAL,
        {
            TONE_HW_CTL, 0, sizeof(dail_tone)/sizeof(struct tone_list), dail_tone,
        },
    },
    {
        SLIC_TONE_HOWLER,
        {
            TONE_HW_CTL, 0, sizeof(howler_tone)/sizeof(struct tone_list), howler_tone,
        },
    },
    {
        SLIC_TONE_CALL_WAITING,
        {
            TONE_SW_CTL, 0, sizeof(call_waiting_tone)/sizeof(struct tone_list), call_waiting_tone,
        },
    },
    {
        SLIC_TONE_ALERTING,
        {
            TONE_SW_CTL, 0, sizeof(alerting_tone)/sizeof(struct tone_list), alerting_tone,
        },
    },
    {
        SLIC_TONE_SERVICE_SUCCES,
        {
            TONE_SW_CTL, 1, sizeof(success_tone)/sizeof(struct tone_list), success_tone,
        },
    },
    {
        SLIC_TONE_SERVICE_FAIL,
        {
            TONE_SW_CTL, 1, sizeof(fail_tone)/sizeof(struct tone_list), fail_tone,
        },
    },
    {
        SLIC_TONE_LOW_POWERE,
        {
            TONE_SW_CTL, 1, sizeof(lower_power_tone)/sizeof(struct tone_list), lower_power_tone,
        },
    },
    {
        SLIC_TONE_VOICE_MAIL,
        {
            TONE_HW_CTL, 0, sizeof(voice_mail_tone)/sizeof(struct tone_list), voice_mail_tone,
        },
    },
    {
        SLIC_TONE_BUSY,
        {
            TONE_HW_CTL, 0, sizeof(busy_tone)/sizeof(struct tone_list), busy_tone,
        },
    },
    {
        SLIC_TONE_CONGESTION,
        {
            TONE_HW_CTL, 0, sizeof(congestion_tone)/sizeof(struct tone_list), congestion_tone,
        },
    },
    {
        SLIC_TONE_RINGBACK,
        {
            TONE_HW_CTL, 0, sizeof(ringback_tone)/sizeof(struct tone_list), ringback_tone,
        },
    },
    {
        SLIC_TONE_PIN,
        {
            TONE_HW_CTL, 0, sizeof(pin_tone)/sizeof(struct tone_list), pin_tone,
        },
    },
    {
        SLIC_TONE_PUK,
        {
            TONE_HW_CTL, 0, sizeof(puk_tone)/sizeof(struct tone_list), puk_tone,
        },
    },
};


int proslic_find_toneset(struct tone_set **toneset, int type)
{
    int i, ret = -1;
    int num = sizeof(tone_configure) / sizeof(tone_configure[0]);

    if (type < SLIC_TONE_TONE_MAX) {
        for (i = 0; i < num; i++) {
            if(tone_configure[i].id == type){
                *toneset = &tone_configure[i].tones;
                return 0;
            }
        }
    }
    return ret;
}

void proslic_play_tonelist(struct proslic_dev *si3217x,
                struct tone_list *playlist)
{
    ProSLIC_ToneGenStop(si3217x->chan);
    if (0 != playlist->preset) {
        ProSLIC_ToneGenSetup(si3217x->chan, playlist->preset);
        if (0 != playlist->time)
            ProSLIC_ToneGenStart(si3217x->chan, 0);
        else
            ProSLIC_ToneGenStart(si3217x->chan, 1);
    }
}

void proslic_play_cas_tone(struct proslic_dev *si3217x)
{
    proslic_play_tonelist(si3217x, cas_tone);
}
