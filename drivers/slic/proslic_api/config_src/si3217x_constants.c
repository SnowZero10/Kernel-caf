/*
** Copyright (c) 2013 Silicon Laboratories, Inc.
** 2013-09-27 09:29:01
**
** Si3217x ProSLIC API Configuration Tool Version 3.0.0
**
** Modified to support multiple BOM options
*/


#include "proslic.h"
#include "si3217x.h"


Si3217x_General_Cfg Si3217x_General_Configuration = {
/* Flags */
0x78,               /* DEVICE_KEY */
BO_DCDC_FLYBACK,    /* BOM_OPT */
BO_GDRV_INSTALLED,  /* GDRV_OPTION */
VDC_7P0_20P0,       /* VDC_RANGE */
VDAA_ENABLED,       /* DAA_ENABLE */
AUTO_ZCAL_ENABLED,  /* ZCAL_EN */
BO_STD_BOM,         /* PM_BOM */
/* Constants */
1100L,              /* I_OITHRESH (1100mA) */
900L,               /* I_OITHRESH_LO (900mA) */
1700L,              /* I_OITHRESH_HI (1700mA) */
136000L,            /* V_OVTHRESH (136000mV) */
5000L,              /* V_UVTHRESH (5000mV) */
1000L,              /* V_UVHYST (1000mV) */
/* RAM Constants */
0x00000000L,        /* DCDC_FSW_VTHLO */
0x00000000L,        /* DCDC_FSW_VHYST */
0x0048D15BL,        /* P_TH_HVIC */
0x07FEB800L,        /* COEF_P_HVIC */
0x00083120L,        /* BAT_HYST */
0x03D70A20L,        /* VBATH_EXPECT (60.00) */
0x070A3D3AL,        /* VBATR_EXPECT (110.00) */
0x0FFF0000L,        /* PWRSAVE_TIMER */
0x01999A00L,        /* OFFHOOK_THRESH */
0x00F00000L,        /* VBAT_TRACK_MIN */
0x00F00000L,        /* VBAT_TRACK_MIN_RNG */
0x00200000L,        /* DCDC_FSW_NORM */
0x00200000L,        /* DCDC_FSW_NORM_LO */
0x00200000L,        /* DCDC_FSW_RING */
0x00200000L,        /* DCDC_FSW_RING_LO */
0x0D980000L,        /* DCDC_DIN_LIM */
0x00C00000L,        /* DCDC_VOUT_LIM */
0x0ADD5500L,        /* DCDC_ANA_SCALE */
0x00800000L,        /* THERM_DBI */
0x00FFFFFFL,        /* VOV_DCDC_SLOPE */
0x00A18937L,        /* VOV_DCDC_OS */
0x00A18937L,        /* VOV_RING_BAT_DCDC */
0x00E49BA5L,        /* VOV_RING_BAT_MAX */
0x01018900L,        /* DCDC_VERR */
0x0080C480L,        /* DCDC_VERR_HYST */
0x00400000L,        /* PD_UVLO */
0x00400000L,        /* PD_OVLO */
0x00200000L,        /* PD_OCLO */
0x00400000L,        /* PD_SWDRV */
0x00000000L,        /* DCDC_UVPOL */
0x00200000L,        /* DCDC_RNGTYPE */
0x00300000L,        /* DCDC_ANA_TOFF */
0x00100000L,        /* DCDC_ANA_TONMIN */
0x00FFC000L,        /* DCDC_ANA_TONMAX */
0xFF,               /* IRQEN1 */
0xFF,               /* IRQEN2 */
0xFF,               /* IRQEN3 */
0xFF,               /* IRQEN4 */
0x30,               /* ENHANCE */
0x01,               /* DAA_CNTL */
0x3F                /* AUTO */
};


#ifdef SIVOICE_MULTI_BOM_SUPPORT

int si3217x_genconf_multi_max_preset = 7;

Si3217x_General_Cfg Si3217x_General_Configuration_MultiBOM[] = {
{/* 0: FLBK + GATE DRIVER */
0x73,                       /* DEVICE_KEY */
BO_DCDC_FLYBACK,            /* BOM_OPT */
BO_GDRV_INSTALLED,          /* GDRV_OPTION */
VDC_7P0_20P0,               /* VDC_RANGE_OPT */
VDAA_ENABLED,               /* DAA_ENABLE */
AUTO_ZCAL_ENABLED,          /* ZCAL_ENABLE */
BO_STD_BOM,                 /* PM_BOM */
1100L,                      /* I_OITHRESH (1100mA) */
900L,                       /* I_OITHRESH_LO (900mA) */
1700L,                      /* I_OITHRESH_HI (1700mA) */
136000L,                    /* V_OVTHRESH (136000mV) */
5000L,                      /* V_UVTHRESH (5000mV) */
1000L,                      /* V_UVHYST (1000mV) */
0x00000000L,                /* DCDC_FSW_VTHLO */
0x00000000L,                /* DCDC_FSW_VHYST */
0x0048D15BL,                /* P_TH_HVIC */
0x07FEB800L,                /* COEF_P_HVIC */
0x00083120L,                /* BAT_HYST */
0x03D70A20L,                /* VBATH_EXPECT (60.00V) */
0x070A3D3AL,                /* VBATR_EXPECT (110.00V) */
0x0FFF0000L,                /* PWRSAVE_TIMER */
0x01999A00L,                /* OFFHOOK_THRESH */
0x00F00000L,                /* VBAT_TRACK_MIN */
0x00F00000L,                /* VBAT_TRACK_MIN_RNG */
0x00200000L,                /* DCDC_FSW_NORM */
0x00200000L,                /* DCDC_FSW_NORM_LO */
0x00200000L,                /* DCDC_FSW_RINGING */
0x00200000L,                /* DCDC_FSW_RINGING_LO */
0x0D980000L,                /* DCDC_DIN_LIM */
0x00C00000L,                /* DCDC_VOUT_LIM */
0x0ADD5500L,                /* DCDC_ANA_SCALE */
0x00800000L,                /* THERM_DBI */
0x00FFFFFFL,                /* VOV_DCDC_SLOPE */
0x00A18937L,                /* VOV_DCDC_OS */
0x00A18937L,                /* VOV_RING_BAT_DCDC */
0x00E49BA5L,                /* VOV_RING_BAT_MAX */
0x01018900L,                /* DCDC_VERR */
0x0080C480L,                /* DCDC_VERR_HYST */
0x00400000L,                /* PD_UVLO */
0x00400000L,                /* PD_OVLO */
0x00200000L,                /* PD_OCLO */
0x00400000L,                /* PD_SWDRV */
0x00000000L,                /* DCDC_UVPOL */
0x00200000L,                /* DCDC_RNGTYPE */
0x00300000L,                /* DCDC_ANA_TOFF */
0x00100000L,                /* DCDC_ANA_TONMIN */
0x00FFC000L,                /* DCDC_ANA_TONMAX */
0x40,                       /* IRQEN1 */
0x12,                       /* IRQEN */
0x03,                       /* IRQEN3 */
0x00,                       /* IRQEN4 */
0x30,                       /* ENHANCE */
0x01,                       /* DAA_CNTL */
0x3F,                       /* AUTO */
},
{/* 1:  PMOS BB 3-6VDC + GATE DRIVER */
0x73,                       /* DEVICE_KEY */
BO_DCDC_PMOS_BUCK_BOOST,    /* BOM_OPT */
BO_GDRV_INSTALLED,          /* GDRV_OPTION */
VDC_3P0_6P0,                /* VDC_RANGE_OPT */
VDAA_ENABLED,               /* DAA_ENABLE */
AUTO_ZCAL_ENABLED,          /* ZCAL_ENABLE */
BO_STD_BOM,                 /* PM_BOM */
1100L,                      /* I_OITHRESH (1100mA) */
900L,                       /* I_OITHRESH_LO (900mA) */
1700L,                      /* I_OITHRESH_HI (1700mA) */
136000L,                    /* V_OVTHRESH (136000mV) */
5000L,                      /* V_UVTHRESH (5000mV) */
1000L,                      /* V_UVHYST (1000mV) */
0x00000000L,                /* DCDC_FSW_VTHLO */
0x00000000L,                /* DCDC_FSW_VHYST */
0x0048D15BL,                /* P_TH_HVIC */
0x07FEB800L,                /* COEF_P_HVIC */
0x00083120L,                /* BAT_HYST */
0x03D70A20L,                /* VBATH_EXPECT (60.00V) */
0x051EB82AL,                /* VBATR_EXPECT (80.00V) */
0x0FFF0000L,                /* PWRSAVE_TIMER */
0x01999A00L,                /* OFFHOOK_THRESH */
0x00C00000L,                /* VBAT_TRACK_MIN */
0x01800000L,                /* VBAT_TRACK_MIN_RNG */
0x00200000L,                /* DCDC_FSW_NORM */
0x00200000L,                /* DCDC_FSW_NORM_LO */
0x00200000L,                /* DCDC_FSW_RINGING */
0x00200000L,                /* DCDC_FSW_RINGING_LO */
0x0D980000L,                /* DCDC_DIN_LIM */
0x00C00000L,                /* DCDC_VOUT_LIM */
0x0ADD5500L,                /* DCDC_ANA_SCALE */
0x00800000L,                /* THERM_DBI */
0x00FFFFFFL,                /* VOV_DCDC_SLOPE */
0x00A18937L,                /* VOV_DCDC_OS */
0x00A18937L,                /* VOV_RING_BAT_DCDC */
0x00E49BA5L,                /* VOV_RING_BAT_MAX */
0x01018900L,                /* DCDC_VERR */
0x0080C480L,                /* DCDC_VERR_HYST */
0x00400000L,                /* PD_UVLO */
0x00400000L,                /* PD_OVLO */
0x00200000L,                /* PD_OCLO */
0x00400000L,                /* PD_SWDRV */
0x00000000L,                /* DCDC_UVPOL */
0x00200000L,                /* DCDC_RNGTYPE */
0x00300000L,                /* DCDC_ANA_TOFF */
0x00200000L,                /* DCDC_ANA_TONMIN */
0x01F00000L,                /* DCDC_ANA_TONMAX */
0x40,                       /* IRQEN1 */
0x12,                       /* IRQEN */
0x03,                       /* IRQEN3 */
0x00,                       /* IRQEN4 */
0x30,                       /* ENHANCE */
0x01,                       /* DAA_CNTL */
0x3F,                       /* AUTO */
},
{/* 2:  PMOS BB 9-24VDC + GATE DRIVER */
0x73,                       /* DEVICE_KEY */
BO_DCDC_PMOS_BUCK_BOOST,    /* BOM_OPT */
BO_GDRV_INSTALLED,          /* GDRV_OPTION */
VDC_9P0_24P0,               /* VDC_RANGE_OPT */
VDAA_ENABLED,               /* DAA_ENABLE */
AUTO_ZCAL_ENABLED,          /* ZCAL_ENABLE */
BO_STD_BOM,                 /* PM_BOM */
1100L,                      /* I_OITHRESH (1100mA) */
900L,                       /* I_OITHRESH_LO (900mA) */
1700L,                      /* I_OITHRESH_HI (1700mA) */
136000L,                    /* V_OVTHRESH (136000mV) */
5000L,                      /* V_UVTHRESH (5000mV) */
1000L,                      /* V_UVHYST (1000mV) */
0x00000000L,                /* DCDC_FSW_VTHLO */
0x00000000L,                /* DCDC_FSW_VHYST */
0x0048D15BL,                /* P_TH_HVIC */
0x07FEB800L,                /* COEF_P_HVIC */
0x00083120L,                /* BAT_HYST */
0x03D70A20L,                /* VBATH_EXPECT (60.00V) */
0x070A3D3AL,                /* VBATR_EXPECT (110.00V) */
0x0FFF0000L,                /* PWRSAVE_TIMER */
0x01999A00L,                /* OFFHOOK_THRESH */
0x00C00000L,                /* VBAT_TRACK_MIN */
0x01800000L,                /* VBAT_TRACK_MIN_RNG */
0x00200000L,                /* DCDC_FSW_NORM */
0x00200000L,                /* DCDC_FSW_NORM_LO */
0x00200000L,                /* DCDC_FSW_RINGING */
0x00200000L,                /* DCDC_FSW_RINGING_LO */
0x0D980000L,                /* DCDC_DIN_LIM */
0x00C00000L,                /* DCDC_VOUT_LIM */
0x0ADD5500L,                /* DCDC_ANA_SCALE */
0x00800000L,                /* THERM_DBI */
0x00FFFFFFL,                /* VOV_DCDC_SLOPE */
0x00A18937L,                /* VOV_DCDC_OS */
0x00A18937L,                /* VOV_RING_BAT_DCDC */
0x00E49BA5L,                /* VOV_RING_BAT_MAX */
0x01018900L,                /* DCDC_VERR */
0x0080C480L,                /* DCDC_VERR_HYST */
0x00400000L,                /* PD_UVLO */
0x00400000L,                /* PD_OVLO */
0x00200000L,                /* PD_OCLO */
0x00400000L,                /* PD_SWDRV */
0x00000000L,                /* DCDC_UVPOL */
0x00200000L,                /* DCDC_RNGTYPE */
0x00300000L,                /* DCDC_ANA_TOFF */
0x00200000L,                /* DCDC_ANA_TONMIN */
0x00D00000L,                /* DCDC_ANA_TONMAX */
0x40,                       /* IRQEN1 */
0x12,                       /* IRQEN */
0x03,                       /* IRQEN3 */
0x00,                       /* IRQEN4 */
0x30,                       /* ENHANCE */
0x01,                       /* DAA_CNTL */
0x3F,                       /* AUTO */
},
{/* 3:  FLBK */
0x73,                       /* DEVICE_KEY */
BO_DCDC_FLYBACK,            /* BOM_OPT */
BO_GDRV_NOT_INSTALLED,      /* GDRV_OPTION */
VDC_7P0_20P0,               /* VDC_RANGE_OPT */
VDAA_ENABLED,               /* DAA_ENABLE */
AUTO_ZCAL_ENABLED,          /* ZCAL_ENABLE */
BO_STD_BOM,                 /* PM_BOM */
1100L,                      /* I_OITHRESH (1100mA) */
900L,                       /* I_OITHRESH_LO (900mA) */
1700L,                      /* I_OITHRESH_HI (1700mA) */
136000L,                    /* V_OVTHRESH (136000mV) */
5000L,                      /* V_UVTHRESH (5000mV) */
1000L,                      /* V_UVHYST (1000mV) */
0x00000000L,                /* DCDC_FSW_VTHLO */
0x00000000L,                /* DCDC_FSW_VHYST */
0x0048D15BL,                /* P_TH_HVIC */
0x07FEB800L,                /* COEF_P_HVIC */
0x00083120L,                /* BAT_HYST */
0x03D70A20L,                /* VBATH_EXPECT (60.00V) */
0x070A3D3AL,                /* VBATR_EXPECT (110.00V) */
0x0FFF0000L,                /* PWRSAVE_TIMER */
0x01999A00L,                /* OFFHOOK_THRESH */
0x00F00000L,                /* VBAT_TRACK_MIN */
0x00F00000L,                /* VBAT_TRACK_MIN_RNG */
0x00200000L,                /* DCDC_FSW_NORM */
0x00200000L,                /* DCDC_FSW_NORM_LO */
0x00200000L,                /* DCDC_FSW_RINGING */
0x00200000L,                /* DCDC_FSW_RINGING_LO */
0x0D980000L,                /* DCDC_DIN_LIM */
0x00C00000L,                /* DCDC_VOUT_LIM */
0x0ADD5500L,                /* DCDC_ANA_SCALE */
0x00800000L,                /* THERM_DBI */
0x00FFFFFFL,                /* VOV_DCDC_SLOPE */
0x00A18937L,                /* VOV_DCDC_OS */
0x00A18937L,                /* VOV_RING_BAT_DCDC */
0x00E49BA5L,                /* VOV_RING_BAT_MAX */
0x01018900L,                /* DCDC_VERR */
0x0080C480L,                /* DCDC_VERR_HYST */
0x00400000L,                /* PD_UVLO */
0x00400000L,                /* PD_OVLO */
0x00200000L,                /* PD_OCLO */
0x00400000L,                /* PD_SWDRV */
0x00000000L,                /* DCDC_UVPOL */
0x00200000L,                /* DCDC_RNGTYPE */
0x00300000L,                /* DCDC_ANA_TOFF */
0x00100000L,                /* DCDC_ANA_TONMIN */
0x00FFC000L,                /* DCDC_ANA_TONMAX */
0x40,                       /* IRQEN1 */
0x12,                       /* IRQEN */
0x03,                       /* IRQEN3 */
0x00,                       /* IRQEN4 */
0x30,                       /* ENHANCE */
0x01,                       /* DAA_CNTL */
0x3F,                       /* AUTO */
},
{/* 4:  BJT BUCK-BOOST */
0x73,                       /* DEVICE_KEY */
BO_DCDC_BUCK_BOOST,         /* BOM_OPT */
BO_GDRV_NOT_INSTALLED,      /* GDRV_OPTION */
VDC_8P0_16P0,               /* VDC_RANGE_OPT */
VDAA_ENABLED,               /* DAA_ENABLE */
AUTO_ZCAL_ENABLED,          /* ZCAL_ENABLE */
BO_STD_BOM,                 /* PM_BOM */
1100L,                      /* I_OITHRESH (1100mA) */
900L,                       /* I_OITHRESH_LO (900mA) */
1700L,                      /* I_OITHRESH_HI (1700mA) */
136000L,                    /* V_OVTHRESH (136000mV) */
5000L,                      /* V_UVTHRESH (5000mV) */
1000L,                      /* V_UVHYST (1000mV) */
0x00000000L,                /* DCDC_FSW_VTHLO */
0x00000000L,                /* DCDC_FSW_VHYST */
0x0048D15BL,                /* P_TH_HVIC */
0x07FEB800L,                /* COEF_P_HVIC */
0x0020C480L,                /* BAT_HYST */
0x03D70A20L,                /* VBATH_EXPECT (60.00V) */
0x06147AB2L,                /* VBATR_EXPECT (95.00V) */
0x0FFF0000L,                /* PWRSAVE_TIMER */
0x01999A00L,                /* OFFHOOK_THRESH */
0x00C00000L,                /* VBAT_TRACK_MIN */
0x01800000L,                /* VBAT_TRACK_MIN_RNG */
0x00200000L,                /* DCDC_FSW_NORM */
0x00200000L,                /* DCDC_FSW_NORM_LO */
0x00200000L,                /* DCDC_FSW_RINGING */
0x00200000L,                /* DCDC_FSW_RINGING_LO */
0x0D980000L,                /* DCDC_DIN_LIM */
0x00C00000L,                /* DCDC_VOUT_LIM */
0x0ADD5500L,                /* DCDC_ANA_SCALE */
0x00800000L,                /* THERM_DBI */
0x00FFFFFFL,                /* VOV_DCDC_SLOPE */
0x00A18937L,                /* VOV_DCDC_OS */
0x00A18937L,                /* VOV_RING_BAT_DCDC */
0x00E49BA5L,                /* VOV_RING_BAT_MAX */
0x01018900L,                /* DCDC_VERR */
0x0080C480L,                /* DCDC_VERR_HYST */
0x00400000L,                /* PD_UVLO */
0x00400000L,                /* PD_OVLO */
0x00200000L,                /* PD_OCLO */
0x00400000L,                /* PD_SWDRV */
0x00000000L,                /* DCDC_UVPOL */
0x00000000L,                /* DCDC_RNGTYPE */
0x00600000L,//0x00600000L,                /* DCDC_ANA_TOFF */
0x003F0000L,//0x00300000L,                /* DCDC_ANA_TONMIN */
0x00800000L,//0x00800000L,                /* DCDC_ANA_TONMAX */
0x40,                       /* IRQEN1 */
0x12,                       /* IRQEN */
0x03,                       /* IRQEN3 */
0x00,                       /* IRQEN4 */
0x30,                       /* ENHANCE PWRSAVE_EN NOT*/
0x00,                       /* DAA_CNTL */
0x3F,                       /* AUTO */
},
{ /* 5:  PMOS BB 3-6VDC */
0x73,                       /* DEVICE_KEY */
BO_DCDC_PMOS_BUCK_BOOST,    /* BOM_OPT */
BO_GDRV_NOT_INSTALLED,      /* GDRV_OPTION */
VDC_3P0_6P0,                /* VDC_RANGE_OPT */
VDAA_ENABLED,               /* DAA_ENABLE */
AUTO_ZCAL_ENABLED,          /* ZCAL_ENABLE */
BO_STD_BOM,                 /* PM_BOM */
1100L,                      /* I_OITHRESH (1100mA) */
900L,                       /* I_OITHRESH_LO (900mA) */
1700L,                      /* I_OITHRESH_HI (1700mA) */
136000L,                    /* V_OVTHRESH (136000mV) */
5000L,                      /* V_UVTHRESH (5000mV) */
1000L,                      /* V_UVHYST (1000mV) */
0x00000000L,                /* DCDC_FSW_VTHLO */
0x00000000L,                /* DCDC_FSW_VHYST */
0x0048D15BL,                /* P_TH_HVIC */
0x07FEB800L,                /* COEF_P_HVIC */
0x00083120L,                /* BAT_HYST */
0x03D70A20L,                /* VBATH_EXPECT (60.00V) */
0x051EB82AL,                /* VBATR_EXPECT (80.00V) */
0x0FFF0000L,                /* PWRSAVE_TIMER */
0x01999A00L,                /* OFFHOOK_THRESH */
0x00C00000L,                /* VBAT_TRACK_MIN */
0x01800000L,                /* VBAT_TRACK_MIN_RNG */
0x00200000L,                /* DCDC_FSW_NORM */
0x00200000L,                /* DCDC_FSW_NORM_LO */
0x00200000L,                /* DCDC_FSW_RINGING */
0x00200000L,                /* DCDC_FSW_RINGING_LO */
0x0D980000L,                /* DCDC_DIN_LIM */
0x00C00000L,                /* DCDC_VOUT_LIM */
0x0ADD5500L,                /* DCDC_ANA_SCALE */
0x00800000L,                /* THERM_DBI */
0x00FFFFFFL,                /* VOV_DCDC_SLOPE */
0x00A18937L,                /* VOV_DCDC_OS */
0x00A18937L,                /* VOV_RING_BAT_DCDC */
0x00E49BA5L,                /* VOV_RING_BAT_MAX */
0x01018900L,                /* DCDC_VERR */
0x0080C480L,                /* DCDC_VERR_HYST */
0x00400000L,                /* PD_UVLO */
0x00400000L,                /* PD_OVLO */
0x00200000L,                /* PD_OCLO */
0x00400000L,                /* PD_SWDRV */
0x00000000L,                /* DCDC_UVPOL */
0x00200000L,                /* DCDC_RNGTYPE */
0x00300000L,                /* DCDC_ANA_TOFF */
0x00200000L,                /* DCDC_ANA_TONMIN */
0x01F00000L,                /* DCDC_ANA_TONMAX */
0x40,                       /* IRQEN1 */
0x12,                       /* IRQEN */
0x03,                       /* IRQEN3 */
0x00,                       /* IRQEN4 */
0x30,                       /* ENHANCE */
0x01,                       /* DAA_CNTL */
0x3F,                       /* AUTO */
},
{ /* 6:  PMOS BB 9-24VDC */
0x73,                       /* DEVICE_KEY */
BO_DCDC_PMOS_BUCK_BOOST,    /* BOM_OPT */
BO_GDRV_NOT_INSTALLED,      /* GDRV_OPTION */
VDC_9P0_24P0,               /* VDC_RANGE_OPT */
VDAA_ENABLED,               /* DAA_ENABLE */
AUTO_ZCAL_ENABLED,          /* ZCAL_ENABLE */
BO_STD_BOM,                 /* PM_BOM */
1100L,                      /* I_OITHRESH (1100mA) */
900L,                       /* I_OITHRESH_LO (900mA) */
1700L,                      /* I_OITHRESH_HI (1700mA) */
136000L,                    /* V_OVTHRESH (136000mV) */
5000L,                      /* V_UVTHRESH (5000mV) */
1000L,                      /* V_UVHYST (1000mV) */
0x00000000L,                /* DCDC_FSW_VTHLO */
0x00000000L,                /* DCDC_FSW_VHYST */
0x0048D15BL,                /* P_TH_HVIC */
0x07FEB800L,                /* COEF_P_HVIC */
0x00083120L,                /* BAT_HYST */
0x03D70A20L,                /* VBATH_EXPECT (60.00V) */
0x070A3D3AL,                /* VBATR_EXPECT (110.00V) */
0x0FFF0000L,                /* PWRSAVE_TIMER */
0x01999A00L,                /* OFFHOOK_THRESH */
0x00C00000L,                /* VBAT_TRACK_MIN */
0x01800000L,                /* VBAT_TRACK_MIN_RNG */
0x00200000L,                /* DCDC_FSW_NORM */
0x00200000L,                /* DCDC_FSW_NORM_LO */
0x00200000L,                /* DCDC_FSW_RINGING */
0x00200000L,                /* DCDC_FSW_RINGING_LO */
0x0D980000L,                /* DCDC_DIN_LIM */
0x00C00000L,                /* DCDC_VOUT_LIM */
0x0ADD5500L,                /* DCDC_ANA_SCALE */
0x00800000L,                /* THERM_DBI */
0x00FFFFFFL,                /* VOV_DCDC_SLOPE */
0x00A18937L,                /* VOV_DCDC_OS */
0x00A18937L,                /* VOV_RING_BAT_DCDC */
0x00E49BA5L,                /* VOV_RING_BAT_MAX */
0x01018900L,                /* DCDC_VERR */
0x0080C480L,                /* DCDC_VERR_HYST */
0x00400000L,                /* PD_UVLO */
0x00400000L,                /* PD_OVLO */
0x00200000L,                /* PD_OCLO */
0x00400000L,                /* PD_SWDRV */
0x00000000L,                /* DCDC_UVPOL */
0x00200000L,                /* DCDC_RNGTYPE */
0x00300000L,                /* DCDC_ANA_TOFF */
0x00200000L,                /* DCDC_ANA_TONMIN */
0x00D00000L,                /* DCDC_ANA_TONMAX */
0x40,                       /* IRQEN1 */
0x12,                       /* IRQEN */
0x03,                       /* IRQEN3 */
0x00,                       /* IRQEN4 */
0x30,                       /* ENHANCE */
0x01,                       /* DAA_CNTL */
0x3F,                       /* AUTO */
}
};

#endif

Si3217x_DTMFDec_Cfg Si3217x_DTMFDec_Presets[] = {
	{0x02d40000L,
     0x1a660000L,
     0x02d40000L,
     0x06ba0000L,
     0x1dcc0000L,
     0x033f0000L,
     0x0bd30000L,
     0x19d20000L,
     0x04150000L,
     0x188F0000L,
     0x04150000L,
     0x0d970000L,
     0x18620000L,
     0x0f1c0000L}
};
Si3217x_GPIO_Cfg Si3217x_GPIO_Configuration = {
0x00,     /* GPIO_OE */
0x00,     /* GPIO_ANA */
0x00,     /* GPIO_DIR */
0x00,     /* GPIO_MAN */
0x00,     /* GPIO_POL */
0x00,     /* GPIO_OD */
0x00     /* BATSELMAP */
};
Si3217x_CI_Cfg Si3217x_CI_Presets [] = {
{0}
};
Si3217x_audioGain_Cfg Si3217x_audioGain_Presets [] = {
{0x1377080L,0, 0x0L, 0x0L, 0x0L, 0x0L},
{0x80C3180L,0, 0x0L, 0x0L, 0x0L, 0x0L}
};

Si3217x_Ring_Cfg Si3217x_Ring_Presets[] ={
{
/*
	Loop = 500.0 ft @ 0.044 ohms/ft, REN = 5, Rcpe = 600 ohms
	Rprot = 30 ohms, Type = LPR, Waveform = SINE
*/
0x00050000L,	/* RTPER */
0x07EFE000L,	/* RINGFR (20.000 Hz) */
0x001B9F2EL,	/* RINGAMP (45.000 vrms)  */
0x00000000L,	/* RINGPHAS */
0x00000000L,	/* RINGOF (0.000 vdc) */
0x15E5200EL,	/* SLOPE_RING (100.000 ohms) */
0x00D16348L,	/* IRING_LIM (90.000 mA) */
0x0087CFF5L,	/* RTACTH (75.000 mA) */
0x0FFFFFFFL,	/* RTDCTH (450.000 mA) */
0x00006000L,	/* RTACDB (75.000 ms) */
0x00006000L,	/* RTDCDB (75.000 ms) */
0x00C49BA0L,	/* VOV_RING_BAT (12.000 v) */
0x00000000L,	/* VOV_RING_GND (0.000 v) */
0x0558ABFCL,	/* VBATR_EXPECT (83.537 v) */
0x80,			/* RINGTALO (2.000 s) */
0x3E,			/* RINGTAHI */
0x00,			/* RINGTILO (4.000 s) */
0x7D,			/* RINGTIHI */
0x00000000L,	/* ADAP_RING_MIN_I */
0x00003000L,	/* COUNTER_IRING_VAL */
0x00051EB8L,	/* COUNTER_VTR_VAL */
0x00000000L,	/* CONST_028 */
0x00000000L,	/* CONST_032 */
0x00000000L,	/* CONST_038 */
0x00000000L,	/* CONST_046 */
0x00000000L,	/* RRD_DELAY */
0x00000000L,	/* RRD_DELAY2 */
0x01893740L,	/* DCDC_VREF_MIN_RNG */
0x58,			/* RINGCON */
0x00,			/* USERSTAT */
0x02AC55FEL,	/* VCM_RING (38.769 v) */
0x02AC55FEL,	/* VCM_RING_FIXED */
0x003126E8L,	/* DELTA_VCM */
0x00000000L,	/* DCDC_RNGTYPE */
},  /* RING_F20_45VRMS_0VDC_LPR */
{
/*
	Loop = 500.0 ft @ 0.044 ohms/ft, REN = 5, Rcpe = 600 ohms
	Rprot = 30 ohms, Type = BALANCED, Waveform = SINE
*/
0x00050000L,	/* RTPER */
0x07EFE000L,	/* RINGFR (20.000 Hz) */
0x001B9F2EL,	/* RINGAMP (45.000 vrms)  */
0x00000000L,	/* RINGPHAS */
0x00000000L,	/* RINGOF (0.000 vdc) */
0x15E5200EL,	/* SLOPE_RING (100.000 ohms) */
0x00D16348L,	/* IRING_LIM (90.000 mA) */
0x0068E9B4L,	/* RTACTH (57.936 mA) */
0x0FFFFFFFL,	/* RTDCTH (450.000 mA) */
0x00006000L,	/* RTACDB (75.000 ms) */
0x00006000L,	/* RTDCDB (75.000 ms) */
0x00C49BA0L,	/* VOV_RING_BAT (12.000 v) */
0x00000000L,	/* VOV_RING_GND (0.000 v) */
0x0558ABFCL,	/* VBATR_EXPECT (83.537 v) */
0x80,			/* RINGTALO (2.000 s) */
0x3E,			/* RINGTAHI */
0x00,			/* RINGTILO (4.000 s) */
0x7D,			/* RINGTIHI */
0x00000000L,	/* ADAP_RING_MIN_I */
0x00003000L,	/* COUNTER_IRING_VAL */
0x00051EB8L,	/* COUNTER_VTR_VAL */
0x00000000L,	/* CONST_028 */
0x00000000L,	/* CONST_032 */
0x00000000L,	/* CONST_038 */
0x00000000L,	/* CONST_046 */
0x00000000L,	/* RRD_DELAY */
0x00000000L,	/* RRD_DELAY2 */
0x01893740L,	/* DCDC_VREF_MIN_RNG */
0x58,			/* RINGCON */
0x00,			/* USERSTAT */
0x02AC55FEL,	/* VCM_RING (38.769 v) */
0x02AC55FEL,	/* VCM_RING_FIXED */
0x003126E8L,	/* DELTA_VCM */
0x00000000L,	/* DCDC_RNGTYPE */
},  /* RING_F20_45VRMS_0VDC_BAL */
{
/*
	Loop = 500.0 ft @ 0.044 ohms/ft, REN = 5, Rcpe = 600 ohms
	Rprot = 30 ohms, Type = LPR, Waveform = SINE
*/
0x00050000L,	/* RTPER */
0x07EFE000L,	/* RINGFR (20.000 Hz) */
0x001B9F2EL,	/* RINGAMP (45.000 vrms)  */
0x00000000L,	/* RINGPHAS */
0x00000000L,	/* RINGOF (0.000 vdc) */
0x15E5200EL,	/* SLOPE_RING (100.000 ohms) */
0x00D16348L,	/* IRING_LIM (90.000 mA) */
0x0068E9B4L,	/* RTACTH (57.936 mA) */
0x0FFFFFFFL,	/* RTDCTH (450.000 mA) */
0x00006000L,	/* RTACDB (75.000 ms) */
0x00006000L,	/* RTDCDB (75.000 ms) */
0x00C49BA0L,	/* VOV_RING_BAT (12.000 v) */
0x00000000L,	/* VOV_RING_GND (0.000 v) */
0x0558ABFCL,	/* VBATR_EXPECT (83.537 v) */
0x40,			/* RINGTALO (1.000 s) */
0x1F,			/* RINGTAHI */
0x00,			/* RINGTILO (4.000 s) */
0x7D,			/* RINGTIHI */
0x00000000L,	/* ADAP_RING_MIN_I */
0x00003000L,	/* COUNTER_IRING_VAL */
0x00051EB8L,	/* COUNTER_VTR_VAL */
0x00000000L,	/* CONST_028 */
0x00000000L,	/* CONST_032 */
0x00000000L,	/* CONST_038 */
0x00000000L,	/* CONST_046 */
0x00000000L,	/* RRD_DELAY */
0x00000000L,	/* RRD_DELAY2 */
0x01893740L,	/* DCDC_VREF_MIN_RNG */
0x58,			/* RINGCON */
0x00,			/* USERSTAT */
0x02AC55FEL,	/* VCM_RING (38.769 v) */
0x02AC55FEL,	/* VCM_RING_FIXED */
0x003126E8L,	/* DELTA_VCM */
0x00000000L,	/* DCDC_RNGTYPE */
},  /* RING_F20_45VRMS_0VDC_LPR_ON1000_OFF4000 */
{
/*
	Loop = 500.0 ft @ 0.044 ohms/ft, REN = 5, Rcpe = 600 ohms
	Rprot = 30 ohms, Type = LPR, Waveform = SINE
*/
0x00040000L,	/* RTPER */
0x07E6C000L,	/* RINGFR (25.000 Hz) */
0x0024DF0DL,	/* RINGAMP (48.000 vrms)  */
0x00000000L,	/* RINGPHAS */
0x00000000L,	/* RINGOF (0.000 vdc) */
0x15E5200EL,	/* SLOPE_RING (100.000 ohms) */
0x00A2DB71L,	/* IRING_LIM (70.000 mA) */
0x0059D242L,	/* RTACTH (62.003 mA) */
0x0FFFFFFFL,	/* RTDCTH (450.000 mA) */
0x00008000L,	/* RTACDB (75.000 ms) */
0x00008000L,	/* RTDCDB (75.000 ms) */
0x00C49BA0L,	/* VOV_RING_BAT (12.000 v) */
0x00000000L,	/* VOV_RING_GND (0.000 v) */
0x05A75077L,	/* VBATR_EXPECT (88.337 v) */
0x40,			/* RINGTALO (1.000 s) */
0x1F,			/* RINGTAHI */
0xA0,			/* RINGTILO (4.500 s) */
0x8C,			/* RINGTIHI */
0x00000000L,	/* ADAP_RING_MIN_I */
0x00003000L,	/* COUNTER_IRING_VAL */
0x00066666L,	/* COUNTER_VTR_VAL */
0x00000000L,	/* CONST_028 */
0x00000000L,	/* CONST_032 */
0x00000000L,	/* CONST_038 */
0x00000000L,	/* CONST_046 */
0x00000000L,	/* RRD_DELAY */
0x00000000L,	/* RRD_DELAY2 */
0x01893740L,	/* DCDC_VREF_MIN_RNG */
0x58,			/* RINGCON */
0x00,			/* USERSTAT */
0x02D3A83BL,	/* VCM_RING (41.169 v) */
0x02D3A83BL,	/* VCM_RING_FIXED */
0x003126E8L,	/* DELTA_VCM */
0x00000000L,	/* DCDC_RNGTYPE */
}    /* RING_F25_48VRMS_0VDC_LPR_ON1000_OFF4500 */
};

Si3217x_DCfeed_Cfg Si3217x_DCfeed_Presets[] = {
{
0x1C8A024CL,	/* SLOPE_VLIM */
0x1F909679L,	/* SLOPE_RFEED */
0x0040A0E0L,	/* SLOPE_ILIM */
0x1D5B21A9L,	/* SLOPE_DELTA1 */
0x1DD87A3EL,	/* SLOPE_DELTA2 */
0x05A38633L,	/* V_VLIM (48.000 v) */
0x050D2839L,	/* V_RFEED (43.000 v) */
0x03FE7F0FL,	/* V_ILIM  (34.000 v) */
0x00B4F3C3L,	/* CONST_RFEED (15.000 mA) */
0x005D0FA6L,	/* CONST_ILIM (20.000 mA) */
0x002D8D96L,	/* I_VLIM (0.000 mA) */
0x005B0AFBL,	/* LCRONHK (10.000 mA) */
0x006D4060L,	/* LCROFFHK (12.000 mA) */
0x00008000L,	/* LCRDBI (5.000 ms) */
0x0048D595L,	/* LONGHITH (8.000 mA) */
0x003FBAE2L,	/* LONGLOTH (7.000 mA) */
0x00008000L,	/* LONGDBI (5.000 ms) */
0x000F0000L,	/* LCRMASK (150.000 ms) */
0x00080000L,	/* LCRMASK_POLREV (80.000 ms) */
0x00140000L,	/* LCRMASK_STATE (200.000 ms) */
0x00140000L,	/* LCRMASK_LINECAP (200.000 ms) */
0x01BA5E35L,	/* VCM_OH (27.000 v) */
0x0051EB85L,	/* VOV_BAT (5.000 v) */
0x00418937L,	/* VOV_GND (4.000 v) */
},  /* DCFEED_48V_20MA */
{
0x1C8A024CL,	/* SLOPE_VLIM */
0x1EE08C11L,	/* SLOPE_RFEED */
0x0040A0E0L,	/* SLOPE_ILIM */
0x1C940D71L,	/* SLOPE_DELTA1 */
0x1DD87A3EL,	/* SLOPE_DELTA2 */
0x05A38633L,	/* V_VLIM (48.000 v) */
0x050D2839L,	/* V_RFEED (43.000 v) */
0x03FE7F0FL,	/* V_ILIM  (34.000 v) */
0x01241BC9L,	/* CONST_RFEED (15.000 mA) */
0x0074538FL,	/* CONST_ILIM (25.000 mA) */
0x002D8D96L,	/* I_VLIM (0.000 mA) */
0x005B0AFBL,	/* LCRONHK (10.000 mA) */
0x006D4060L,	/* LCROFFHK (12.000 mA) */
0x00008000L,	/* LCRDBI (5.000 ms) */
0x0048D595L,	/* LONGHITH (8.000 mA) */
0x003FBAE2L,	/* LONGLOTH (7.000 mA) */
0x00008000L,	/* LONGDBI (5.000 ms) */
0x000F0000L,	/* LCRMASK (150.000 ms) */
0x00080000L,	/* LCRMASK_POLREV (80.000 ms) */
0x00140000L,	/* LCRMASK_STATE (200.000 ms) */
0x00140000L,	/* LCRMASK_LINECAP (200.000 ms) */
0x01BA5E35L,	/* VCM_OH (27.000 v) */
0x0051EB85L,	/* VOV_BAT (5.000 v) */
0x00418937L,	/* VOV_GND (4.000 v) */
},  /* DCFEED_48V_25MA */
{
0x1C8A024CL,	/* SLOPE_VLIM */
0x1D16D76CL,	/* SLOPE_RFEED */
0x0040A0E0L,	/* SLOPE_ILIM */
0xF1753922L,	/* SLOPE_DELTA1 */
0x1DD87A3EL,	/* SLOPE_DELTA2 */
0x05A38633L,	/* V_VLIM (48.000 v) */
0x050D2839L,	/* V_RFEED (43.000 v) */
0x03FE7F0FL,	/* V_ILIM  (34.000 v) */
0x02451D74L,	/* CONST_RFEED (15.000 mA) */
0x00B0D0EEL,	/* CONST_ILIM (38.000 mA) */
0x002D8D96L,	/* I_VLIM (0.000 mA) */
0x005B0AFBL,	/* LCRONHK (10.000 mA) */
0x006D4060L,	/* LCROFFHK (12.000 mA) */
0x00008000L,	/* LCRDBI (5.000 ms) */
0x0048D595L,	/* LONGHITH (8.000 mA) */
0x003FBAE2L,	/* LONGLOTH (7.000 mA) */
0x00008000L,	/* LONGDBI (5.000 ms) */
0x000F0000L,	/* LCRMASK (150.000 ms) */
0x00080000L,	/* LCRMASK_POLREV (80.000 ms) */
0x00140000L,	/* LCRMASK_STATE (200.000 ms) */
0x00140000L,	/* LCRMASK_LINECAP (200.000 ms) */
0x01BA5E35L,	/* VCM_OH (27.000 v) */
0x0051EB85L,	/* VOV_BAT (5.000 v) */
0x00418937L,	/* VOV_GND (4.000 v) */
},	/* DCFEED_48V_38MA */
{
0x1E655196L,	/* SLOPE_VLIM */
0x001904EFL,	/* SLOPE_RFEED */
0x0040A0E0L,	/* SLOPE_ILIM */
0x1B4CAD9EL,	/* SLOPE_DELTA1 */
0x1BB0F47CL,	/* SLOPE_DELTA2 */
0x05A38633L,	/* V_VLIM (48.000 v) */
0x043AA4A6L,	/* V_RFEED (36.000 v) */
0x025977EAL,	/* V_ILIM  (20.000 v) */
0x0068B19AL,	/* CONST_RFEED (18.000 mA) */
0x005D0FA6L,	/* CONST_ILIM (20.000 mA) */
0x002D8D96L,	/* I_VLIM (0.000 mA) */
0x005B0AFBL,	/* LCRONHK (10.000 mA) */
0x006D4060L,	/* LCROFFHK (12.000 mA) */
0x00008000L,	/* LCRDBI (5.000 ms) */
0x0048D595L,	/* LONGHITH (8.000 mA) */
0x003FBAE2L,	/* LONGLOTH (7.000 mA) */
0x00008000L,	/* LONGDBI (5.000 ms) */
0x000F0000L,	/* LCRMASK (150.000 ms) */
0x00080000L,	/* LCRMASK_POLREV (80.000 ms) */
0x00140000L,	/* LCRMASK_STATE (200.000 ms) */
0x00140000L,	/* LCRMASK_LINECAP (200.000 ms) */
0x01BA5E35L,	/* VCM_OH (27.000 v) */
0x0051EB85L,	/* VOV_BAT (5.000 v) */
0x00418937L,	/* VOV_GND (4.000 v) */
},  /* DCFEED_PSTN_DET_1 */
{
0x1A10433FL,	/* SLOPE_VLIM */
0x1C206275L,	/* SLOPE_RFEED */
0x0040A0E0L,	/* SLOPE_ILIM */
0x1C1F426FL,	/* SLOPE_DELTA1 */
0x1EB51625L,	/* SLOPE_DELTA2 */
0x041C91DBL,	/* V_VLIM (35.000 v) */
0x03E06C43L,	/* V_RFEED (33.000 v) */
0x038633E0L,	/* V_ILIM  (30.000 v) */
0x022E5DE5L,	/* CONST_RFEED (10.000 mA) */
0x005D0FA6L,	/* CONST_ILIM (20.000 mA) */
0x0021373DL,	/* I_VLIM (0.000 mA) */
0x005B0AFBL,	/* LCRONHK (10.000 mA) */
0x006D4060L,	/* LCROFFHK (12.000 mA) */
0x00008000L,	/* LCRDBI (5.000 ms) */
0x0048D595L,	/* LONGHITH (8.000 mA) */
0x003FBAE2L,	/* LONGLOTH (7.000 mA) */
0x00008000L,	/* LONGDBI (5.000 ms) */
0x000F0000L,	/* LCRMASK (150.000 ms) */
0x00080000L,	/* LCRMASK_POLREV (80.000 ms) */
0x00140000L,	/* LCRMASK_STATE (200.000 ms) */
0x00140000L,	/* LCRMASK_LINECAP (200.000 ms) */
0x01BA5E35L,	/* VCM_OH (27.000 v) */
0x0051EB85L,	/* VOV_BAT (5.000 v) */
0x00418937L,	/* VOV_GND (4.000 v) */
}    /* DCFEED_PSTN_DET_2 */
};

Si3217x_Impedance_Cfg Si3217x_Impedance_Presets[] ={
/* Source: Database file: cwdb.db */
/* Database information: */
/* parameters: zref=600_0_0 rprot=30 rfuse=0 emi_cap=10*/
{
{0x07F46C00L, 0x000E4600L, 0x00008580L, 0x1FFD6100L,    /* TXACEQ */
 0x07EF5000L, 0x0013F580L, 0x1FFDE000L, 0x1FFCB280L},   /* RXACEQ */
{0x0027CB00L, 0x1F8A8880L, 0x02801180L, 0x1F625C80L,    /* ECFIR/ECIIR */
 0x0314FB00L, 0x1E6B8E80L, 0x00C5FF00L, 0x1FC96F00L,
 0x1FFD1200L, 0x00023C00L, 0x0ED29D00L, 0x192A9400L},
{0x00810E00L, 0x1EFEBE80L, 0x00803500L, 0x0FF66D00L,    /* ZSYNTH */
 0x18099080L, 0x59},
 0x088E0D80L,   /* TXACGAIN */
 0x01456D80L,   /* RXACGAIN */
 0x07ABE580L, 0x18541B00L, 0x0757CB00L,    /* RXACHPF */
 0, 0  /* TXGAIN, RXGAIN */
 },  /* ZSYN_600_0_0_30_0 */
/* Source: Database file: j:\proj\sandbox\config_tool\build\cwdb.db */
/* Database information: version: 1.0.0 build date: 2010-01-06*/
/* parameters: zref=270_750_150 rprot=30 rfuse=0 emi_cap=10*/
{
{0x0715CB80L, 0x1FD56880L, 0x000D4480L, 0x1FFEAE00L,    /* TXACEQ */
 0x0A834F80L, 0x1BA7E500L, 0x0080D300L, 0x1FDC1580L},   /* RXACEQ */
{0x0017A080L, 0x1FDE3600L, 0x0129F900L, 0x01915280L,    /* ECFIR/ECIIR */
 0x01434280L, 0x018E9E00L, 0x1FFE1200L, 0x0085E000L,
 0x001ECE80L, 0x1FDF3C80L, 0x0CC9EA00L, 0x1B2E1180L},
{0x00C66800L, 0x1DD9CB80L, 0x015F8F00L, 0x0CB97F00L,    /* ZSYNTH */
 0x1B44F480L, 0x94},
 0x08000000L,   /* TXACGAIN */
 0x0108FB80L,   /* RXACGAIN */
 0x07BB6980L, 0x18449700L, 0x0776D380L,    /* RXACHPF */
 0, 0  /* TXGAIN, RXGAIN */
 },  /* ZSYN_270_750_150_30_0 */
/* Source: Database file: j:\proj\sandbox\config_tool\build\cwdb.db */
/* Database information: version: 1.0.0 build date: 2010-01-06*/
/* parameters: zref=370_620_310 rprot=30 rfuse=0 emi_cap=10*/
{
{0x07E59E80L, 0x1FD33400L, 0x1FFDF800L, 0x1FFD8300L,    /* TXACEQ */
 0x09F38000L, 0x1C1C5A00L, 0x1F94D700L, 0x1FDE5800L},   /* RXACEQ */
{0x00234480L, 0x1F9CDD00L, 0x01F5D580L, 0x1FF39000L,    /* ECFIR/ECIIR */
 0x02C17180L, 0x1FBE2500L, 0x00DFFE80L, 0x00441A80L,
 0x003BF800L, 0x1FC42400L, 0x0D9EB380L, 0x1A514580L},
{0x003ED200L, 0x1F5D6B80L, 0x0063B100L, 0x0F12E200L,    /* ZSYNTH */
 0x18EC9380L, 0x8B},
 0x08000000L,   /* TXACGAIN */
 0x0127C700L,   /* RXACGAIN */
 0x07B51200L, 0x184AEE80L, 0x076A2480L,    /* RXACHPF */
 0, 0  /* TXGAIN, RXGAIN */
 },  /* ZSYN_370_620_310_30_0 */
/* Source: Database file: j:\proj\sandbox\config_tool\build\cwdb.db */
/* Database information: version: 1.0.0 build date: 2010-01-06*/
/* parameters: zref=220_820_120 rprot=30 rfuse=0 emi_cap=10*/
{
{0x06E2A580L, 0x1FD1DF80L, 0x00068880L, 0x1FFCE200L,    /* TXACEQ */
 0x0A7AFB00L, 0x1BC11F80L, 0x009C4E80L, 0x1FD60300L},   /* RXACEQ */
{0x002C9880L, 0x1F530400L, 0x02CF4D80L, 0x1E895880L,    /* ECFIR/ECIIR */
 0x055F7200L, 0x1E034600L, 0x023B9080L, 0x1FB01780L,
 0x00339380L, 0x1FC98F80L, 0x0B7EA900L, 0x1C760400L},
{0x022C8200L, 0x1A9F3E80L, 0x03332100L, 0x0A0D4700L,    /* ZSYNTH */
 0x1DEBC480L, 0x8D},
 0x08000000L,   /* TXACGAIN */
 0x01013A80L,   /* RXACGAIN */
 0x07BEF980L, 0x18410700L, 0x077DF280L,    /* RXACHPF */
 0, 0  /* TXGAIN, RXGAIN */
 },  /* ZSYN_220_820_120_30_0 */
/* Source: Database file: j:\proj\sandbox\config_tool\build\cwdb.db */
/* Database information: version: 1.0.0 build date: 2010-01-06*/
/* parameters: zref=600_0_0 rprot=30 rfuse=0 emi_cap=10*/
{
{0x07F46C00L, 0x000E4600L, 0x00008580L, 0x1FFD6100L,    /* TXACEQ */
 0x07EF5000L, 0x0013F580L, 0x1FFDE000L, 0x1FFCB280L},   /* RXACEQ */
{0x0027CB00L, 0x1F8A8880L, 0x02801180L, 0x1F625C80L,    /* ECFIR/ECIIR */
 0x0314FB00L, 0x1E6B8E80L, 0x00C5FF00L, 0x1FC96F00L,
 0x1FFD1200L, 0x00023C00L, 0x0ED29D00L, 0x192A9400L},
{0x00810E00L, 0x1EFEBE80L, 0x00803500L, 0x0FF66D00L,    /* ZSYNTH */
 0x18099080L, 0x59},
 0x088E0D80L,   /* TXACGAIN */
 0x01456D80L,   /* RXACGAIN */
 0x07ABE580L, 0x18541B00L, 0x0757CB00L,    /* RXACHPF */
 0, 0  /* TXGAIN, RXGAIN */
 },  /* ZSYN_600_0_1000_30_0 */
/* Source: Database file: j:\proj\sandbox\config_tool\build\cwdb.db */
/* Database information: version: 1.0.0 build date: 2010-01-06*/
/* parameters: zref=200_680_100 rprot=30 rfuse=0 emi_cap=10*/
{
{0x07365D80L, 0x1FC64180L, 0x00022980L, 0x1FFCE300L,    /* TXACEQ */
 0x09C28580L, 0x1D1FD880L, 0x0071A280L, 0x1FDF7500L},   /* RXACEQ */
{0x1FF15A00L, 0x005C0600L, 0x00828200L, 0x01B11D00L,    /* ECFIR/ECIIR */
 0x027BB800L, 0x1EE9F200L, 0x028BAB80L, 0x1E57BE80L,
 0x01007580L, 0x1EF8B780L, 0x0556EE80L, 0x028DFB80L},
{0x014F2C00L, 0x1C7A1180L, 0x02369A00L, 0x0A138100L,    /* ZSYNTH */
 0x1DEA2280L, 0x8E},
 0x08000000L,   /* TXACGAIN */
 0x010C7E80L,   /* RXACGAIN */
 0x07BB2500L, 0x1844DB80L, 0x07764980L,    /* RXACHPF */
 0, 0  /* TXGAIN, RXGAIN */
 },  /* ZSYN_200_680_100_30_0 */
/* Source: Database file: j:\proj\sandbox\config_tool\build\cwdb.db */
/* Database information: version: 1.0.0 build date: 2010-01-06*/
/* parameters: zref=220_820_115 rprot=30 rfuse=0 emi_cap=10*/
{
{0x06D56400L, 0x1FDF1780L, 0x00095A80L, 0x1FFDA880L,    /* TXACEQ */
 0x0A596300L, 0x1C067880L, 0x0095EF00L, 0x1FD7AF00L},   /* RXACEQ */
{0x00164300L, 0x1FD81880L, 0x0150CC80L, 0x0151BB80L,    /* ECFIR/ECIIR */
 0x01DA1A00L, 0x0142CB80L, 0x0027DE80L, 0x0076A180L,
 0x0012F980L, 0x1FEAE000L, 0x0CC70C80L, 0x1B2DF000L},
{0x00246300L, 0x1E5E0580L, 0x017D2300L, 0x0A138100L,    /* ZSYNTH */
 0x1DEA2280L, 0xA7},
 0x08000000L,   /* TXACGAIN */
 0x01009500L,   /* RXACGAIN */
 0x07BBEE80L, 0x18441200L, 0x0777DD80L,    /* RXACHPF */
 0, 0  /* TXGAIN, RXGAIN */
 },    /* ZSYN_220_820_115_30_0 */
 /* Source: Database file:  */
/* Database information: version: 1.0.0 build date: 2010-06-24*/
/* parameters: zref=600_0_0 rprot=54 rfuse=0 emi_cap=10*/
{
{0x07EF4080L, 0x00184A00L, 0x0000E100L, 0x1FFED900L,    /* TXACEQ */
 0x07E8F080L, 0x001B8400L, 0x1FFCF300L, 0x1FFD2680L},   /* RXACEQ */
{0x00218680L, 0x1FAB6080L, 0x021C1200L, 0x00080300L,    /* ECFIR/ECIIR */
 0x02322C00L, 0x1F354100L, 0x003AB400L, 0x00133200L,
 0x1FD87980L, 0x00213380L, 0x0288F400L, 0x0554C800L},
{0x0050B500L, 0x1FBD9980L, 0x1FF20D80L, 0x0A13AD00L,    /* ZSYNTH */
 0x1DEA1780L, 0x5F},
 0x08EEBA80L,   /* TXACGAIN */
 0x0153CB80L,   /* RXACGAIN */
 0x07B13100L, 0x184ECF80L, 0x07626200L,    /* RXACHPF */
 0, 0  /* TXGAIN, RXGAIN */
 },    /* ZSYN_600_0_0_54_0 */
 /* Source: Database file:  */
/* Database information: version: 1.0.0 build date: 2010-06-24*/
/* parameters: zref=270_750_150 rprot=54 rfuse=0 emi_cap=10*/
{
{0x0753B880L, 0x1FC7CD80L, 0x000B3680L, 0x1FFCD900L,    /* TXACEQ */
 0x0A8D4600L, 0x1B920480L, 0x0083CF00L, 0x1FDAEC80L},   /* RXACEQ */
{0x002FAD00L, 0x1F696080L, 0x0254A300L, 0x1F73A480L,    /* ECFIR/ECIIR */
 0x03D2FE00L, 0x1F440300L, 0x01627E00L, 0x00026700L,
 0x00302800L, 0x1FCDAE00L, 0x0C287B80L, 0x1BCDE380L},
{0x1F60AD80L, 0x00A52700L, 0x1FF9F980L, 0x0D90F100L,    /* ZSYNTH */
 0x1A6E1880L, 0xB4},
 0x08000000L,   /* TXACGAIN */
 0x0110FF80L,   /* RXACGAIN */
 0x07BBF000L, 0x18441080L, 0x0777E000L,    /* RXACHPF */
 0, 0  /* TXGAIN, RXGAIN */
 },  /* ZSYN_250_750_150_54_0 */
/* Source: Database file: C:\Program Files\Silicon Laboratories\ProSLIC API General Release Version 5.4.1\proslic_api\config_tools\cwdb.db */
/* Database information: version: 1.0.0 build date: 2010-06-24*/
/* parameters: zref=200_680_100 rprot=54 rfuse=0 emi_cap=10*/
{
{0x077B1700L, 0x1FBD9780L, 0x00032800L, 0x1FFC7180L,    /* TXACEQ */
 0x09C89800L, 0x1D164880L, 0x0075BB80L, 0x1FDE9200L},   /* RXACEQ */
{0x1FFC3D80L, 0x001ED880L, 0x01220380L, 0x00975280L,    /* ECFIR/ECIIR */
 0x03A78C80L, 0x1E090F80L, 0x02D4B480L, 0x1E7F2D80L,
 0x00BE6F80L, 0x1F3CA600L, 0x0643AC00L, 0x01A39C00L},
{0x01596600L, 0x1C593F80L, 0x024D1E00L, 0x0A12AC00L,    /* ZSYNTH */
 0x1DEAF780L, 0x95},
 0x08000000L,   /* TXACGAIN */
 0x0115E000L,   /* RXACGAIN */
 0x07BA6580L, 0x18459B00L, 0x0774CA80L,    /* RXACHPF */
 0, 0  /* TXGAIN, RXGAIN */
 },    /* ZSYN_200_680_100_54_0 */
/* Source: Database file: C:\Program Files\Silicon Laboratories\ProSLIC API General Release Version 5.4.1\proslic_api\config_tools\cwdb.db */
/* Database information: version: 1.0.0 build date: 2010-06-24*/
/* parameters: zref=220_820_115 rprot=54 rfuse=0 emi_cap=10*/
{
{0x070B3080L, 0x1FCF2780L, 0x0009B700L, 0x1FFCC000L,    /* TXACEQ */
 0x0A674600L, 0x1BEA8C80L, 0x009DEF80L, 0x1FD58380L},   /* RXACEQ */
{0x00021480L, 0x00020180L, 0x01381700L, 0x00FF9B00L,    /* ECFIR/ECIIR */
 0x028E7580L, 0x00657A00L, 0x00C32D00L, 0x0039ED00L,
 0x001A1200L, 0x1FE37080L, 0x0C89C080L, 0x1B6D9800L},
{0x1FEF6480L, 0x1EA98680L, 0x01669600L, 0x0A138100L,    /* ZSYNTH */
 0x1DEA2280L, 0xB2},
 0x08000000L,   /* TXACGAIN */
 0x01070500L,   /* RXACGAIN */
 0x07BDBB80L, 0x18424500L, 0x077B7680L,    /* RXACHPF */
 0, 0  /* TXGAIN, RXGAIN */
 }    /* ZSYN_220_820_115_54_0 */
};

Si3217x_FSK_Cfg Si3217x_FSK_Presets[] ={
{
0x02232000L,	 /* FSK01 */
0x077C2000L,	 /* FSK10 */
0x003C0000L,	 /* FSKAMP0 (0.220 vrms )*/
0x00200000L,	 /* FSKAMP1 (0.220 vrms) */
0x06B60000L,	 /* FSKFREQ0 (2200.0 Hz space) */
0x079C0000L,	 /* FSKFREQ1 (1200.0 Hz mark) */
0x00,			 /* FSK8 */
0x00,			 /* FSKDEPTH (1 deep fifo) */
}    /* DEFAULT_FSK */
};

Si3217x_PulseMeter_Cfg Si3217x_PulseMeter_Presets[] ={
    /* inputs:  freq = 12kHz, amp = 1.000Vrms, cal = First, ramp = 24kHz, power = Normal */
    { 0x7A2B6AL, 0x0, 0x0 }    /* DEFAULT_PULSE_METERING */
};

/*
msec, lo, hi
{ 0, 0x0, 0x0 },
{ 80, 0x80, 0x02 },
{ 100, 0x20, 0x03 },
{ 200, 0x40, 0x06 },
{ 300, 0x60, 0x09 },
{ 350, 0xF0, 0x0A },
{ 500, 0xA0, 0x0F },
{ 700, 0xE0, 0x15 },
{ 1000, 0x40, 0x1F },
{ 4000, 0x00, 0x7D },
*/
Si3217x_Tone_Cfg Si3217x_Tone_Presets[] ={
{
	{
	0x08000000L,	 /* OSC1FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC1AMP (0.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x00 			 /* OMODE */
},  /* TONE_NONE */
{
	{
	0x078F0000L,	 /* OSC1FREQ (425.000 Hz) */
	0x00260000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F425 */
{
	{
	0x078F0000L,	 /* OSC1FREQ (425.000 Hz) */
	0x004BE000L,	 /* OSC1AMP (-4.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F425_M4 */
{
	{
	0x078F0000L,	 /* OSC1FREQ (425.000 Hz) */
	0x00260000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x40,			 /* O1TALO (200 ms) */
	0x06,			 /* O1TAHI */
	0x40,			 /* O1TILO (200 ms) */
	0x06			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F425_ON200_OFF200 */
{
	{
	0x078F0000L,	 /* OSC1FREQ (425.000 Hz) */
	0x00260000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x40,			 /* O1TALO (1000 ms) */
	0x1F,			 /* O1TAHI */
	0xA0,			 /* O1TILO (4500 ms) */
	0x8C			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F425_ON1000_OFF4500 */
{
	{
	0x078F0000L,	 /* OSC1FREQ (425.000 Hz) */
	0x00260000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0xA0,			 /* O1TALO (500 ms) */
	0x0F,			 /* O1TAHI */
	0xA0,			 /* O1TILO (500 ms) */
	0x0F			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F425_ON500_OFF500 */
{
	{
	0x078F0000L,	 /* OSC1FREQ (425.000 Hz) */
	0x00260000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x80,			 /* O1TALO (240 ms) */
	0x07,			 /* O1TAHI */
	0x80,			 /* O1TILO (240 ms) */
	0x07			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F425_ON240_OFF240 */
{
	{
	0x07870000L,	 /* OSC1FREQ (440.000 Hz) */
	0x00276000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F440 */
{
	{
	0x07B30000L,	 /* OSC1FREQ (350.000 Hz) */
	0x00162000L,	 /* OSC1AMP (-13.000 dBm) hubo change 20170222 */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x07870000L,	 /* OSC2FREQ (440.000 Hz) */
	0x001BE000L,	 /* OSC2AMP (-13.000 dBm) hubo change 20170222 */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x22 			 /* OMODE */
},  /* TONE_F440_F350 */
{
	{
	0x07B30000L,	 /* OSC1FREQ (350.000 Hz) */
	0x001F4000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x20,			 /* O1TALO (100 ms) */
	0x03,			 /* O1TAHI */
	0x20,			 /* O1TILO (100 ms) */
	0x03			 /* O1TIHI */
	},
	{
	0x07870000L,	 /* OSC2FREQ (440.000 Hz) */
	0x00276000L,	 /* OSC2AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x20,			 /* O2TALO (100 ms) */
	0x03,			 /* O2TAHI */
	0x20,			 /* O2TILO (100 ms) */
	0x03 			 /* O2TIHI */
	},
	0x22 			 /* OMODE */
},  /* TONE_F440_F350_ON100_OFF100 */
{
	{
	0x07810000L,	 /* OSC1FREQ (450.000 Hz) */
	0x00286000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F450 */
#if 0 /*hubo change busy_tone from TONE_F450_ON350_OFF350 to TONE_F480_F620_ON500_OFF500 begin*/
{
	{
	0x07810000L,	 /* OSC1FREQ (450.000 Hz) */
	0x00286000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0xF0,			 /* O1TALO (350 ms) */
	0x0A,			 /* O1TAHI */
	0xF0,			 /* O1TILO (350 ms) */
	0x0A			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F450_ON350_OFF350 */

#else
{
	{
	0x07700000L,	 /* OSC1FREQ (480.000 Hz) */
	0x00088000L,	 /* OSC1AMP (-24.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0xA0,			 /* O1TALO (500 ms) */
	0x0F,			 /* O1TAHI */
	0xA0,			 /* O1TILO (500 ms) */
	0x0F			 /* O1TIHI */
	},
	{
	0x07120000L,	 /* OSC2FREQ (620.000 Hz) */
	0x000B2000L,	 /* OSC2AMP (-24.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0xA0,			 /* O2TALO (500 ms) */
	0x0F,			 /* O2TAHI */
	0xA0,			 /* O2TILO (500 ms) */
	0x0F 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
}, /*hubo change busy_tone from TONE_F450_ON350_OFF350 to TONE_F480_F620_ON500_OFF500 end*/
#endif

{
	{
	0x07810000L,	 /* OSC1FREQ (450.000 Hz) */
	0x00286000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0xE0,			 /* O1TALO (700 ms) */
	0x15,			 /* O1TAHI */
	0xE0,			 /* O1TILO (700 ms) */
	0x15			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F450_ON700_OFF700 */
#if 0  /*hubo change ringback_tone from TONE_F450_ON1000_OFF4000 to TONE_F440_F480_ON2000_OFF4000 20170222 begin*/
{
	{
	0x07810000L,	 /* OSC1FREQ (450.000 Hz) */
	0x00286000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x40,			 /* O1TALO (1000 ms) */
	0x1F,			 /* O1TAHI */
	0x00,			 /* O1TILO (4000 ms) */
	0x7D			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F450_ON1000_OFF4000 */
#else
{
	{
	0x07870000L,	 /* OSC1FREQ (440.000 Hz) */
	0x000DE000L,	 /* OSC1AMP (-19.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x80,			 /* O1TALO (2000 ms) */
	0x3E,			 /* O1TAHI */
	0x00,			 /* O1TILO (4000 ms) */
	0x7D			 /* O1TIHI */
	},
	{
	0x07700000L,	 /* OSC2FREQ (480.000 Hz) */
	0x000F4000L,	 /* OSC2AMP (-19.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x80,			 /* O2TALO (2000 ms) */
	0x3E,			 /* O2TAHI */
	0x00,			 /* O2TILO (4000 ms) */
	0x7D 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},
#endif /*hubo change ringback_tone from TONE_F450_ON1000_OFF4000 to TONE_F440_F480_ON2000_OFF4000 20170222 end*/

{
	{
	0x07700000L,	 /* OSC1FREQ (480.000 Hz) */
	0x00112000L,	 /* OSC1AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x40,			 /* O1TALO (200 ms) */
	0x06,			 /* O1TAHI */
	0x40,			 /* O1TILO (200 ms) */
	0x06			 /* O1TIHI */
	},
	{
	0x07120000L,	 /* OSC2FREQ (620.000 Hz) */
	0x00164000L,	 /* OSC2AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x40,			 /* O2TALO (200 ms) */
	0x06,			 /* O2TAHI */
	0x40,			 /* O2TILO (200 ms) */
	0x06 			 /* O2TIHI */
	},
	0x22 			 /* OMODE */
},  /* TONE_F480_F620_ON200_OFF200 */
{
	{
	0x07700000L,	 /* OSC1FREQ (480.000 Hz) */
	0x00112000L,	 /* OSC1AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x60,			 /* O1TALO (300 ms) */
	0x09,			 /* O1TAHI */
	0x40,			 /* O1TILO (200 ms) */
	0x06			 /* O1TIHI */
	},
	{
	0x07120000L,	 /* OSC2FREQ (620.000 Hz) */
	0x00164000L,	 /* OSC2AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x60,			 /* O2TALO (300 ms) */
	0x09,			 /* O2TAHI */
	0x40,			 /* O2TILO (200 ms) */
	0x06 			 /* O2TIHI */
	},
	0x22 			 /* OMODE */
},  /* TONE_F480_F620_ON300_OFF200 */
{
	{
	0x07700000L,	 /* OSC1FREQ (480.000 Hz) */
	0x00112000L,	 /* OSC1AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x60,			 /* O1TALO (300 ms) */
	0x09,			 /* O1TAHI */
	0x60,			 /* O1TILO (300 ms) */
	0x09			 /* O1TIHI */
	},
	{
	0x07120000L,	 /* OSC2FREQ (620.000 Hz) */
	0x00164000L,	 /* OSC2AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x60,			 /* O2TALO (300 ms) */
	0x09,			 /* O2TAHI */
	0x60,			 /* O2TILO (300 ms) */
	0x09 			 /* O2TIHI */
	},
	0x22 			 /* OMODE */
},  /* TONE_F480_F620_ON300_OFF300 */
{
	{
	0x07210000L,	 /* OSC1FREQ (600.000 Hz) */
	0x00158000L,	 /* OSC1AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x20,			 /* O1TALO (100 ms) */
	0x03,			 /* O1TAHI */
	0x20,			 /* O1TILO (100 ms) */
	0x03			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F600_ON100_OFF100 */
{
	{
	0x05F30000L,	 /* OSC1FREQ (932.000 Hz) */
	0x0056A000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F932 */
{
	{
	0x05E00000L,	 /* OSC1FREQ (950.000 Hz) */
	0x00586000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F950 */
{
	{
	0x03A60000L,	 /* OSC1FREQ (1397.000 Hz) */
	0x008A0000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F1397 */
{
	{
	0x01800000L,	 /* OSC1FREQ (1760.000 Hz) */
	0x00BAE000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F1760 */
{
	{
	0x1F2F0000L,	 /* OSC1FREQ (2130.000 Hz) */
	0x008CC000L,	 /* OSC1AMP (-15.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x80,			 /* O1TALO (80 ms) */
	0x02,			 /* O1TAHI */
	0x00,			 /* O1TILO (4000 ms) */
	0x7D			 /* O1TIHI */
	},
	{
	0x1B8E0000L,	 /* OSC2FREQ (2750.000 Hz) */
	0x00EDA000L,	 /* OSC2AMP (-15.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x80,			 /* O2TALO (80 ms) */
	0x02,			 /* O2TAHI */
	0x00,			 /* O2TILO (4000 ms) */
	0x7D 			 /* O2TIHI */
	},
	0x22 			 /* OMODE */
},  /* TONE_F2130_F2750 */
{
	{
	0x1B8E0000L,	 /* OSC1FREQ (2750.000 Hz) */
	0x01A6A000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F2750 */
{
	{
	0x1ACE0000L,	 /* OSC1FREQ (2900.000 Hz) */
	0x01EA0000L,	 /* OSC1AMP (-10.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x08000000L,	 /* OSC2FREQ (0.000 Hz) */
	0x00000000L,	 /* OSC2AMP (0.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x02 			 /* OMODE */
},  /* TONE_F2900 */
{
	{ 0x3fc4000L ,0x1284000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x5e9a000L, 0xc64000L ,0 , 0x80 , 0x2 , 0x20 , 0x3},
	0x22
},	// DTMF(0)--Fre (1336)(941)
{
	{ 0x4a80000L ,0x1072000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x6d4c000L, 0x8fc000L ,0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(1)--Fre (1209)(697)
{
	{ 0x3fc4000L ,0x1284000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x6d4c000L, 0x8fc000L ,0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(2)--Fre (1336)(697)
{
	{ 0x331c000L ,0x14f8000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x6d4c000L, 0x8fc000L ,0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(3)--Fre (1477)(697)
{
	{ 0x4a80000L ,0x1072000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x694c000L, 0x9fc000L ,0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(4)--Fre (1209)(770)
{
	{ 0x3fc4000L ,0x1284000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x694c000L, 0x9fc000L ,0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(5)--Fre (1336)(770)
{
	{ 0x331c000L ,0x14f8000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x694c000L, 0x9fc000L ,0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(6)--Fre (1477)(770)
{
	{ 0x4a80000L ,0x1072000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x6466000L, 0xb20000L ,0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},     // DTMF(7)--Fre (1209)(852)
{
	{ 0x3fc4000L ,0x1284000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x6466000L, 0xb20000L ,0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(8)--Fre (1336)(852)
{
	{ 0x331c000L ,0x14f8000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x6466000L, 0xb20000L ,0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(9)--Fre (1477)(852)
{
	{ 0x2462000L ,0x17e4000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x6d4c000L, 0x8fc000L, 0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(A)--Fre (1633)(697)
{
	{ 0x2462000L ,0x17e4000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x694c000L, 0x9fc000L, 0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(B)--Fre (1633)(770)
{
	{ 0x2462000L ,0x17e4000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x6466000L, 0xb20000L, 0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(C)--Fre (1633)(852)
{
	{ 0x2462000L ,0x17e4000L , 0 , 0x80 , 0x2 , 0x20 , 0x3 } ,
	{ 0x5e9a000L, 0xc64000L, 0 , 0x80 , 0x2 , 0x20 , 0x3 },
	0x22
},	// DTMF(D)--Fre (1633)(941)
};

Si3217x_PCM_Cfg Si3217x_PCM_Presets[] ={
	{
	0x01, 	 /* PCM_FMT - u-Law */
	0x00, 	 /* WIDEBAND - DISABLED (3.4kHz BW) */
	0x00, 	 /* PCM_TRI - PCLK RISING EDGE */
	0x00, 	 /* TX_EDGE - PCLK RISING EDGE */
	0x00 	 /* A-LAW -  INVERT NONE */
	},  /* ULAW_PCM */
	{
	0x03, 	 /* PCM_FMT - 16-bit Linear */
	0x00, 	 /* WIDEBAND - DISABLED (3.4kHz BW) */
	0x00, 	 /* PCM_TRI - PCLK RISING EDGE */
	0x00, 	 /* TX_EDGE - PCLK RISING EDGE */
	0x00 	 /* A-LAW -  INVERT NONE */
	},  /* LIN16_PCM */
	{
	0x03, 	 /* PCM_FMT - 16-bit Linear */
	0x01, 	 /* WIDEBAND - DISABLED (7kHz BW) */
	0x00, 	 /* PCM_TRI - PCLK RISING EDGE */
	0x00, 	 /* TX_EDGE - PCLK RISING EDGE */
	0x00 	 /* A-LAW -  INVERT NONE */
	},  /* WB_LIN16_PCM */
};

