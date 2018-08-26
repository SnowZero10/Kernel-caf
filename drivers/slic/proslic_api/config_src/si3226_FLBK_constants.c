/*
** Copyright (c) 2015 Silicon Laboratories, Inc.
** 2015-02-24 16:49:07
**
** Si322x ProSLIC API Configuration Tool Version 3.4.0
** Last Updated in API Release: 7.4.0
** Auto generated file from configuration tool
*/


#include "proslic.h"
#include "si3226.h"
Si3226_General_Cfg Si3226_General_Configuration  = {
BO_DCDC_FLYBACK,    /* BOM_OPT */
0x05C28F30L,    /* VBATR_EXPECT */
0x03D70A20L,    /* VBATH_EXPECT */
0x00000000L,    /* DCDC_FSW_VTHLO */
0x00000000L,    /* DCDC_FSW_VHYST */
0x01300000L,    /* DCDC_VREF_MIN */
0x05A00000L,    /* DCDC_VREF_MIN_RNG */
0x00200000L,    /* DCDC_FSW_NORM */
0x00200000L,    /* DCDC_FSW_NORM_LO */
0x00200000L,    /* DCDC_FSW_RING */
0x00100000L,    /* DCDC_FSW_RING_LO */
0x0D980000L,    /* DCDC_DIN_LIM */
0x00C00000L,    /* DCDC_VOUT_LIM */
0x00000000L,    /* DCDC_DCFF_ENABLE */
0x00500000L,    /* DCDC_UVHYST */
0x00600000L,    /* DCDC_UVTHRESH */
0x00B00000L,    /* DCDC_OVTHRESH */
0x01F00000L,    /* DCDC_OITHRESH */
0x00100000L,    /* DCDC_SWDRV_POL */
0x00300000L,    /* DCDC_SWFET */
0x00600000L,    /* DCDC_VREF_CTRL */
0x00200000L,    /* DCDC_RNGTYPE */
0x00000000L,    /* DCDC_ANA_GAIN */
0x00000000L,    /* DCDC_ANA_TOFF */
0x00000000L,    /* DCDC_ANA_TONMIN */
0x00000000L,    /* DCDC_ANA_TONMAX */
0x00000000L,    /* DCDC_ANA_DSHIFT */
0x00000000L,    /* DCDC_ANA_LPOLY */
0x07FEB86CL,    /* COEF_P_HVIC */
0x4C756DL,    /* P_TH_HVIC */
VDC_7P0_20P0,    /* VDC_RANGE_OPT */
0x3F,     /* AUTO */
0x00,     /* DAA_CNTL - NOT USED*/
0x00,     /* IRQEN1 */
0x00,     /* IRQEN2 */
0x03,     /* IRQEN3 */
0x00,     /* IRQEN4 */
0x00,     /* ENHANCE */
0,       /* DAA_ENABLE - NOT USED*/
0x08000000L,    /* SCALE_KAUDIO */
0x0151EB80L,    /* AC_ADC_GAIN */
};

Si3226_GPIO_Cfg Si3226_GPIO_Configuration = {
0x00,     /* GPIO_OE */
0x00,     /* GPIO_ANA */
0x00,     /* GPIO_DIR */
0x00,     /* GPIO_MAN */
0x00,     /* GPIO_POL */
0x00,     /* GPIO_OD */
0x00     /* BATSELMAP */
};
Si3226_CI_Cfg Si3226_CI_Presets [] = {
	{0}
};
Si3226_audioGain_Cfg Si3226_audioGain_Presets [] = {
	{0x1377080L,0, 0x0L, 0x0L, 0x0L, 0x0L},
{0x80C3180L,0, 0x0L, 0x0L, 0x0L, 0x0L}
};

Si3226_Ring_Cfg Si3226_Ring_Presets[] ={
{
/*
	Loop = 500.0 ft @ 0.044 ohms/ft, REN = 3, Rcpe = 600 ohms
	Rprot = 30 ohms, Type = BALANCED, Waveform = SINE
*/
0x00050000L,	/* RTPER */
0x07EFE000L,	/* RINGFR (20.000 Hz) */
0x0025D5BFL,	/* RINGAMP (64.490 vrms)  */
0x00000000L,	/* RINGPHAS */
0x00000000L,	/* RINGOF (0.000 vdc) */
0x15E5200EL,	/* SLOPE_RING (100.000 ohms) */
0x008B9ACAL,	/* IRING_LIM (90.000 mA) */
0x00608636L,	/* RTACTH (53.304 mA) */
0x0FFFFFFFL,	/* RTDCTH (450.000 mA) */
0x00006000L,	/* RTACDB (75.000 ms) */
0x00006000L,	/* RTDCDB (75.000 ms) */
0x00C49BA0L,	/* VOV_RING_BAT (12.000 v) */
0x00000000L,	/* VOV_RING_GND (0.000 v) */
0x070A0AA8L,	/* VBATR_EXPECT (109.988 v) */
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
0x40,			/* RINGCON */
0x00,			/* USERSTAT */
0x03850554L,	/* VCM_RING (51.994 v) */
0x03850554L,	/* VCM_RING_FIXED */
0x003126E8L,	/* DELTA_VCM */
0x00200000L,	/* DCDC_RNGTYPE */
},  /* RING_MAX_VBAT_PROVISIONING */
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
0x008B9ACAL,	/* IRING_LIM (90.000 mA) */
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
0x40,			/* RINGCON */
0x00,			/* USERSTAT */
0x02AC55FEL,	/* VCM_RING (38.769 v) */
0x02AC55FEL,	/* VCM_RING_FIXED */
0x003126E8L,	/* DELTA_VCM */
0x00200000L,	/* DCDC_RNGTYPE */
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
0x008B9ACAL,	/* IRING_LIM (90.000 mA) */
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
0x40,			/* RINGCON */
0x00,			/* USERSTAT */
0x02AC55FEL,	/* VCM_RING (38.769 v) */
0x02AC55FEL,	/* VCM_RING_FIXED */
0x003126E8L,	/* DELTA_VCM */
0x00200000L,	/* DCDC_RNGTYPE */
}   /* RING_F20_45VRMS_0VDC_BAL */
};

Si3226_DCfeed_Cfg Si3226_DCfeed_Presets[] = {
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
}   /* DCFEED_PSTN_DET_2 */
};

Si3226_Impedance_Cfg Si3226_Impedance_Presets[] ={
/* Source: Database file: cwdb.db */
/* Database information: */
/* parameters: zref=600_0_0 rprot=30 rfuse=0 emi_cap=10*/
{
{0x08061880L, 0x1FF7E500L, 0x0004C880L, 0x1FFE8200L,    /* TXACEQ */
 0x08004200L, 0x1FF73400L, 0x0001F400L, 0x1FFB3600L},   /* RXACEQ */
{0x0011AD00L, 0x1FEDA700L, 0x01DEDA00L, 0x00561180L,    /* ECFIR/ECIIR */
 0x02153C80L, 0x1EF2E200L, 0x00A74600L, 0x1FAC9F80L,
 0x00263B80L, 0x1FD3EB00L, 0x01A37700L, 0x0637F980L},
{0x01D6FC00L, 0x1CE95C80L, 0x01403800L, 0x0B73EA00L,    /* ZSYNTH */
 0x1C88EC80L, 0x36},
 0x0875A080L,   /* TXACGAIN */
 0x0144C380L,   /* RXACGAIN */
 0x07B53400L, 0x184ACC80L, 0x076A6780L,    /* RXACHPF */
#ifdef ENABLE_HIRES_GAIN
 0, 0  /* TXGAIN*10, RXGAIN*10 (hi_res) */
#else
 0, 0  /* TXGAIN, RXGAIN */
#endif
 },  /* ZSYN_600_0_0_30_0 */
/* Source: Database file: cwdb.db */
/* Database information: */
/* parameters: zref=270_750_150 rprot=30 rfuse=0 emi_cap=10*/
{
{0x0713F500L, 0x1FDA3000L, 0x0009B600L, 0x1FFE1A00L,    /* TXACEQ */
 0x0A7D1480L, 0x1BB25D00L, 0x0077AD80L, 0x1FDE2F00L},   /* RXACEQ */
{0x00233780L, 0x1FB73F00L, 0x019AF680L, 0x01120900L,    /* ECFIR/ECIIR */
 0x01B6C400L, 0x0128DB80L, 0x001E8B00L, 0x007A4A80L,
 0x001D6B00L, 0x1FE0FB00L, 0x0CDFA480L, 0x1B10C880L},
{0x0066D100L, 0x1EB12280L, 0x00E7D600L, 0x0D707F00L,    /* ZSYNTH */
 0x1A8E3C80L, 0x9A},
 0x08000000L,   /* TXACGAIN */
 0x010BC000L,   /* RXACGAIN */
 0x07BB6700L, 0x18449980L, 0x0776CE00L,    /* RXACHPF */
#ifdef ENABLE_HIRES_GAIN
 0, 0  /* TXGAIN*10, RXGAIN*10 (hi_res) */
#else
 0, 0  /* TXGAIN, RXGAIN */
#endif
 },  /* ZSYN_270_750_150_30_0 */
/* Source: Database file: cwdb.db */
/* Database information: */
/* parameters: zref=370_620_310 rprot=30 rfuse=0 emi_cap=10*/
{
{0x07E57C80L, 0x1FD36080L, 0x1FFDFC80L, 0x1FFD5600L,    /* TXACEQ */
 0x09F29800L, 0x1C1E7B80L, 0x1F951700L, 0x1FDDF900L},   /* RXACEQ */
{0x00180000L, 0x1FDE7B80L, 0x01724100L, 0x00DCC900L,    /* ECFIR/ECIIR */
 0x01BBCF00L, 0x006AB600L, 0x0086D980L, 0x00573F80L,
 0x003A8280L, 0x1FC5F780L, 0x0D9B3100L, 0x1A52AF00L},
{0x00666100L, 0x1F0FED80L, 0x0089A100L, 0x0F14FA00L,    /* ZSYNTH */
 0x18EA8480L, 0x88},
 0x08000000L,   /* TXACGAIN */
 0x012ABB80L,   /* RXACGAIN */
 0x07B44A80L, 0x184BB600L, 0x07689500L,    /* RXACHPF */
#ifdef ENABLE_HIRES_GAIN
 0, 0  /* TXGAIN*10, RXGAIN*10 (hi_res) */
#else
 0, 0  /* TXGAIN, RXGAIN */
#endif
 },  /* ZSYN_370_620_310_30_0 */
/* Source: Database file: cwdb.db */
/* Database information: */
/* parameters: zref=220_820_120 rprot=30 rfuse=0 emi_cap=10*/
{
{0x06DF0200L, 0x1FE00A80L, 0x000A1D00L, 0x1FFDB600L,    /* TXACEQ */
 0x0A6D9A80L, 0x1BDD8E80L, 0x00995100L, 0x1FD6B000L},   /* RXACEQ */
{0x1FF1F980L, 0x00821900L, 0x1FE0CF00L, 0x0385F000L,    /* ECFIR/ECIIR */
 0x1F85A380L, 0x02CB3980L, 0x1F6D6380L, 0x00A01E00L,
 0x00154180L, 0x1FE8F000L, 0x0CCE6D80L, 0x1B1BD080L},
{0x00A38500L, 0x1D812380L, 0x01DAD000L, 0x0AEB1100L,    /* ZSYNTH */
 0x1D126880L, 0xAC},
 0x08000000L,   /* TXACGAIN */
 0x01047800L,   /* RXACGAIN */
 0x07BC0400L, 0x1843FC80L, 0x07780800L,    /* RXACHPF */
#ifdef ENABLE_HIRES_GAIN
 0, 0  /* TXGAIN*10, RXGAIN*10 (hi_res) */
#else
 0, 0  /* TXGAIN, RXGAIN */
#endif
 },  /* ZSYN_220_820_120_30_0 */
/* Source: Database file: cwdb.db */
/* Database information: */
/* parameters: zref=600_0_0 rprot=30 rfuse=0 emi_cap=10*/
{
{0x08061880L, 0x1FF7E500L, 0x0004C880L, 0x1FFE8200L,    /* TXACEQ */
 0x08004200L, 0x1FF73400L, 0x0001F400L, 0x1FFB3600L},   /* RXACEQ */
{0x0011AD00L, 0x1FEDA700L, 0x01DEDA00L, 0x00561180L,    /* ECFIR/ECIIR */
 0x02153C80L, 0x1EF2E200L, 0x00A74600L, 0x1FAC9F80L,
 0x00263B80L, 0x1FD3EB00L, 0x01A37700L, 0x0637F980L},
{0x01D6FC00L, 0x1CE95C80L, 0x01403800L, 0x0B73EA00L,    /* ZSYNTH */
 0x1C88EC80L, 0x36},
 0x0875A080L,   /* TXACGAIN */
 0x0144C380L,   /* RXACGAIN */
 0x07B53400L, 0x184ACC80L, 0x076A6780L,    /* RXACHPF */
#ifdef ENABLE_HIRES_GAIN
 0, 0  /* TXGAIN*10, RXGAIN*10 (hi_res) */
#else
 0, 0  /* TXGAIN, RXGAIN */
#endif
 },  /* ZSYN_600_0_1000_30_0 */
/* Source: Database file: cwdb.db */
/* Database information: */
/* parameters: zref=200_680_100 rprot=30 rfuse=0 emi_cap=10*/
{
{0x07361880L, 0x1FE64480L, 0x00076600L, 0x1FFDB780L,    /* TXACEQ */
 0x09A5E200L, 0x1D545800L, 0x006D0800L, 0x1FE0D480L},   /* RXACEQ */
{0x1FF14700L, 0x007AD780L, 0x0009B380L, 0x02F6B600L,    /* ECFIR/ECIIR */
 0x007BA700L, 0x00D5E180L, 0x0145AE00L, 0x1E75FC00L,
 0x01B02500L, 0x1E4B5D80L, 0x01B20200L, 0x06299400L},
{0x1F5E0480L, 0x1FD83180L, 0x00C91200L, 0x0A11FA00L,    /* ZSYNTH */
 0x1DEA8980L, 0xB9},
 0x08000000L,   /* TXACGAIN */
 0x01122B00L,   /* RXACGAIN */
 0x07B94700L, 0x1846B980L, 0x07728D80L,    /* RXACHPF */
#ifdef ENABLE_HIRES_GAIN
 0, 0  /* TXGAIN*10, RXGAIN*10 (hi_res) */
#else
 0, 0  /* TXGAIN, RXGAIN */
#endif
 },  /* ZSYN_200_680_100_30_0 */
/* Source: Database file: cwdb.db */
/* Database information: */
/* parameters: zref=220_820_115 rprot=30 rfuse=0 emi_cap=10*/
{
{0x06D3C300L, 0x1FE3A500L, 0x000A2D80L, 0x1FFDF280L,    /* TXACEQ */
 0x0A558A00L, 0x1C0E6680L, 0x00953500L, 0x1FD7C880L},   /* RXACEQ */
{0x00636A00L, 0x1EC6E200L, 0x0389E700L, 0x1EB70900L,    /* ECFIR/ECIIR */
 0x03CCB980L, 0x0091E080L, 0x1FC0B600L, 0x00FFCE00L,
 0x1FDDA580L, 0x0016D200L, 0x046CBC80L, 0x1D3CB200L},
{0x00CFD200L, 0x1CFD1180L, 0x02328000L, 0x0A12B400L,    /* ZSYNTH */
 0x1DEA5880L, 0xAA},
 0x08000000L,   /* TXACGAIN */
 0x01032400L,   /* RXACGAIN */
 0x07BB4F00L, 0x1844B180L, 0x07769E00L,    /* RXACHPF */
#ifdef ENABLE_HIRES_GAIN
 0, 0  /* TXGAIN*10, RXGAIN*10 (hi_res) */
#else
 0, 0  /* TXGAIN, RXGAIN */
#endif
 }   /* ZSYN_220_820_115_30_0 */
};

Si3226_FSK_Cfg Si3226_FSK_Presets[] ={
{
0x02232000L,	 /* FSK01 */
0x077C2000L,	 /* FSK10 */
0x003C0000L,	 /* FSKAMP0 (0.220 vrms )*/
0x00200000L,	 /* FSKAMP1 (0.220 vrms) */
0x06B60000L,	 /* FSKFREQ0 (2200.0 Hz space) */
0x079C0000L,	 /* FSKFREQ1 (1200.0 Hz mark) */
0x00,			 /* FSK8 */
0x00,			 /* FSKDEPTH (1 deep fifo) */
}   /* DEFAULT_FSK */
};

Si3226_Tone_Cfg Si3226_Tone_Presets[] = {
{
	{
	0x07B30000L,	 /* OSC1FREQ (350.000 Hz) */
	0x000C6000L,	 /* OSC1AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x00,			 /* O1TALO (0 ms) */
	0x00,			 /* O1TAHI */
	0x00,			 /* O1TILO (0 ms) */
	0x00			 /* O1TIHI */
	},
	{
	0x07870000L,	 /* OSC2FREQ (440.000 Hz) */
	0x000FA000L,	 /* OSC2AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x00,			 /* O2TALO (0 ms) */
	0x00,			 /* O2TAHI */
	0x00,			 /* O2TILO (0 ms) */
	0x00 			 /* O2TIHI */
	},
	0x66 			 /* OMODE */
},  /* TONEGEN_FCC_DIAL */
{
	{
	0x07700000L,	 /* OSC1FREQ (480.000 Hz) */
	0x00112000L,	 /* OSC1AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0xA0,			 /* O1TALO (500 ms) */
	0x0F,			 /* O1TAHI */
	0xA0,			 /* O1TILO (500 ms) */
	0x0F			 /* O1TIHI */
	},
	{
	0x07120000L,	 /* OSC2FREQ (620.000 Hz) */
	0x00164000L,	 /* OSC2AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0xA0,			 /* O2TALO (500 ms) */
	0x0F,			 /* O2TAHI */
	0xA0,			 /* O2TILO (500 ms) */
	0x0F 			 /* O2TIHI */
	},
	0x66 			 /* OMODE */
},  /* TONEGEN_FCC_BUSY */
{
	{
	0x07700000L,	 /* OSC1FREQ (480.000 Hz) */
	0x00112000L,	 /* OSC1AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC1PHAS (0.000 rad) */
	0x80,			 /* O1TALO (2000 ms) */
	0x3E,			 /* O1TAHI */
	0x00,			 /* O1TILO (4000 ms) */
	0x7D			 /* O1TIHI */
	},
	{
	0x07870000L,	 /* OSC2FREQ (440.000 Hz) */
	0x000FA000L,	 /* OSC2AMP (-18.000 dBm) */
	0x00000000L,	 /* OSC2PHAS (0.000 rad) */
	0x80,			 /* O2TALO (2000 ms) */
	0x3E,			 /* O2TAHI */
	0x00,			 /* O2TILO (4000 ms) */
	0x7D 			 /* O2TIHI */
	},
	0x66 			 /* OMODE */
},  /* TONEGEN_FCC_RINGBACK */
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
	0x40,			 /* O2TILO (200 ms) */
	0x06 			 /* O2TIHI */
	},
	0x66 			 /* OMODE */
},  /* TONEGEN_FCC_REORDER */
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
	0x66 			 /* OMODE */
}   /* TONEGEN_FCC_CONGESTION */
};

Si3226_PCM_Cfg Si3226_PCM_Presets[] ={
	{
	0x01, 	 /* PCM_FMT - u-Law */
	0x00, 	 /* WIDEBAND - DISABLED (3.4kHz BW) */
	0x00, 	 /* PCM_TRI - PCLK RISING EDGE */
	0x00, 	 /* TX_EDGE - PCLK RISING EDGE */
	0x00 	 /* A-LAW -  INVERT NONE */
	},  /* PCM_8ULAW */
	{
	0x00, 	 /* PCM_FMT - A-Law */
	0x00, 	 /* WIDEBAND - DISABLED (3.4kHz BW) */
	0x00, 	 /* PCM_TRI - PCLK RISING EDGE */
	0x00, 	 /* TX_EDGE - PCLK RISING EDGE */
	0x00 	 /* A-LAW -  INVERT NONE */
	},  /* PCM_8ALAW */
	{
	0x03, 	 /* PCM_FMT - 16-bit Linear */
	0x00, 	 /* WIDEBAND - DISABLED (3.4kHz BW) */
	0x00, 	 /* PCM_TRI - PCLK RISING EDGE */
	0x00, 	 /* TX_EDGE - PCLK RISING EDGE */
	0x00 	 /* A-LAW -  INVERT NONE */
	},  /* PCM_16LIN */
	{
	0x03, 	 /* PCM_FMT - 16-bit Linear */
	0x01, 	 /* WIDEBAND - ENABLED (7kHz BW) */
	0x00, 	 /* PCM_TRI - PCLK RISING EDGE */
	0x00, 	 /* TX_EDGE - PCLK RISING EDGE */
	0x00 	 /* A-LAW -  INVERT NONE */
	}   /* PCM_16LIN_WB */
};

