#ifndef _TOUCHSCREEN_FW_
#define _TOUCHSCREEN_FW_

#if (defined(CONFIG_BOARD_LAVENDER) || defined(CONFIG_BOARD_BAFFIN))
#define FTC_LIANCHUANG_FW_NAME	"ZTEMT_Z716BL_FT5346i_LC_LCFB04012360_Ver0x0C_ID0x87_V0C_D01_20150728_app.bin"
#elif (defined(CONFIG_BOARD_FORBES) || defined(CONFIG_BOARD_ELDEN))
#define SYN_LCE_FW_NAME	"Nubia_Z799VL_LCE_S2333_DS3.5.5_PR2081494_51503039.img"
#elif defined(CONFIG_BOARD_GEMI)
#define SYN_DJN_FW_NAME "ZTE_P890A08_DJN_S3603_DS5_PR2611852_55493532.img"
#define SYN_HOLITCH_FW_NAME "ZTE_P890A08_Holitch_S3603_DS5_PR2611852_55523034.img"
#elif (defined(CONFIG_BOARD_DRACO) || defined(CONFIG_BOARD_JEFF))
#define SYN_LCE_FW_NAME "Nubia_Z835_S2333_DS3.5.5_PR2081494_51503035.img"
#elif (defined(CONFIG_BOARD_KELLY) || defined(CONFIG_BOARD_LEWIS) \
	|| defined(CONFIG_BOARD_GRAYJOYLITE) || defined(CONFIG_BOARD_LOFT))
#define SYN_HOLITCH_FW_NAME "ZTE_P890A16_Holitech_S3603_DS5_PR2605936_55523033.img"
#define SYN_OFILM_FW_NAME "ZTE_P890A16_Ofilm_S3603_DS5_PR2605936_55343033.img"
#elif defined(CONFIG_BOARD_CALBEE)
#define SYN_HUAXINGDA_FW_NAME "ZTE_P890T18_HXD_S3603_DS5_PR2605936_55543032.img"
#define SYN_HOLITCH_FW_NAME "ZTE_P890A16_Holitech_S3603_DS5_PR2605936_55523033.img"
#define SYN_OFILM_FW_NAME "ZTE_P890A16_Ofilm_S3603_DS5_PR2605936_55343033.img"
#elif defined(CONFIG_BOARD_SWEET)
#define SYN_HOLITCH_FW_NAME	"ZTE_P890V07_Holitech_5P0_854_480_S3603_DS5_17.0.0.1085_PR2605936_55523036.img"
#define SYN_OFILM_FW_NAME "ZTE_P890V07_Ofilm_5P0_854_480_S3603_DS5_17.0.0.1085_PR2731570_55343033.img"
#elif (defined(CONFIG_BOARD_LENA) || defined(CONFIG_BOARD_CASTELLA))
#define SYN_HOLITCH_FW_NAME	"Holitech_Synaptics_S3603_Z5000A_Z559DL.img"
#else
#define SYN_TPK_FW_NAME		""
#define SYN_TURLY_FW_NAME	""
#define SYN_SUCCESS_FW_NAME	""
#define SYN_OFILM_FW_NAME	""
#define SYN_LEAD_FW_NAME	""
#define SYN_WINTEK_FW_NAME	""
#define SYN_LAIBAO_FW_NAME	""
#define SYN_CMI_FW_NAME		""
#define SYN_ECW_FW_NAME		""
#define SYN_GOWORLD_FW_NAME	""
#define SYN_BAOMING_FW_NAME	""
#define SYN_EACHOPTO_FW_NAME	""
#define SYN_MUTTO_FW_NAME	""
#define SYN_JUNDA_FW_NAME	""
#define SYN_BOE_FW_NAME	""
#define SYN_TIANMA_FW_NAME	""
#define SYN_SAMSUNG_FW_NAME  ""
#define SYN_DIJING_FW_NAME  ""
#define SYN_LCE_FW_NAME	""
#define SYN_LSE_FW_NAME ""
#define SYN_GUOXIAN_FW_NAME	""
#define SYN_HOLITCH_FW_NAME ""

#define FTC_TPK_FW_NAME		""
#define FTC_TURLY_FW_NAME	""
#define FTC_SUCCESS_FW_NAME	""
#define FTC_OFILM_FW_NAME	""
#define FTC_LEAD_FW_NAME	""
#define FTC_WINTEK_FW_NAME	""
#define FTC_LAIBAO_FW_NAME	""
#define FTC_CMI_FW_NAME		""
#define FTC_ECW_FW_NAME		""
#define FTC_GOWORLD_FW_NAME	""
#define FTC_BAOMING_FW_NAME	""
#define FTC_JUNDA_FW_NAME	""
#define FTC_JIAGUAN_FW_NAME	""
#define FTC_MUDONG_FW_NAME	""
#define FTC_EACHOPTO_FW_NAME	""
#define FTC_AVC_FW_NAME	""

#define GTP_SENSOR_ID_1_FW_NAME		""
#define GTP_SENSOR_ID_2_FW_NAME		""
#define GTP_SENSOR_ID_3_FW_NAME		""
#define GTP_SENSOR_ID_4_FW_NAME		""
#define GTP_SENSOR_ID_5_FW_NAME		""
#define GTP_SENSOR_ID_6_FW_NAME		""

#define CY_TPK_FW_NAME		""
#define CY_TURLY_FW_NAME	""
#define CY_SUCCESS_FW_NAME	""
#define CY_OFILM_FW_NAME	""
#define CY_LEAD_FW_NAME	""
#define CY_WINTEK_FW_NAME	""
#define CY_LAIBAO_FW_NAME	""
#define CY_CMI_FW_NAME		""
#define CY_ECW_FW_NAME		""
#define CY_GOWORLD_FW_NAME	""
#define CY_BAOMING_FW_NAME	""
#define CY_JUNDA_FW_NAME	""
#define CY_JIAGUAN_FW_NAME	""
#define CY_MUDONG_FW_NAME	""
#define CY_EACHOPTO_FW_NAME	""
#define SYN_DJN_FW_NAME	""
#define SYN_HOLITCH_FW_NAME	""
#define SYN_HUAXINGDA_FW_NAME	""
#endif

#ifndef SYN_TPK_FW_NAME
#define SYN_TPK_FW_NAME		""
#endif
#ifndef SYN_TURLY_FW_NAME
#define SYN_TURLY_FW_NAME	""
#endif
#ifndef SYN_SUCCESS_FW_NAME
#define SYN_SUCCESS_FW_NAME	""
#endif
#ifndef SYN_OFILM_FW_NAME
#define SYN_OFILM_FW_NAME	""
#endif
#ifndef SYN_LEAD_FW_NAME
#define SYN_LEAD_FW_NAME	""
#endif
#ifndef SYN_WINTEK_FW_NAME
#define SYN_WINTEK_FW_NAME	""
#endif
#ifndef SYN_LAIBAO_FW_NAME
#define SYN_LAIBAO_FW_NAME	""
#endif
#ifndef SYN_CMI_FW_NAME
#define SYN_CMI_FW_NAME		""
#endif
#ifndef SYN_ECW_FW_NAME
#define SYN_ECW_FW_NAME		""
#endif
#ifndef SYN_GOWORLD_FW_NAME
#define SYN_GOWORLD_FW_NAME	""
#endif
#ifndef SYN_BAOMING_FW_NAME
#define SYN_BAOMING_FW_NAME	""
#endif
#ifndef SYN_EACHOPTO_FW_NAME
#define SYN_EACHOPTO_FW_NAME	""
#endif
#ifndef SYN_MUTTO_FW_NAME
#define SYN_MUTTO_FW_NAME	""
#endif
#ifndef SYN_JUNDA_FW_NAME
#define SYN_JUNDA_FW_NAME	""
#endif
#ifndef SYN_BOE_FW_NAME
#define SYN_BOE_FW_NAME	""
#endif
#ifndef SYN_TIANMA_FW_NAME
#define SYN_TIANMA_FW_NAME	""
#endif
#ifndef SYN_SAMSUNG_FW_NAME
#define SYN_SAMSUNG_FW_NAME  ""
#endif
#ifndef SYN_DIJING_FW_NAME
#define SYN_DIJING_FW_NAME  ""
#endif
#ifndef SYN_LCE_FW_NAME
#define SYN_LCE_FW_NAME	""
#endif
#ifndef SYN_DJN_FW_NAME
#define SYN_DJN_FW_NAME	""
#endif
#ifndef SYN_GUOXIAN_FW_NAME
#define SYN_GUOXIAN_FW_NAME ""
#endif
#ifndef SYN_LSE_FW_NAME
#define SYN_LSE_FW_NAME ""
#endif
#ifndef SYN_HOLITCH_FW_NAME
#define SYN_HOLITCH_FW_NAME	""
#endif
#ifndef SYN_HUAXINGDA_FW_NAME
#define SYN_HUAXINGDA_FW_NAME	""
#endif

#ifndef FTC_TPK_FW_NAME
#define FTC_TPK_FW_NAME		""
#endif
#ifndef FTC_TURLY_FW_NAME
#define FTC_TURLY_FW_NAME	""
#endif
#ifndef FTC_SUCCESS_FW_NAME
#define FTC_SUCCESS_FW_NAME	""
#endif
#ifndef FTC_OFILM_FW_NAME
#define FTC_OFILM_FW_NAME	""
#endif
#ifndef FTC_LEAD_FW_NAME
#define FTC_LEAD_FW_NAME	""
#endif
#ifndef FTC_WINTEK_FW_NAME
#define FTC_WINTEK_FW_NAME	""
#endif
#ifndef FTC_LAIBAO_FW_NAME
#define FTC_LAIBAO_FW_NAME	""
#endif
#ifndef FTC_CMI_FW_NAME
#define FTC_CMI_FW_NAME		""
#endif
#ifndef FTC_ECW_FW_NAME
#define FTC_ECW_FW_NAME		""
#endif
#ifndef FTC_GOWORLD_FW_NAME
#define FTC_GOWORLD_FW_NAME	""
#endif
#ifndef FTC_BAOMING_FW_NAME
#define FTC_BAOMING_FW_NAME	""
#endif
#ifndef FTC_JUNDA_FW_NAME
#define FTC_JUNDA_FW_NAME	""
#endif
#ifndef FTC_JIAGUAN_FW_NAME
#define FTC_JIAGUAN_FW_NAME	""
#endif
#ifndef FTC_MUDONG_FW_NAME
#define FTC_MUDONG_FW_NAME	""
#endif
#ifndef FTC_EACHOPTO_FW_NAME
#define FTC_EACHOPTO_FW_NAME	""
#endif
#ifndef FTC_AVC_FW_NAME
#define FTC_AVC_FW_NAME	""
#endif
#ifndef FTC_MUTTO_FW_NAME
#define FTC_MUTTO_FW_NAME ""
#endif
#ifndef FTC_BOE_FW_NAME
#define FTC_BOE_FW_NAME ""
#endif
#ifndef FTC_TIANMA_FW_NAME
#define FTC_TIANMA_FW_NAME ""
#endif
#ifndef FTC_SAMSUNG_FW_NAME
#define FTC_SAMSUNG_FW_NAME ""
#endif
#ifndef FTC_DIJING_FW_NAME
#define FTC_DIJING_FW_NAME ""
#endif
#ifndef FTC_LCE_FW_NAME
#define FTC_LCE_FW_NAME ""
#endif
#ifndef SYN_GUOXIAN_FW_NAME
#define SYN_GUOXIAN_FW_NAME	""
#endif
#ifndef SYN_HOLITCH_FW_NAME
#define SYN_HOLITCH_FW_NAME	""
#endif

#ifndef GTP_SENSOR_ID_1_FW_NAME
#define GTP_SENSOR_ID_1_FW_NAME		""
#endif
#ifndef GTP_SENSOR_ID_2_FW_NAME
#define GTP_SENSOR_ID_2_FW_NAME		""
#endif
#ifndef GTP_SENSOR_ID_3_FW_NAME
#define GTP_SENSOR_ID_3_FW_NAME		""
#endif
#ifndef GTP_SENSOR_ID_4_FW_NAME
#define GTP_SENSOR_ID_4_FW_NAME		""
#endif
#ifndef GTP_SENSOR_ID_5_FW_NAME
#define GTP_SENSOR_ID_5_FW_NAME		""
#endif
#ifndef GTP_SENSOR_ID_6_FW_NAME
#define GTP_SENSOR_ID_6_FW_NAME		""
#endif

#ifndef CY_TPK_FW_NAME
#define CY_TPK_FW_NAME		""
#endif
#ifndef CY_TURLY_FW_NAME
#define CY_TURLY_FW_NAME	""
#endif
#ifndef CY_SUCCESS_FW_NAME
#define CY_SUCCESS_FW_NAME	""
#endif
#ifndef CY_OFILM_FW_NAME
#define CY_OFILM_FW_NAME	""
#endif
#ifndef CY_LEAD_FW_NAME
#define CY_LEAD_FW_NAME	""
#endif
#ifndef CY_WINTEK_FW_NAME
#define CY_WINTEK_FW_NAME	""
#endif
#ifndef CY_LAIBAO_FW_NAME
#define CY_LAIBAO_FW_NAME	""
#endif
#ifndef CY_CMI_FW_NAME
#define CY_CMI_FW_NAME		""
#endif
#ifndef CY_ECW_FW_NAME
#define CY_ECW_FW_NAME		""
#endif
#ifndef CY_GOWORLD_FW_NAME
#define CY_GOWORLD_FW_NAME	""
#endif
#ifndef CY_BAOMING_FW_NAME
#define CY_BAOMING_FW_NAME	""
#endif
#ifndef CY_JUNDA_FW_NAME
#define CY_JUNDA_FW_NAME	""
#endif
#ifndef CY_JIAGUAN_FW_NAME
#define CY_JIAGUAN_FW_NAME	""
#endif
#ifndef CY_MUDONG_FW_NAME
#define CY_MUDONG_FW_NAME	""
#endif
#ifndef CY_EACHOPTO_FW_NAME
#define CY_EACHOPTO_FW_NAME	""
#endif

struct tp_vendor_fw_t {
	int vendor_id;
	char *vendor_name;
	char *firmware_name;
};

#define SYN_MOUDLE_NUM_MAX 27
#define FTC_MOUDLE_NUM_MAX 19
#define GTP_MOUDLE_NUM_MAX 6
#define CY_MOUDLE_NUM_MAX 15
#endif
