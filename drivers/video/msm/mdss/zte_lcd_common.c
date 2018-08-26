#include "zte_lcd_common.h"
#include "mdss_dsi_cmd.h"

#ifdef CONFIG_ZTE_LCD_REG_DEBUG
extern void zte_lcd_reg_debug_func(void);
#endif

extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds, u32 flags);

struct mdss_dsi_ctrl_pdata *g_zte_ctrl_pdata;
/********************read lcm hardware info begin****************/

/*file path: proc/driver/lcd_id/ or proc/msm_lcd*/
static int zte_lcd_proc_info_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_panel_name);
	return 0;
}
static int zte_lcd_proc_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, zte_lcd_proc_info_show, NULL);
}
static const struct file_operations zte_lcd_common_func_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= zte_lcd_proc_info_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static int zte_lcd_proc_info_display(struct mdss_dsi_ctrl_pdata *ctrl_pdata, struct device_node *node)
{

	proc_create_data("driver/lcd_id", 0, NULL, &zte_lcd_common_func_proc_fops, NULL);
	/*proc_create_data("msm_lcd", 0, NULL, &zte_lcd_common_func_proc_fops, NULL);*/
	g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_panel_name = of_get_property(node,
	"qcom,mdss-dsi-panel-name", NULL);

	if (!g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_panel_name) {
		pr_info("%s:%d, panel name not found!\n", __func__, __LINE__);
		return -ENODEV;
	}
	pr_info("[MSM_LCD]%s: Panel Name = %s\n", __func__, g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_panel_name);

	return 0;
}
/********************read lcm hardware info end***********************/

/********************extreme power save mode cabc begin*****************/
#ifdef CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE
static ssize_t lcd_cabc_show(struct device *dev, struct device_attribute *attr, char *buf)
{	int retval = 0;
	mutex_lock(&g_zte_ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	retval = snprintf(buf, 32, "%d\n", g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_value);
	pr_info("[MSM_LCD]%s:ZTE_mdss_lcd_cabc: 0x%x\n", __func__, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_value);
	mutex_unlock(&g_zte_ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	return retval;
}
static int zte_set_panel_cabc(int cabc_mode)
{
	pr_info("[MSM_LCD]%s:cabc_mode= 0x%x\n", __func__, cabc_mode);
	switch (cabc_mode) {
	case 0:
		mdss_dsi_panel_cmds_send(g_zte_ctrl_pdata,
			g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_off_cmds, CMD_REQ_COMMIT);
		break;
	case 1:
		mdss_dsi_panel_cmds_send(g_zte_ctrl_pdata,
			g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_low_cmds, CMD_REQ_COMMIT);
		break;
	case 2:
		mdss_dsi_panel_cmds_send(g_zte_ctrl_pdata,
			g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_medium_cmds, CMD_REQ_COMMIT);
		break;
	case 3:
		mdss_dsi_panel_cmds_send(g_zte_ctrl_pdata,
			g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_high_cmds, CMD_REQ_COMMIT);
		break;
	default:
		mdss_dsi_panel_cmds_send(g_zte_ctrl_pdata,
			g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_off_cmds, CMD_REQ_COMMIT);
		break;
	}
	return 0;
}
static ssize_t lcd_cabc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (kstrtoint(buf, 16, &input) != 0)
		return -EINVAL;
	input = input > 0 ? input : 0;
	mutex_lock(&g_zte_ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_value = input;
	zte_set_panel_cabc(input);
	mutex_unlock(&g_zte_ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	return count;
}
static DEVICE_ATTR(lcd_cabc, 0664,  lcd_cabc_show, lcd_cabc_store);
static struct attribute *lcd_cabc_attributes[] = {
	&dev_attr_lcd_cabc.attr,
		NULL
};
static struct attribute_group lcd_cabc_attribute_group = {
	.attrs = lcd_cabc_attributes
};
int zte_create_cabc_sys(struct mdss_dsi_ctrl_pdata *pdata)
{
	int err;
	struct kobject *lcd_cabc_kobj;

	lcd_cabc_kobj = kobject_create_and_add("cabc", g_zte_ctrl_pdata->zte_lcd_ctrl->kobj);
	if (!lcd_cabc_kobj) {
		err = -EINVAL;
		pr_info("%s() - ERROR Unable to create lcd_cabc_kobj.\n", __func__);
		return -EIO;
	}
	err = sysfs_create_group(lcd_cabc_kobj, &lcd_cabc_attribute_group);
	if (err != 0) {
		pr_info("%s - ERROR lcd_cabc_kobj failed.\n", __func__);
		kobject_put(lcd_cabc_kobj);
		return -EIO;
	}
	pr_info("%s succeeded.\n", __func__);
	return err;
}
#endif
/********************extreme power save mode cabc end*****************/

/********************lcd gpio power ctrl***********************/
#ifdef CONFIG_ZTE_LCD_GPIO_CTRL_POWER
static int zte_gpio_ctrl_lcd_power_enable(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pr_info("%s:%s\n", __func__, enable ? "enable":"disable");
	if (enable) {
		usleep_range(1000, 1100);
		if (ctrl_pdata->zte_lcd_ctrl->disp_avdd_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl->disp_avdd_en_gpio, 1);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl->disp_iovdd_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl->disp_iovdd_en_gpio, 1);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl->disp_vsp_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl->disp_vsp_en_gpio, 1);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl->disp_vsn_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl->disp_vsn_en_gpio, 1);
			usleep_range(5000, 5100);
		}
	} else {
		usleep_range(1000, 1100);
		if (ctrl_pdata->zte_lcd_ctrl->disp_vsn_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl->disp_vsn_en_gpio, 0);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl->disp_vsp_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl->disp_vsp_en_gpio, 0);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl->disp_iovdd_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl->disp_iovdd_en_gpio, 0);
			usleep_range(5000, 5100);
		}
		if (ctrl_pdata->zte_lcd_ctrl->disp_avdd_en_gpio > 0) {
			gpio_set_value(ctrl_pdata->zte_lcd_ctrl->disp_avdd_en_gpio, 0);
			usleep_range(5000, 5100);
		}
	}
	return 0;
}
int zte_gpio_ctrl_lcd_power_init(struct platform_device *ctrl_pdev, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	ctrl_pdata->zte_lcd_ctrl->disp_avdd_en_gpio = of_get_named_gpio(ctrl_pdev->dev.of_node,
		"zte,disp_avdd_en_gpio", 0);
	if (!gpio_is_valid(ctrl_pdata->zte_lcd_ctrl->disp_avdd_en_gpio)) {
		pr_info("%s:%d, zte,disp_avdd_en_gpio not specified\n", __func__, __LINE__);
	} else {
		if (gpio_request(ctrl_pdata->zte_lcd_ctrl->disp_avdd_en_gpio, "disp_avdd_en_gpio")) {
			pr_info("request disp_avdd_en_gpio failed\n");
		} else {
			gpio_direction_output(ctrl_pdata->zte_lcd_ctrl->disp_avdd_en_gpio, 1);
			pr_info("%s:request disp_avdd_en_gpio success\n", __func__);
		}
	}
	ctrl_pdata->zte_lcd_ctrl->disp_iovdd_en_gpio = of_get_named_gpio(ctrl_pdev->dev.of_node,
		"zte,disp_iovdd_en_gpio", 0);
	if (!gpio_is_valid(ctrl_pdata->zte_lcd_ctrl->disp_iovdd_en_gpio)) {
		pr_info("%s:%d, zte,disp_iovdd_en_gpio not specified\n", __func__, __LINE__);
	} else {
		if (gpio_request(ctrl_pdata->zte_lcd_ctrl->disp_iovdd_en_gpio, "disp_iovdd_en_gpio")) {
			pr_info("request disp_iovdd_en_gpio failed %d\n", ctrl_pdata->zte_lcd_ctrl->disp_iovdd_en_gpio);
		} else {
			gpio_direction_output(ctrl_pdata->zte_lcd_ctrl->disp_iovdd_en_gpio, 1);
			pr_info("%s:request disp_iovdd_en_gpio success\n", __func__);
		}
	}

	ctrl_pdata->zte_lcd_ctrl->disp_vsp_en_gpio = of_get_named_gpio(ctrl_pdev->dev.of_node,
		"zte,disp_vsp_en_gpio", 0);
	if (!gpio_is_valid(ctrl_pdata->zte_lcd_ctrl->disp_vsp_en_gpio)) {
		pr_info("%s:%d, zte,disp_vsp_en_gpio not specified\n", __func__, __LINE__);
	} else {
		if (gpio_request(ctrl_pdata->zte_lcd_ctrl->disp_vsp_en_gpio, "disp_vsp_en_gpio")) {
			pr_info("request disp_vsp_en_gpio failed\n");
		} else {
			gpio_direction_output(ctrl_pdata->zte_lcd_ctrl->disp_vsp_en_gpio, 1);
			pr_info("%s:request disp_vsp_en_gpio success\n", __func__);
		}
	}
	ctrl_pdata->zte_lcd_ctrl->disp_vsn_en_gpio = of_get_named_gpio(ctrl_pdev->dev.of_node,
		"zte,disp_vsn_en_gpio", 0);
	if (!gpio_is_valid(ctrl_pdata->zte_lcd_ctrl->disp_vsn_en_gpio)) {
		pr_info("%s:%d, zte,disp_vsn_en_gpio not specified\n", __func__, __LINE__);
	} else {
		if (gpio_request(ctrl_pdata->zte_lcd_ctrl->disp_vsn_en_gpio, "disp_vsn_en_gpio")) {
			pr_info("request disp_vsn_en_gpio failed %d\n", ctrl_pdata->zte_lcd_ctrl->disp_vsn_en_gpio);
		} else {
			gpio_direction_output(ctrl_pdata->zte_lcd_ctrl->disp_vsn_en_gpio, 1);
			pr_info("%s:request disp_vsn_en_gpio success\n", __func__);
		}
	}

	return 0;
}
#endif

/********************lcd backlight level curve begin*****************/
#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
enum {	/* lcd curve mode */
	CURVE_MATRIX_MAX_350_LUX = 1,
	CURVE_MATRIX_MAX_400_LUX,
	CURVE_MATRIX_MAX_450_LUX,
};

int zte_backlight_curve_matrix_max_350_lux[256] = {
0, 1, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9,
10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 16, 16, 17, 17, 18,
18, 19, 19, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 26, 27,
28, 28, 29, 29, 30, 31, 31, 32, 33, 33, 34, 35, 35, 36, 36, 37,
38, 38, 39, 40, 40, 41, 42, 43, 43, 44, 45, 45, 46, 47, 47, 48,
49, 50, 50, 51, 52, 52, 53, 54, 55, 55, 56, 57, 58, 58, 59, 60,
61, 61, 62, 63, 64, 64, 65, 66, 67, 68, 68, 69, 70, 71, 72, 73,
74, 75, 76, 77, 77, 78, 79, 80, 81, 82, 82, 83, 84, 85, 86, 87,
88, 88, 89, 90, 91, 92, 93, 94, 95, 96, 96, 97, 98, 99, 100, 101,
102, 103, 104, 105, 106, 107, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132,
133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 146, 147,
148, 149, 151, 152, 153, 155, 156, 157, 159, 160, 162, 163, 164, 166, 167, 169,
170, 172, 173, 175, 176, 178, 179, 181, 183, 184, 186, 187, 189, 191, 192, 194,
196, 197, 199, 201, 203, 204, 206, 208, 210, 212, 214, 215, 217, 219, 221, 223,
225, 227, 229, 231, 233, 235, 237, 239, 241, 243, 245, 248, 250, 252, 254, 255
};

int zte_backlight_curve_matrix_max_400_lux[256] = {
0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 8,
8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16,
16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24,
25, 25, 26, 26, 27, 27, 28, 28, 29, 30, 30, 31, 31, 32, 32, 33,
34, 34, 35, 35, 36, 37, 37, 38, 38, 39, 40, 40, 41, 42, 42, 43,
43, 44, 45, 45, 46, 47, 47, 48, 49, 49, 50, 51, 51, 52, 53, 53,
54, 55, 55, 56, 57, 57, 58, 59, 59, 60, 61, 61, 62, 63, 64, 64,
65, 66, 66, 67, 68, 69, 69, 70, 71, 72, 72, 73, 74, 74, 75, 76,
77, 78, 78, 79, 80, 81, 81, 82, 83, 84, 84, 85, 86, 87, 88, 88,
89, 90, 91, 92, 92, 93, 94, 95, 96, 96, 97, 98, 99, 100, 101, 101,
102, 103, 104, 105, 106, 107, 107, 108, 109, 110, 111, 112, 113, 113, 114, 115,
116, 117, 118, 119, 120, 121, 121, 122, 123, 124, 125, 126, 127, 128, 129, 129,
130, 132, 133, 134, 136, 137, 139, 140, 142, 143, 145, 147, 148, 150, 151, 153,
155, 156, 158, 160, 162, 163, 165, 167, 169, 170, 172, 174, 176, 178, 180, 182,
184, 186, 188, 190, 192, 194, 196, 198, 200, 203, 205, 207, 209, 212, 214, 216,
219, 221, 223, 226, 228, 231, 233, 236, 238, 241, 243, 246, 249, 252, 254, 255
};

int zte_backlight_curve_matrix_max_450_lux[256] = {
0, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7,
8, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14,
14, 15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21,
22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29,
30, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 36, 36, 37, 37, 38,
38, 39, 39, 40, 41, 41, 42, 42, 43, 43, 44, 45, 45, 46, 46, 47,
48, 48, 49, 49, 50, 51, 51, 52, 52, 53, 54, 54, 55, 56, 56, 57,
57, 58, 59, 59, 60, 61, 61, 62, 63, 63, 64, 65, 65, 66, 67, 67,
68, 69, 69, 70, 71, 71, 72, 73, 73, 74, 75, 75, 76, 77, 78, 78,
79, 80, 80, 81, 82, 83, 83, 84, 85, 85, 86, 87, 88, 88, 89, 90,
91, 91, 92, 93, 94, 94, 95, 96, 97, 97, 98, 99, 100, 101, 101, 102,
103, 104, 105, 105, 106, 107, 108, 109, 109, 110, 111, 112, 112, 112, 113, 113,
114, 116, 117, 119, 120, 122, 123, 125, 126, 128, 130, 131, 133, 135, 136, 138,
140, 142, 143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169,
171, 173, 176, 178, 180, 183, 185, 187, 190, 192, 194, 197, 199, 202, 205, 207,
210, 213, 215, 218, 221, 224, 226, 229, 232, 235, 238, 241, 244, 248, 251, 255
};
static int zte_convert_backlevel_function(int level, u32 bl_max)
{
	int bl, convert_level;

	if (level == 0)
		return 0;

	if (bl_max > 1023) {
		bl = level>>4;
	} else {
		bl = level;
	}

	if (!bl && level)
		bl = 1;/*ensure greater than 0 and less than 16 equal to 1*/

	switch (g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode) {
	case CURVE_MATRIX_MAX_350_LUX:
		convert_level = zte_backlight_curve_matrix_max_350_lux[bl];
		break;
	case CURVE_MATRIX_MAX_400_LUX:
		convert_level = zte_backlight_curve_matrix_max_400_lux[bl];
		break;
	case CURVE_MATRIX_MAX_450_LUX:
		convert_level = zte_backlight_curve_matrix_max_450_lux[bl];
		break;
	default:
		convert_level = zte_backlight_curve_matrix_max_350_lux[bl];
		break;
	}
	if (bl_max > 1023) {
		convert_level = (convert_level >= 255) ? 4095 : (convert_level<<4);
	}

	return convert_level;
}
#endif
/********************lcd backlight level curve end*****************/

/********************lcd common function start*****************/
static void zte_lcd_panel_parse_dt(struct mdss_dsi_ctrl_pdata *ctrl_pdata, struct device_node *node)
{
#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
	const char *data;
#endif
#if defined(CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE) || defined(CONFIG_ZTE_LCD_CABC_53H_CTRL)
	u32 tmp;
	int rc;
#endif

#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
	data = of_get_property(node, "zte,lcm_backlight_curve_mode", NULL);
	if (data) {
		if (!strcmp(data, "lcd_brightness_max_350_lux"))
			ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode = CURVE_MATRIX_MAX_350_LUX;
		else if (!strcmp(data, "lcd_brightness_max_400_lux"))
			ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode = CURVE_MATRIX_MAX_400_LUX;
		else if (!strcmp(data, "lcd_brightness_max_450_lux"))
			ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode = CURVE_MATRIX_MAX_450_LUX;
		else
			ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode = CURVE_MATRIX_MAX_350_LUX;
	} else
		ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode = CURVE_MATRIX_MAX_350_LUX;

	pr_info("[MSM_LCD]%s:dtsi_mode=%s matrix_mode=%d\n", __func__, data,
			ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode);
#endif
	ctrl_pdata->zte_lcd_ctrl->lcd_tp_binding_resetpin = of_property_read_bool(node,
															"zte,lcm_tp_incell_binding_reset_pin");
	ctrl_pdata->zte_lcd_ctrl->lcd_reset_high_sleeping = of_property_read_bool(node,
															"zte,lcm_reset_pin_keep_high_sleeping");

#ifdef CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE
	rc = of_property_read_u32(node, "zte,mdss-dsi-cabc-default-value", &tmp);
	g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_value = (!rc ? tmp : 0x01);
	pr_info("[MSM_LCD]%s:cabc_value =%d\n", __func__, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_value);

	mdss_dsi_parse_dcs_cmds(node, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_off_cmds,
		"zte,mdss-dsi-cabc-off-command", "zte,mdss-dsi-cabc-command-state");
	mdss_dsi_parse_dcs_cmds(node, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_low_cmds,
		"zte,mdss-dsi-cabc-low-command", "zte,mdss-dsi-cabc-command-state");
	mdss_dsi_parse_dcs_cmds(node, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_medium_cmds,
		"zte,mdss-dsi-cabc-medium-command", "zte,mdss-dsi-cabc-command-state");
	mdss_dsi_parse_dcs_cmds(node, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_high_cmds,
		"zte,mdss-dsi-cabc-high-command", "zte,mdss-dsi-cabc-command-state");
#endif

#ifdef CONFIG_ZTE_LCD_CABC_53H_CTRL
	rc = of_property_read_u32(node, "zte,mdss-dsi-cabc-53h-value", &tmp);
	g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_53h = (!rc ? tmp : 0x2c);
	pr_info("[MSM_LCD]%s:cabc_53h =0x%x\n", __func__, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_53h);
#endif
}

static ssize_t zte_show_esd_num(struct device *d, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 80, "esd num:0x%x\n", g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_esd_num);
}
static DEVICE_ATTR(esd_num, 0660, zte_show_esd_num, NULL);
static struct attribute *esd_num_attrs[] = {
	&dev_attr_esd_num.attr,
	NULL,
};
static struct attribute_group esd_num_attr_group = {
	.attrs = esd_num_attrs,
};

static void zte_lcd_common_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata, struct device_node *node)
{
	int ret = 0;

	ctrl_pdata->zte_lcd_ctrl = kzalloc(sizeof(struct zte_lcd_ctrl_data), GFP_KERNEL);
	if (!ctrl_pdata->zte_lcd_ctrl) {
		pr_err("[MSM_LCD] no mem to save zte_lcd_ctrl_data info: kzalloc fail\n");
		return;
	}

	pr_info("[MSM_LCD] %s:alloc zte_lcd_ctrl_data success!\n", __func__);

	/*create /sys/lcd_sys/ path to add other lcd ctrl point*/
	g_zte_ctrl_pdata->zte_lcd_ctrl->kobj = kobject_create_and_add("lcd_sys", NULL);
	if (!g_zte_ctrl_pdata->zte_lcd_ctrl->kobj) {
		pr_info("%s: unable to create lcd_sys\n", __func__);
	}

	/*add /sys/lcd_sys/esd_num point*/
	if (ctrl_pdata->panel_data.panel_info.esd_check_enabled && g_zte_ctrl_pdata->zte_lcd_ctrl->kobj) {
		ret = sysfs_create_group(g_zte_ctrl_pdata->zte_lcd_ctrl->kobj, &esd_num_attr_group);
		if (ret) {
			pr_err("%s:failed to create esd_num_attr_group attributes\n", __func__);
		}
	} else
		pr_info("[MSM_LCD] esd_check_enabled is %d\n", ctrl_pdata->panel_data.panel_info.esd_check_enabled);

#ifdef CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE
	g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_off_cmds = kzalloc(sizeof(struct dsi_panel_cmds), GFP_KERNEL);
	if (!g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_off_cmds) {
		pr_err("[MSM_LCD] %s: cabc_off_cmds kzalloc fail\n", __func__);
	}

	g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_low_cmds = kzalloc(sizeof(struct dsi_panel_cmds), GFP_KERNEL);
	if (!g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_low_cmds) {
		pr_err("[MSM_LCD] %s: cabc_low_cmds kzalloc fail\n", __func__);
	}

	g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_medium_cmds = kzalloc(sizeof(struct dsi_panel_cmds), GFP_KERNEL);
	if (!g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_medium_cmds) {
		pr_err("[MSM_LCD] %s: cabc_medium_cmds kzalloc fail\n", __func__);
	}

	g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_high_cmds = kzalloc(sizeof(struct dsi_panel_cmds), GFP_KERNEL);
	if (!g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_high_cmds) {
		pr_err("[MSM_LCD] %s: cabc_high_cmds kzalloc fail\n", __func__);
	}
#endif

}
void zte_lcd_common_func(struct mdss_dsi_ctrl_pdata *ctrl_pdata, struct device_node *node)
{
	g_zte_ctrl_pdata = ctrl_pdata;

	zte_lcd_common_init(ctrl_pdata, node);
	zte_lcd_panel_parse_dt(ctrl_pdata, node);
#ifdef CONFIG_ZTE_LCD_GPIO_CTRL_POWER
	g_zte_ctrl_pdata->zte_lcd_ctrl->gpio_enable_lcd_power = zte_gpio_ctrl_lcd_power_enable;
#endif
	zte_lcd_proc_info_display(ctrl_pdata, node);
#ifdef CONFIG_ZTE_LCD_REG_DEBUG
	zte_lcd_reg_debug_func();
#endif
#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
	g_zte_ctrl_pdata->zte_lcd_ctrl->zte_convert_brightness = zte_convert_backlevel_function;
#endif

#ifdef CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE
	mutex_init(&ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	zte_create_cabc_sys(ctrl_pdata);
	g_zte_ctrl_pdata->zte_lcd_ctrl->zte_set_cabc_mode = zte_set_panel_cabc;
#endif

}
