/*
* STMicroelectronics lsm6dse driver
*
* Copyright 2014 STMicroelectronics Inc.
*
* Giuseppe Barba <giuseppe.barba@st.com>
* v 1.1.0
* Licensed under the GPL-2.
*/

#ifndef __LSM6DSE_H__
#define __LSM6DSE_H__

#define LSM6DSE_ACC_GYR_DEV_NAME		"lsm6dse"
#define LSM6DSE_ACC_INPUT_DEV_NAME	"lsm6dse_acc"
#define LSM6DSE_GYR_INPUT_DEV_NAME	"lsm6dse_gyr"

struct lsm6dse_platform_data {
	u8 drdy_int_pin;
};

#endif /* __LSM6DSE_H__ */
