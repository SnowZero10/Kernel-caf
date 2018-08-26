/*
** Copyright © 2008-2010 by Silicon Laboratories
**
** $Id: vdaa_api_config.h 4461 2014-07-16 15:39:20Z nizajerk $
**
** vdaa_api_config.h
** VoiceDAA header config file
**
** Author(s):
** naqamar, laj
**
** Distributed by:
** Silicon Laboratories, Inc
**
** This file contains proprietary information.
** No dissemination allowed without prior written permission from
** Silicon Laboratories, Inc.
**
** File Description:
** This file is used
** in the VoiceDAA demonstration code.
**
**
*/

#ifndef VDAA_API_CFG_H
#define VDAA_API_CFG_H

/* #define DISABLE_MALLOC */
/* #define DISABLE_VDAA_RING_DETECT_SETUP */
/* #define DISABLE_VDAA_AUDIO_GAIN_SETUP */
/* #define DISABLE_VDAA_PCM_SETUP */
/* #define DISABLE_VDAA_COUNTRY_SETUP */
/* #define DISABLE_VDAA_HYBRID_SETUP */
#define DISABLE_VDAA_LOOPBACK_SETUP
#define DISABLE_VDAA_IMPEDANCE_SETUP

#ifndef ENABLE_DEBUG
#define ENABLE_DEBUG
#endif

#ifdef CONFIG_PROSLIC_SI3217X
#define PROSLIC_LINUX_KERNEL
#if !defined(PROSLIC_LINUX_KERNEL)
#include "stdio.h"
#define LOGPRINT printf
#else /* Linux kernel space */
#include <linux/printk.h>
#define LOGPRINT(...) printk(KERN_ERR __VA_ARGS__)
#endif
#else
#include "stdio.h"
#define LOGPRINT printf
#endif

#endif
