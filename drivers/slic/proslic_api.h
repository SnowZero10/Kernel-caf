#ifndef __PROSLIC_API_H_
#define __PROSLIC_API_H_

#include "proslic_api_config.h"
#include "si3217x_constants.h"
#include "si3217x_common_registers.h"
#include "si3217x_revb_registers.h"
#include "si3217x_revc_registers.h"
#include "vdaa_registers.h"
#include "vdaa_constants.h"
#include "proslic.h"
#include "vdaa.h"
#include "si_voice_datatypes.h"
#include "si_voice_timer_intf.h"


/*
** Function: ctrl_ResetWrapper
**
** Description:
** Sets the reset pin of the ProSLIC
*/
int ctrl_ResetWrapper (void *hCtrl, int status);

/*
** register read
**
** Description:
** Reads ProSLIC registers
*/
uInt8 ctrl_ReadRegisterWrapper (void *hCtrl, uInt8 channel, uInt8 regAddr);

/*
** Function: ctrl_WriteRegisterWrapper
**
** Description:
** Writes ProSLIC registers
*/
int ctrl_WriteRegisterWrapper (void *hSpiGci, uInt8 channel, uInt8 regAddr, uInt8 data);

/*
** Function: ctrl_WriteRAMWrapper
**
** Description:
** Writes ProSLIC RAM
*/
int ctrl_WriteRAMWrapper (void *hSpiGci, uInt8 channel, uInt16 ramAddr, ramData data);

/*
** Function: ctrl_ReadRAMWrapper
**
** Description:
** Reads ProSLIC RAM
*/
ramData ctrl_ReadRAMWrapper  (void *hSpiGci, uInt8 channel, uInt16 ramAddr);
/*
** Function: DelayWrapper
**
** Description:
** Waits the specified number of ms
**
** Input Parameters:
** hTimer: timer pointer
** timeInMs: time in ms to wait
**
** Return:
** none
*/
int time_DelayWrapper (void *hTimer, int timeInMs);


/*
** Function: TimeElapsedWrapper
**
** Description:
** Calculates number of ms that have elapsed
**
** Input Parameters:
** hTImer: pointer to timer object
** startTime: timer value when function last called
** timeInMs: pointer to time elapsed
**
** Return:
** timeInMs: time elapsed since start time
*/
int time_TimeElapsedWrapper (void *hTimer, void *startTime, int *timeInMs);

int time_GetTimeWrapper (void *hTimer, void *time);
#endif
