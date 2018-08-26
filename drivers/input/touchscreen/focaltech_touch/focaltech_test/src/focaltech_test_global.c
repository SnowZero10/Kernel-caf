/************************************************************************
* Copyright (C) 2012-2017, Focaltech Systems (R)��All Rights Reserved.
*
* File Name: focaltech_test_global.c
*
* Author: Software Development Team, AE
*
* Created: 2016-08-01
*
* Abstract: global function for test
*
************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/delay.h>	/*msleep*/

#include "../include/focaltech_test_ini.h"
#include "../focaltech_test_config.h"
#include "../include/focaltech_test_main.h"
#include "../include/focaltech_ic_table.h"

#define DEVIDE_MODE_ADDR    0x00

/*buff length*/
#define BUFF_LEN_STORE_MSG_AREA     (1024 * 100)
#define BUFF_LEN_MSG_AREA_LINE2     (1024 * 4)
#define BUFF_LEN_STORE_DATA_AREA        (1024 * 80)

/*---------------------About Store Test Dat*/
char *g_pTmpBuff = NULL;
char *g_pStoreMsgArea = NULL;
int g_lenStoreMsgArea = 0;
char *g_pMsgAreaLine2 = NULL;
int g_lenMsgAreaLine2 = 0;
char *g_pStoreDataArea = NULL;
int g_lenStoreDataArea = 0;
unsigned char m_ucTestItemCode = 0;
int m_iStartLine = 0;
int m_iTestDataCount = 0;

char *TestResult = NULL;
int TestResultLen = 0;

/*�ڴ����뷽ʽ*/
#define FTS_MALLOC_TYPE         1	/*0: kmalloc, 1: vmalloc*/
enum enum_malloc_mode {
	kmalloc_mode = 0,
	vmalloc_mode = 1,
};

struct StruScreenSeting g_ScreenSetParam;	/*set screen parameters*/
struct stTestItem g_stTestItem[1][MAX_TEST_ITEM];
struct structSCapConfEx g_stSCapConfEx;
/*struct structSCapConf g_stSCapConf;*/
int g_TestItemNum = 0;
char g_strIcName[20] = { 0 };

char *g_pStoreAllData = NULL;

int GetPrivateProfileString(char *section, char *ItemName, char *defaultvalue,
			    char *returnValue, char *IniFile)
{
	char value[512] = { 0 };
	int len = 0;

	if (returnValue == NULL) {
		FTS_TEST_DBG("[FTS] returnValue is NULL in function.");
		return 0;
	}

	if (ini_get_key(IniFile, section, ItemName, value) < 0) {
		if (defaultvalue != NULL)
			memcpy(value, defaultvalue, strlen(defaultvalue));
		len = snprintf(returnValue, 512, "%s", value);
	} else {
		len = snprintf(returnValue, 512, "%s", value);
	}
	return len;
}

void focal_msleep(int ms)
{
	msleep(ms);
}

void SysDelay(int ms)
{
	msleep(ms);
}

int focal_abs(int value)
{
	if (value < 0)
		value = 0 - value;

	return value;
}

void *fts_malloc(size_t size)
{
	if (kmalloc_mode == FTS_MALLOC_TYPE) {
		goto KMALLOC;
	} else if (vmalloc_mode == FTS_MALLOC_TYPE) {
		goto VMALLOC;
	} else {
		FTS_TEST_DBG("invalid malloc.\n");
		goto INVALID;
	}
KMALLOC:
	kmalloc(size, GFP_ATOMIC);
VMALLOC:
	return vmalloc(size);
INVALID:
	return NULL;
}

void fts_free(void *p)
{
	if (kmalloc_mode == FTS_MALLOC_TYPE) {
		kfree(p);
	} else if (vmalloc_mode == FTS_MALLOC_TYPE) {
		vfree(p);
	} else {
		FTS_TEST_DBG("invalid free.\n");
	}
}

void OnInit_InterfaceCfg(char *strIniFile)
{
	char str[128] = { 0 };

	FTS_TEST_FUNC_ENTER();
	/*IC_Type*/
	GetPrivateProfileString("Interface", "IC_Type", "FT5X36", str, strIniFile);
	g_ScreenSetParam.iSelectedIC = fts_ic_table_get_ic_code_from_ic_name(str);
	FTS_TEST_INFO(" IC code :0x%02x. ", g_ScreenSetParam.iSelectedIC);

	/*Normalize Type*/
	GetPrivateProfileString("Interface", "Normalize_Type", 0, str, strIniFile);
	g_ScreenSetParam.isNormalize = fts_atoi(str);

	FTS_TEST_FUNC_EXIT();

}

/************************************************************************
* Name: ReadReg(Same function name as FT_MultipleTest)
* Brief:  Read Register
* Input: RegAddr
* Output: RegData
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
int ReadReg(unsigned char RegAddr, unsigned char *RegData)
{
	int iRet;

	if (fts_i2c_read_test == NULL) {
		FTS_TEST_DBG("[focal] %s fts_i2c_read_test == NULL  !!! ", __func__);
		return ERROR_CODE_INVALID_COMMAND;
	}

	iRet = fts_i2c_read_test(&RegAddr, 1, RegData, 1);

	if (iRet >= 0)
		return ERROR_CODE_OK;
	else
		return ERROR_CODE_COMM_ERROR;
}

/************************************************************************
* Name: WriteReg(Same function name as FT_MultipleTest)
* Brief:  Write Register
* Input: RegAddr, RegData
* Output: null
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
int WriteReg(unsigned char RegAddr, unsigned char RegData)
{
	int iRet;
	unsigned char cmd[2] = { 0 };

	if (fts_i2c_write_test == NULL) {
		FTS_TEST_DBG("[focal] %s fts_i2c_write_test == NULL  !!!",  __func__);
		return ERROR_CODE_INVALID_COMMAND;
	}

	cmd[0] = RegAddr;
	cmd[1] = RegData;
	iRet = fts_i2c_write_test(cmd, 2);

	if (iRet >= 0)
		return ERROR_CODE_OK;
	else
		return ERROR_CODE_COMM_ERROR;
}

/************************************************************************
* Name: Comm_Base_IIC_IO(Same function name as FT_MultipleTest)
* Brief:  Write/Read Data by IIC
* Input: pWriteBuffer, iBytesToWrite, iBytesToRead
* Output: pReadBuffer
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char Comm_Base_IIC_IO(unsigned char *pWriteBuffer, int iBytesToWrite,
			       unsigned char *pReadBuffer, int iBytesToRead)
{
	int iRet;

	if (fts_i2c_read_test == NULL) {
		FTS_TEST_DBG("[focal] %s fts_i2c_read_test == NULL  !!! ", __func__);
		return ERROR_CODE_INVALID_COMMAND;
	}

	if (iBytesToRead > 0) {
		iRet = fts_i2c_read_test(pWriteBuffer, iBytesToWrite, pReadBuffer, iBytesToRead);
	} else {
		iRet = fts_i2c_write_test(pWriteBuffer, iBytesToWrite);
	}

	if (iRet >= 0)
		return ERROR_CODE_OK;
	else
		return ERROR_CODE_COMM_ERROR;
}

/************************************************************************
* Name: EnterWork(Same function name as FT_MultipleTest)
* Brief:  Enter Work Mode
* Input: null
* Output: null
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char EnterWork(void)
{
	unsigned char RunState = 0;
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	FTS_TEST_FUNC_ENTER();
	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
	if (ReCode == ERROR_CODE_OK) {
		if (((RunState >> 4) & 0x07) == 0x00) {
			ReCode = ERROR_CODE_OK;
		} else {
			ReCode = WriteReg(DEVIDE_MODE_ADDR, 0);
			if (ReCode == ERROR_CODE_OK) {
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
				if (ReCode == ERROR_CODE_OK) {
					if (((RunState >> 4) & 0x07) == 0x00) {
						ReCode = ERROR_CODE_OK;
					} else {
						ReCode = ERROR_CODE_COMM_ERROR;
					}
				} else
					FTS_TEST_ERROR("EnterWork read DEVIDE_MODE_ADDR error 3.");
			} else
				FTS_TEST_ERROR("EnterWork write DEVIDE_MODE_ADDR error 2.");
		}
	} else
		FTS_TEST_ERROR("EnterWork read DEVIDE_MODE_ADDR error 1.");

	FTS_TEST_FUNC_EXIT();

	return ReCode;

}

/************************************************************************
* Name: EnterFactory
* Brief:  enter Fcatory Mode
* Input: null
* Output: null
* Return: Comm Code. Code = 0 is OK, else fail.
***********************************************************************/
unsigned char EnterFactory(void)
{
	unsigned char RunState = 0;
	int index = 0;
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	FTS_TEST_FUNC_ENTER();
	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
	if (ReCode == ERROR_CODE_OK) {
		if (((RunState >> 4) & 0x07) == 0x04) {
			ReCode = ERROR_CODE_OK;
		} else {
			ReCode = WriteReg(DEVIDE_MODE_ADDR, 0x40);
			if (ReCode == ERROR_CODE_OK) {
				for (index = 0; index < 20; ++index) {
					ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
					if (ReCode == ERROR_CODE_OK) {
						if (((RunState >> 4) & 0x07) == 0x04) {
							ReCode = ERROR_CODE_OK;
							goto READ_CODE_OK;
						} else {
							ReCode = ERROR_CODE_COMM_ERROR;
						}
					}
					SysDelay(50);
				}
READ_CODE_OK:
				if (ReCode != ERROR_CODE_OK)
					FTS_TEST_ERROR("EnterFactory read DEVIDE_MODE_ADDR error 3.");
			} else
				FTS_TEST_ERROR("EnterFactory write DEVIDE_MODE_ADDR error 2.");
		}
	} else
		FTS_TEST_ERROR("EnterFactory read DEVIDE_MODE_ADDR error 1.");

	FTS_TEST_FUNC_EXIT();
	return ReCode;
}

/************************************************************************
* Name: fts_SetTestItemCodeName
* Brief:  set test item code and name
* Input: null
* Output: null
* Return:
**********************************************************************/

void fts_SetTestItemCodeName(unsigned char ucitemcode)
{
	g_stTestItem[0][g_TestItemNum].ItemCode = ucitemcode;
/*g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT8607[ucitemcode];*/
	g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
	g_stTestItem[0][g_TestItemNum].TestResult = RESULT_NULL;
	g_TestItemNum++;
}

/************************************************************************
* Name: InitTest
* Brief:  Init all param before test
* Input: none
* Output: none
* Return: none
***********************************************************************/
int InitTest(void)
{
	int ret = 0;

	ret = AllocateMemory();	/*Allocate pointer Memory*/
	if (ret < 0)
		return -EPERM;

	InitStoreParamOfTestData();

	g_stSCapConfEx.ChannelXNum = 0;
	g_stSCapConfEx.ChannelYNum = 0;
	g_stSCapConfEx.KeyNum = 0;
	g_stSCapConfEx.KeyNumTotal = 6;

	return 0;

}

/************************************************************************
* Name: FinishTest
* Brief:  Init all param before test
* Input: none
* Output: none
* Return: none
***********************************************************************/
void FinishTest(void)
{
	MergeAllTestData();	/*Merge Test Result*/
	FreeMemory();		/*Release pointer memory*/
}

/************************************************************************
* Name: InitStoreParamOfTestData
* Brief:  Init store param of test data
* Input: none
* Output: none
* Return: none
***********************************************************************/
void InitStoreParamOfTestData(void)
{
	g_lenStoreMsgArea = 0;
	/*Msg Area, Add Line1*/
	g_lenStoreMsgArea +=
	    snprintf(g_pStoreMsgArea, 128, "ECC, 85, 170, IC Name, %s, IC Code, %x\n",
		    g_strIcName, g_ScreenSetParam.iSelectedIC);

	/*Line2*/
	/*g_pMsgAreaLine2 = NULL;*/
	g_lenMsgAreaLine2 = 0;

	/*Data Area*/
	/*g_pStoreDataArea = NULL;*/
	g_lenStoreDataArea = 0;
	m_iStartLine = 11;	/*The Start Line of Data Area is 11*/

	m_iTestDataCount = 0;
}

/************************************************************************
* Name: MergeAllTestData
* Brief:  Merge All Data of test result
* Input: none
* Output: none
* Return: none
***********************************************************************/
void MergeAllTestData(void)
{
	int iLen = 0;

	/*Add the head part of Line2*/
	iLen = snprintf(g_pTmpBuff, 128, "TestItem, %d, ", m_iTestDataCount);
	memcpy(g_pStoreMsgArea + g_lenStoreMsgArea, g_pTmpBuff, iLen);
	g_lenStoreMsgArea += iLen;

	/*Add other part of Line2, except for "\n"*/
	memcpy(g_pStoreMsgArea + g_lenStoreMsgArea, g_pMsgAreaLine2,  g_lenMsgAreaLine2);
	g_lenStoreMsgArea += g_lenMsgAreaLine2;

	/*Add Line3 ~ Line10*/
	iLen = snprintf(g_pTmpBuff, 128, "\n\n\n\n\n\n\n\n\n");
	memcpy(g_pStoreMsgArea + g_lenStoreMsgArea, g_pTmpBuff, iLen);
	g_lenStoreMsgArea += iLen;

	/*1.Add Msg Area*/
	memcpy(g_pStoreAllData, g_pStoreMsgArea, g_lenStoreMsgArea);

	/*2.Add Data Area*/
	if (g_lenStoreDataArea != 0) {
		memcpy(g_pStoreAllData + g_lenStoreMsgArea, g_pStoreDataArea,  g_lenStoreDataArea);
	}

	FTS_TEST_DBG("lenStoreMsgArea=%d,  lenStoreDataArea = %d", g_lenStoreMsgArea, g_lenStoreDataArea);
}

/**/
/************************************************************************
* Name: AllocateMemory
* Brief:  Allocate pointer Memory
* Input: none
* Output: none
* Return: none
***********************************************************************/
int AllocateMemory(void)
{
	/*New buff*/
	g_pStoreMsgArea = NULL;
	if (g_pStoreMsgArea == NULL)
		g_pStoreMsgArea = fts_malloc(BUFF_LEN_STORE_MSG_AREA);
	if (g_pStoreMsgArea == NULL)
		goto ERR;

	g_pMsgAreaLine2 = NULL;
	if (g_pMsgAreaLine2 == NULL)
		g_pMsgAreaLine2 = fts_malloc(BUFF_LEN_MSG_AREA_LINE2);
	if (g_pMsgAreaLine2 == NULL)
		goto ERR;

	g_pStoreDataArea = NULL;
	if (g_pStoreDataArea == NULL)
		g_pStoreDataArea = fts_malloc(BUFF_LEN_STORE_DATA_AREA);
	if (g_pStoreDataArea == NULL)
		goto ERR;

	g_pTmpBuff = NULL;
	if (g_pTmpBuff == NULL)
		g_pTmpBuff = fts_malloc(BUFF_LEN_TMP_BUFFER);
	if (g_pTmpBuff == NULL)
		goto ERR;

	TestResult = NULL;
	if (TestResult == NULL)
		TestResult = fts_malloc(BUFF_LEN_TMP_BUFFER);
	if (TestResult == NULL)
		goto ERR;

	return 0;

ERR:
	FTS_TEST_ERROR("fts_malloc memory failed in function.");
	return -EPERM;

}

/************************************************************************
* Name: FreeMemory
* Brief:  Release pointer memory
* Input: none
* Output: none
* Return: none
***********************************************************************/
void FreeMemory(void)
{
	/*Release buff*/
	if (g_pStoreMsgArea != NULL)
		fts_free(g_pStoreMsgArea);

	if (g_pMsgAreaLine2 != NULL)
		fts_free(g_pMsgAreaLine2);

	if (g_pStoreDataArea != NULL)
		fts_free(g_pStoreDataArea);

	/*if(NULL == g_pStoreAllData)
	   fts_free(g_pStoreAllData); */

	if (g_pTmpBuff != NULL)
		fts_free(g_pTmpBuff);
}
