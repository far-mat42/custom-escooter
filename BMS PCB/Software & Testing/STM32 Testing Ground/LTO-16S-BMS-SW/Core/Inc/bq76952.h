/*
 * bq76952.h
 * Header file for defining addresses and functions to communicate with the BQ76952 AFE
 *  Created on: Jul 7, 2024
 *      Author: Farris matar
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef INC_BQ76952_H_
#define INC_BQ76952_H_

/* Addresses for R/W registers used for communication with the AFE ---------- */
// Address R/W registers
#define LOWER_ADDR_REG_READ 0x3E
#define UPPER_ADDR_REG_READ 0x3F
#define LOWER_ADDR_REG_WRITE 0xBE
#define UPPER_ADDR_REG_WRITE 0xBF

// Data buffer LSB addresses
#define READ_DATA_BUFF_LSB 0x40
#define WRITE_DATA_BUFF_LSB 0xC0

// Addresses for writing checksum and data length info for transactions
#define READ_CHECKSUM_ADDR 0x60
#define READ_DATALEN_ADDR 0x61
#define WRITE_CHECKSUM_ADDR 0xE0
#define WRITE_DATALEN_ADDR 0xE1

/**
 * Addresses for the RAM registers
 */

/* Calibration RAM registers ------------------------------------------ */
// Calibration gains
#define CAL_GAIN_CL1 			0x9180
#define CAL_GAIN_CL2 			0x9182
#define CAL_GAIN_CL3 			0x9184
#define CAL_GAIN_CL4 			0x9186
#define CAL_GAIN_CL5 			0x9188
#define CAL_GAIN_CL6 			0x918A
#define CAL_GAIN_CL7 			0x918C
#define CAL_GAIN_CL8 			0x918E
#define CAL_GAIN_CL9 			0x9190
#define CAL_GAIN_CL10 			0x9192
#define CAL_GAIN_CL11 			0x9194
#define CAL_GAIN_CL12 			0x9196
#define CAL_GAIN_CL13 			0x9198
#define CAL_GAIN_CL14 			0x919A
#define CAL_GAIN_CL15 			0x919C
#define CAL_GAIN_CL16 			0x919E
#define CAL_GAIN_PACK 			0x91A0
#define CAL_GAIN_TOS 			0x91A2
#define CAL_GAIN_LD 			0x91A4
#define CAL_GAIN_ADC 			0x91A6
#define CAL_GAIN_CC 			0x91A8
#define CAL_GAIN_CAP 			0x91AC

// Calibration temperature offsets
#define CAL_OFST_VCELL 			0x91B0
#define CAL_OFST_VDIV 			0x91B2
#define CAL_OFST_CLMB_CTR 		0x91C6
#define CAL_OFST_BRD 			0x91C6
#define CAL_OFST_INT_TMP 		0x91CA
#define CAL_OFST_CFETOFF_TEMP 	0x91CB
#define CAL_OFST_DFETOFF_TEMP 	0x91CC
#define CAL_OFST_ALERT_TEMP 	0x91CD
#define CAL_OFST_TS1_TEMP 		0x91CE
#define CAL_OFST_TS2_TEMP 		0x91CF
#define CAL_OFST_TS3_TEMP 		0x91D0
#define CAL_OFST_HDQ_TEMP 		0x91D1
#define CAL_OFST_DCHG_TEMP 		0x91D2
#define CAL_OFST_DDSG_TEMP 		0x91D3

// Calibration internal temperature model
#define CAL_INTTEMP_GAIN 		0x91E2
#define CAL_INTTEMP_BASE 		0x91E4
#define CAL_INTTEMP_MAX_AD 		0x91E6
#define CAL_INTTEMP_MAX_TEMP 	0x91E8

// Calibration 18K temperature model
#define CAL_18KTEMP_CF_A1 		0x91EA
#define CAL_18KTEMP_CF_A2 		0x91EC
#define CAL_18KTEMP_CF_A3 		0x91EE
#define CAL_18KTEMP_CF_A4 		0x91F0
#define CAL_18KTEMP_CF_A5 		0x91F2
#define CAL_18KTEMP_CF_B1 		0x91F4
#define CAL_18KTEMP_CF_B2 		0x91F6
#define CAL_18KTEMP_CF_B3 		0x91F8
#define CAL_18KTEMP_CF_B4 		0x91FA
#define CAL_18KTEMP_ADC0 		0x91FE

// Calibration 180K temperature model
#define CAL_180KTEMP_CF_A1 		0x9200
#define CAL_180KTEMP_CF_A2 		0x9202
#define CAL_180KTEMP_CF_A3 		0x9204
#define CAL_180KTEMP_CF_A4 		0x9206
#define CAL_180KTEMP_CF_A5 		0x9208
#define CAL_180KTEMP_CF_B1 		0x920A
#define CAL_180KTEMP_CF_B2 		0x920C
#define CAL_180KTEMP_CF_B3 		0x920E
#define CAL_180KTEMP_CF_B4 		0x9210
#define CAL_180KTEMP_ADC0 		0x9214

// Calibration custom temperature model
#define CAL_CSTMTEMP_CF_A1 		0x9216
#define CAL_CSTMTEMP_CF_A2 		0x9218
#define CAL_CSTMTEMP_CF_A3 		0x921A
#define CAL_CSTMTEMP_CF_A4 		0x921C
#define CAL_CSTMTEMP_CF_A5 		0x921E
#define CAL_CSTMTEMP_CF_B1 		0x9220
#define CAL_CSTMTEMP_CF_B2 		0x9222
#define CAL_CSTMTEMP_CF_B3 		0x9224
#define CAL_CSTMTEMP_CF_B4 		0x9226
#define CAL_CSTMTEMP_RC0 		0x9228
#define CAL_CSTMTEMP_ADC0 		0x922A

// Misc. calibration
#define CAL_CLMB_CURRDB 		0x922D
#define CAL_CUV_OVERRIDE 		0x91D4
#define CAL_COV_OVERRIDE 		0x91D6

/* Settings RAM registers --------------------------------------------------- */
// Settings for the fuse
#define SET_FUSE_MINV 			0x9231
#define SET_FUSE_TO 			0x9233

// Settings for configuration
#define SET_CONF_PWR 			0x9234
#define SET_CONF_REG12 			0x9236
#define SET_CONF_REG0 			0x9237
#define SET_CONF_HWDREG 		0x9238
#define SET_CONF_COMM_TYPE 		0x9239
#define SET_CONF_I2C_ADDR 		0x923A
#define SET_CONF_SPI_CFG 		0x923C
#define SET_CONF_COMM_IDLE 		0x923D
#define SET_CONF_CFETOFF_CFG 	0x92FA
#define SET_CONF_DFETOFF_CFG 	0x92FB
#define SET_CONF_ALERT_CFG 		0x92FC
#define SET_CONF_TS1_CFG 		0x92FD
#define SET_CONF_TS2_CFG 		0x92FE
#define SET_CONF_TS3_CFG 		0x92FF
#define SET_CONF_HDQ_CFG 		0x9300
#define SET_CONF_DCHG_CFG 		0x9301
#define SET_CONF_DDSG_CFG 		0x9302
#define SET_CONF_DA_CFG 		0x9303
#define SET_CONF_VCELL_MODE 	0x9304
#define SET_CONF_CC3_SMPLS 		0x9307

// Settings for protection
#define SET_PROT_CONFIG 		0x925F
#define SET_PROT_ENPROT_A 		0x9261
#define SET_PROT_ENPROT_B 		0x9262
#define SET_PROT_ENPROT_C 		0x9263
#define SET_PROT_CHGFET_PROT_A 	0x9265
#define SET_PROT_CHGFET_PROT_B 	0x9266
#define SET_PROT_CHGFET_PROT_C 	0x9267
#define SET_PROT_DSGFET_PROT_A 	0x9269
#define SET_PROT_DSGFET_PROT_B 	0x926A
#define SET_PROT_DSGFET_PROT_C 	0x926B
#define SET_PROT_BODY_DIODE 	0x9273

// Settings for alarm
#define SET_ALRM_DFLT_MSK 		0x926D
#define SET_ALRM_SFALRT_MSK_A 	0x926F
#define SET_ALRM_SFALRT_MSK_B 	0x9270
#define SET_ALRM_SFALRT_MSK_C 	0x9271
#define SET_ALRM_PFALRT_MSK_A 	0x92C4
#define SET_ALRM_PFALRT_MSK_B 	0x92C5
#define SET_ALRM_PFALRT_MSK_C 	0x92C6
#define SET_ALRM_PFALRT_MSK_D 	0x92C7

// Settings for permanent failure
#define SET_PFAIL_ENPF_A 		0x92C0
#define SET_PFAIL_ENPF_B 		0x92C1
#define SET_PFAIL_ENPF_C 		0x92C2
#define SET_PFAIL_ENPF_D 		0x92C3

// Settings for FET
#define SET_FET_OPTIONS 		0x9308
#define SET_FET_CHG_PMP_CTRL 	0x9309
#define SET_FET_PCHG_STRT_V 	0x930A
#define SET_FET_PCHG_STP_V 		0x930C
#define SET_FET_PDSG_TO 		0x930E
#define SET_FET_PDSG_STP_DLT 	0x930F

// Settings for current thresholds
#define SET_CURRTH_DSG_CURRTH 	0x9310
#define SET_CURRTH_CHG_CURRTH	0x9312

// Settings for interconnect resistances
#define SET_INTRES_CL1 			0x9315
#define SET_INTRES_CL2 			0x9317
#define SET_INTRES_CL3 			0x9319
#define SET_INTRES_CL4 			0x931B
#define SET_INTRES_CL5 			0x931D
#define SET_INTRES_CL6 			0x931F
#define SET_INTRES_CL7 			0x9321
#define SET_INTRES_CL8 			0x9323
#define SET_INTRES_CL9 			0x9325
#define SET_INTRES_CL10			0x9327
#define SET_INTRES_CL11			0x9329
#define SET_INTRES_CL12			0x932B
#define SET_INTRES_CL13			0x932D
#define SET_INTRES_CL14			0x932F
#define SET_INTRES_CL15			0x9331
#define SET_INTRES_CL16			0x9333

// Settings for cell balancing configuration
#define SET_CLBCFG_CONFIG		0x9335
#define SET_CLBCFG_CLTEMP_MIN 	0x9336
#define SET_CLBCFG_CLTEMP_MAX 	0x9337
#define SET_CLBCFG_INTTEMP_MAX	0x9338
#define SET_CLBCFG_CB_INTRVL	0x9339
#define SET_CLBCFG_CB_MAX_CLS	0x933A
#define SET_CLBCFG_CHG_MIN_V	0x933B
#define SET_CLBCFG_CHG_MIN_DLT	0x933D
#define SET_CLBCFG_CHG_STP_DLT	0x933E
#define SET_CLBCFG_RLX_MIN_V	0x933F
#define SET_CLBCFG_RLX_MIN_DLT	0x9341
#define SET_CLBCFG_RLX_STP_DLT	0x9342

// Misc. settings
#define SET_CELLOW_CHECK_TIME 	0x9314
#define SET_MFG_STATUS_INIT 	0x9333

/* Power RAM registers ------------------------------------------------------ */
// Shutdown settings
#define PWR_SHDN_CELL_V			0x923F
#define PWR_SHDN_BATT_V			0x9241
#define PWR_SHDN_LOWV_DLY		0x9243
#define PWR_SHDN_TEMP			0x9244
#define PWR_SHDN_TEMP_DLY		0x9245
#define PWR_SHDN_FETOFF_DLY		0x9252
#define PWR_SHDN_CMD_DLY		0x9253
#define PWR_SHDN_AUTOSHDN_TIM	0x9254
#define PWR_SHDN_RAMFAIL_TIM	0x9255

// Sleep settings
#define PWR_SLP_CURR			0x9248
#define PWR_SLP_VTIME			0x924A
#define PWR_SLP_WAKE_CMP_CURR	0x924B
#define PWR_SLP_HYST_TIME		0x924D
#define PWR_SLP_CHG_V_THLD		0x924E
#define PWR_SLP_CHG_PACKTOS_DLT	0x9250

/* System Data RAM registers ------------------------------------------------ */
// System data integrity
#define SYSDAT_ING_CFG_RAM_SGN	0x91E0

/* Protections RAM registers ------------------------------------------------ */
// CUV settings
#define PROT_CUV_THLD			0x9275
#define PROT_CUV_DLY			0x9276
#define PROT_CUV_RCVR_HYST		0x927B

// COV settings
#define PROT_COV_THLD			0x9278
#define PROT_COV_DLY			0x9279
#define PROT_COV_RCVR_HYST		0x927C

// COVL settings
#define PROT_COVL_LTCH_LMT		0x927D
#define PROT_COVL_CTR_DEC_DLY	0x927E
#define PROT_COVL_RCVR_TIME		0x927F

// OCC settings
#define PROT_OCC_THLD			0x9280
#define PROT_OCC_DLY			0x9281
#define PROT_OCC_RCVR_THLD		0x9288
#define PROT_OCC_PACKTOS_DLT	0x92B0

// OCD settings
#define PROT_OCD_RCVR_THLD		0x928D

#define PROT_OCD1_THLD			0x9282
#define PROT_OCD1_DLY			0x9283

#define PROT_OCD2_THLD			0x9284
#define PROT_OCD2_DLY			0x9285

#define PROT_OCD3_THLD			0x928A
#define PROT_OCD3_DLY			0x928C

#define PROT_OCDL_LTCH_LMT		0x928F
#define PROT_OCDL_CTR_DEC_DLY	0x9290
#define PROT_OCDL_RCVR_TIME		0x9291
#define PROT_OCDL_RCVR_THLD		0x9292

// SCD settings
#define PROT_SCD_THLD			0x9286
#define PROT_SCD_DLY			0x9287
#define PROT_SCD_RCVR_TIME		0x9294

#define PROT_SCDL_LTCH_LMT		0x9295
#define PROT_SCDL_CTR_DEC_DLY	0x9296
#define PROT_SCDL_RCVR_TIME		0x9297
#define PROT_SCDL_RCVR_THLD		0x9298

// OTC settings
#define PROT_OTC_THLD			0x929A
#define PROT_OTC_DLY			0x929B
#define PROT_OTC_RCVR			0x929C

// OTD settings
#define PROT_OTD_THLD			0x929D
#define PROT_OTD_DLY			0x929E
#define PROT_OTD_RCVR			0x929F

// OTF settings
#define PROT_OTF_THLD			0x92A0
#define PROT_OTF_DLY			0x92A1
#define PROT_OTF_RCVR			0x92A2

// OTINT settings
#define PROT_OTINT_THLD			0x92A3
#define PROT_OTINT_DLY			0x92A4
#define PROT_OTINT_RCVR			0x92A5

// UTC settings
#define PROT_UTC_THLD			0x92A6
#define PROT_UTC_DLY			0x92A7
#define PROT_UTC_RCVR			0x92A8

// UTD settings
#define PROT_UTD_THLD			0x92A9
#define PROT_UTD_DLY			0x92AA
#define PROT_UTD_RCVR			0x92AB

// UTINT settings
#define PROT_UTINT_THLD			0x92AC
#define PROT_UTINT_DLY			0x92AD
#define PROT_UTINT_RCVR			0x92AE

// Load Detect settings
#define PROT_LD_ACTV_TIME		0x92B4
#define PROT_LD_RTRY_DLY		0x92B5
#define PROT_LD_TO				0x92B6

// PTO settings
#define PROT_PTO_CHG_THLD		0x92BA
#define PROT_PTO_DLY			0x92BC
#define PROT_PTO_RST			0x92BE

// Misc. settings
#define PROT_RCVR_TIME			0x92AF
#define PROT_HWD_DLY			0x92B2

/* Permanent Fail RAM registers --------------------------------------------- */
// CUDEP settings
#define PFAIL_CUDEP_THLD		0x92C8
#define PFAIL_CUDEP_DLY			0x92CA

// SUV settings
#define PFAIL_SUV_THLD			0x92CB
#define PFAIL_SUV_DLY			0x92CD

// SOV settings
#define PFAIL_SOV_THLD			0x92CE
#define PFAIL_SOV_DLY			0x92D0

// TOS settings
#define PFAIL_TOS_THLD			0x92D1
#define PFAIL_TOS_DLY			0x92D3

// SOCC settings
#define PFAIL_SOCC_THLD			0x92D4
#define PFAIL_SOCC_DLY			0x92D6

// SOCD settings
#define PFAIL_SOCD_THLD			0x92D7
#define PFAIL_SOCD_DLY			0x92D9

// SOT settings
#define PFAIL_SOT_THLD			0x92DA
#define PFAIL_SOT_DLY			0x92DB

// SOTF settings
#define PFAIL_SOTF_THLD			0x92DC
#define PFAIL_SOTF_DLY			0x92DD

// VIMR settings
#define PFAIL_VIMR_CHK_V		0x92DE
#define PFAIL_VIMR_MAX_RLX_I	0x92E0
#define PFAIL_VIMR_THLD			0x92E2
#define PFAIL_VIMR_DLY			0x92E4
#define PFAIL_VIMR_RLX_MIN_DUR	0x92E5

// VIMA settings
#define PFAIL_VIMA_CHK_V		0x92E7
#define PFAIL_VIMA_MIN_ACTV_I	0x92E9
#define PFAIL_VIMA_THLD			0x92EB
#define PFAIL_VIMA_DLY			0x92ED

// CFETF settings
#define PFAIL_CFETF_OFF_THLD	0x92EE
#define PFAIL_CFETF_OFF_DLY		0x92F0

// DFETF settings
#define PFAIL_DFETF_OFF_THLD	0x92F1
#define PFAIL_DFETF_OFF_DLY		0x92F3

// VSSF settings
#define PFAIL_VSSF_FAIL_THLD	0x92F4
#define PFAIL_VSSF_DLY			0x92F6

// Misc. settings
#define PFAIL_2LVL_DLY			0x92F7
#define PFAIL_LFOF_DLY			0x92F8
#define PFAIL_HWMX_DLY			0x92F9

/* Security RAM registers --------------------------------------------------- */
// Security settings
#define SCRTY_SET				0x9256

// Security keys
#define SCRTY_KEY_UNSL_1		0x9257
#define SCRTY_KEY_UNSL_2		0x9259
#define SCRTY_KEY_FL_ACS_1		0x925B
#define SCRTY_KEY_FL_ACS_2		0x925D


/**
 * Helper functions for formatting data types into the AFE's data buffer
 */
void format_uint16(uint8_t *dataArr, uint16_t data);
void format_uint32(uint8_t *dataArr, uint32_t data);

void format_int16(uint8_t *dataArr, int data);
void format_int32(uint8_t *dataArr, int data);

#endif /* INC_BQ76952_H_ */
