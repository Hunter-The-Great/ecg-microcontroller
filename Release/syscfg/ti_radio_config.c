/*
 *  ======== ti_radio_config.c ========
 *  Configured RadioConfig module definitions
 *
 *  DO NOT EDIT - This file is generated for the CC2651R3SIPA
 *  by the SysConfig tool.
 *
 *  Radio Config module version : 1.18.0
 *  SmartRF Studio data version : 2.31.0
 */

#include "ti_radio_config.h"
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_bt5.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_mce_bt5.h)

// Custom overrides
#include <ti/ble5stack_flash/icall/inc/ble_overrides.h>


// *********************************************************************************
//   RF Frontend configuration
// *********************************************************************************
// RF design based on: LP_CC2651R3SIPA

// TX Power tables
// The RF_TxPowerTable_DEFAULT_PA_ENTRY and RF_TxPowerTable_HIGH_PA_ENTRY macros are defined in RF.h.
// The following arguments are required:
// RF_TxPowerTable_DEFAULT_PA_ENTRY(bias, gain, boost, coefficient)
// RF_TxPowerTable_HIGH_PA_ENTRY(bias, ibboost, boost, coefficient, ldoTrim)
// See the Technical Reference Manual for further details about the "txPower" Command field.
// The PA settings require the CCFG_FORCE_VDDR_HH = 0 unless stated otherwise.

// 2400 MHz, 5 dBm
RF_TxPowerTable_Entry txPowerTable_2400_pa5[TXPOWERTABLE_2400_PA5_SIZE] =
{
    {-20, RF_TxPowerTable_DEFAULT_PA_ENTRY(8, 3, 0, 0) }, // 0x00C8
    {-18, RF_TxPowerTable_DEFAULT_PA_ENTRY(10, 3, 0, 0) }, // 0x00CA
    {-15, RF_TxPowerTable_DEFAULT_PA_ENTRY(13, 3, 0, 0) }, // 0x00CD
    {-12, RF_TxPowerTable_DEFAULT_PA_ENTRY(16, 3, 0, 0) }, // 0x00D0
    {-10, RF_TxPowerTable_DEFAULT_PA_ENTRY(18, 3, 0, 0) }, // 0x00D2
    {-9, RF_TxPowerTable_DEFAULT_PA_ENTRY(20, 3, 0, 0) }, // 0x00D4
    {-6, RF_TxPowerTable_DEFAULT_PA_ENTRY(26, 3, 0, 4) }, // 0x08DA
    {-5, RF_TxPowerTable_DEFAULT_PA_ENTRY(28, 3, 0, 4) }, // 0x08DC
    {-3, RF_TxPowerTable_DEFAULT_PA_ENTRY(36, 3, 0, 7) }, // 0x0EE4
    {0, RF_TxPowerTable_DEFAULT_PA_ENTRY(34, 2, 0, 16) }, // 0x20A2
    {1, RF_TxPowerTable_DEFAULT_PA_ENTRY(37, 2, 0, 16) }, // 0x20A5
    {2, RF_TxPowerTable_DEFAULT_PA_ENTRY(36, 1, 0, 23) }, // 0x2E64
    {3, RF_TxPowerTable_DEFAULT_PA_ENTRY(42, 1, 0, 18) }, // 0x246A
    {4, RF_TxPowerTable_DEFAULT_PA_ENTRY(30, 0, 0, 48) }, // 0x601E
    {5, RF_TxPowerTable_DEFAULT_PA_ENTRY(46, 0, 0, 82) }, // 0xA42E
    RF_TxPowerTable_TERMINATION_ENTRY
};



//*********************************************************************************
//  RF Setting:   BLE, 2 Mbps, LE 2M
//
//  PHY:          bt5le2m
//  Setting file: setting_bt5_le_2m.json
//*********************************************************************************

// PARAMETER SUMMARY
// NB! Setting RF parameters in this design has no effect as no RF commands are selected.

// TI-RTOS RF Mode Object
RF_Mode RF_modeBle =
{
    .rfMode = RF_MODE_AUTO,
    .cpePatchFxn = &rf_patch_cpe_bt5,
    .mcePatchFxn = &rf_patch_mce_bt5,
    .rfePatchFxn = 0
};

// Overrides for CMD_BLE5_RADIO_SETUP
uint32_t pOverrides_bleCommon[] =
{
    // override_ble5_setup_override_common.json
    // DC/DC regulator: In Tx, use DCDCCTL5[3:0]=0x3 (DITHER_EN=0 and IPEAK=3).
    (uint32_t)0x00F388D3,
    // Bluetooth 5: Set pilot tone length to 20 us Common
    HW_REG_OVERRIDE(0x6024,0x2E20),
    // Bluetooth 5: Compensate for reduced pilot tone length
    (uint32_t)0x01280263,
    // Bluetooth 5: Default to no CTE. 
    HW_REG_OVERRIDE(0x5328,0x0000),
    // Synth: Increase mid code calibration time to 5 us
    (uint32_t)0x00058683,
    // Synth: Increase mid code calibration time to 5 us
    HW32_ARRAY_OVERRIDE(0x4004,1),
    // Synth: Increase mid code calibration time to 5 us
    (uint32_t)0x38183C30,
    // Bluetooth 5: Move synth start code
    HW_REG_OVERRIDE(0x4064,0x3C),
    // Bluetooth 5: Set DTX gain -5% for 1 Mbps
    (uint32_t)0x00E787E3,
    // Bluetooth 5: Set DTX threshold 1 Mbps
    (uint32_t)0x00950803,
    // Bluetooth 5: Set DTX gain -2.5% for 2 Mbps
    (uint32_t)0x00F487F3,
    // Bluetooth 5: Set DTX threshold 2 Mbps
    (uint32_t)0x012A0823,
    // Bluetooth 5: Set synth fine code calibration interval
    HW32_ARRAY_OVERRIDE(0x4020,1),
    // Bluetooth 5: Set synth fine code calibration interval
    (uint32_t)0x41005F00,
    // Bluetooth 5: Adapt to synth fine code calibration interval
    (uint32_t)0xC0040141,
    // Bluetooth 5: Adapt to synth fine code calibration interval
    (uint32_t)0x0007DD44,
    // Bluetooth 5: Set enhanced TX shape
    (uint32_t)0x000D8C73,
    // ti/ble5stack_flash/icall/inc/ble_overrides.h
    BLE_STACK_OVERRIDES(),
    (uint32_t)0xFFFFFFFF
};

// Overrides for CMD_BLE5_RADIO_SETUP
uint32_t pOverrides_ble1Mbps[] =
{
    // override_ble5_setup_override_1mbps.json
    // Bluetooth 5: Set pilot tone length to 20 us
    HW_REG_OVERRIDE(0x5320,0x03C0),
    // Bluetooth 5: Compensate syncTimeadjust
    (uint32_t)0x015302A3,
    // override_ble5_symbol_error_tracking.json
    // Symbol tracking: timing correction
    HW_REG_OVERRIDE(0x50D4,0x00F9),
    // Symbol tracking: reduce sample delay
    HW_REG_OVERRIDE(0x50E0,0x0087),
    // Symbol tracking: demodulation order
    HW_REG_OVERRIDE(0x50F8,0x0014),
    (uint32_t)0xFFFFFFFF
};

// Overrides for CMD_BLE5_RADIO_SETUP
uint32_t pOverrides_ble2Mbps[] =
{
    // override_ble5_setup_override_2mbps_residual_per.json
    // PHY: Use MCE RAM (patch), RFE ROM
    MCE_RFE_OVERRIDE(1,0,2,0,3,2),
    // Rx: increase AGC hysteresis (HIGH_GAIN=15, LOW_GAIN=11)
    HW_REG_OVERRIDE(0x6098,0x75FB),
    // override_ble5_setup_override_2mbps.json
    // Bluetooth 5: Set pilot tone length to 20 us
    HW_REG_OVERRIDE(0x5320,0x03C0),
    // Bluetooth 5: Compensate syncTimeAdjust
    (uint32_t)0x00F102A3,
    // Bluetooth 5: increase low gain AGC delay for 2 Mbps
    HW_REG_OVERRIDE(0x60A4,0x7D00),
    // override_ble5_symbol_error_tracking.json
    // Symbol tracking: timing correction
    HW_REG_OVERRIDE(0x50D4,0x00F9),
    // Symbol tracking: reduce sample delay
    HW_REG_OVERRIDE(0x50E0,0x0087),
    // Symbol tracking: demodulation order
    HW_REG_OVERRIDE(0x50F8,0x0014),
    (uint32_t)0xFFFFFFFF
};

// Overrides for CMD_BLE5_RADIO_SETUP
uint32_t pOverrides_bleCoded[] =
{
    // override_ble5_setup_override_coded.json
    // Bluetooth 5: Set pilot tone length to 20 us
    HW_REG_OVERRIDE(0x5320,0x03C0),
    // Bluetooth 5: Compensate syncTimeadjust
    (uint32_t)0x07A902A3,
    // Rx: Set AGC reference level to 0x21 (default: 0x2E)
    HW_REG_OVERRIDE(0x609C,0x0021),
    (uint32_t)0xFFFFFFFF
};




