/******************************************************************************

   @file  main.c

   @brief main entry of the BLE stack sample application.

   Group: WCS, BTS
   Target Device: cc13xx_cc26xx

 ******************************************************************************

 Copyright (c) 2013-2024, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************


 *****************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "ble.h"
#include <stdint.h>

#include <xdc/runtime/Error.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>

// Comment this in to use xdc.runtime.Log, but also remove UartLog_init below.
// #include <xdc/runtime/Log.h>
#include <ti/common/cc26xx/uartlog/UartLog.h> // Comment out to use xdc Log.

#include <common/cc26xx/flash_interface/flash_interface.h>
#ifndef EXCLUDE_OAD
#include "find_stack_entry.h"
#endif // EXLUDE_OAD

#include "bcomdef.h"
#include "hal_assert.h"
#include "project_zero.h"
#include <icall.h>

#ifndef USE_DEFAULT_USER_CFG
#include "ble_user_config.h"
// BLE user defined configuration
icall_userCfg_t user0Cfg = BLE_USER_CFG;
#endif // USE_DEFAULT_USER_CFG

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

#ifndef EXCLUDE_OAD
// The stack's image header is found at runtime, available for entire app to use
const imgHdr_t *stackImageHeader = NULL;
#endif // EXLUDE_OAD

/*******************************************************************************
 * EXTERNS
 */

extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);

/*******************************************************************************
 * @fn          Main
 *
 * @brief       Application Main
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */

// pls
#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>
#include <stdint.h>
#include <stdlib.h>

#define ADXL362_CS_PIN  CONFIG_GPIO_1  // Define CS GPIO

SPI_Handle spi;
SPI_Params spiParams;
SPI_Transaction spiTransaction;

// ADXL362 Register Addresses
#define ADXL362_REG_XDATA_L   0x0E  // X-axis lower byte
#define ADXL362_REG_XDATA_H   0x0F  // X-axis higher byte
#define ADXL362_REG_YDATA_L   0x10  // Y-axis lower byte
#define ADXL362_REG_YDATA_H   0x11  // Y-axis higher byte
#define ADXL362_REG_ZDATA_L   0x12  // Z-axis lower byte
#define ADXL362_REG_ZDATA_H   0x13  // Z-axis higher byte
#define ADXL362_REG_POWER_CTL  0x2D
#define ADXL362_MEASURE_MODE   0x02 



uint8_t SPI_readRegister(uint8_t regAddr)
{
    uint8_t txBuffer[2] = {0x0B, regAddr};  // Read command (0x0B) + Register
    uint8_t rxBuffer[2] = {0};

    spiTransaction.count = 2;
    spiTransaction.txBuf = txBuffer;
    spiTransaction.rxBuf = rxBuffer;

    GPIO_write(ADXL362_CS_PIN, 0);  // Pull CS Low
    SPI_transfer(spi, &spiTransaction);
    GPIO_write(ADXL362_CS_PIN, 1);  // Pull CS High

    return rxBuffer[1];  // Return register value
}

void SPI_writeRegister(uint8_t regAddr, uint8_t value)
{
    uint8_t txBuffer[3] = {0x0A, regAddr, value};  // Write command (0x0A) + Register + Value
    uint8_t rxBuffer[3] = {0};

    spiTransaction.count = 3;
    spiTransaction.txBuf = txBuffer;
    spiTransaction.rxBuf = rxBuffer;

    GPIO_write(ADXL362_CS_PIN, 0);  // Pull CS Low
    bool success = SPI_transfer(spi, &spiTransaction);
    GPIO_write(ADXL362_CS_PIN, 1);  // Pull CS High

    if (!success) {
        Log_info0("SPI Write failed!\n");
    }
}

void readAccelerometerData(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t xLow = SPI_readRegister(ADXL362_REG_XDATA_L);
    uint8_t xHigh = SPI_readRegister(ADXL362_REG_XDATA_H);
    uint8_t yLow = SPI_readRegister(ADXL362_REG_YDATA_L);
    uint8_t yHigh = SPI_readRegister(ADXL362_REG_YDATA_H);
    uint8_t zLow = SPI_readRegister(ADXL362_REG_ZDATA_L);
    uint8_t zHigh = SPI_readRegister(ADXL362_REG_ZDATA_H);

    *x = (int16_t)((xHigh << 8) | xLow);
    *y = (int16_t)((yHigh << 8) | yLow);
    *z = (int16_t)((zHigh << 8) | zLow);
}

#define ADXL362_DEVID_AD  0x00 // Device ID Register (should return 0xAD)

void test_ADXL362()
{
    uint8_t dev_id = SPI_readRegister(ADXL362_DEVID_AD);
    Log_info1("ADXL362 Device ID: 0x%d\n", dev_id);

    if (dev_id != 0xAD) {
        Log_info0("Error: Invalid Device ID. Check SPI wiring!\n");
    }
}

void SPI_init_ADXL362()
{
    SPI_init();  // Initialize SPI
    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL0_PHA0;  // Mode 0
    spiParams.dataSize = 8;
    spiParams.bitRate = 115200; // 1 MHz
    spi = SPI_open(CONFIG_SPI_0, &spiParams);
    
    if (spi == NULL) {
        Log_info0("man ----------------------------------------");  // SPI initialization failed
    }

    SPI_writeRegister(ADXL362_REG_POWER_CTL, ADXL362_MEASURE_MODE);
    volatile uint32_t i;
    for (i = 0; i < (1000 * 8000); i++); 
}

int main() {
  // test
  /* Register Application callback to trap asserts raised in the Stack */
  RegisterAssertCback(AssertHandler);

  Board_initGeneral();



  int16_t x, y, z;

    GPIO_init();
    SPI_init_ADXL362();
      test_ADXL362();

        readAccelerometerData(&x, &y, &z);
        
        // Print or process acceleration data
        Log_info0("on god ---------------------------------------");
        Log_info3("X: %d, Y: %d, Z: %d\n", x, y, z);



  

#if !defined(POWER_SAVING)
  /* Set constraints for Standby, powerdown and idle mode */
  // PowerCC26XX_SB_DISALLOW may be redundant
  Power_setConstraint(PowerCC26XX_SB_DISALLOW);
  Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
#endif // POWER_SAVING

  /* Update User Configuration of the stack */
  user0Cfg.appServiceInfo->timerTickPeriod = Clock_tickPeriod;
  user0Cfg.appServiceInfo->timerMaxMillisecond = ICall_getMaxMSecs();

  /* Initialize the RTOS Log formatting and output to UART in Idle thread.
   * Note: Define xdc_runtime_Log_DISABLE_ALL and remove define UARTLOG_ENABLE
   *       to remove all impact of Log statements.
   * Note: NULL as Params gives 115200,8,N,1 and Blocking mode */

  // Initialize UART2 parameters
  UART2_Params params;
  UART2_Params_init(&params);
  // Open the UART
  UartLog_init(UART2_open(CONFIG_DISPLAY_UART, &params));

  /* Initialize ICall module */
  ICall_init();

#ifndef STACK_LIBRARY
  {
    /* Find stack entry page */
    uint32_t stackAddr = findStackBoundaryAddr();

    if (stackAddr == 0xFFFFFFFF) {
      // If we cannot find the stack start address, exit
      ICall_abort();
    }

    /* set the stack image header based on the stack addr */
    stackImageHeader = (imgHdr_t *)stackAddr;

    /* Start tasks of external images - Priority 5 */
    const ICall_RemoteTask_t remoteTaskTbl[] = {
        (ICall_RemoteTaskEntry)(stackImageHeader->prgEntry), 5, 1000,
        &user0Cfg};

    /* Start tasks of external images - Priority 5 */
    ICall_createRemoteTasksAtRuntime(
        (ICall_RemoteTask_t *)remoteTaskTbl,
        (sizeof(remoteTaskTbl) / sizeof(ICall_RemoteTask_t)));
  }
#else

  /* Start tasks of external images - Priority 5 */
  ICall_createRemoteTasks();
#endif

  ProjectZero_createTask();

  /* enable interrupts and start SYS/BIOS */
  BIOS_start();

  return (0);
}

/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.  When EXT_HAL_ASSERT is defined in the Stack
 * Wrapper project this function will be called when an assert is raised, and
 * can be used to observe or trap a violation from expected behavior.
 *
 *              As an example, for Heap allocation failures the Stack will raise
 *              HAL_ASSERT_CAUSE_OUT_OF_MEMORY as the assertCause and
 *              HAL_ASSERT_SUBCAUSE_NONE as the assertSubcause.  An application
 *              developer could trap any malloc failure on the stack by calling
 *              HAL_ASSERT_SPINLOCK under the matching case.
 *
 *              An application developer is encouraged to extend this function
 *              for use by their own application.  To do this, add hal_assert.c
 *              to your project workspace, the path to hal_assert.h (this can
 *              be found on the stack side). Asserts are raised by including
 *              hal_assert.h and using macro HAL_ASSERT(cause) to raise an
 *              assert with argument assertCause.  the assertSubcause may be
 *              optionally set by macro HAL_ASSERT_SET_SUBCAUSE(subCause) prior
 *              to asserting the cause it describes. More information is
 *              available in hal_assert.h.
 *
 * input parameters
 *
 * @param       assertCause    - Assert cause as defined in hal_assert.h.
 * @param       assertSubcause - Optional assert subcause (see hal_assert.h).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void AssertHandler(uint8_t assertCause, uint8_t assertSubcause) {
  Log_error2(">>>STACK ASSERT Cause 0x%02x subCause 0x%02x", assertCause,
             assertSubcause);

  // check the assert cause
  switch (assertCause) {
  case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
    Log_error0("***ERROR***");
    Log_error0(">> OUT OF MEMORY!");
    break;

  case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
    // check the subcause
    if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR) {
      Log_error0("***ERROR***");
      Log_error0(">> INTERNAL FW ERROR!");
    } else {
      Log_error0("***ERROR***");
      Log_error0(">> INTERNAL ERROR!");
    }
    break;

  case HAL_ASSERT_CAUSE_ICALL_ABORT:
    Log_error0("***ERROR***");
    Log_error0(">> ICALL ABORT!");
    // HAL_ASSERT_SPINLOCK;
    break;

  case HAL_ASSERT_CAUSE_ICALL_TIMEOUT:
    Log_error0("***ERROR***");
    Log_error0(">> ICALL TIMEOUT!");
    // HAL_ASSERT_SPINLOCK;
    break;

  case HAL_ASSERT_CAUSE_WRONG_API_CALL:
    Log_error0("***ERROR***");
    Log_error0(">> WRONG API CALL!");
    // HAL_ASSERT_SPINLOCK;
    break;

  default:
    Log_error0("***ERROR***");
    Log_error0(">> DEFAULT SPINLOCK!");
    // HAL_ASSERT_SPINLOCK;
  }

  return;
}

/*******************************************************************************
 */
