/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_clint.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* This implementation allows use of the Core Local Interrupts (CLINT).
 * Those are 4 local level-type interrupt sources that are reserved.
 *
 * | ID  |        Description        | Priority |
 * |  0  | U mode software interrupt |    1     |
 * |  3  | M mode software interrupt |    3     |
 * |  4  |  U mode timer interrupt   |    0     |
 * |  7  |  M mode timer interrupt   |    1     |
 *
 * It is possible to use those as simple software interrupt or as timer
 * counter and interrupt. They are not available as a common interrupt,
 * and require some specific configuration.
 *
 * This is, for now, mostly used for system integration testing.
 *
 * For more details, see Technical Reference Manual, section:
 * - 1.7 for ESP32|C6|H2;
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "riscv_internal.h"

#include "esp_attr.h"
#include "esp_bit_defs.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "riscv/interrupt.h"
#include "soc/soc.h"
#include "soc/clint_reg.h"

#include "esp_clint.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_COMPARE_VALUE_DEFAULT_L 0x2710
#define TIMER_COMPARE_VALUE_DEFAULT_H 0x0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_clint_setup
 *
 * Description:
 *   Default callback function for CLINT interrupt.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR static int esp_clint_default_isr(int argc, char *argv[])
{
  /* Interrupt is cleared by disabling MTIE bit or modifying
   * the timer compare value.
   */

  esp_clint_timer_int_disable();
  esp_clint_reset_timer_counter();
  esp_clint_timer_int_enable();

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_clint_setup
 *
 * Description:
 *   .
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_clint_setup(void)
{
  int ret;

  /* Clear mideleg CSR bit to use machine timer interrupt instead of the
   * default user mode interrupt. Default bits set:
   * 0: User software interrupt (CLINT)
   * 4: User timer interrupt (CLINT)
   * 8: User external interrupt
   */

  /* READ_AND_SET_CSR(CSR_MIDELEG, (0x1 << 7)); */

  /* Set mie CSR MTIE bit to enable machine timer interrupt */

  READ_AND_SET_CSR(CSR_MIE, MIE_MTIE);

  /* Set timer compare value that should trigger an interrupt */

  esp_clint_set_timer_compare(0, TIMER_COMPARE_VALUE_DEFAULT_L);
  esp_clint_set_timer_compare(1, TIMER_COMPARE_VALUE_DEFAULT_H);

  /* Enable IRQ */

  ret = irq_attach(ESP_IRQ_FROM_CPU_INTR0,
                   (xcpt_t)esp_clint_default_isr, NULL);
  if (ret < 0)
    {
      irqerr("Failed to attach irq: %d\n", ret);
      return;
    }

  /* Enable the allocated CPU interrupt */

  up_enable_irq(ESP_IRQ_FROM_CPU_INTR0);
}

/****************************************************************************
 * Name: esp_clint_reset_timer_counter
 *
 * Description:
 *   Reset the timer counter register to 0.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_clint_reset_timer_counter(void)
{
  putreg32(0x0, CLINT_MINT_MTIME_L_REG);
  putreg32(0x0, CLINT_MINT_MTIME_H_REG);
}

/****************************************************************************
 * Name: esp_clint_set_timer_compare
 *
 * Description:
 *   Set the timer comparator value for timer interrupt.
 *
 * Input Parameters:
 *   high  - If true, set value for high register bits [63:32]. Otherwise,
 *           set value for low register bits [31:0].
 *   value - Value to set.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_clint_set_timer_compare(bool high, uint32_t value)
{
  if (high)
    {
      putreg32(value, CLINT_MINT_MTIMECMP_H_REG);
    }
  else
    {
      putreg32(value, CLINT_MINT_MTIMECMP_L_REG);
    }
}

/****************************************************************************
 * Name: esp_clint_timer_counter_enable
 *
 * Description:
 *   Enable machine timer counter by setting the MTCE bit of the
 *   MTIMECTL register.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_clint_timer_counter_enable(void)
{
    uint32_t val;

    val = getreg32(CLINT_MINT_TIMECTL_REG);
    val |= CLINT_MINT_COUNTER_EN;
    putreg32(val, CLINT_MINT_TIMECTL_REG);
}

/****************************************************************************
 * Name: esp_clint_timer_counter_disable
 *
 * Description:
 *   Disable machine timer counter by clearing the MTCE bit of the
 *   MTIMECTL register.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_clint_timer_counter_disable(void)
{
    uint32_t val;

    val = getreg32(CLINT_MINT_TIMECTL_REG);
    val &= ~CLINT_MINT_COUNTER_EN;
    putreg32(val, CLINT_MINT_TIMECTL_REG);
}

/****************************************************************************
 * Name: esp_clint_timer_int_enable
 *
 * Description:
 *   Enable machine timer interrupt by setting the MTIE bit of the
 *   MTIMECTL register.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_clint_timer_int_enable(void)
{
    uint32_t val;

    val = getreg32(CLINT_MINT_TIMECTL_REG);
    val |= CLINT_MINT_TIMERINT_EN;
    putreg32(val, CLINT_MINT_TIMECTL_REG);
}

/****************************************************************************
 * Name: esp_clint_timer_int_disable
 *
 * Description:
 *   Disable machine timer interrupt by clearing the MTIE bit of the
 *   MTIMECTL register.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

IRAM_ATTR void esp_clint_timer_int_disable(void)
{
    uint32_t val;

    val = getreg32(CLINT_MINT_TIMECTL_REG);
    val &= ~CLINT_MINT_TIMERINT_EN;
    putreg32(val, CLINT_MINT_TIMECTL_REG);
}

/****************************************************************************
 * Name: get_clint_timer_value
 *
 * Description:
 *   Returns current timer counter register value.
 *   This is a 64 bit register that is read in two blocks of 32 bits each.
 *
 * Input Parameters:
 *   high - True to return the register with high values [63:32].
 *    False to return low values [31:0].
 *
 * Returned Value:
 *   Current register value.
 *
 ****************************************************************************/

IRAM_ATTR uint32_t get_clint_timer_value(bool high)
{
  if (high)
    {
      return getreg32(CLINT_MINT_MTIME_H_REG);
    }
  else
    {
      return getreg32(CLINT_MINT_MTIME_L_REG);
    }
}

void esp_clint_debug(void)
{
  uint32_t val;

  val = getreg32(CLINT_MINT_TIMECTL_REG);
  irqinfo("CLINT_MINT_TIMECTL_REG: 0x%x\n", val);
  val = getreg32(CLINT_MINT_MTIMECMP_H_REG);
  irqinfo("CLINT_MINT_MTIMECMP_H_REG: 0x%lx\n", val);
  val = getreg32(CLINT_MINT_MTIMECMP_L_REG);
  irqinfo("CLINT_MINT_MTIMECMP_L_REG: 0x%lx\n", val);
  val = READ_CSR(CSR_MIDELEG);
  irqinfo("CSR_MIDELEG: 0x%x\n", val);
  val = READ_CSR(CSR_MIE);
  irqinfo("CSR_MIE: 0x%x\n", val);
}
