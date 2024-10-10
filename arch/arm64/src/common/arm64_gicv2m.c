/****************************************************************************
 * arch/arm64/src/common/arm64_gicv2m.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/bits.h>
#include <nuttx/spinlock.h>

#include "arm64_internal.h"
#include "arm64_gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPCORE_ICM_VBASE                CONFIG_GICM_BASE

/* V2m Registers */

#define GIC_V2MTYPER_OFFSET             0x008
#define GIC_V2MSETSPI_OFFSET            0x040
#define GIC_V2MTYPER                    (MPCORE_ICM_VBASE + GIC_V2MTYPER_OFFSET)
#define GIC_V2MSETSPI                   (MPCORE_ICM_VBASE + GIC_V2MSETSPI_OFFSET)

#define GIC_V2MTYPES_BASE_SHIFT         16
#define GIC_V2MTYPES_BASE_MASK          0x3FF
#define GIC_V2MTYPES_NUM_MASK           0x3FF
#define GIC_V2MTYPES_BASE(x)            \
  (((x) >> GIC_V2MTYPES_BASE_SHIFT) & GIC_V2MTYPES_NUM_MASK)
#define GIC_V2MTYPES_NUMBER(x)          ((x) & GIC_V2MTYPES_NUM_MASK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gic_v2m_s
{
  spinlock_t lock;
  uint32_t spi_start;
  uint32_t spi_number;
  unsigned long *spi_bitmap;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gic_v2m_s g_v2m =
{
  SP_UNLOCKED
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool is_valid_spi(uint32_t base, uint32_t num)
{
  if (base < GIC_SPI_INT_BASE)
    {
      return false;
    }

  if ((num == 0) || (base + num > NR_IRQS))
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int arm64_gic_v2m_initialize(void)
{
  uint32_t typer;

  typer = getreg32(GIC_V2MTYPER);
  g_v2m.spi_start = GIC_V2MTYPES_BASE(typer);
  g_v2m.spi_number = GIC_V2MTYPES_NUMBER(typer);

  if (!is_valid_spi(g_v2m.spi_start, g_v2m.spi_number))
    {
      return -EINVAL;
    }

  g_v2m.spi_bitmap = kmm_zalloc(BITS_TO_LONGS(g_v2m.spi_number));
  if (g_v2m.spi_bitmap == NULL)
    {
      return -ENOMEM;
    }

  return 0;
}

int up_alloc_irq_msi(uint8_t busno, uint32_t devfn, int *pirq, int num)
{
  irqstate_t flags;
  int offset;
  int irq;
  int i;

  flags = spin_lock_irqsave(&g_v2m.lock);
  offset = bitmap_find_free_region(g_v2m.spi_bitmap, g_v2m.spi_number, num);
  spin_unlock_irqrestore(&g_v2m.lock, flags);
  irq = g_v2m.spi_start + offset;
  for (i = 0; i < num; i++)
    {
      arm64_gicv_irq_trigger(i + irq, true);
      pirq[i] = irq + i;
    }

  return num;
}

void up_release_irq_msi(int *irq, int num)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_v2m.lock);
  bitmap_release_region(g_v2m.spi_bitmap, *irq - g_v2m.spi_start, num);
  spin_unlock_irqrestore(&g_v2m.lock, flags);
}

int up_connect_irq(const int *irq, int num,
                   uintptr_t *mar, uint32_t *mdr)
{
  *mar = GIC_V2MSETSPI;
  *mdr = *irq;
  return 0;
}