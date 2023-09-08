/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_mpu6050.c
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

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sensors/mpu60x0.h>
#include <nuttx/i2c/i2c_master.h>

#include "esp32_board_i2c.h"
#include "esp32_i2c.h"
#include "esp32_bmp280.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_mpu6050_initialize
 *
 * Description:
 *   Initialize and register the MPU6050 Inertial Measurement Unit driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/imuN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_mpu6050_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  struct mpu_config_s *mpu_config;
  int ret;

  sninfo("Initializing MPU6050!\n");

  /* Initialize MPU6050 */

  i2c = esp32_i2cbus_initialize(busno);

  mpu_config = kmm_zalloc(sizeof(struct mpu_config_s));
  if (mpu_config == NULL)
    {
      snerr("ERROR: Failed to allocate mpu6050 driver\n");
      return -ENOMEM;
    }

  if (i2c)
    {
      /* Try to register the IMU in I2C0 */
      char device[12];
      mpu_config->i2c = i2c;
      mpu_config->addr = 0x68;

      snprintf(device, sizeof(device), "/dev/imu%d", device, devno);
      ret = mpu60x0_register(device, mpu_config);
      if (ret < 0)
        {
          snerr("ERROR: Error registering MPU6050 in I2C%d\n", busno);
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}
