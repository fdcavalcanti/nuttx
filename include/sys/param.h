/****************************************************************************
 * include/sys/param.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_SYS_PARAM_H
#define __INCLUDE_SYS_PARAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAXHOSTNAMELEN HOST_NAME_MAX

/* Macros for min/max. */

#ifndef MIN
#  define MIN(a,b)      (((a) < (b)) ? (a) : (b))
#endif  /* MIN */

#ifndef MAX
#  define MAX(a,b)      (((a) > (b)) ? (a) : (b))
#endif  /* MAX */

/* Macros for number of items.
 * (aka. ARRAY_SIZE, ArraySize, Size of an Array)
 */

#ifndef nitems
#  define nitems(_a)    (sizeof(_a) / sizeof(0[(_a)]))
#endif /* nitems */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_SYS_PARAM_H */
