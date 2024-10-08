/****************************************************************************
 * graphics/nxglib/fb/nxglib_getrectangle.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/video/fb.h>
#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_lowresmemcpy
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL < 8
static inline void nxgl_lowresmemcpy(FAR uint8_t *dline,
                                     FAR const uint8_t *sline,
                                     unsigned int width,
                                     uint8_t leadmask,
                                     uint8_t tailmask)
{
  FAR const uint8_t *sptr;
  FAR uint8_t *dptr;
  uint8_t mask;
  int lnlen;

  /* Handle masking of the fractional initial byte */

  mask  = leadmask;
  sptr  = sline;
  dptr  = dline;
  lnlen = width;

  if (lnlen > 1 && mask)
    {
      dptr[0] = (dptr[0] & ~mask) | (sptr[0] & mask);
      mask = 0xff;
      dptr++;
      sptr++;
      lnlen--;
    }

  /* Handle masking of the fractional final byte */

  mask &= tailmask;
  if (lnlen > 0 && mask)
    {
      dptr[lnlen - 1] = (dptr[lnlen - 1] & ~mask) | (sptr[lnlen - 1] & mask);
      lnlen--;
    }

  /* Handle all of the unmasked bytes in-between */

  if (lnlen > 0)
    {
      NXGL_MEMCPY(dptr, sptr, lnlen);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_getrectangle_*bpp
 *
 * Description:
 *   Fetch a rectangular region from framebuffer memory.  The source is
 *   expressed as a rectangle.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_getrectangle, NXGLIB_SUFFIX)
(
  FAR struct fb_planeinfo_s *pinfo,
  FAR const struct nxgl_rect_s *rect,
  FAR void *dest, unsigned int deststride)
{
  FAR const uint8_t *sline;
  FAR uint8_t *dline;
  unsigned int width;
  unsigned int fbstride;
  unsigned int rows;

#if NXGLIB_BITSPERPIXEL < 8
  uint8_t leadmask;
  uint8_t tailmask;
#endif

  /* Get the width of the framebuffer in bytes */

  fbstride = pinfo->stride;

  /* Get the dimensions of the rectangle to copy: width in pixels, height
   * in rows
   */

  width = rect->pt2.x - rect->pt1.x + 1;
  rows  = rect->pt2.y - rect->pt1.y + 1;

#if NXGLIB_BITSPERPIXEL < 8
#  ifdef CONFIG_NX_PACKEDMSFIRST

  /* Get the mask for pixels that are ordered so that they pack from the
   * MS byte down.
   */

  leadmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(rect->pt1.x)));
  tailmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(rect->pt2.x - 1)));
#  else
  /* Get the mask for pixels that are ordered so that they pack from the
   * LS byte up.
   */

  leadmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(rect->pt1.x)));
  tailmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(rect->pt1.x - 1)));
#  endif
#endif

  /* sline = address of the first pixel in the top row of the source in
   * framebuffer memory
   */

  sline = pinfo->fbmem + rect->pt1.y * fbstride + NXGL_SCALEX(rect->pt1.x);

  /* dline = address of the first row pixel */

  dline = (FAR uint8_t *)dest;

  /* Yes.. Copy the rectangle */

  while (rows--)
    {
      /* Copy the row */

#if NXGLIB_BITSPERPIXEL < 8
      nxgl_lowresmemcpy(dline, sline, width, leadmask, tailmask);
#else
      NXGL_MEMCPY(dline, sline, width);
#endif
      /* Point to the next source/dest row below the current one */

      dline += deststride;
      sline += fbstride;
    }
}
