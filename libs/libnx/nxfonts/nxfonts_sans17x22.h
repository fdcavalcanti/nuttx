/****************************************************************************
 * libs/libnx/nxfonts/nxfonts_sans17x22.h
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

#ifndef __LIBS_LIBNX_NXFONTS_NXFONTS_SANS17X22_H
#define __LIBS_LIBNX_NXFONTS_NXFONTS_SANS17X22_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Font ID */

#define NXFONT_ID         FONTID_SANS17X22

/* Ranges of 7-bit and 8-bit fonts */

#define NXFONT_MIN7BIT    33
#define NXFONT_MAX7BIT    126

#define NXFONT_MIN8BIT    161
#define NXFONT_MAX8BIT    255

/* Maximum height and width of any glyph in the set */

#define NXFONT_MAXHEIGHT  22
#define NXFONT_MAXWIDTH   17

/* The width of a space */

#define NXFONT_SPACEWIDTH 4

/* exclam (33) */
#define NXFONT_METRICS_33 {1, 1, 11, 2, 6, 0}
#define NXFONT_BITMAP_33 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0, 0x80, 0x80}

/* quotedbl (34) */
#define NXFONT_METRICS_34 {1, 3, 3, 1, 6, 0}
#define NXFONT_BITMAP_34 {0xa0, 0xa0, 0xa0}

/* numbersign (35) */
#define NXFONT_METRICS_35 {1, 7, 10, 0, 7, 0}
#define NXFONT_BITMAP_35 {0x14, 0x14, 0x14, 0x7e, 0x28, 0x28, 0xfc, 0x50, 0x50, 0x50}

/* dollar (36) */
#define NXFONT_METRICS_36 {1, 7, 14, 0, 5, 0}
#define NXFONT_BITMAP_36 {0x10, 0x7c, 0x92, 0x92, 0x90, 0x50, 0x38, 0x14, 0x12, 0x92, 0x92, 0x7c, 0x10, 0x10}

/* percent (37) */
#define NXFONT_METRICS_37 {2, 11, 11, 0, 6, 0}
#define NXFONT_BITMAP_37 {0x70, 0x80, 0x89, 0x0, 0x89, 0x0, 0x72, 0x0, 0x2, 0x0, 0x4, 0x0, 0x8, 0x0, 0x9, 0xc0, 0x12, 0x20, 0x12, 0x20, 0x21, 0xc0}

/* ampersand (38) */
#define NXFONT_METRICS_38 {1, 8, 10, 1, 7, 0}
#define NXFONT_BITMAP_38 {0x30, 0x48, 0x48, 0x30, 0x20, 0x52, 0x8a, 0x84, 0x8a, 0x71}

/* quotesingle (39) */
#define NXFONT_METRICS_39 {1, 1, 3, 1, 6, 0}
#define NXFONT_BITMAP_39 {0x80, 0x80, 0x80}

/* parenleft (40) */
#define NXFONT_METRICS_40 {1, 3, 14, 1, 6, 0}
#define NXFONT_BITMAP_40 {0x20, 0x40, 0x40, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x40, 0x40, 0x20}

/* parenright (41) */
#define NXFONT_METRICS_41 {1, 3, 14, 1, 6, 0}
#define NXFONT_BITMAP_41 {0x80, 0x40, 0x40, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x40, 0x40, 0x80}

/* asterisk (42) */
#define NXFONT_METRICS_42 {1, 5, 5, 1, 6, 0}
#define NXFONT_BITMAP_42 {0x20, 0xa8, 0x70, 0xa8, 0x20}

/* plus (43) */
#define NXFONT_METRICS_43 {1, 7, 7, 1, 9, 0}
#define NXFONT_BITMAP_43 {0x10, 0x10, 0x10, 0xfe, 0x10, 0x10, 0x10}

/* comma (44) */
#define NXFONT_METRICS_44 {1, 2, 4, 0, 15, 0}
#define NXFONT_BITMAP_44 {0x40, 0x40, 0x40, 0x80}

/* hyphen (45) */
#define NXFONT_METRICS_45 {1, 5, 1, 0, 12, 0}
#define NXFONT_BITMAP_45 {0x70}

/* period (46) */
#define NXFONT_METRICS_46 {1, 1, 2, 1, 15, 0}
#define NXFONT_BITMAP_46 {0x80, 0x80}

/* slash (47) */
#define NXFONT_METRICS_47 {1, 4, 11, 0, 6, 0}
#define NXFONT_BITMAP_47 {0x10, 0x10, 0x20, 0x20, 0x20, 0x40, 0x40, 0x40, 0x80, 0x80, 0x80}

/* zero (48) */
#define NXFONT_METRICS_48 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_48 {0x78, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x78}

/* one (49) */
#define NXFONT_METRICS_49 {1, 3, 11, 2, 6, 0}
#define NXFONT_BITMAP_49 {0x20, 0xe0, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20}

/* two (50) */
#define NXFONT_METRICS_50 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_50 {0x78, 0x84, 0x84, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x80, 0xfc}

/* three (51) */
#define NXFONT_METRICS_51 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_51 {0x78, 0x84, 0x84, 0x4, 0x4, 0x38, 0x4, 0x4, 0x84, 0x84, 0x78}

/* four (52) */
#define NXFONT_METRICS_52 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_52 {0x4, 0xc, 0x14, 0x24, 0x44, 0x84, 0x84, 0xfe, 0x4, 0x4, 0x4}

/* five (53) */
#define NXFONT_METRICS_53 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_53 {0xfc, 0x80, 0x80, 0x80, 0xf8, 0x4, 0x4, 0x4, 0x84, 0x84, 0x78}

/* six (54) */
#define NXFONT_METRICS_54 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_54 {0x78, 0x84, 0x80, 0x80, 0xb8, 0xc4, 0x84, 0x84, 0x84, 0x84, 0x78}

/* seven (55) */
#define NXFONT_METRICS_55 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_55 {0xfc, 0x4, 0x8, 0x8, 0x10, 0x10, 0x20, 0x20, 0x40, 0x40, 0x40}

/* eight (56) */
#define NXFONT_METRICS_56 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_56 {0x78, 0x84, 0x84, 0x84, 0x84, 0x78, 0x84, 0x84, 0x84, 0x84, 0x78}

/* nine (57) */
#define NXFONT_METRICS_57 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_57 {0x78, 0x84, 0x84, 0x84, 0x84, 0x7c, 0x4, 0x4, 0x84, 0x84, 0x78}

/* colon (58) */
#define NXFONT_METRICS_58 {1, 1, 8, 1, 9, 0}
#define NXFONT_BITMAP_58 {0x80, 0x80, 0x0, 0x0, 0x0, 0x0, 0x80, 0x80}

/* semicolon (59) */
#define NXFONT_METRICS_59 {1, 2, 10, 0, 9, 0}
#define NXFONT_BITMAP_59 {0x40, 0x40, 0x0, 0x0, 0x0, 0x0, 0x40, 0x40, 0x40, 0x80}

/* less (60) */
#define NXFONT_METRICS_60 {1, 6, 5, 1, 10, 0}
#define NXFONT_BITMAP_60 {0xc, 0x30, 0xc0, 0x30, 0xc}

/* equal (61) */
#define NXFONT_METRICS_61 {1, 6, 3, 1, 11, 0}
#define NXFONT_BITMAP_61 {0xfc, 0x0, 0xfc}

/* greater (62) */
#define NXFONT_METRICS_62 {1, 6, 5, 1, 10, 0}
#define NXFONT_BITMAP_62 {0xc0, 0x30, 0xc, 0x30, 0xc0}

/* question (63) */
#define NXFONT_METRICS_63 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_63 {0x30, 0xcc, 0x84, 0x84, 0x4, 0x8, 0x10, 0x20, 0x0, 0x20, 0x20}

/* at (64) */
#define NXFONT_METRICS_64 {2, 11, 12, 1, 6, 0}
#define NXFONT_BITMAP_64 {0xf, 0x0, 0x30, 0xc0, 0x40, 0x20, 0x46, 0xa0, 0x89, 0x20, 0x91, 0x20, 0x91, 0x20, 0x93, 0x40, 0x8d, 0x80, 0x40, 0x0, 0x60, 0x80, 0x1f, 0x0}

/* A (65) */
#define NXFONT_METRICS_65 {2, 9, 11, 0, 6, 0}
#define NXFONT_BITMAP_65 {0x8, 0x0, 0x1c, 0x0, 0x14, 0x0, 0x14, 0x0, 0x22, 0x0, 0x22, 0x0, 0x41, 0x0, 0x7f, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80}

/* B (66) */
#define NXFONT_METRICS_66 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_66 {0xfc, 0x86, 0x82, 0x82, 0x84, 0xf8, 0x84, 0x82, 0x82, 0x86, 0xfc}

/* C (67) */
#define NXFONT_METRICS_67 {1, 8, 11, 1, 6, 0}
#define NXFONT_BITMAP_67 {0x1c, 0x63, 0x41, 0x80, 0x80, 0x80, 0x80, 0x80, 0x41, 0x63, 0x1c}

/* D (68) */
#define NXFONT_METRICS_68 {1, 8, 11, 1, 6, 0}
#define NXFONT_BITMAP_68 {0xf8, 0x86, 0x82, 0x81, 0x81, 0x81, 0x81, 0x81, 0x82, 0x86, 0xf8}

/* E (69) */
#define NXFONT_METRICS_69 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_69 {0xfe, 0x80, 0x80, 0x80, 0x80, 0xfc, 0x80, 0x80, 0x80, 0x80, 0xfe}

/* F (70) */
#define NXFONT_METRICS_70 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_70 {0xfe, 0x80, 0x80, 0x80, 0x80, 0xfc, 0x80, 0x80, 0x80, 0x80, 0x80}

/* G (71) */
#define NXFONT_METRICS_71 {2, 9, 11, 1, 6, 0}
#define NXFONT_BITMAP_71 {0x1e, 0x0, 0x61, 0x80, 0x40, 0x80, 0x80, 0x0, 0x80, 0x0, 0x87, 0x80, 0x80, 0x80, 0x80, 0x80, 0x40, 0x80, 0x63, 0x80, 0x1c, 0x80}

/* H (72) */
#define NXFONT_METRICS_72 {1, 8, 11, 1, 6, 0}
#define NXFONT_BITMAP_72 {0x81, 0x81, 0x81, 0x81, 0x81, 0xff, 0x81, 0x81, 0x81, 0x81, 0x81}

/* I (73) */
#define NXFONT_METRICS_73 {1, 2, 11, 2, 6, 0}
#define NXFONT_BITMAP_73 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* J (74) */
#define NXFONT_METRICS_74 {1, 6, 11, 0, 6, 0}
#define NXFONT_BITMAP_74 {0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x84, 0x84, 0x78}

/* K (75) */
#define NXFONT_METRICS_75 {1, 8, 11, 1, 6, 0}
#define NXFONT_BITMAP_75 {0x82, 0x84, 0x88, 0x90, 0xa0, 0xe0, 0x90, 0x88, 0x84, 0x82, 0x81}

/* L (76) */
#define NXFONT_METRICS_76 {1, 6, 11, 2, 6, 0}
#define NXFONT_BITMAP_76 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xfc}

/* M (77) */
#define NXFONT_METRICS_77 {2, 11, 11, 0, 6, 0}
#define NXFONT_BITMAP_77 {0x80, 0x20, 0xc0, 0x60, 0xc0, 0x60, 0xa0, 0xa0, 0xa0, 0xa0, 0x91, 0x20, 0x91, 0x20, 0x8a, 0x20, 0x8a, 0x20, 0x84, 0x20, 0x84, 0x20}

/* N (78) */
#define NXFONT_METRICS_78 {1, 8, 11, 1, 6, 0}
#define NXFONT_BITMAP_78 {0xc1, 0xa1, 0xa1, 0x91, 0x91, 0x89, 0x89, 0x85, 0x85, 0x83, 0x83}

/* O (79) */
#define NXFONT_METRICS_79 {2, 9, 11, 1, 6, 0}
#define NXFONT_BITMAP_79 {0x1c, 0x0, 0x63, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x41, 0x0, 0x63, 0x0, 0x1c, 0x0}

/* P (80) */
#define NXFONT_METRICS_80 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_80 {0xfc, 0x86, 0x82, 0x82, 0x86, 0xfc, 0x80, 0x80, 0x80, 0x80, 0x80}

/* Q (81) */
#define NXFONT_METRICS_81 {2, 9, 11, 1, 6, 0}
#define NXFONT_BITMAP_81 {0x1c, 0x0, 0x63, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x88, 0x80, 0x84, 0x80, 0x43, 0x0, 0x63, 0x0, 0x1c, 0x80}

/* R (82) */
#define NXFONT_METRICS_82 {1, 8, 11, 1, 6, 0}
#define NXFONT_BITMAP_82 {0xfe, 0x83, 0x81, 0x81, 0x82, 0xfc, 0x82, 0x81, 0x81, 0x81, 0x81}

/* S (83) */
#define NXFONT_METRICS_83 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_83 {0x38, 0xc6, 0x82, 0x80, 0x60, 0x18, 0x6, 0x2, 0x82, 0xc6, 0x38}

/* T (84) */
#define NXFONT_METRICS_84 {2, 9, 11, 0, 6, 0}
#define NXFONT_BITMAP_84 {0xff, 0x80, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0}

/* U (85) */
#define NXFONT_METRICS_85 {1, 8, 11, 1, 6, 0}
#define NXFONT_BITMAP_85 {0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3c}

/* V (86) */
#define NXFONT_METRICS_86 {2, 9, 11, 0, 6, 0}
#define NXFONT_BITMAP_86 {0x80, 0x80, 0x80, 0x80, 0x41, 0x0, 0x41, 0x0, 0x63, 0x0, 0x22, 0x0, 0x22, 0x0, 0x14, 0x0, 0x14, 0x0, 0x8, 0x0, 0x8, 0x0}

/* W (87) */
#define NXFONT_METRICS_87 {2, 13, 11, 0, 6, 0}
#define NXFONT_BITMAP_87 {0x82, 0x8, 0x82, 0x8, 0x85, 0x8, 0x45, 0x10, 0x45, 0x10, 0x45, 0x10, 0x28, 0xa0, 0x28, 0xa0, 0x28, 0xa0, 0x10, 0x40, 0x10, 0x40}

/* X (88) */
#define NXFONT_METRICS_88 {2, 9, 11, 0, 6, 0}
#define NXFONT_BITMAP_88 {0x80, 0x80, 0x41, 0x0, 0x22, 0x0, 0x14, 0x0, 0x8, 0x0, 0x8, 0x0, 0x14, 0x0, 0x22, 0x0, 0x41, 0x0, 0x41, 0x0, 0x80, 0x80}

/* Y (89) */
#define NXFONT_METRICS_89 {2, 9, 11, 0, 6, 0}
#define NXFONT_BITMAP_89 {0x80, 0x80, 0xc1, 0x80, 0x41, 0x0, 0x22, 0x0, 0x22, 0x0, 0x14, 0x0, 0x1c, 0x0, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0}

/* Z (90) */
#define NXFONT_METRICS_90 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_90 {0xfe, 0x2, 0x4, 0x8, 0x18, 0x10, 0x20, 0x60, 0x40, 0x80, 0xfe}

/* bracketleft (91) */
#define NXFONT_METRICS_91 {1, 3, 14, 1, 6, 0}
#define NXFONT_BITMAP_91 {0xe0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xe0}

/* backslash (92) */
#define NXFONT_METRICS_92 {1, 4, 11, 0, 6, 0}
#define NXFONT_BITMAP_92 {0x80, 0x80, 0x40, 0x40, 0x40, 0x20, 0x20, 0x20, 0x10, 0x10, 0x10}

/* bracketright (93) */
#define NXFONT_METRICS_93 {1, 3, 14, 0, 6, 0}
#define NXFONT_BITMAP_93 {0xe0, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0xe0}

/* asciicircum (94) */
#define NXFONT_METRICS_94 {1, 5, 5, 1, 6, 0}
#define NXFONT_BITMAP_94 {0x20, 0x50, 0x50, 0x88, 0x88}

/* underscore (95) */
#define NXFONT_METRICS_95 {1, 8, 1, 0, 19, 0}
#define NXFONT_BITMAP_95 {0xff}

/* grave (96) */
#define NXFONT_METRICS_96 {1, 2, 2, 1, 6, 0}
#define NXFONT_BITMAP_96 {0x80, 0x40}

/* a (97) */
#define NXFONT_METRICS_97 {1, 7, 8, 1, 9, 0}
#define NXFONT_BITMAP_97 {0x78, 0xcc, 0x4, 0x7c, 0xc4, 0x84, 0xcc, 0x76}

/* b (98) */
#define NXFONT_METRICS_98 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_98 {0x80, 0x80, 0x80, 0xb8, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0xb8}

/* c (99) */
#define NXFONT_METRICS_99 {1, 6, 8, 1, 9, 0}
#define NXFONT_BITMAP_99 {0x78, 0xcc, 0x80, 0x80, 0x80, 0x84, 0xcc, 0x78}

/* d (100) */
#define NXFONT_METRICS_100 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_100 {0x4, 0x4, 0x4, 0x74, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x74}

/* e (101) */
#define NXFONT_METRICS_101 {1, 6, 8, 1, 9, 0}
#define NXFONT_BITMAP_101 {0x78, 0xcc, 0x84, 0xfc, 0x80, 0x80, 0xcc, 0x78}

/* f (102) */
#define NXFONT_METRICS_102 {1, 4, 11, 0, 6, 0}
#define NXFONT_BITMAP_102 {0x30, 0x40, 0x40, 0xe0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}

/* g (103) */
#define NXFONT_METRICS_103 {1, 6, 11, 1, 9, 0}
#define NXFONT_BITMAP_103 {0x74, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x74, 0x4, 0xcc, 0x78}

/* h (104) */
#define NXFONT_METRICS_104 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_104 {0x80, 0x80, 0x80, 0xb8, 0xcc, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84}

/* i (105) */
#define NXFONT_METRICS_105 {1, 1, 11, 1, 6, 0}
#define NXFONT_BITMAP_105 {0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* j (106) -- NOTE: Xoffset should be -1, not 0. */
#define NXFONT_METRICS_106 {1, 3, 14, 0, 6, 0}
#define NXFONT_BITMAP_106 {0x20, 0x20, 0x0, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0xc0}

/* k (107) */
#define NXFONT_METRICS_107 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_107 {0x80, 0x80, 0x80, 0x88, 0x90, 0xa0, 0xc0, 0xa0, 0x90, 0x88, 0x84}

/* l (108) */
#define NXFONT_METRICS_108 {1, 1, 11, 1, 6, 0}
#define NXFONT_BITMAP_108 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* m (109) */
#define NXFONT_METRICS_109 {2, 9, 8, 1, 9, 0}
#define NXFONT_BITMAP_109 {0xb3, 0x0, 0xcc, 0x80, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x88, 0x80}

/* n (110) */
#define NXFONT_METRICS_110 {1, 6, 8, 1, 9, 0}
#define NXFONT_BITMAP_110 {0xb8, 0xcc, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84}

/* o (111) */
#define NXFONT_METRICS_111 {1, 6, 8, 1, 9, 0}
#define NXFONT_BITMAP_111 {0x78, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x78}

/* p (112) */
#define NXFONT_METRICS_112 {1, 6, 11, 1, 9, 0}
#define NXFONT_BITMAP_112 {0xb8, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0xb8, 0x80, 0x80, 0x80}

/* q (113) */
#define NXFONT_METRICS_113 {1, 6, 11, 1, 9, 0}
#define NXFONT_BITMAP_113 {0x74, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x74, 0x4, 0x4, 0x4}

/* r (114) */
#define NXFONT_METRICS_114 {1, 4, 8, 1, 9, 0}
#define NXFONT_BITMAP_114 {0xb0, 0xc0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* s (115) */
#define NXFONT_METRICS_115 {1, 5, 8, 1, 9, 0}
#define NXFONT_BITMAP_115 {0x70, 0x88, 0xc0, 0x70, 0x18, 0x8, 0x88, 0x70}

/* t (116) */
#define NXFONT_METRICS_116 {1, 4, 10, 0, 7, 0}
#define NXFONT_BITMAP_116 {0x40, 0x40, 0xe0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x30}

/* u (117) */
#define NXFONT_METRICS_117 {1, 6, 8, 1, 9, 0}
#define NXFONT_BITMAP_117 {0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x74}

/* v (118) */
#define NXFONT_METRICS_118 {1, 7, 8, 0, 9, 0}
#define NXFONT_BITMAP_118 {0x82, 0x82, 0x44, 0x44, 0x44, 0x28, 0x28, 0x10}

/* w (119) */
#define NXFONT_METRICS_119 {2, 9, 8, 0, 9, 0}
#define NXFONT_BITMAP_119 {0x88, 0x80, 0x88, 0x80, 0x88, 0x80, 0x49, 0x0, 0x49, 0x0, 0x55, 0x0, 0x22, 0x0, 0x22, 0x0}

/* x (120) */
#define NXFONT_METRICS_120 {1, 7, 8, 0, 9, 0}
#define NXFONT_BITMAP_120 {0xc6, 0x44, 0x28, 0x10, 0x10, 0x28, 0x44, 0xc6}

/* y (121) */
#define NXFONT_METRICS_121 {1, 7, 11, 0, 9, 0}
#define NXFONT_BITMAP_121 {0x82, 0xc2, 0x44, 0x44, 0x24, 0x28, 0x18, 0x10, 0x10, 0x30, 0x60}

/* z (122) */
#define NXFONT_METRICS_122 {1, 6, 8, 0, 9, 0}
#define NXFONT_BITMAP_122 {0xfc, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0xfc}

/* braceleft (123) */
#define NXFONT_METRICS_123 {1, 5, 14, 0, 6, 0}
#define NXFONT_BITMAP_123 {0x18, 0x20, 0x20, 0x20, 0x20, 0x40, 0x80, 0x40, 0x20, 0x20, 0x20, 0x20, 0x20, 0x18}

/* bar (124) */
#define NXFONT_METRICS_124 {1, 1, 14, 1, 6, 0}
#define NXFONT_BITMAP_124 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* braceright (125) */
#define NXFONT_METRICS_125 {1, 5, 14, 0, 6, 0}
#define NXFONT_BITMAP_125 {0xc0, 0x20, 0x20, 0x20, 0x20, 0x10, 0x8, 0x10, 0x20, 0x20, 0x20, 0x20, 0x20, 0xc0}

/* asciitilde (126) */
#define NXFONT_METRICS_126 {1, 6, 3, 1, 11, 0}
#define NXFONT_BITMAP_126 {0x64, 0xb4, 0x98}

/* exclamdown (161) */
#define NXFONT_METRICS_161 {1, 1, 11, 1, 9, 0}
#define NXFONT_BITMAP_161 {0x80, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* cent (162) */
#define NXFONT_METRICS_162 {1, 6, 10, 1, 8, 0}
#define NXFONT_BITMAP_162 {0x4, 0x78, 0xcc, 0x90, 0x90, 0xa0, 0xa4, 0xcc, 0x78, 0x80}

/* sterling (163) */
#define NXFONT_METRICS_163 {1, 7, 11, 0, 6, 0}
#define NXFONT_BITMAP_163 {0x38, 0x44, 0x40, 0x40, 0xf8, 0x20, 0x20, 0x20, 0x40, 0x62, 0xdc}

/* currency (164) */
#define NXFONT_METRICS_164 {1, 6, 6, 1, 9, 0}
#define NXFONT_BITMAP_164 {0x84, 0x78, 0x48, 0x48, 0x78, 0x84}

/* yen (165) */
#define NXFONT_METRICS_165 {1, 7, 11, 0, 6, 0}
#define NXFONT_BITMAP_165 {0x82, 0x82, 0x82, 0x44, 0x44, 0x28, 0xfe, 0x10, 0xfe, 0x10, 0x10}

/* brokenbar (166) */
#define NXFONT_METRICS_166 {1, 1, 14, 1, 6, 0}
#define NXFONT_BITMAP_166 {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* section (167) */
#define NXFONT_METRICS_167 {1, 5, 14, 2, 6, 0}
#define NXFONT_BITMAP_167 {0x70, 0xd8, 0x88, 0xc0, 0x70, 0x98, 0x88, 0x88, 0xc8, 0x70, 0x18, 0x88, 0xd8, 0x70}

/* dieresis (168) */
#define NXFONT_METRICS_168 {1, 5, 1, 0, 7, 0}
#define NXFONT_BITMAP_168 {0xd8}

/* copyright (169) */
#define NXFONT_METRICS_169 {2, 10, 11, 1, 6, 0}
#define NXFONT_BITMAP_169 {0x1e, 0x0, 0x61, 0x80, 0x5c, 0x80, 0xa2, 0xc0, 0xa2, 0x40, 0xa0, 0x40, 0xa2, 0x40, 0x9c, 0x40, 0x40, 0x80, 0x61, 0x80, 0x1e, 0x0}

/* ordfeminine (170) */
#define NXFONT_METRICS_170 {1, 4, 7, 1, 6, 0}
#define NXFONT_BITMAP_170 {0xe0, 0x10, 0x70, 0x90, 0xd0, 0x0, 0xf0}

/* guillemotleft (171) */
#define NXFONT_METRICS_171 {1, 6, 5, 1, 10, 0}
#define NXFONT_BITMAP_171 {0x24, 0x48, 0x90, 0x48, 0x24}

/* logicalnot (172) */
#define NXFONT_METRICS_172 {1, 7, 4, 1, 11, 0}
#define NXFONT_BITMAP_172 {0xfe, 0x2, 0x2, 0x2}

/* hyphen (173) */
#define NXFONT_METRICS_173 {1, 3, 1, 0, 12, 0}
#define NXFONT_BITMAP_173 {0xe0}

/* registered (174) */
#define NXFONT_METRICS_174 {2, 10, 11, 1, 6, 0}
#define NXFONT_BITMAP_174 {0x1e, 0x0, 0x61, 0x80, 0x5c, 0x80, 0x92, 0x40, 0x92, 0x40, 0x9c, 0x40, 0x92, 0x40, 0x92, 0x40, 0x40, 0x80, 0x61, 0x80, 0x1e, 0x0}

/* macron (175) */
#define NXFONT_METRICS_175 {1, 4, 1, 0, 7, 0}
#define NXFONT_BITMAP_175 {0xf0}

/* degree (176) */
#define NXFONT_METRICS_176 {1, 4, 4, 1, 6, 0}
#define NXFONT_BITMAP_176 {0x60, 0x90, 0x90, 0x60}

/* plusminus (177) */
#define NXFONT_METRICS_177 {1, 7, 9, 1, 8, 0}
#define NXFONT_BITMAP_177 {0x10, 0x10, 0x10, 0xfe, 0x10, 0x10, 0x10, 0x0, 0xfe}

/* twosuperior (178) */
#define NXFONT_METRICS_178 {1, 4, 6, 0, 6, 0}
#define NXFONT_BITMAP_178 {0x60, 0x90, 0x10, 0x20, 0x40, 0xf0}

/* threesuperior (179) */
#define NXFONT_METRICS_179 {1, 4, 6, 0, 6, 0}
#define NXFONT_BITMAP_179 {0x60, 0x90, 0x20, 0x10, 0x90, 0x60}

/* acute (180) */
#define NXFONT_METRICS_180 {1, 2, 2, 2, 6, 0}
#define NXFONT_BITMAP_180 {0x40, 0x80}

/* mu (181) */
#define NXFONT_METRICS_181 {1, 6, 11, 1, 9, 0}
#define NXFONT_BITMAP_181 {0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0xcc, 0xb4, 0x80, 0x80, 0x80}

/* paragraph (182) */
#define NXFONT_METRICS_182 {1, 7, 14, 0, 6, 0}
#define NXFONT_BITMAP_182 {0x3e, 0x74, 0xf4, 0xf4, 0xf4, 0x74, 0x34, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14}

/* periodcentered (183) */
#define NXFONT_METRICS_183 {1, 2, 1, 1, 12, 0}
#define NXFONT_BITMAP_183 {0xc0}

/* cedilla (184) */
#define NXFONT_METRICS_184 {1, 4, 3, 0, 17, 0}
#define NXFONT_BITMAP_184 {0x20, 0x90, 0x60}

/* onesuperior (185) */
#define NXFONT_METRICS_185 {1, 2, 6, 1, 6, 0}
#define NXFONT_BITMAP_185 {0x40, 0xc0, 0x40, 0x40, 0x40, 0x40}

/* ordmasculine (186) */
#define NXFONT_METRICS_186 {1, 4, 7, 1, 6, 0}
#define NXFONT_BITMAP_186 {0x60, 0x90, 0x90, 0x90, 0x60, 0x0, 0xf0}

/* guillemotright (187) */
#define NXFONT_METRICS_187 {1, 6, 5, 1, 10, 0}
#define NXFONT_BITMAP_187 {0x90, 0x48, 0x24, 0x48, 0x90}

/* onequarter (188) */
#define NXFONT_METRICS_188 {2, 10, 11, 1, 6, 0}
#define NXFONT_BITMAP_188 {0x42, 0x0, 0xc2, 0x0, 0x44, 0x0, 0x44, 0x0, 0x48, 0x0, 0x48, 0x80, 0x9, 0x80, 0x12, 0x80, 0x14, 0x80, 0x27, 0xc0, 0x20, 0x80}

/* onehalf (189) */
#define NXFONT_METRICS_189 {2, 9, 11, 1, 6, 0}
#define NXFONT_BITMAP_189 {0x42, 0x0, 0xc2, 0x0, 0x44, 0x0, 0x44, 0x0, 0x48, 0x0, 0x4b, 0x0, 0x14, 0x80, 0x10, 0x80, 0x11, 0x0, 0x22, 0x0, 0x27, 0x80}

/* threequarters (190) */
#define NXFONT_METRICS_190 {2, 11, 11, 0, 6, 0}
#define NXFONT_BITMAP_190 {0x61, 0x0, 0x91, 0x0, 0x22, 0x0, 0x12, 0x0, 0x94, 0x0, 0x64, 0x40, 0x4, 0xc0, 0x9, 0x40, 0xa, 0x40, 0x13, 0xe0, 0x10, 0x40}

/* questiondown (191) */
#define NXFONT_METRICS_191 {1, 6, 11, 1, 9, 0}
#define NXFONT_BITMAP_191 {0x10, 0x10, 0x0, 0x10, 0x20, 0x40, 0x80, 0x84, 0x84, 0xcc, 0x30}

/* Agrave (192) */
#define NXFONT_METRICS_192 {2, 9, 14, 0, 3, 0}
#define NXFONT_BITMAP_192 {0x10, 0x0, 0x8, 0x0, 0x0, 0x0, 0x8, 0x0, 0x1c, 0x0, 0x14, 0x0, 0x14, 0x0, 0x22, 0x0, 0x22, 0x0, 0x41, 0x0, 0x7f, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80}

/* Aacute (193) */
#define NXFONT_METRICS_193 {2, 9, 14, 0, 3, 0}
#define NXFONT_BITMAP_193 {0x4, 0x0, 0x8, 0x0, 0x0, 0x0, 0x8, 0x0, 0x1c, 0x0, 0x14, 0x0, 0x14, 0x0, 0x22, 0x0, 0x22, 0x0, 0x41, 0x0, 0x7f, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80}

/* Acircumflex (194) */
#define NXFONT_METRICS_194 {2, 9, 14, 0, 3, 0}
#define NXFONT_BITMAP_194 {0xc, 0x0, 0x12, 0x0, 0x0, 0x0, 0x8, 0x0, 0x1c, 0x0, 0x14, 0x0, 0x14, 0x0, 0x22, 0x0, 0x22, 0x0, 0x41, 0x0, 0x7f, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80}

/* Atilde (195) */
#define NXFONT_METRICS_195 {2, 9, 14, 0, 3, 0}
#define NXFONT_BITMAP_195 {0x1a, 0x0, 0x2c, 0x0, 0x0, 0x0, 0x8, 0x0, 0x1c, 0x0, 0x14, 0x0, 0x14, 0x0, 0x22, 0x0, 0x22, 0x0, 0x41, 0x0, 0x7f, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80}

/* Adieresis (196) */
#define NXFONT_METRICS_196 {2, 9, 13, 0, 4, 0}
#define NXFONT_BITMAP_196 {0x36, 0x0, 0x0, 0x0, 0x8, 0x0, 0x1c, 0x0, 0x14, 0x0, 0x14, 0x0, 0x22, 0x0, 0x22, 0x0, 0x41, 0x0, 0x7f, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80}

/* Aring (197) */
#define NXFONT_METRICS_197 {2, 9, 14, 0, 3, 0}
#define NXFONT_BITMAP_197 {0xc, 0x0, 0x12, 0x0, 0x12, 0x0, 0xc, 0x0, 0xc, 0x0, 0x14, 0x0, 0x14, 0x0, 0x22, 0x0, 0x22, 0x0, 0x41, 0x0, 0x7f, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80}

/* AE (198) */
#define NXFONT_METRICS_198 {2, 12, 11, 1, 6, 0}
#define NXFONT_BITMAP_198 {0x1f, 0xf0, 0x12, 0x0, 0x12, 0x0, 0x22, 0x0, 0x22, 0x0, 0x23, 0xf0, 0x7e, 0x0, 0x42, 0x0, 0x42, 0x0, 0x82, 0x0, 0x83, 0xf0}

/* Ccedilla (199) */
#define NXFONT_METRICS_199 {1, 8, 14, 1, 6, 0}
#define NXFONT_BITMAP_199 {0x1c, 0x63, 0x41, 0x80, 0x80, 0x80, 0x80, 0x80, 0x41, 0x63, 0x1c, 0x8, 0x24, 0x18}

/* Egrave (200) */
#define NXFONT_METRICS_200 {1, 7, 14, 1, 3, 0}
#define NXFONT_BITMAP_200 {0x20, 0x10, 0x0, 0xfe, 0x80, 0x80, 0x80, 0x80, 0xfc, 0x80, 0x80, 0x80, 0x80, 0xfe}

/* Eacute (201) */
#define NXFONT_METRICS_201 {1, 7, 14, 1, 3, 0}
#define NXFONT_BITMAP_201 {0x8, 0x10, 0x0, 0xfe, 0x80, 0x80, 0x80, 0x80, 0xfc, 0x80, 0x80, 0x80, 0x80, 0xfe}

/* Ecircumflex (202) */
#define NXFONT_METRICS_202 {1, 7, 14, 1, 3, 0}
#define NXFONT_BITMAP_202 {0x18, 0x24, 0x0, 0xfe, 0x80, 0x80, 0x80, 0x80, 0xfc, 0x80, 0x80, 0x80, 0x80, 0xfe}

/* Edieresis (203) */
#define NXFONT_METRICS_203 {1, 7, 13, 1, 4, 0}
#define NXFONT_BITMAP_203 {0x6c, 0x0, 0xfe, 0x80, 0x80, 0x80, 0x80, 0xfc, 0x80, 0x80, 0x80, 0x80, 0xfe}

/* Igrave (204) */
#define NXFONT_METRICS_204 {1, 2, 14, 1, 3, 0}
#define NXFONT_BITMAP_204 {0x80, 0x40, 0x0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}

/* Iacute (205) */
#define NXFONT_METRICS_205 {1, 2, 14, 2, 3, 0}
#define NXFONT_BITMAP_205 {0x40, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* Icircumflex (206) */
#define NXFONT_METRICS_206 {1, 4, 14, 1, 3, 0}
#define NXFONT_BITMAP_206 {0x60, 0x90, 0x0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}

/* Idieresis (207) */
#define NXFONT_METRICS_207 {1, 5, 13, 0, 4, 0}
#define NXFONT_BITMAP_207 {0xd8, 0x0, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20}

/* Eth (208) */
#define NXFONT_METRICS_208 {2, 9, 11, 0, 6, 0}
#define NXFONT_BITMAP_208 {0x7c, 0x0, 0x43, 0x0, 0x41, 0x0, 0x40, 0x80, 0x40, 0x80, 0xf0, 0x80, 0x40, 0x80, 0x40, 0x80, 0x41, 0x0, 0x43, 0x0, 0x7c, 0x0}

/* Ntilde (209) */
#define NXFONT_METRICS_209 {1, 8, 14, 1, 3, 0}
#define NXFONT_BITMAP_209 {0x1a, 0x2c, 0x0, 0xc1, 0xa1, 0xa1, 0x91, 0x91, 0x89, 0x89, 0x85, 0x85, 0x83, 0x83}

/* Ograve (210) */
#define NXFONT_METRICS_210 {2, 9, 14, 1, 3, 0}
#define NXFONT_BITMAP_210 {0x10, 0x0, 0x8, 0x0, 0x0, 0x0, 0x1c, 0x0, 0x63, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x41, 0x0, 0x63, 0x0, 0x1c, 0x0}

/* Oacute (211) */
#define NXFONT_METRICS_211 {2, 9, 14, 1, 3, 0}
#define NXFONT_BITMAP_211 {0x4, 0x0, 0x8, 0x0, 0x0, 0x0, 0x1c, 0x0, 0x63, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x41, 0x0, 0x63, 0x0, 0x1c, 0x0}

/* Ocircumflex (212) */
#define NXFONT_METRICS_212 {2, 9, 14, 1, 3, 0}
#define NXFONT_BITMAP_212 {0xc, 0x0, 0x12, 0x0, 0x0, 0x0, 0x1c, 0x0, 0x63, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x41, 0x0, 0x63, 0x0, 0x1c, 0x0}

/* Otilde (213) */
#define NXFONT_METRICS_213 {2, 9, 14, 1, 3, 0}
#define NXFONT_BITMAP_213 {0x1a, 0x0, 0x2c, 0x0, 0x0, 0x0, 0x1c, 0x0, 0x63, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x41, 0x0, 0x63, 0x0, 0x1c, 0x0}

/* Odieresis (214) */
#define NXFONT_METRICS_214 {2, 9, 13, 1, 4, 0}
#define NXFONT_BITMAP_214 {0x33, 0x0, 0x0, 0x0, 0x1c, 0x0, 0x63, 0x0, 0x41, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x41, 0x0, 0x63, 0x0, 0x1c, 0x0}

/* multiply (215) */
#define NXFONT_METRICS_215 {1, 7, 7, 1, 9, 0}
#define NXFONT_BITMAP_215 {0x82, 0x44, 0x28, 0x10, 0x28, 0x44, 0x82}

/* Oslash (216) */
#define NXFONT_METRICS_216 {2, 11, 11, 0, 6, 0}
#define NXFONT_BITMAP_216 {0xe, 0x20, 0x31, 0xc0, 0x20, 0x80, 0x41, 0x40, 0x42, 0x40, 0x44, 0x40, 0x48, 0x40, 0x50, 0x40, 0x20, 0x80, 0x71, 0x80, 0x8e, 0x0}

/* Ugrave (217) */
#define NXFONT_METRICS_217 {1, 8, 14, 1, 3, 0}
#define NXFONT_BITMAP_217 {0x10, 0x8, 0x0, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3c}

/* Uacute (218) */
#define NXFONT_METRICS_218 {1, 8, 14, 1, 3, 0}
#define NXFONT_BITMAP_218 {0x4, 0x8, 0x0, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3c}

/* Ucircumflex (219) */
#define NXFONT_METRICS_219 {1, 8, 14, 1, 3, 0}
#define NXFONT_BITMAP_219 {0x18, 0x24, 0x0, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3c}

/* Udieresis (220) */
#define NXFONT_METRICS_220 {1, 8, 13, 1, 4, 0}
#define NXFONT_BITMAP_220 {0x66, 0x0, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3c}

/* Yacute (221) */
#define NXFONT_METRICS_221 {2, 9, 14, 0, 3, 0}
#define NXFONT_BITMAP_221 {0x4, 0x0, 0x8, 0x0, 0x0, 0x0, 0x80, 0x80, 0xc1, 0x80, 0x41, 0x0, 0x22, 0x0, 0x22, 0x0, 0x14, 0x0, 0x1c, 0x0, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0, 0x8, 0x0}

/* Thorn (222) */
#define NXFONT_METRICS_222 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_222 {0x80, 0x80, 0xfc, 0x86, 0x82, 0x82, 0x86, 0xfc, 0x80, 0x80, 0x80}

/* germandbls (223) */
#define NXFONT_METRICS_223 {1, 5, 11, 1, 6, 0}
#define NXFONT_BITMAP_223 {0x70, 0x88, 0x88, 0x88, 0xb0, 0x90, 0x88, 0x88, 0x88, 0x88, 0xb0}

/* agrave (224) */
#define NXFONT_METRICS_224 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_224 {0x20, 0x10, 0x0, 0x78, 0xcc, 0x4, 0x7c, 0xc4, 0x84, 0xcc, 0x76}

/* aacute (225) */
#define NXFONT_METRICS_225 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_225 {0x10, 0x20, 0x0, 0x78, 0xcc, 0x4, 0x7c, 0xc4, 0x84, 0xcc, 0x76}

/* acircumflex (226) */
#define NXFONT_METRICS_226 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_226 {0x30, 0x48, 0x0, 0x78, 0xcc, 0x4, 0x7c, 0xc4, 0x84, 0xcc, 0x76}

/* atilde (227) */
#define NXFONT_METRICS_227 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_227 {0x34, 0x58, 0x0, 0x78, 0xcc, 0x4, 0x7c, 0xc4, 0x84, 0xcc, 0x76}

/* adieresis (228) */
#define NXFONT_METRICS_228 {1, 7, 11, 1, 6, 0}
#define NXFONT_BITMAP_228 {0x48, 0x48, 0x0, 0x78, 0xcc, 0x4, 0x7c, 0xc4, 0x84, 0xcc, 0x76}

/* aring (229) */
#define NXFONT_METRICS_229 {1, 7, 12, 1, 5, 0}
#define NXFONT_BITMAP_229 {0x30, 0x48, 0x30, 0x0, 0x78, 0xcc, 0x4, 0x7c, 0xc4, 0x84, 0xcc, 0x76}

/* ae (230) */
#define NXFONT_METRICS_230 {2, 11, 8, 1, 9, 0}
#define NXFONT_BITMAP_230 {0x7b, 0xc0, 0xc6, 0x60, 0x4, 0x20, 0x7f, 0xe0, 0xc4, 0x0, 0x84, 0x0, 0xce, 0x60, 0x7b, 0xc0}

/* ccedilla (231) */
#define NXFONT_METRICS_231 {1, 6, 11, 1, 9, 0}
#define NXFONT_BITMAP_231 {0x78, 0xcc, 0x80, 0x80, 0x80, 0x84, 0xcc, 0x78, 0x10, 0x48, 0x30}

/* egrave (232) */
#define NXFONT_METRICS_232 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_232 {0x20, 0x10, 0x0, 0x78, 0xcc, 0x84, 0xfc, 0x80, 0x80, 0xcc, 0x78}

/* eacute (233) */
#define NXFONT_METRICS_233 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_233 {0x10, 0x20, 0x0, 0x78, 0xcc, 0x84, 0xfc, 0x80, 0x80, 0xcc, 0x78}

/* ecircumflex (234) */
#define NXFONT_METRICS_234 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_234 {0x30, 0x48, 0x0, 0x78, 0xcc, 0x84, 0xfc, 0x80, 0x80, 0xcc, 0x78}

/* edieresis (235) */
#define NXFONT_METRICS_235 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_235 {0x48, 0x48, 0x0, 0x78, 0xcc, 0x84, 0xfc, 0x80, 0x80, 0xcc, 0x78}

/* igrave (236) */
#define NXFONT_METRICS_236 {1, 2, 11, 1, 6, 0}
#define NXFONT_BITMAP_236 {0x80, 0x40, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* iacute (237) */
#define NXFONT_METRICS_237 {1, 2, 11, 1, 6, 0}
#define NXFONT_BITMAP_237 {0x40, 0x80, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}

/* icircumflex (238) */
#define NXFONT_METRICS_238 {1, 4, 11, 0, 6, 0}
#define NXFONT_BITMAP_238 {0x60, 0x90, 0x0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}

/* idieresis (239) */
#define NXFONT_METRICS_239 {1, 3, 11, 0, 6, 0}
#define NXFONT_BITMAP_239 {0xa0, 0xa0, 0x0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}

/* eth (240) */
#define NXFONT_METRICS_240 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_240 {0xd8, 0x70, 0x90, 0x78, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x78}

/* ntilde (241) */
#define NXFONT_METRICS_241 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_241 {0x68, 0xb0, 0x0, 0xb8, 0xcc, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84}

/* ograve (242) */
#define NXFONT_METRICS_242 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_242 {0x20, 0x10, 0x0, 0x78, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x78}

/* oacute (243) */
#define NXFONT_METRICS_243 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_243 {0x10, 0x20, 0x0, 0x78, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x78}

/* ocircumflex (244) */
#define NXFONT_METRICS_244 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_244 {0x30, 0x48, 0x0, 0x78, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x78}

/* otilde (245) */
#define NXFONT_METRICS_245 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_245 {0x68, 0xb0, 0x0, 0x78, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x78}

/* odieresis (246) */
#define NXFONT_METRICS_246 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_246 {0x48, 0x48, 0x0, 0x78, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x78}

/* divide (247) */
#define NXFONT_METRICS_247 {1, 7, 7, 1, 9, 0}
#define NXFONT_BITMAP_247 {0x10, 0x10, 0x0, 0xfe, 0x0, 0x10, 0x10}

/* oslash (248) */
#define NXFONT_METRICS_248 {1, 8, 8, 0, 9, 0}
#define NXFONT_BITMAP_248 {0x3d, 0x62, 0x46, 0x4a, 0x52, 0x62, 0x46, 0xbc}

/* ugrave (249) */
#define NXFONT_METRICS_249 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_249 {0x20, 0x10, 0x0, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x74}

/* uacute (250) */
#define NXFONT_METRICS_250 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_250 {0x10, 0x20, 0x0, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x74}

/* ucircumflex (251) */
#define NXFONT_METRICS_251 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_251 {0x30, 0x48, 0x0, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x74}

/* udieresis (252) */
#define NXFONT_METRICS_252 {1, 6, 11, 1, 6, 0}
#define NXFONT_BITMAP_252 {0x48, 0x48, 0x0, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x74}

/* yacute (253) */
#define NXFONT_METRICS_253 {1, 7, 14, 0, 6, 0}
#define NXFONT_BITMAP_253 {0x8, 0x10, 0x0, 0x82, 0xc2, 0x44, 0x44, 0x24, 0x28, 0x18, 0x10, 0x10, 0x30, 0x60}

/* thorn (254) */
#define NXFONT_METRICS_254 {1, 6, 14, 1, 6, 0}
#define NXFONT_BITMAP_254 {0x80, 0x80, 0x80, 0xb8, 0xcc, 0x84, 0x84, 0x84, 0x84, 0xcc, 0xb8, 0x80, 0x80, 0x80}

/* ydieresis (255) */
#define NXFONT_METRICS_255 {1, 7, 14, 0, 6, 0}
#define NXFONT_BITMAP_255 {0x24, 0x24, 0x0, 0x82, 0xc2, 0x44, 0x44, 0x24, 0x28, 0x18, 0x10, 0x10, 0x30, 0x60}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __LIBS_LIBNX_NXFONTS_NXFONTS_SANS17X22_H */
