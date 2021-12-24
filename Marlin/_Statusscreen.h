/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

// #define CONFIG_EXAMPLES_DIR "Creality/Ender-3/BigTreeTech SKR Mini E3 3.0"

/**
 * Custom Status Screen bitmap
 *
 * Place this file in the root with your configuration files
 * and enable CUSTOM_STATUS_SCREEN_IMAGE in Configuration.h.
 *
 * Use the Marlin Bitmap Converter to make your own:
 * https://marlinfw.org/tools/u8glib/converter.html
 */

//
// Status Screen Logo bitmap
//
#define STATUS_LOGO_Y            8
#define STATUS_LOGO_WIDTH       39

const unsigned char status_logo_bmp[] PROGMEM = {
  B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000000,B01000000,B00000001,B11111110,
  B01111100,B00000000,B01000000,B00000001,B11111110,
  B01000000,B00000000,B01000000,B00000000,B00001100,
  B01000001,B01100011,B11001110,B01011000,B00011100,
  B01111001,B10010100,B01010001,B01100100,B00011000,
  B01000001,B00010100,B01011110,B01000000,B00110000,
  B01000001,B00010100,B01010000,B01000000,B01100000,
  B01111101,B00010011,B11001110,B01000000,B11111000,
  B00000000,B00000000,B00000000,B00000000,B11111100,
  B00000000,B00000000,B00000000,B00000000,B00000110,
  B00000000,B01111000,B00000000,B00000000,B00000011,
  B00000000,B01000100,B00000000,B00000000,B00000011,
  B00000000,B01000101,B01100011,B10000000,B00000011,
  B00000000,B01111001,B10010100,B01000011,B00000011,
  B00000000,B01000001,B00000100,B01000011,B00000110,
  B00000000,B01000001,B00000100,B01000001,B11111110,
  B00000000,B01000001,B00000011,B10000000,B11111000,
  B00000000,B00000000,B00000000,B00000000,B00000000,
};

//
// Use default bitmaps
//
#define STATUS_HOTEND_ANIM
#define STATUS_BED_ANIM
#define STATUS_HEATERS_XSPACE   20
#if HOTENDS < 2
  #define STATUS_HEATERS_X      48
  #define STATUS_BED_X          72
#else
  #define STATUS_HEATERS_X      40
  #define STATUS_BED_X          80
#endif
