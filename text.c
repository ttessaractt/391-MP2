/*
 * tab:4
 *
 * text.c - font data and text to graphics conversion utility
 *
 * "Copyright (c) 2004-2009 by Steven S. Lumetta."
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice and the following
 * two paragraphs appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE AUTHOR OR THE UNIVERSITY OF ILLINOIS BE LIABLE TO 
 * ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL 
 * DAMAGES ARISING OUT  OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, 
 * EVEN IF THE AUTHOR AND/OR THE UNIVERSITY OF ILLINOIS HAS BEEN ADVISED 
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND THE UNIVERSITY OF ILLINOIS SPECIFICALLY DISCLAIM ANY 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE 
 * PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND NEITHER THE AUTHOR NOR
 * THE UNIVERSITY OF ILLINOIS HAS ANY OBLIGATION TO PROVIDE MAINTENANCE, 
 * SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Author:        Steve Lumetta
 * Version:        2
 * Creation Date:   Thu Sep  9 22:06:29 2004
 * Filename:        text.c
 * History:
 *    SL    1    Thu Sep  9 22:06:29 2004
 *        First written.
 *    SL    2    Sat Sep 12 13:45:33 2009
 *        Integrated original release back into main code base.
 */

#include <string.h>

#include "text.h"

/*
Given string, produce a buffer that holds a graphical image of the ASCII characters in the string

buffer will be written at video mem address 0x0000
*/

int string_to_font(const char *string, unsigned char buf2[]){
    int size = 0;   //width
    int x = 1;
    int y = 1;
    //unsigned char* addr; // addr into buffer
    //unsigned char buf2[0x140*0x012];    // buffer to write into, size of status bar
    size_t g;
    g = strlen(string);
    //static unsigned short target_img2 = 0x0000;

    size_t a; int b; int c;
    for (a = 0; a < g; a++) {   // go through string
        int z = string[a];      // get ASCII code of character
        // font_data[(ASCII code)*16]

        //addr = (x >> 2) + y * (320-y);   
        // add ascii char to buffer
        for (b = x; b <= x+8; b++){
            for (c = y; c <= 17; c++){
                //addr = target_img + (b >> 2) + c * size;
                if (x <= 319){
                    buf2[b*c] = font_data[z*16][c];
                }
            }

        }
        size += 8;
        x += 8;
    }
    size += 2;

    return size;
};

/* 
 * These font data were read out of video memory during text mode and
 * saved here.  They could be read in the same manner at the start of a
 * game, but keeping a copy allows us to run the game to fix text mode
 * if it is broken (font data missing, usually).
 *
 * Each character is 8x16 pixels and occupies two lines in the table below.
 * Each byte represents a single bitmapped line of a single character.
 */
unsigned char font_data[256][16] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7E, 0x81, 0xA5, 0x81, 0x81, 0xBD,
     0x99, 0x81, 0x81, 0x7E, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7E, 0xFF, 0xDB, 0xFF, 0xFF, 0xC3,
     0xE7, 0xFF, 0xFF, 0x7E, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x6C, 0xFE, 0xFE, 0xFE,
     0xFE, 0x7C, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x7C, 0xFE,
     0x7C, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x18, 0x3C, 0x3C, 0xE7, 0xE7,
     0xE7, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x18, 0x3C, 0x7E, 0xFF, 0xFF,
     0x7E, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x3C,
     0x3C, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xC3,
     0xC3, 0xE7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x66, 0x42,
     0x42, 0x66, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC3, 0x99, 0xBD,
     0xBD, 0x99, 0xC3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0x00, 0x00, 0x1E, 0x0E, 0x1A, 0x32, 0x78, 0xCC,
     0xCC, 0xCC, 0xCC, 0x78, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x3C, 0x66, 0x66, 0x66, 0x66, 0x3C,
     0x18, 0x7E, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x3F, 0x33, 0x3F, 0x30, 0x30, 0x30,
     0x30, 0x70, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7F, 0x63, 0x7F, 0x63, 0x63, 0x63,
     0x63, 0x67, 0xE7, 0xE6, 0xC0, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x18, 0x18, 0xDB, 0x3C, 0xE7,
     0x3C, 0xDB, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFE, 0xF8,
     0xF0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x02, 0x06, 0x0E, 0x1E, 0x3E, 0xFE, 0x3E,
     0x1E, 0x0E, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x18, 0x3C, 0x7E, 0x18, 0x18, 0x18,
     0x7E, 0x3C, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66,
     0x66, 0x00, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7F, 0xDB, 0xDB, 0xDB, 0x7B, 0x1B,
     0x1B, 0x1B, 0x1B, 0x1B, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x7C, 0xC6, 0x60, 0x38, 0x6C, 0xC6, 0xC6,
     0x6C, 0x38, 0x0C, 0xC6, 0x7C, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xFE, 0xFE, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x18, 0x3C, 0x7E, 0x18, 0x18, 0x18,
     0x7E, 0x3C, 0x18, 0x7E, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x18, 0x3C, 0x7E, 0x18, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
     0x18, 0x7E, 0x3C, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x0C, 0xFE,
     0x0C, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x60, 0xFE,
     0x60, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0,
     0xC0, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x6C, 0xFE,
     0x6C, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x38, 0x7C,
     0x7C, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0x7C, 0x7C,
     0x38, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x18, 0x3C, 0x3C, 0x3C, 0x18, 0x18,
     0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x66, 0x66, 0x66, 0x24, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x6C, 0x6C, 0xFE, 0x6C, 0x6C,
     0x6C, 0xFE, 0x6C, 0x6C, 0x00, 0x00, 0x00, 0x00},
    {0x18, 0x18, 0x7C, 0xC6, 0xC2, 0xC0, 0x7C, 0x06,
     0x06, 0x86, 0xC6, 0x7C, 0x18, 0x18, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0xC2, 0xC6, 0x0C, 0x18,
     0x30, 0x60, 0xC6, 0x86, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x38, 0x6C, 0x6C, 0x38, 0x76, 0xDC,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x30, 0x30, 0x30, 0x60, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x30,
     0x30, 0x30, 0x18, 0x0C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x0C,
     0x0C, 0x0C, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x3C, 0xFF,
     0x3C, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x7E,
     0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x18, 0x18, 0x18, 0x30, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x02, 0x06, 0x0C, 0x18,
     0x30, 0x60, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x38, 0x6C, 0xC6, 0xC6, 0xC6, 0xC6,
     0xC6, 0xC6, 0x6C, 0x38, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x18, 0x38, 0x78, 0x18, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x7E, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7C, 0xC6, 0x06, 0x0C, 0x18, 0x30,
     0x60, 0xC0, 0xC6, 0xFE, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7C, 0xC6, 0x06, 0x06, 0x3C, 0x06,
     0x06, 0x06, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x0C, 0x1C, 0x3C, 0x6C, 0xCC, 0xFE,
     0x0C, 0x0C, 0x0C, 0x1E, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xFE, 0xC0, 0xC0, 0xC0, 0xFC, 0x06,
     0x06, 0x06, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x38, 0x60, 0xC0, 0xC0, 0xFC, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xFE, 0xC6, 0x06, 0x06, 0x0C, 0x18,
     0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0x7C, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0x7E, 0x06,
     0x06, 0x06, 0x0C, 0x78, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00,
     0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00,
     0x00, 0x18, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x06, 0x0C, 0x18, 0x30, 0x60,
     0x30, 0x18, 0x0C, 0x06, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00,
     0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x60, 0x30, 0x18, 0x0C, 0x06,
     0x0C, 0x18, 0x30, 0x60, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7C, 0xC6, 0xC6, 0x0C, 0x18, 0x18,
     0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xDE, 0xDE,
     0xDE, 0xDC, 0xC0, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x10, 0x38, 0x6C, 0xC6, 0xC6, 0xFE,
     0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xFC, 0x66, 0x66, 0x66, 0x7C, 0x66,
     0x66, 0x66, 0x66, 0xFC, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x3C, 0x66, 0xC2, 0xC0, 0xC0, 0xC0,
     0xC0, 0xC2, 0x66, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xF8, 0x6C, 0x66, 0x66, 0x66, 0x66,
     0x66, 0x66, 0x6C, 0xF8, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xFE, 0x66, 0x62, 0x68, 0x78, 0x68,
     0x60, 0x62, 0x66, 0xFE, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xFE, 0x66, 0x62, 0x68, 0x78, 0x68,
     0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x3C, 0x66, 0xC2, 0xC0, 0xC0, 0xDE,
     0xC6, 0xC6, 0x66, 0x3A, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xFE, 0xC6,
     0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x3C, 0x18, 0x18, 0x18, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
     0xCC, 0xCC, 0xCC, 0x78, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xE6, 0x66, 0x66, 0x6C, 0x78, 0x78,
     0x6C, 0x66, 0x66, 0xE6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xF0, 0x60, 0x60, 0x60, 0x60, 0x60,
     0x60, 0x62, 0x66, 0xFE, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xC6, 0xEE, 0xFE, 0xFE, 0xD6, 0xC6,
     0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xC6, 0xE6, 0xF6, 0xFE, 0xDE, 0xCE,
     0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xFC, 0x66, 0x66, 0x66, 0x7C, 0x60,
     0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6,
     0xC6, 0xD6, 0xDE, 0x7C, 0x0C, 0x0E, 0x00, 0x00},
    {0x00, 0x00, 0xFC, 0x66, 0x66, 0x66, 0x7C, 0x6C,
     0x66, 0x66, 0x66, 0xE6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7C, 0xC6, 0xC6, 0x60, 0x38, 0x0C,
     0x06, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7E, 0x7E, 0x5A, 0x18, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6,
     0xC6, 0x6C, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xD6, 0xD6,
     0xD6, 0xFE, 0xEE, 0x6C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xC6, 0xC6, 0x6C, 0x7C, 0x38, 0x38,
     0x7C, 0x6C, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18,
     0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xFE, 0xC6, 0x86, 0x0C, 0x18, 0x30,
     0x60, 0xC2, 0xC6, 0xFE, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x30,
     0x30, 0x30, 0x30, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0x70, 0x38,
     0x1C, 0x0E, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x3C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
     0x0C, 0x0C, 0x0C, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x10, 0x38, 0x6C, 0xC6, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00},
    {0x30, 0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x0C, 0x7C,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xE0, 0x60, 0x60, 0x78, 0x6C, 0x66,
     0x66, 0x66, 0x66, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC0,
     0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x1C, 0x0C, 0x0C, 0x3C, 0x6C, 0xCC,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xFE,
     0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x38, 0x6C, 0x64, 0x60, 0xF0, 0x60,
     0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xCC, 0xCC,
     0xCC, 0xCC, 0xCC, 0x7C, 0x0C, 0xCC, 0x78, 0x00},
    {0x00, 0x00, 0xE0, 0x60, 0x60, 0x6C, 0x76, 0x66,
     0x66, 0x66, 0x66, 0xE6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x18, 0x18, 0x00, 0x38, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x06, 0x06, 0x00, 0x0E, 0x06, 0x06,
     0x06, 0x06, 0x06, 0x06, 0x66, 0x66, 0x3C, 0x00},
    {0x00, 0x00, 0xE0, 0x60, 0x60, 0x66, 0x6C, 0x78,
     0x78, 0x6C, 0x66, 0xE6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xEC, 0xFE, 0xD6,
     0xD6, 0xD6, 0xD6, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x66, 0x66,
     0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x66, 0x66,
     0x66, 0x66, 0x66, 0x7C, 0x60, 0x60, 0xF0, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xCC, 0xCC,
     0xCC, 0xCC, 0xCC, 0x7C, 0x0C, 0x0C, 0x1E, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x76, 0x66,
     0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0x60,
     0x38, 0x0C, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x10, 0x30, 0x30, 0xFC, 0x30, 0x30,
     0x30, 0x30, 0x36, 0x1C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xCC, 0xCC, 0xCC,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x66, 0x66,
     0x66, 0x66, 0x3C, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xD6,
     0xD6, 0xD6, 0xFE, 0x6C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0x6C, 0x38,
     0x38, 0x38, 0x6C, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7E, 0x06, 0x0C, 0xF8, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xCC, 0x18,
     0x30, 0x60, 0xC6, 0xFE, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x0E, 0x18, 0x18, 0x18, 0x70, 0x18,
     0x18, 0x18, 0x18, 0x0E, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x00, 0x18,
     0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x70, 0x18, 0x18, 0x18, 0x0E, 0x18,
     0x18, 0x18, 0x18, 0x70, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x76, 0xDC, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x6C, 0xC6,
     0xC6, 0xC6, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x3C, 0x66, 0xC2, 0xC0, 0xC0, 0xC0,
     0xC2, 0x66, 0x3C, 0x0C, 0x06, 0x7C, 0x00, 0x00},
    {0x00, 0x00, 0xCC, 0x00, 0x00, 0xCC, 0xCC, 0xCC,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x0C, 0x18, 0x30, 0x00, 0x7C, 0xC6, 0xFE,
     0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x10, 0x38, 0x6C, 0x00, 0x78, 0x0C, 0x7C,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xCC, 0x00, 0x00, 0x78, 0x0C, 0x7C,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x60, 0x30, 0x18, 0x00, 0x78, 0x0C, 0x7C,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x38, 0x6C, 0x38, 0x00, 0x78, 0x0C, 0x7C,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x3C, 0x66, 0x60, 0x60,
     0x66, 0x3C, 0x0C, 0x06, 0x3C, 0x00, 0x00, 0x00},
    {0x00, 0x10, 0x38, 0x6C, 0x00, 0x7C, 0xC6, 0xFE,
     0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xC6, 0x00, 0x00, 0x7C, 0xC6, 0xFE,
     0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x60, 0x30, 0x18, 0x00, 0x7C, 0xC6, 0xFE,
     0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x66, 0x00, 0x00, 0x38, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x18, 0x3C, 0x66, 0x00, 0x38, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x60, 0x30, 0x18, 0x00, 0x38, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0xC6, 0x00, 0x10, 0x38, 0x6C, 0xC6, 0xC6,
     0xFE, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x38, 0x6C, 0x38, 0x00, 0x38, 0x6C, 0xC6, 0xC6,
     0xFE, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x18, 0x30, 0x60, 0x00, 0xFE, 0x66, 0x60, 0x7C,
     0x60, 0x60, 0x66, 0xFE, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xCC, 0x76, 0x36,
     0x7E, 0xD8, 0xD8, 0x6E, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x3E, 0x6C, 0xCC, 0xCC, 0xFE, 0xCC,
     0xCC, 0xCC, 0xCC, 0xCE, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x10, 0x38, 0x6C, 0x00, 0x7C, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xC6, 0x00, 0x00, 0x7C, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x60, 0x30, 0x18, 0x00, 0x7C, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x30, 0x78, 0xCC, 0x00, 0xCC, 0xCC, 0xCC,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x60, 0x30, 0x18, 0x00, 0xCC, 0xCC, 0xCC,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xC6, 0x00, 0x00, 0xC6, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7E, 0x06, 0x0C, 0x78, 0x00},
    {0x00, 0xC6, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0xC6, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x18, 0x18, 0x3C, 0x66, 0x60, 0x60, 0x60,
     0x66, 0x3C, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x38, 0x6C, 0x64, 0x60, 0xF0, 0x60, 0x60,
     0x60, 0x60, 0xE6, 0xFC, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x66, 0x66, 0x3C, 0x18, 0x7E, 0x18,
     0x7E, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0xF8, 0xCC, 0xCC, 0xF8, 0xC4, 0xCC, 0xDE,
     0xCC, 0xCC, 0xCC, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x0E, 0x1B, 0x18, 0x18, 0x18, 0x7E, 0x18,
     0x18, 0x18, 0x18, 0x18, 0xD8, 0x70, 0x00, 0x00},
    {0x00, 0x18, 0x30, 0x60, 0x00, 0x78, 0x0C, 0x7C,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x0C, 0x18, 0x30, 0x00, 0x38, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x18, 0x30, 0x60, 0x00, 0x7C, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x18, 0x30, 0x60, 0x00, 0xCC, 0xCC, 0xCC,
     0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x76, 0xDC, 0x00, 0xDC, 0x66, 0x66,
     0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00},
    {0x76, 0xDC, 0x00, 0xC6, 0xE6, 0xF6, 0xFE, 0xDE,
     0xCE, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x3C, 0x6C, 0x6C, 0x3E, 0x00, 0x7E, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x38, 0x6C, 0x6C, 0x38, 0x00, 0x7C, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x30, 0x30, 0x00, 0x30, 0x30, 0x60,
     0xC0, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xC0,
     0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x06,
     0x06, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0xC0, 0xC0, 0xC2, 0xC6, 0xCC, 0x18, 0x30,
     0x60, 0xDC, 0x86, 0x0C, 0x18, 0x3E, 0x00, 0x00},
    {0x00, 0xC0, 0xC0, 0xC2, 0xC6, 0xCC, 0x18, 0x30,
     0x66, 0xCE, 0x9E, 0x3E, 0x06, 0x06, 0x00, 0x00},
    {0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18,
     0x3C, 0x3C, 0x3C, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x6C, 0xD8,
     0x6C, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xD8, 0x6C, 0x36,
     0x6C, 0xD8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44,
     0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44},
    {0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA,
     0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA},
    {0xDD, 0x77, 0xDD, 0x77, 0xDD, 0x77, 0xDD, 0x77,
     0xDD, 0x77, 0xDD, 0x77, 0xDD, 0x77, 0xDD, 0x77},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xF8,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0xF8, 0x18, 0xF8,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xF6,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x18, 0xF8,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0xF6, 0x06, 0xF6,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x06, 0xF6,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0xF6, 0x06, 0xFE,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xFE,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0xF8, 0x18, 0xF8,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1F,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xFF,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1F,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xFF,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x1F, 0x18, 0x1F,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x37,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0x37, 0x30, 0x3F,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x30, 0x37,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0xF7, 0x00, 0xFF,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xF7,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0x37, 0x30, 0x37,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0xF7, 0x00, 0xF7,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0xFF, 0x00, 0xFF,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xFF,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x3F,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x1F, 0x18, 0x1F,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x18, 0x1F,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xFF,
     0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0xFF, 0x18, 0xFF,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xF8,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
     0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0},
    {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
     0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xDC, 0xD8,
     0xD8, 0xD8, 0xDC, 0x76, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x78, 0xCC, 0xCC, 0xCC, 0xD8, 0xCC,
     0xC6, 0xC6, 0xC6, 0xCC, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0xFE, 0xC6, 0xC6, 0xC0, 0xC0, 0xC0,
     0xC0, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0xFE, 0x6C, 0x6C, 0x6C,
     0x6C, 0x6C, 0x6C, 0x6C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0xFE, 0xC6, 0x60, 0x30, 0x18,
     0x30, 0x60, 0xC6, 0xFE, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0xD8, 0xD8,
     0xD8, 0xD8, 0xD8, 0x70, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x66, 0x66, 0x66, 0x66,
     0x66, 0x7C, 0x60, 0x60, 0xC0, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x76, 0xDC, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x7E, 0x18, 0x3C, 0x66, 0x66,
     0x66, 0x3C, 0x18, 0x7E, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x38, 0x6C, 0xC6, 0xC6, 0xFE,
     0xC6, 0xC6, 0x6C, 0x38, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x38, 0x6C, 0xC6, 0xC6, 0xC6, 0x6C,
     0x6C, 0x6C, 0x6C, 0xEE, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x1E, 0x30, 0x18, 0x0C, 0x3E, 0x66,
     0x66, 0x66, 0x66, 0x3C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0xDB, 0xDB,
     0xDB, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x03, 0x06, 0x7E, 0xDB, 0xDB,
     0xF3, 0x7E, 0x60, 0xC0, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x1C, 0x30, 0x60, 0x60, 0x7C, 0x60,
     0x60, 0x60, 0x30, 0x1C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6,
     0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0xFE,
     0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x7E, 0x18,
     0x18, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x30, 0x18, 0x0C, 0x06, 0x0C,
     0x18, 0x30, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x0C, 0x18, 0x30, 0x60, 0x30,
     0x18, 0x0C, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x0E, 0x1B, 0x1B, 0x18, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
     0xD8, 0xD8, 0xD8, 0x70, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x7E,
     0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xDC, 0x00,
     0x76, 0xDC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x38, 0x6C, 0x6C, 0x38, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
     0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x0F, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0xEC,
     0x6C, 0x6C, 0x3C, 0x1C, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0xD8, 0x6C, 0x6C, 0x6C, 0x6C, 0x6C, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x70, 0xD8, 0x30, 0x60, 0xC8, 0xF8, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x7C, 0x7C, 0x7C, 0x7C,
     0x7C, 0x7C, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};
