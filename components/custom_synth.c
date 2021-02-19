/*******************************************************************************
* Written by Maanika Kenneth Koththioda, for PSoC5LP
* Last Modified on 4/1/2021
*
* File: custom_synth.c
* Version: 1.0.0
*
* Brief: Project dependent LPC audio constants and functions.
*
* Target device:
*    CY8C5888LTI - LP097
*
* Code Tested With:
*    - Silicon: PSoC 5LP
*    - IDE: PSoC Creator 4.3
*    - Compiler: GCC 5.4
*
* Creating LPC encoded sound samples:
*   Download python_wizard from https://github.com/ptwz/python_wizard.
*   This program is used to convert audio streams with a .WAV extension into LPC 
*   bit streams.
*
* Using python_wizard:
*   1. Download python_wizard and install prerequisites.
*   2. Ensure python_wizard and python_wizard_gui file has a .py extension to be 
*   identifed by python editors.
*   3. Place EXAMPLE.wav file in the same directory as python_wizard.py.
*   4. Open the command line, cd to the project path and enter the command 
*   python python_wizard.py -f arduino EXAMPLE.wav > EXAMPLE.h
*   5. Open EXAMPLE.h to obtain reuquired LPC code.
*
* Notes:
*   Optimized LPC codes stored in the ROM of Speech Synthesisers can be found here
*   https://github.com/going-digital/Talkie/tree/master/Talkie/examples
*
*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "lpc_synth.h"
#include "custom_synth.h"

/*******************************************************************************
*   Constant definitions
*******************************************************************************/

/* LPC encoded word "Hargrave library" */
static const uint8_t HARGRAVE[]= {
    0x6C, 0xCD, 0x69, 0xCD, 0xB3, 0xDC, 0xA9, 0xBF, 0xC6, 0x71,
    0x8F, 0x74, 0xAB, 0xAC, 0x52, 0xCA, 0xC4, 0x42, 0x8A, 0xB1,
    0x63, 0x35, 0xD7, 0x0E, 0xA6, 0xCE, 0x8A, 0x4D, 0x53, 0xDA,
    0xD5, 0x04, 0x33, 0x66, 0x2B, 0xCF, 0x64, 0xED, 0x8C, 0x98,
    0xA4, 0xD2, 0x9C, 0xB5, 0x30, 0x7C, 0x92, 0x4C, 0x0D, 0x25,
    0xC2, 0x48, 0x89, 0x23, 0xD5, 0x14, 0x33, 0x23, 0x7A, 0x76,
    0x8B, 0x50, 0x2D, 0x4C, 0x93, 0xA4, 0xCB, 0x92, 0x31, 0xB3,
    0x75, 0xE5, 0x4E, 0x2F, 0x47, 0xCA, 0x0E, 0x91, 0x3B, 0xA2,
    0x14, 0x39, 0x27, 0x19, 0xA9, 0xAC, 0x42, 0xEC, 0x9C, 0xA4,
    0x7C, 0xDA, 0x93, 0x91, 0x73, 0xA3, 0xB6, 0xCD, 0x50, 0x55,
    0xC6, 0x8D, 0xC6, 0x37, 0x42, 0x59, 0x09, 0x37, 0x69, 0xAD,
    0x4C, 0xA7, 0xCD, 0x3C, 0xAF, 0xA4, 0xC3, 0x5D, 0x16, 0xF1,
    0xAD, 0xB4, 0x5C, 0x72, 0xC5, 0xC4, 0xA7, 0xB9, 0x23, 0x24,
    0x1C, 0x31, 0x5F, 0x0E, 0xF6, 0x4E, 0x63, 0xAC, 0xFC, 0x30,
    0x28, 0xCB, 0xD8, 0x95, 0x99, 0x62, 0x17, 0x4F, 0x13, 0x8D,
    0x6E, 0x8A, 0xC5, 0xD4, 0xDD, 0x30, 0x39, 0x3F, 0x16, 0x17,
    0x37, 0xE7, 0xE4, 0xBC, 0x94, 0x2D, 0x3C, 0x94, 0x83, 0x73,
    0x53, 0xD6, 0x1C, 0x77, 0x4E, 0xCE, 0x8D, 0x51, 0xBB, 0xCD,
    0x38, 0x2B, 0xCF, 0x67, 0xAB, 0x56, 0x57, 0xCD, 0x7C, 0x9F,
    0x34, 0xC2, 0x4C, 0x09, 0xF1, 0x7D, 0x10, 0x4F, 0x31, 0x39,
    0x2C, 0x90, 0x55, 0xBA, 0xD9, 0x64, 0xB3, 0x50, 0x17, 0xE9,
    0xA0, 0x90, 0x25, 0x22, 0x5F, 0xB5, 0x13, 0x43, 0x8E, 0x88,
    0x7D, 0x96, 0x09, 0x72, 0xC5, 0x2A, 0x09, 0x91, 0x3B, 0xD8,
    0x58, 0xAB, 0x24, 0x44, 0x6A, 0xA7, 0x60, 0x25, 0x12, 0x9F,
    0x68, 0x84, 0x52, 0xB6, 0x48, 0x5C, 0xA2, 0x51, 0x09, 0xC9,
    0x22, 0xB5, 0x89, 0xC6, 0xD8, 0x69, 0x8B, 0xD4, 0x05, 0xEE,
    0x42, 0x57, 0xCC, 0x0E, 0x15, 0xBC, 0x52, 0x0C, 0x31, 0x4B,
    0xB5, 0xCD, 0x76, 0x76, 0xDA, 0x24, 0x55, 0x2A, 0xDA, 0xD4,
    0x68, 0x93, 0xD4, 0x70, 0xEF, 0xA2, 0x90, 0x4D, 0x04, 0xC5,
    0xB5, 0xDD, 0xD3, 0x36, 0xE2, 0xA9, 0xB0, 0x4C, 0x4F, 0x3B,
    0x48, 0x64, 0xD2, 0x23, 0x2C, 0xE2, 0x20, 0x91, 0xAB, 0x70,
    0x37, 0x73, 0x82, 0x12, 0x63, 0xC2, 0xDD, 0xD5, 0x36, 0xFF, 
    0xFF
};

/* LPC encoded word "Campbell hall" */
static const uint8_t CAMPBELL[] = {
    0x80, 0x80, 0x95, 0xDD, 0x1D, 0x30, 0xA3, 0x86, 0x02, 0xA6,
    0x91, 0x50, 0xDE, 0x30, 0x1C, 0x55, 0xA6, 0xD8, 0xB8, 0x51,
    0xDA, 0x24, 0x8B, 0x62, 0xE3, 0x06, 0x9B, 0x93, 0x22, 0x8E,
    0x9C, 0x1B, 0x7C, 0x4C, 0xB8, 0x3A, 0x76, 0x5E, 0xC8, 0x51,
    0x65, 0xAE, 0xD8, 0x78, 0xA1, 0x58, 0xAE, 0x9B, 0x6D, 0xE5,
    0x85, 0x62, 0xD6, 0x11, 0x8A, 0x99, 0x67, 0xAA, 0x69, 0x5A,
    0x26, 0x61, 0xBE, 0x68, 0x2E, 0xE9, 0xE1, 0x5A, 0x78, 0x26,
    0xA5, 0x9B, 0x85, 0x22, 0xE5, 0xDA, 0xA7, 0x5A, 0x6D, 0x0A,
    0x85, 0x63, 0x1F, 0x6B, 0x8D, 0xC8, 0x52, 0x8E, 0xBF, 0x2C,
    0x3D, 0xA6, 0x48, 0x38, 0xFE, 0x92, 0xCD, 0xA8, 0x62, 0xE5,
    0xFA, 0x29, 0x9E, 0xC1, 0x89, 0x95, 0x17, 0x92, 0x7A, 0x25,
    0x39, 0x36, 0x7E, 0x74, 0x6A, 0xE6, 0x2E, 0x5B, 0xBD, 0xC9,
    0x9B, 0x49, 0x84, 0x2C, 0xE5, 0xC6, 0x6C, 0x22, 0xE1, 0x8C,
    0x8D, 0x17, 0x96, 0x5A, 0xA4, 0xB2, 0x36, 0x5E, 0x78, 0xAA,
    0xE3, 0xCA, 0xC6, 0xF8, 0xE1, 0xB1, 0x56, 0x08, 0x67, 0x13,
    0x84, 0x25, 0x9A, 0xA5, 0x68, 0x54, 0x18, 0x36, 0x7B, 0x87,
    0xB0, 0x51, 0x91, 0xDF, 0xAA, 0x53, 0xC6, 0x44, 0x45, 0xFE,
    0xB0, 0x4F, 0x09, 0x6B, 0x15, 0x87, 0x4B, 0xBE, 0xAE, 0xAC,
    0x45, 0x1C, 0x2E, 0xE9, 0xBA, 0xB2, 0x16, 0x89, 0x3F, 0xA8,
    0xE3, 0xC2, 0x8A, 0x25, 0x6E, 0xA1, 0x4C, 0x08, 0x6B, 0x92,
    0xF2, 0x41, 0xD2, 0xA9, 0x48, 0x88, 0x40, 0xAB, 0x50, 0x66,
    0x48, 0x46, 0xA2, 0x4A, 0x46, 0xE6, 0x69, 0x19, 0x89, 0x36,
    0xA6, 0xA8, 0x85, 0x45, 0xC4, 0x7B, 0x57, 0xA6, 0xEA, 0xB6,
    0xD1, 0x13, 0x5D, 0xB9, 0x99, 0x39, 0x46, 0x4F, 0xB2, 0x69,
    0x2A, 0x2A, 0x1B, 0xFF, 0xFF
};         
                
/* LPC encoded word "Campus Centre" */
static const uint8_t CENTRE[] = {
    0x20, 0x80, 0x95, 0x2D, 0x02, 0x30, 0x83, 0x84, 0x01, 0x86,
    0xE6, 0x70, 0x6E, 0xD7, 0x9C, 0xE9, 0xAA, 0xC8, 0xB9, 0x41,
    0xE7, 0x24, 0x89, 0x23, 0xE7, 0x46, 0x9B, 0xE3, 0xAA, 0x8E,
    0x82, 0x17, 0x7D, 0xB4, 0x9B, 0xD9, 0x76, 0x5E, 0x48, 0x5E,
    0x65, 0xAE, 0xC8, 0x79, 0xB1, 0x58, 0x8C, 0xBB, 0x22, 0xE5,
    0xB9, 0x62, 0x56, 0x9E, 0xB2, 0x85, 0xCF, 0x73, 0x7A, 0x99,
    0x3B, 0x61, 0xBE, 0xAD, 0xE1, 0xC1, 0x26, 0x9B, 0xF8, 0xA6,
    0xA4, 0x95, 0x98, 0x2C, 0x16, 0x44, 0x13, 0x19, 0xEE, 0xB4,
    0x8D, 0xE3, 0xBD, 0xB7, 0xA7, 0xA8, 0x72, 0x8E, 0x2B, 0x5E,
    0x9E, 0xE4, 0x5A, 0xB9, 0x3E, 0x56, 0x9A, 0x0B, 0x13, 0xE1,
    0x04, 0x93, 0x91, 0x1A, 0x4C, 0x98, 0x9B, 0x94, 0x47, 0x58,
    0xC8, 0x62, 0x72, 0xB6, 0xEE, 0xEE, 0xA1, 0x98, 0xC9, 0x45,
    0xBB, 0x87, 0x8F, 0x6A, 0xE1, 0x16, 0xED, 0x9E, 0xD1, 0x4E,
    0xC8, 0x95, 0x65, 0x44, 0x65, 0x46, 0x26, 0x6F, 0x33, 0x6E,
    0xEE, 0x6D, 0x9B, 0x19, 0x45, 0x5B, 0x58, 0x85, 0x94, 0xE0,
    0x86, 0xE4, 0xE5, 0x4D, 0x89, 0x93, 0x9F, 0xA2, 0x56, 0x14,
    0x26, 0x0E, 0x7E, 0x8C, 0x52, 0x91, 0x98, 0xD8, 0x04, 0x3E,
    0x72, 0x87, 0x51, 0x2A, 0x15, 0xD0, 0x6A, 0x9D, 0x66, 0x09,
    0x45, 0xC0, 0x8A, 0x75, 0x99, 0x64, 0x14, 0x81, 0x72, 0x16,
    0x1D, 0xC6, 0xC6, 0x04, 0x99, 0xB9, 0xA6, 0x97, 0x62, 0x15,
    0x4C, 0xC2, 0x36, 0x5C, 0xB6, 0x45, 0xD0, 0x39, 0x7B, 0xF1,
    0xC8, 0x36, 0x91, 0x0B, 0x99, 0xAE, 0x81, 0xC4, 0xC4, 0xB6,
    0x7B, 0x9A, 0x3A, 0x13, 0x93, 0xDA, 0x22, 0x65, 0x6A, 0x4C,
    0x54, 0xAA, 0x33, 0x77, 0xA0, 0x33, 0x16, 0xA9, 0x2A, 0x92,
    0x89, 0xA6, 0x98, 0x65, 0x3A, 0x69, 0x16, 0x29, 0x63, 0x92,
    0x39, 0xEF, 0x9E, 0x64, 0x72, 0x48, 0x1A, 0x42, 0x58, 0xAB,
    0xD3, 0x41, 0x69, 0xB0, 0x65, 0x65, 0x2E, 0x0B, 0x6D, 0xD1,
    0xA4, 0x95, 0xB9, 0x2C, 0x64, 0x64, 0xED, 0x1A, 0xEA, 0xB6,
    0xD1, 0x5C, 0x74, 0x68, 0x58, 0x49, 0x06, 0xFF, 0xFF
};

/* LPC encoded sentence "You have arrived at your destination" */
static const uint8_t ARRIVED[] = {
    0x80, 0x1C, 0xE2, 0x2E, 0x44, 0xAC, 0x31, 0x73, 0x44, 0x98,
    0x16, 0xF4, 0xD4, 0xCA, 0xD1, 0xA6, 0x36, 0x85, 0x35, 0x2B,
    0x47, 0xF9, 0x98, 0x08, 0x65, 0x2D, 0x3C, 0xED, 0x3C, 0xBD,
    0x0C, 0x91, 0xF1, 0x82, 0x33, 0xF7, 0x0A, 0xC6, 0xCE, 0x2B,
    0xCE, 0x42, 0xDA, 0x18, 0x07, 0xB7, 0x38, 0x77, 0xCF, 0x50,
    0x14, 0x9C, 0x64, 0x3C, 0x2D, 0x5C, 0x51, 0xB2, 0x62, 0x88,
    0xF0, 0x34, 0xC5, 0xC9, 0x8C, 0xC5, 0xC3, 0xD2, 0x1D, 0x27,
    0x23, 0x15, 0x0F, 0x2F, 0x55, 0x9D, 0xB4, 0x54, 0x35, 0x22,
    0xC8, 0xA3, 0xD3, 0x7C, 0x31, 0x0B, 0x13, 0xD7, 0x46, 0x8B,
    0xC1, 0xD5, 0xD4, 0x5D, 0x2B, 0x2D, 0x7A, 0x77, 0x71, 0x57,
    0xAB, 0x14, 0x9B, 0x2C, 0x2C, 0x93, 0xB6, 0x51, 0x4D, 0xB1,
    0x96, 0x0C, 0x3A, 0x46, 0xD5, 0x55, 0x5B, 0x33, 0xE9, 0x28,
    0x4D, 0x77, 0x69, 0x73, 0x97, 0xCD, 0x4C, 0x36, 0xA5, 0xC2,
    0xDC, 0x32, 0xB3, 0xD8, 0xD2, 0x74, 0x33, 0x5B, 0xCA, 0xB6,
    0x5B, 0xC2, 0xDA, 0x94, 0x38, 0x27, 0xB4, 0x08, 0x59, 0x47,
    0x1B, 0xCE, 0x30, 0xCD, 0xAD, 0x55, 0x4B, 0x70, 0x42, 0x0F,
    0xD3, 0x32, 0x26, 0xCE, 0x0D, 0x35, 0xCC, 0xCA, 0x99, 0x38,
    0x37, 0x54, 0xB7, 0x2C, 0x65, 0xE2, 0xBC, 0x10, 0xD3, 0xA5,
    0x42, 0xB6, 0x0B, 0x7C, 0x76, 0x93, 0x54, 0xC7, 0x2E, 0xF4,
    0xC1, 0xDD, 0x42, 0x58, 0x9B, 0x28, 0xC4, 0x28, 0x73, 0x61,
    0x6D, 0x22, 0x1F, 0xAA, 0x58, 0x85, 0x35, 0xEB, 0x79, 0x08,
    0x33, 0x73, 0x25, 0x2C, 0xA6, 0x26, 0xC2, 0xC3, 0x95, 0x90,
    0x18, 0xDB, 0xD2, 0xB0, 0x50, 0xAD, 0x84, 0x22, 0x95, 0xD3,
    0xDC, 0xB6, 0x09, 0x6C, 0xE8, 0x68, 0x15, 0xC5, 0x26, 0xD0,
    0x35, 0x22, 0x98, 0x35, 0xA9, 0x40, 0xE5, 0x88, 0x14, 0xD1,
    0xA4, 0x82, 0x40, 0x2A, 0xCB, 0xC3, 0xB6, 0x01, 0x26, 0x27,
    0x73, 0xC0, 0xD6, 0x2C, 0xC6, 0x69, 0x84, 0x72, 0xD2, 0x69,
    0x3B, 0xD7, 0xF9, 0xAA, 0x52, 0xA3, 0xEB, 0x3C, 0x57, 0xA2,
    0xD7, 0x5C, 0x89, 0xF3, 0x7C, 0xB1, 0x59, 0x0F, 0x25, 0xC6,
    0xF7, 0x41, 0xA6, 0x43, 0x69, 0x0B, 0x5F, 0xE4, 0x88, 0x66,
    0xA1, 0xC5, 0x7C, 0xE1, 0x3C, 0x46, 0x45, 0xB6, 0x82, 0xA2,
    0x56, 0xF3, 0x72, 0xD9, 0xC6, 0xB3, 0x25, 0x2A, 0x13, 0x9D,
    0x38, 0xCF, 0xF6, 0xAA, 0x48, 0x76, 0x62, 0x3C, 0xD7, 0x2A,
    0xBD, 0x28, 0xB5, 0xF1, 0x7C, 0xEE, 0xD2, 0xC1, 0x24, 0xCA,
    0xB3, 0xA5, 0xCA, 0x83, 0x15, 0x2B, 0x3F, 0x28, 0x8F, 0x08,
    0x91, 0x25, 0xFC, 0x20, 0x2D, 0xDC, 0x4D, 0x36, 0xF3, 0xA3,
    0x4A, 0x4F, 0x77, 0xDB, 0xC8, 0x77, 0x76, 0x22, 0xDC, 0x24,
    0x21, 0x3F, 0xA9, 0x88, 0xF4, 0xB0, 0x25, 0xBC, 0x2C, 0x23,
    0x33, 0xD3, 0xB1, 0x72, 0x6D, 0xA8, 0xCA, 0x20, 0x36, 0xCA,
    0x33, 0x33, 0xD2, 0x8C, 0x5A, 0x0A, 0x4F, 0x75, 0xAB, 0x50,
    0x49, 0xC0, 0x3C, 0x59, 0xA5, 0x4A, 0x2B, 0xA1, 0x70, 0x65,
    0xF6, 0x4A, 0xB3, 0x94, 0xC1, 0xF3, 0xA1, 0x26, 0x5C, 0x5D,
    0x07, 0x3F, 0xEA, 0xD8, 0x70, 0x4E, 0x1C, 0xFC, 0x28, 0x6D,
    0xD3, 0xA9, 0x71, 0xF0, 0xA3, 0xB4, 0x35, 0x95, 0x54, 0x2E,
    0x88, 0xD2, 0x96, 0x59, 0x5B, 0x99, 0x40, 0xDB, 0x1E, 0x45,
    0xF7, 0x68, 0x82, 0xC8, 0xB3, 0x4F, 0x9D, 0xB1, 0x1B, 0x06,
    0x63, 0xDE, 0x68, 0x89, 0x06, 0xB8, 0x90, 0xC5, 0x00, 0x0F,
    0x8B, 0x1A, 0xE0, 0x29, 0x25, 0x05, 0x5C, 0xE7, 0x68, 0x82,
    0x0A, 0x26, 0xCA, 0x4C, 0xAC, 0x8B, 0x7D, 0x1C, 0x1D, 0x15,
    0x36, 0x2E, 0x76, 0xB9, 0x75, 0x44, 0x54, 0x99, 0x44, 0xA7,
    0x90, 0x61, 0x95, 0x6D, 0x12, 0xE9, 0x9C, 0x5B, 0x42, 0x8A,
    0xDA, 0x98, 0x57, 0x19, 0x4B, 0xDB, 0x2C, 0x01, 0xBE, 0x79,
    0xDC, 0x94, 0x10, 0x11, 0xA4, 0xB2, 0x76, 0x57, 0x42, 0x44,
    0x90, 0x5B, 0xCB, 0x9D, 0x0E, 0x11, 0x41, 0x69, 0x2D, 0x33,
    0xD9, 0x48, 0x04, 0x69, 0x3C, 0x3D, 0x95, 0x20, 0x11, 0xD8,
    0x8E, 0x8A, 0x70, 0x8C, 0x36, 0xA0, 0xD3, 0x3B, 0x32, 0x0D,
    0x92, 0xA8, 0x76, 0xAD, 0x48, 0xC7, 0x88, 0x67, 0x3A, 0x2C,
    0x34, 0x1C, 0xA3, 0x44, 0xE8, 0xD4, 0xD4, 0x90, 0x0D, 0xFF,
    0xFF                                                  
};

/* LPC encoded sentence "Waiting for a GPS fix" */
static const uint8_t FIX[] = {
    0xC8, 0x61, 0x23, 0x85, 0x23, 0x63, 0x23, 0x47, 0x3C, 0xD4,
    0xED, 0x90, 0x44, 0x6C, 0xF9, 0xD0, 0xBA, 0x43, 0x91, 0xB0,
    0xD4, 0xE1, 0x8C, 0x66, 0xC7, 0xC1, 0x0E, 0x51, 0xDB, 0x97,
    0x1C, 0x07, 0x37, 0x5A, 0x5B, 0x5B, 0xB1, 0x1C, 0xDC, 0xA4,
    0xF5, 0xA2, 0x29, 0x76, 0x70, 0xB3, 0x92, 0x8B, 0xA4, 0x58,
    0xCE, 0xCB, 0x4A, 0x36, 0x83, 0x2B, 0x1B, 0x3F, 0x2A, 0xDD,
    0x08, 0xAC, 0xA4, 0x7C, 0x63, 0x7D, 0xCD, 0xC4, 0x96, 0x09,
    0xA2, 0xD6, 0x09, 0xB3, 0x44, 0x26, 0x74, 0xC6, 0xD7, 0x85,
    0x12, 0x9B, 0x28, 0x68, 0x5F, 0x13, 0xCA, 0x6C, 0xE2, 0x60,
    0x7C, 0x48, 0xB8, 0x86, 0x8A, 0xBD, 0x89, 0x06, 0x8B, 0x08,
    0x2A, 0x96, 0x31, 0x12, 0x33, 0x25, 0x8A, 0x98, 0x35, 0x35,
    0x8D, 0xA0, 0xC5, 0x62, 0x56, 0xCD, 0x24, 0x92, 0x36, 0x8B,
    0x41, 0x71, 0x77, 0x0F, 0x39, 0x24, 0x06, 0x25, 0xCC, 0x34,
    0x9D, 0x10, 0x0E, 0xF8, 0x8C, 0xB0, 0x74, 0x82, 0x38, 0x60,
    0xAB, 0x32, 0xC2, 0x31, 0x93, 0x3C, 0x17, 0x57, 0xF7, 0x34,
    0x62, 0xCB, 0xDE, 0x92, 0x54, 0x6C, 0xB3, 0x3E, 0x05, 0x4F,
    0x0E, 0xB3, 0x4D, 0x2C, 0x5B, 0xC2, 0xD5, 0x4D, 0x36, 0x73,
    0xCD, 0x47, 0xCB, 0x0C, 0x39, 0xC2, 0x75, 0x8F, 0x3A, 0xD3,
    0xE9, 0x38, 0x2F, 0x46, 0x1E, 0x5F, 0x53, 0x15, 0xFC, 0xE4,
    0x79, 0xEC, 0x5D, 0x49, 0xF2, 0x53, 0x90, 0x8A, 0x62, 0x3B,
    0xC1, 0x0F, 0x51, 0x3B, 0x83, 0x6D, 0x2B, 0xDF, 0x06, 0xAD,
    0x34, 0x51, 0x2C, 0x7C, 0x15, 0xD5, 0x53, 0x4C, 0x8D, 0xF0,
    0x4D, 0xCC, 0x28, 0x53, 0x5A, 0xCE, 0xEF, 0xDC, 0x2D, 0xCC,
    0x5D, 0x19, 0x60, 0x4B, 0x61, 0x07, 0x1C, 0x67, 0xEC, 0x9C,
    0x8C, 0x35, 0xB6, 0x82, 0xB6, 0x73, 0x13, 0xB3, 0x3E, 0x17,
    0x35, 0xCE, 0xCD, 0x5C, 0x7A, 0x4D, 0xD4, 0x18, 0x37, 0x71,
    0x99, 0x36, 0x56, 0xAD, 0xDC, 0x20, 0xBC, 0x5B, 0x4C, 0x09,
    0x71, 0x23, 0xB1, 0x58, 0x35, 0x5B, 0xC8, 0x0B, 0xA8, 0x62,
    0xD4, 0x1C, 0x21, 0x3F, 0xE3, 0x88, 0xD2, 0x90, 0xC5, 0xF8,
    0x2C, 0xD9, 0x3D, 0x82, 0x96, 0xD2, 0x12, 0xE7, 0xCE, 0x12,
    0xD9, 0x46, 0xF3, 0xD2, 0x37, 0x15, 0x1D, 0x3B, 0x23, 0x71,
    0xD9, 0x32, 0x72, 0xE4, 0x8C, 0x20, 0x74, 0x4B, 0xD8, 0xA1,
    0x33, 0xA3, 0xD4, 0x6D, 0x13, 0xD7, 0xCE, 0x0E, 0xDA, 0x37,
    0x48, 0x53, 0x39, 0xD7, 0xC7, 0x5A, 0x63, 0x4B, 0xE4, 0x7C,
    0x1F, 0x72, 0x54, 0x34, 0x91, 0x0B, 0x5C, 0xAD, 0x64, 0x67,
    0xDB, 0x2E, 0xF4, 0x35, 0xD2, 0x8D, 0x15, 0xBB, 0x30, 0x26,
    0xEF, 0x6A, 0x66, 0xEC, 0xC2, 0x18, 0xAC, 0xA3, 0x51, 0x89,
    0x09, 0xBD, 0xF7, 0xB2, 0x24, 0xC6, 0x22, 0x08, 0xC6, 0x23,
    0x4C, 0xE8, 0xB2, 0x30, 0x28, 0x4D, 0xF7, 0x94, 0x2D, 0xC2,
    0xAC, 0xD5, 0xCC, 0x83, 0xB2, 0x0A, 0xAB, 0x0A, 0x71, 0x0F,
    0x56, 0x2C, 0x28, 0xD2, 0x34, 0x2C, 0x68, 0xA1, 0x20, 0x93,
    0x92, 0xD2, 0xA4, 0x45, 0xC2, 0x6E, 0xD4, 0x42, 0x43, 0x26,
    0x59, 0xBA, 0x55, 0x53, 0x0F, 0x9B, 0x0C, 0xEF, 0xDA, 0x34,
    0x2C, 0x14, 0x39, 0xD7, 0xFB, 0x98, 0x70, 0xA3, 0x95, 0xBC,
    0x68, 0x7C, 0xAA, 0x5D, 0x61, 0x08, 0x82, 0x8B, 0xF6, 0x50,
    0x46, 0x2E, 0x0E, 0xA1, 0x26, 0x5C, 0x59, 0xB9, 0xD8, 0x85,
    0x6A, 0x65, 0x71, 0x69, 0xE2, 0xA0, 0x6C, 0x4D, 0x49, 0x91,
    0x8A, 0xA3, 0x94, 0x09, 0x52, 0x56, 0x24, 0xE6, 0x3E, 0xD3,
    0x51, 0x59, 0x93, 0x5E, 0x99, 0xCC, 0x14, 0xA1, 0x8D, 0x36,
    0x63, 0x2B, 0x53, 0x84, 0x16, 0x19, 0x92, 0x93, 0xC8, 0x74,
    0x49, 0x82, 0x6E, 0x51, 0x43, 0xD3, 0x15, 0x8B, 0xA1, 0x05,
    0x17, 0xF7, 0x51, 0xC4, 0xFE, 0xE6, 0x4C, 0x3D, 0x43, 0x36,
    0x31, 0x9A, 0x33, 0x77, 0x0F, 0xD9, 0xC4, 0x6C, 0x39, 0x24,
    0xA4, 0x65, 0x13, 0xAA, 0x45, 0x93, 0xF0, 0x94, 0xCD, 0xBE,
    0xE6, 0x42, 0xD3, 0x4B, 0x16, 0xBB, 0x5B, 0x0E, 0xB3, 0x48,
    0x49, 0xEC, 0xAF, 0x21, 0x2C, 0x22, 0x6D, 0x93, 0xA3, 0x9A,
    0x30, 0xAF, 0x50, 0x42, 0xCE, 0x1A, 0x5C, 0x23, 0xD3, 0x36,
    0x79, 0xAA, 0x33, 0xF5, 0x4A, 0x47, 0xE8, 0xAC, 0xDE, 0x2C,
    0x3C, 0x25, 0xA3, 0xA4, 0x18, 0x11, 0x17, 0x37, 0x05, 0xFF,
    0xFF     
};                                                  
                                                    
/* LPC encoded sentence "The battery level is at" */
static const uint8_t BATTERY[] = {
    0xE4, 0xE8, 0x3E, 0x33, 0x23, 0x1D, 0x2B, 0xBF, 0x39, 0x73,
    0xD7, 0xB0, 0x6D, 0x5C, 0x9F, 0xAC, 0xB2, 0x8D, 0x89, 0x71,
    0x5D, 0xB5, 0xAA, 0x74, 0x4C, 0xCC, 0x53, 0xCD, 0x3C, 0xCD,
    0x19, 0x13, 0x1F, 0x97, 0xB4, 0x16, 0x63, 0x82, 0x7C, 0x98,
    0xC3, 0xD3, 0x59, 0x2E, 0xF1, 0x99, 0x4B, 0x0F, 0x37, 0xD9,
    0xCE, 0xF5, 0xC5, 0x3A, 0x5C, 0x55, 0x05, 0x27, 0x34, 0x6F,
    0x4F, 0x71, 0x99, 0x9C, 0xD8, 0xBC, 0x2D, 0x39, 0x75, 0xB0,
    0x63, 0xB7, 0x52, 0x97, 0xD4, 0xC9, 0x4A, 0x59, 0x2B, 0x42,
    0x1C, 0x27, 0x33, 0x65, 0xA9, 0x4C, 0x76, 0x9C, 0x8C, 0x94,
    0x2D, 0xAB, 0xC8, 0x71, 0xD0, 0x62, 0xD6, 0x2A, 0x27, 0x4D,
    0x46, 0x0B, 0xD1, 0x3D, 0x5C, 0xD8, 0x24, 0x2D, 0x5A, 0x0B,
    0x6F, 0x51, 0x92, 0x8C, 0x68, 0x35, 0xA2, 0x4D, 0x8A, 0xB3,
    0x5D, 0xD2, 0xEC, 0x54, 0xBB, 0xC6, 0x35, 0xD1, 0xDA, 0xCD,
    0xA5, 0x1A, 0xDF, 0x04, 0xEF, 0xD2, 0xA0, 0x63, 0x02, 0xE9,
    0xB2, 0xDD, 0x9C, 0x91, 0x0A, 0x45, 0xCA, 0xB6, 0x14, 0xC5,
    0x22, 0x50, 0x36, 0xC7, 0x32, 0x14, 0xB1, 0x88, 0xA7, 0x8C,
    0x70, 0x63, 0x44, 0x62, 0x52, 0xCD, 0x32, 0x4C, 0x96, 0x88,
    0x54, 0x13, 0x4F, 0x67, 0x45, 0x26, 0xF2, 0x43, 0xBA, 0x84,
    0x12, 0xBA, 0x28, 0x36, 0xE9, 0x76, 0x76, 0x15, 0xC2, 0x58,
    0x35, 0xAB, 0xC4, 0x53, 0x08, 0x62, 0xD5, 0xC8, 0x64, 0x8F,
    0x26, 0x08, 0x85, 0xA3, 0xD4, 0x54, 0x89, 0xC0, 0x25, 0xB1,
    0x30, 0x53, 0xA2, 0x5C, 0x9F, 0xD5, 0xC5, 0x53, 0xB6, 0x72,
    0xC2, 0xC6, 0x3A, 0x57, 0x45, 0xCA, 0x09, 0x1B, 0xEB, 0x9C,
    0x6D, 0x2A, 0x27, 0x6C, 0xAC, 0x0B, 0xB2, 0x2D, 0x5C, 0x77,
    0xA8, 0x36, 0xD4, 0xA1, 0xF2, 0xDC, 0xE4, 0xD8, 0x56, 0x57,
    0xEC, 0xF0, 0x83, 0x7D, 0x4A, 0x5D, 0xB0, 0xCE, 0x67, 0x8F,
    0x68, 0x4D, 0x68, 0x7C, 0xEF, 0xA2, 0x63, 0xD8, 0x96, 0xF3,
    0x4D, 0xCC, 0xF1, 0x26, 0xCB, 0xCA, 0x57, 0x25, 0x3A, 0x82,
    0x14, 0x33, 0x9F, 0xE6, 0xA8, 0x30, 0x66, 0xCC, 0x7C, 0xE9,
    0x2A, 0xC3, 0x54, 0xB6, 0xF0, 0xB9, 0xAD, 0x4C, 0x53, 0x5A,
    0xCA, 0x57, 0xD5, 0x3A, 0x55, 0x14, 0x39, 0xDF, 0x27, 0x99,
    0x34, 0x8E, 0x15, 0x82, 0x98, 0xB8, 0x4D, 0xC5, 0x4E, 0x08,
    0x53, 0xD6, 0xF0, 0xA6, 0x58, 0x2E, 0x8A, 0x45, 0xD3, 0x93,
    0x6C, 0xB9, 0x2D, 0x25, 0x4D, 0x4B, 0x72, 0xEC, 0xE2, 0x98,
    0x24, 0xCC, 0xC9, 0x91, 0x8A, 0x43, 0xD4, 0xF0, 0x40, 0x57,
    0xA2, 0x37, 0x49, 0xDD, 0x9C, 0x5C, 0x11, 0xD6, 0x05, 0x31,
    0x57, 0x71, 0x4D, 0x58, 0xEF, 0x4D, 0x5D, 0x55, 0x89, 0xE0,
    0xB2, 0x75, 0x57, 0x75, 0xDB, 0x8C, 0xCB, 0x32, 0x3D, 0x34,
    0x1C, 0x32, 0xA6, 0x09, 0xB3, 0x30, 0x57, 0x44, 0xE8, 0xAE,
    0xC4, 0x22, 0x54, 0x21, 0xEA, 0xAB, 0x37, 0x0E, 0x77, 0xDA,
    0xA8, 0x2C, 0xCA, 0x2D, 0x5C, 0x65, 0x03, 0xFF, 0xFF
};

/* LPC encoded word "ten" */
static const uint8_t TEN[] = {
    0x0A, 0xD8, 0x5C, 0x4D, 0x03, 0x2B, 0xAB, 0x5E, 0xC4, 0x33,
    0x2B, 0xAF, 0x62, 0x84, 0x12, 0x0D, 0x7B, 0xB3, 0xCA, 0x66,
    0x43, 0xA2, 0xE3, 0xF6, 0xAA, 0xAA, 0x4E, 0xC9, 0x89, 0xDB,
    0xAB, 0x6E, 0xBA, 0xC5, 0xDB, 0x66, 0xAF, 0xB9, 0xE8, 0xE6,
    0x4C, 0xBF, 0x3D, 0xE6, 0x6A, 0xC4, 0x4B, 0xCA, 0x49, 0xD9,
    0xBA, 0x61, 0x2B, 0x09, 0x25, 0xED, 0xE8, 0x5A, 0xB4, 0xC4,
    0xED, 0xA6, 0x6B, 0x18, 0xE1, 0x56, 0xB7, 0x9A, 0xAE, 0xA6,
    0x44, 0x47, 0xDC, 0x6E, 0xBE, 0xC2, 0xDD, 0xA5, 0xF0, 0xB8,
    0xD9, 0xFD, 0x7F            
};

/* LPC encoded word "twenty" */
static const uint8_t TWENTY[] = {
    0x0A, 0xE8, 0x4A, 0xCD, 0x01, 0xDB, 0xB9, 0x33, 0xC0, 0xA6,
    0x54, 0x0C, 0xA4, 0x34, 0xD9, 0xF2, 0x0A, 0x6C, 0xBB, 0xB3,
    0x53, 0x0E, 0x5D, 0xA6, 0x25, 0x9B, 0x6F, 0x75, 0xCA, 0x61,
    0x52, 0xDC, 0x74, 0x49, 0xA9, 0x8A, 0xC4, 0x76, 0x4D, 0xD7,
    0xB1, 0x76, 0xC0, 0x55, 0xA6, 0x65, 0xD8, 0x26, 0x99, 0x5C,
    0x56, 0xAD, 0xB9, 0x25, 0x23, 0xD5, 0x7C, 0x32, 0x96, 0xE9,
    0x9B, 0x20, 0x7D, 0xCB, 0x3C, 0xFA, 0x55, 0xAE, 0x99, 0x1A,
    0x30, 0xFC, 0x4B, 0x3C, 0xFF, 0x1F              
};

/* LPC encoded word "thirty" */
static const uint8_t THIRTY[]= {
    0x02, 0x18, 0xA2, 0x52, 0x02, 0x16, 0x60, 0xC0, 0x50, 0x13,
    0x25, 0x6B, 0x2C, 0xC4, 0xDC, 0x52, 0xAF, 0xB2, 0x8B, 0x70,
    0x2A, 0xCD, 0xBA, 0xAA, 0xAA, 0xC7, 0x60, 0xCC, 0xFD, 0xAA,
    0x8B, 0x5D, 0x85, 0x35, 0xED, 0xA3, 0x2B, 0xD3, 0x31, 0x52,
    0xF2, 0x2A, 0xA0, 0x7A, 0xA5, 0x00, 0x2C, 0xED, 0xD2, 0xFA,
    0x9E, 0x8C, 0x45, 0x7D, 0xF5, 0xD8, 0xBA, 0x55, 0xB2, 0xAC,
    0xD5, 0xED, 0xE8, 0xDE, 0x51, 0x2A, 0x57, 0x97, 0xA7, 0x07,
    0x41, 0xAF, 0x5A, 0xEC, 0xB6, 0xEE, 0x19, 0x7D, 0x7A, 0xB1,
    0x9B, 0xBA, 0x23, 0xCC, 0xE9, 0x5A, 0xFF, 0x0F
};

/* LPC encoded word "fourty" */
static const uint8_t FOURTY[]= {
    0x04, 0xC8, 0xCE, 0x8C, 0x01, 0xCB, 0x94, 0x33, 0x60, 0xDA,
    0x0C, 0x01, 0x0C, 0x13, 0xAE, 0x80, 0xEA, 0xD3, 0x4A, 0x1D,
    0xC4, 0xB4, 0x26, 0x39, 0x1E, 0x75, 0xA2, 0xB3, 0x9C, 0xAC,
    0x7E, 0x54, 0x51, 0xEC, 0x52, 0xAA, 0xFA, 0x51, 0x05, 0x73,
    0xC1, 0xE5, 0xDC, 0x47, 0x9D, 0xC2, 0x8A, 0xB5, 0x6A, 0x57,
    0x40, 0xF5, 0x4A, 0x01, 0x58, 0xDA, 0xA5, 0x8D, 0xC3, 0x24,
    0x89, 0x5B, 0xAF, 0xB1, 0xB4, 0x20, 0xE4, 0x5E, 0x8B, 0xC7,
    0xDA, 0x9D, 0xA3, 0x54, 0xBE, 0x4E, 0xF7, 0x8C, 0x41, 0x98,
    0xF1, 0xC6, 0x3D, 0x3D, 0x0A, 0xEA, 0xF4, 0x23, 0xF7, 0x34,
    0x47, 0x9C, 0x53, 0x93, 0xDC, 0xD3, 0x15, 0x63, 0x6D, 0x4D,
    0x32, 0xCF, 0x98, 0x41, 0xA4, 0xB9, 0xF8, 0xFF, 0x01
};

/* LPC encoded word "fifty" */
static const uint8_t FIFTY[]= {
    0x08, 0x68, 0x3A, 0x05, 0x01, 0x5D, 0xA4, 0x12, 0x60, 0xD8,
    0x0A, 0x02, 0x4C, 0x5F, 0x21, 0x80, 0xE9, 0xD2, 0x15, 0xD0,
    0x6C, 0xD9, 0x4A, 0x9B, 0x09, 0x91, 0x94, 0x2D, 0x2B, 0xEF,
    0x2A, 0x95, 0x52, 0x37, 0xAF, 0xA2, 0xD9, 0x52, 0x09, 0xFD,
    0x4C, 0x80, 0x61, 0x2B, 0x08, 0x30, 0x7D, 0x85, 0x00, 0xA6,
    0x4B, 0x47, 0xE0, 0x14, 0xA5, 0x2A, 0x54, 0x03, 0x0A, 0x30,
    0x74, 0xD9, 0xE8, 0xBB, 0x34, 0xF2, 0x8C, 0xD5, 0x63, 0xE9,
    0x56, 0x49, 0x2B, 0x1F, 0xAF, 0x6D, 0x04, 0x27, 0xAA, 0x58,
    0x53, 0xEE, 0x19, 0x93, 0x30, 0xFD, 0x4B, 0xB9, 0x67, 0x70,
    0xE4, 0xAA, 0xD5, 0xEE, 0x6D, 0x96, 0xB1, 0xA6, 0x66, 0x87,
    0xB7, 0x67, 0x22, 0x8F, 0xA9, 0xFD, 0xFF
};

/* LPC encoded word "sixty" */
static const uint8_t SIXTY[]= {
    0x06, 0x78, 0x90, 0xC4, 0x00, 0x3F, 0x66, 0x18, 0xE0, 0xA7,
    0x8C, 0x04, 0x7C, 0x9F, 0x11, 0x80, 0xDF, 0xDA, 0x57, 0xD1,
    0x5D, 0xBB, 0x2B, 0x2D, 0x59, 0x45, 0x6B, 0xA9, 0xC6, 0xB2,
    0xA6, 0x14, 0xAD, 0x9B, 0x18, 0x59, 0x17, 0x04, 0x4E, 0x51,
    0xAA, 0x42, 0x35, 0x40, 0x01, 0x35, 0x8D, 0x17, 0xE0, 0xC4,
    0x9A, 0x02, 0xBC, 0x54, 0x85, 0xC0, 0x29, 0x4A, 0x55, 0xA8,
    0x06, 0x04, 0x60, 0x48, 0x37, 0x07, 0x6C, 0xAE, 0xB6, 0xAA,
    0xA9, 0x53, 0x54, 0xA5, 0xCB, 0xAA, 0xBB, 0x4B, 0x12, 0xB7,
    0x37, 0x6B, 0x18, 0x31, 0x51, 0xDC, 0x5F, 0xAF, 0x79, 0x34,
    0x47, 0xD1, 0x58, 0x3D, 0xD6, 0xD1, 0x14, 0xD5, 0x72, 0x71,
    0xDB, 0x47, 0x51, 0xD4, 0xC8, 0xD7, 0xFF, 0x0F  
};                                            

/* LPC encoded word "seventy" */
static const uint8_t SEVENTY[]= {
    0x0C, 0xF8, 0x29, 0x45, 0x01, 0xBF, 0x95, 0x5A, 0x20, 0x00,
    0xBF, 0xA5, 0x3A, 0xE0, 0x97, 0x32, 0x03, 0xFC, 0x5C, 0xB6,
    0x8A, 0x2E, 0x42, 0xDD, 0x6C, 0xD3, 0x2A, 0x3B, 0x2F, 0xD3,
    0xB4, 0xCD, 0xAB, 0xCA, 0xA6, 0xD4, 0x2B, 0x37, 0xB5, 0x3A,
    0x3A, 0x8F, 0x10, 0x5B, 0x99, 0xFA, 0x6C, 0xC3, 0xCC, 0x78,
    0xD5, 0x6A, 0x8A, 0x0D, 0x0B, 0xF5, 0x47, 0x63, 0xAC, 0x9A,
    0x33, 0xC5, 0x63, 0xA7, 0xB1, 0x2A, 0xEC, 0x50, 0xD7, 0x9C,
    0x96, 0x66, 0xB0, 0x92, 0xC2, 0x95, 0x01, 0x9E, 0x32, 0x2D,
    0xDB, 0x0B, 0xE3, 0xE2, 0xAC, 0xAD, 0xED, 0x4D, 0x39, 0xB9,
    0xD5, 0xC6, 0xB5, 0x0F, 0x13, 0xCC, 0xE1, 0x9B, 0xC7, 0xD1,
    0x9D, 0x93, 0x66, 0x7C, 0x2E, 0xE7, 0x70, 0x49, 0x94, 0xBE,
    0xA5, 0x5C, 0xC3, 0x25, 0x72, 0xC5, 0xA6, 0x72, 0x0F, 0xEF,
    0x28, 0x59, 0xAB, 0xDD, 0x3D, 0xAA, 0x91, 0x58, 0xAD, 0xFA,
    0x7F        
};

/* LPC encoded word "eighty" */
static const uint8_t EIGHTY[] = {
    0x63, 0xEA, 0xD2, 0x28, 0x37, 0x67, 0xAD, 0x7E, 0xF9, 0x26,
    0xAC, 0x58, 0xB3, 0x9A, 0x91, 0x1C, 0x34, 0xB3, 0xC9, 0x68,
    0x46, 0x13, 0x54, 0xEF, 0x25, 0xA3, 0x1E, 0x85, 0xD1, 0x7C,
    0x96, 0x88, 0x21, 0x52, 0xB3, 0x71, 0x4F, 0xA4, 0x80, 0x25,
    0x42, 0x4B, 0x37, 0x82, 0x82, 0xD9, 0x2C, 0x1E, 0xD3, 0x4C,
    0x06, 0xEA, 0xB5, 0x64, 0xAC, 0xCB, 0x3B, 0x50, 0xE5, 0xE2,
    0xB2, 0x2F, 0xEB, 0x40, 0x53, 0x8F, 0xCB, 0x39, 0xBD, 0x21,
    0x75, 0x2D, 0xCE, 0x6F, 0x7A, 0x67, 0x12, 0x90, 0xEC, 0xD9,
    0xFF, 0x0F                                            
};

/* LPC encoded word "ninety" */
static const uint8_t NINETY[]= {
    0x6E, 0x28, 0x1C, 0xE3, 0x38, 0xB4, 0xB8, 0x3E, 0x33, 0xAA,
    0xE5, 0xF0, 0xB2, 0xBA, 0x26, 0xD2, 0x5D, 0x6C, 0xF5, 0xA9,
    0x9A, 0x9C, 0xB0, 0x90, 0x35, 0xA7, 0xAA, 0x76, 0xDC, 0x52,
    0xD6, 0x9C, 0xA2, 0x56, 0xB7, 0x70, 0x9D, 0xB3, 0xB2, 0x56,
    0x4C, 0xC2, 0x7D, 0xCA, 0xCA, 0x7A, 0x11, 0xF1, 0x88, 0x31,
    0xAD, 0xE8, 0x91, 0x34, 0xA2, 0xCA, 0x88, 0xD6, 0x08, 0xAD,
    0xF4, 0x4C, 0x5C, 0x80, 0xA9, 0x52, 0x5A, 0xB7, 0xD2, 0xB8,
    0x30, 0xB6, 0x19, 0xC3, 0x48, 0x4A, 0xE2, 0xB1, 0x64, 0xCD,
    0x33, 0x97, 0xA0, 0xE9, 0x96, 0xB1, 0xCC, 0xD8, 0x02, 0x6E,
    0x5F, 0xDA, 0x3A, 0xD3, 0x30, 0xB8, 0x7F, 0x29, 0xEB, 0x4C,
    0x8D, 0x10, 0xB1, 0xE5, 0xFF, 0x01                    
};

/* LPC encoded word "percent" */
static const uint8_t PERCENT[] = {
    0x02, 0xC8, 0xD9, 0x5C, 0x03, 0x2D, 0x8A, 0xB1, 0x30, 0x46,
    0x52, 0xAF, 0xBA, 0x86, 0x26, 0x1A, 0xF6, 0x77, 0x9B, 0xD3,
    0xD5, 0x18, 0x68, 0x69, 0x59, 0x63, 0xEF, 0x80, 0x5F, 0x5A,
    0x2D, 0x60, 0x01, 0x0B, 0x68, 0xC0, 0x03, 0xAB, 0x6E, 0xDE,
    0x25, 0x2D, 0x17, 0xDF, 0xFA, 0x36, 0xBB, 0x1D, 0x53, 0xB1,
    0x6E, 0x23, 0x5D, 0xA7, 0x5D, 0x23, 0x92, 0xB9, 0xA7, 0x62,
    0x7F, 0x20, 0x50, 0x84, 0x72, 0x17, 0x91, 0x0D, 0x00, 0xA0,
    0x80, 0xA5, 0x33, 0x0C, 0xF0, 0xB3, 0x27, 0x02, 0x5A, 0x4A,
    0xFD, 0x7F                                            
};

/* LPC encoded short pause */
static const uint8_t PAUSE[] = {
    0x00,0x00,0x00,0x00,0xFF,0x0F
};

static const uint8_t welcome[] = { 
    0xCC,0x81,0x4B,0x22,0x52,0x5D,0x2B,0x4B,0x5C,0x8C,
    0x28,0x73,0xE9,0x2C,0xBF,0xD1,0xBB,0x8D,0x67,0xB2,
    0xC2,0x60,0xF3,0x51,0x9E,0xC5,0x0A,0x95,0x3D,0x4B,
    0x38,0x17,0x33,0x34,0xF6,0x0A,0xE1,0x9C,0x8C,0x30,
    0xC8,0x33,0xC4,0x93,0x31,0x42,0x25,0xAB,0x60,0x5F,
    0xC2,0xB0,0x8D,0xAD,0x43,0x55,0x09,0xC3,0x34,0xCA,
    0x36,0x75,0xE5,0x8C,0xBA,0x39,0xAC,0xC4,0xB5,0x93,
    0x5C,0xD0,0x88,0x11,0xC5,0x45,0x32,0x35,0xD3,0x07,
    0x5D,0x17,0xD9,0x0E,0xE9,0x56,0x71,0x15,0x64,0x19,
    0xB9,0x32,0xCD,0xB1,0x53,0x78,0xD6,0xF2,0x32,0xD9,
    0x46,0xA5,0x55,0x3B,0xC2,0x64,0x1B,0x88,0x27,0x2D,
    0x29,0xB3,0x13,0xFA,0x24,0x2C,0xCC,0x5C,0xB1,0x82,
    0x92,0xB4,0xF2,0x30,0xD5,0xC2,0x17,0x3E,0xCB,0xD5,
    0x54,0x0B,0x5F,0xBA,0xE8,0x28,0xA1,0x23,0x7C,0x2B,
    0xBD,0xA2,0x4C,0x36,0xF3,0x95,0xF4,0x72,0x4F,0x59,
    0x24,0x08,0xCA,0xC2,0x2D,0x54,0xB3,0x20,0x0B,0xB5,
    0x90,0xB0,0x23,0xA8,0x2A,0xD5,0x9C,0x5D,0xB6,0xB3,
    0x42,0xD4,0x8C,0x24,0xD5,0xC5,0x4E,0xD1,0xD2,0x8B,
    0x55,0x17,0x3B,0x25,0x4B,0x4F,0x51,0x5C,0x9C,0x94,
    0x2D,0x2A,0x94,0x76,0xB1,0x53,0x91,0x68,0x17,0x25,
    0xC9,0x8A,0x99,0x63,0x54,0x9C,0x24,0x33,0x64,0xB6,
    0x35,0x75,0x92,0x8C,0x54,0xD8,0xD7,0xC4,0x51,0x30,
    0x7C,0x34,0xCD,0x56,0x25,0xC6,0x90,0x59,0x6C,0x52,
    0x1D,0x29,0x43,0x26,0xF6,0x35,0x73,0x2C,0x4C,0x57,
    0xC4,0xCB,0xDC,0xB1,0x72,0x78,0x95,0xD0,0x08,0xA5,
    0xC6,0xE5,0xC9,0x5A,0xDA,0x99,0x28,0x97,0x67,0x19,
    0x2F,0x61,0xED,0x5C,0x93,0xA4,0xB3,0x58,0x55,0xF1,
    0x62,0xD4,0xF2,0x64,0xD5,0xCD,0x8F,0xC1,0xCA,0x5C,
    0xDC,0xB4,0x20,0x06,0x2F,0x75,0x91,0x5B,0xC2,0x50,
    0x3C,0xD5,0xC8,0x76,0x09,0x43,0x89,0x32,0x17,0x39,
    0x29,0x4C,0xD5,0x52,0x93,0xDD,0xA4,0x28,0x0D,0x4D,
    0x71,0x74,0x95,0x82,0x34,0x2C,0x2D,0xC1,0x91,0xEB,
    0x63,0xB7,0xD0,0x42,0x56,0xEA,0x8D,0x55,0x5C,0x9C,
    0x35,0xB1,0x3E,0x16,0x4A,0x67,0x76,0xCC,0xE2,0x90,
    0xD9,0x9D,0x59,0x35,0x61,0x63,0x13,0x73,0x16,0x45,
    0x8A,0x2E,0xC6,0x52,0x23,0x68,0x2A,0xBA,0x18,0x33,
    0x8B,0x86,0xC5,0xBE,0x64,0xDC,0xC5,0x9D,0x11,0xEA,
    0x83,0x0E,0x09,0x36,0xC6,0x88,0xC9,0x3A,0xC4,0xCD,
    0xE9,0x20,0xA6,0x28,0x37,0x11,0x93,0x0D,0xFF,0x0F
};


void sayWelocome()
{
synth_say(welcome);
}

void sayPause()
{
synth_say(PAUSE);
}
/*******************************************************************************
* Function Name: sayHargraveLibrary
********************************************************************************
* Summary:
*    
*******************************************************************************/
void sayHargraveLibrary()
{
    synth_say(HARGRAVE);
}

/*******************************************************************************
* Function Name: sayCampbellHall
********************************************************************************
* Summary:
*    
*******************************************************************************/
void sayCampbellHall()
{
    synth_say(CAMPBELL);
}

/*******************************************************************************
* Function Name: sayCampusCentre
********************************************************************************
* Summary:
*    
*******************************************************************************/
void sayCampusCentre()
{
    synth_say(CENTRE);
}

/*******************************************************************************
* Function Name: sayArrived
********************************************************************************
* Summary:
*    
*******************************************************************************/
void sayArrived()
{
    synth_say(ARRIVED);
}

/*******************************************************************************
* Function Name: sayFix
********************************************************************************
* Summary:
*    
*******************************************************************************/
void sayFix()
{
    synth_say(FIX);
}

/*******************************************************************************
* Function Name: sayBatteryLevel
********************************************************************************
* Summary:
*    
*******************************************************************************/
void sayBatteryLevel()
{
    synth_say(BATTERY);
}

/*******************************************************************************
* Function Name: sayBatteryPercent
********************************************************************************
* Summary:
*    
*******************************************************************************/
void sayBatteryPercent(int number)
{
    switch(number)
    {   
        case 10:
        synth_say(TEN);
        break;
        case 20:
        synth_say(TWENTY);
        break;
        case 30:
        synth_say(THIRTY);
        break;
        case 40:
        synth_say(FOURTY);
        break;
        case 50:
        synth_say(FIFTY);
        break;
        case 60:
        synth_say(SIXTY);
        break;
        case 70:
        synth_say(SEVENTY);
        break;
        case 80:
        synth_say(EIGHTY);
        break;
        case 90:
        synth_say(NINETY); 
        break;
        default:
        /* error */
        break;
    }
    synth_say(PERCENT);
    return;
}

/* [] END OF FILE */
