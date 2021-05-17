/*******************************************************************************
* Written by Jaroslav Groman, for PSoC Analog Coprocessor
* https://www.hackster.io/jardag/touch-controlled-talking-clock-for-psoc-analog-coprocessor-98c5a0
*
* Modified by Maanika Kenneth Koththioda, for use on PSoC5LP
* Last Modified on 25/10/2020
*
* File:     lpc_synth.c
* Version:  1.0.0
*
* Brief: Project independent LPC audio decoder implementation.
*
* Target device:
*    CY8C5888LTI - LP097
*
* Code Tested With:
*    - Silicon: PSoC 5LP
*    - IDE: PSoC Creator 4.3
*    - Compiler: GCC 5.4
*
* Components:
*    - cy_clock       [Clock_Synth]
*    - TCPWM_P4       [Timer_Synth]
*    - cy_isr         [isr_Synth]
*    - UAB_VDAC       [VDAC_Synth]
*    - cy_pins        [Pin_Synth_Out]
*
* Source:
*    LPC synth code adapted from https://github.com/going-digital/Talkie
*
*******************************************************************************/
#include "lpc_synth.h"

/*******************************************************************************
*   Private Function Declarations
*******************************************************************************/

// Brief: Read LPC coefficients from bitstream.
// Return: none
static void synth_core(void);

// Brief: Reverse bit order (LSB/MSB) in given byte.
// Param: byte Byte to reverse bits from.
// Return: uint8_t Byte with bit order reversed.
static uint8_t reverse_bit_order(uint8_t byte);

// Brief: Read given number of bits from LPC bitstream.
// Param: bit_count Number of bits to read.
// Return: uint8_t Binary value of read bits.
static uint8_t get_bits(uint8_t bit_count);

// Brief: Generate single LPC audio sample based on global coefficients.
// Return: int16_t Generated sample value.
static int16_t generate_sample(void);


/*******************************************************************************
*   Constant definitions
*******************************************************************************/
/* LPC 'chirp' sample array */
static const int8_t CHIRP[CHIRP_SIZE] =
{
    0x00, 0x2A, 0xD4, 0x32, 0xB2, 0x12, 0x25, 0x14,
    0x02, 0xE1, 0xC5, 0x02, 0x5F, 0x5A, 0x05, 0x0F,
    0x26, 0xFC, 0xA5, 0xA5, 0xD6, 0xDD, 0xDC, 0xFC,
    0x25, 0x2B, 0x22, 0x21, 0x0F, 0xFF, 0xF8, 0xEE,
    0xED, 0xEF, 0xF7, 0xF6, 0xFA, 0x00, 0x03, 0x02,
    0x01
};

/* LPC energy lookup array */
static const uint8_t ENERGY[0x10] =
{
    0x00, 0x02, 0x03, 0x04, 0x05, 0x07, 0x0A, 0x0F,
    0x14, 0x20, 0x29, 0x39, 0x51, 0x72, 0xA1, 0xFF
};

/* LPC pitch lookup array */
static const uint8_t PITCH[0x40] =
{
    0x00, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
    0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
    0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26,
    0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2D, 0x2F, 0x31,
    0x33, 0x35, 0x36, 0x39, 0x3B, 0x3D, 0x3F, 0x42,
    0x45, 0x47, 0x49, 0x4D, 0x4F, 0x51, 0x55, 0x57,
    0x5C, 0x5F, 0x63, 0x66, 0x6A, 0x6E, 0x73, 0x77,
    0x7B, 0x80, 0x85, 0x8A, 0x8F, 0x95, 0x9A, 0xA0
};

/* LPC coefficient K1 lookup array */
static const int16_t K1[0x20] =
{
    0x82C0, 0x8380, 0x83C0, 0x8440, 0x84C0, 0x8540, 0x8600, 0x8780,
    0x8880, 0x8980, 0x8AC0, 0x8C00, 0x8D40, 0x8F00, 0x90C0, 0x92C0,
    0x9900, 0xA140, 0xAB80, 0xB840, 0xC740, 0xD8C0, 0xEBC0, 0x0000,
    0x1440, 0x2740, 0x38C0, 0x47C0, 0x5480, 0x5EC0, 0x6700, 0x6D40
};

/* LPC coefficient K2 lookup array */
static const int16_t K2[0x20] =
{
    0xAE00, 0xB480, 0xBB80, 0xC340, 0xCB80, 0xD440, 0xDDC0, 0xE780,
    0xF180, 0xFBC0, 0x0600, 0x1040, 0x1A40, 0x2400, 0x2D40, 0x3600,
    0x3E40, 0x45C0, 0x4CC0, 0x5300, 0x5880, 0x5DC0, 0x6240, 0x6640,
    0x69C0, 0x6CC0, 0x6F80, 0x71C0, 0x73C0, 0x7580, 0x7700, 0x7E80
};

/* LPC coefficient K3 lookup array */
static const int8_t K3[0x10] =
{
    0x92, 0x9F, 0xAD, 0xBA, 0xC8, 0xD5, 0xE3, 0xF0,
    0xFE, 0x0B, 0x19, 0x26, 0x34, 0x41, 0x4F, 0x5C
};

/* LPC coefficient K4 lookup array */
static const int8_t K4[0x10] =
{
    0xAE, 0xBC, 0xCA, 0xD8, 0xE6, 0xF4, 0x01, 0x0F,
    0x1D, 0x2B, 0x39, 0x47, 0x55, 0x63, 0x71, 0x7E
};

/* LPC coefficient K5 lookup array */
static const int8_t K5[0x10] =
{
    0xAE, 0xBA, 0xC5, 0xD1, 0xDD, 0xE8, 0xF4, 0xFF,
    0x0B, 0x17, 0x22, 0x2E, 0x39, 0x45, 0x51, 0x5C
};

/* LPC coefficient K6 lookup array */
static const int8_t K6[0x10] =
{
    0xC0, 0xCB, 0xD6, 0xE1, 0xEC, 0xF7, 0x03, 0x0E,
    0x19, 0x24, 0x2F, 0x3A, 0x45, 0x50, 0x5B, 0x66
};

/* LPC coefficient K7 lookup array */
static const int8_t K7[0x10] =
{
    0xB3, 0xBF, 0xCB, 0xD7, 0xE3, 0xEF, 0xFB, 0x07,
    0x13, 0x1F, 0x2B, 0x37, 0x43, 0x4F, 0x5A, 0x66
};

/* LPC coefficient K8 lookup array */
static const int8_t K8[0x08] =
{
    0xC0, 0xD8, 0xF0, 0x07, 0x1F, 0x37, 0x4F, 0x66
};

/* LPC coefficient K9 lookup array */
static const int8_t K9[0x08] =
{
    0xC0, 0xD4, 0xE8, 0xFC, 0x10, 0x25, 0x39, 0x4D
};

/* LPC coefficient K10 lookup array */
static const int8_t K10[0x08] =
{
    0xCD, 0xDF, 0xF1, 0x04, 0x16, 0x20, 0x3B, 0x4D
};

/*******************************************************************************
*   Variable definitions
*******************************************************************************/

/* LPC bitstream byte pointer */
static uint8_t *gp_byte;

/* LPC bitstream bit pointer */
static uint8_t  g_bit_pointer;

/* LPC bitstream current read byte */
static uint8_t  g_byte_1;

/* LPC bitstream next read byte */
static uint8_t  g_byte_2;

/* Boolean flag whether FRAM is being used as LPC bitstream source */
static uint8_t  gb_using_fram;

/* LPC synthesizer current pitch */
static uint8_t  g_synth_pitch;

/* LPC synthesizer current energy */
static uint16_t g_synth_energy;

/* LPC synthesizer current coefficient k1 */
static int32_t  g_synth_k1;

/* LPC synthesizer current coefficient k2 */
static int32_t  g_synth_k2;

/* LPC synthesizer current coefficient k3 */
static int16_t  g_synth_k3;

/* LPC synthesizer current coefficient k4 */
static int16_t  g_synth_k4;

/* LPC synthesizer current coefficient k5 */
static int16_t  g_synth_k5;

/* LPC synthesizer current coefficient k6 */
static int16_t  g_synth_k6;

/* LPC synthesizer current coefficient k7 */
static int16_t  g_synth_k7;

/* LPC synthesizer current coefficient k8 */
static int16_t  g_synth_k8;

/* LPC synthesizer current coefficient k9 */
static int16_t  g_synth_k9;

/* LPC synthesizer current coefficient k10 */
static int16_t  g_synth_k10;


/*******************************************************************************
* Function Name: synth_hw_init
********************************************************************************
* @par Summary
*    Initializes all involved hardware components. Configures timer and ISR.
*******************************************************************************/
void synthInitialize(void)
{
    /* Start components */
    VDAC_Synth_Start();
    Timer_Synth_Init();
    Opamp_Synth_Start();
    
    /* Timer generates interrupts at sampling rate */
    Timer_Synth_WritePeriod((CLOCK_SYNTH / SAMPLE_RATE) - 1);

    /* Attach ISR */
    isr_Synth_StartEx(synth_isr);

}

/*******************************************************************************
* Function Name: synth_say
****************************************************************************//**
* @par Summary
*    Initializes reading of LPC bitstream from flash memory and starts audio
* generator.
*******************************************************************************/
void synth_say(const uint8_t *p_lpc_data)
{
	/* Initialize pointers to LPC data bitstream */
    gp_byte = (uint8_t *)p_lpc_data;
	g_bit_pointer = 0u;
    gb_using_fram = 0u;
    
    /* Read the first and the second byte from bitstream */
    g_byte_1 = reverse_bit_order(*gp_byte);
    g_byte_2 = reverse_bit_order(*(gp_byte + 1));

    synth_core();
}

/*******************************************************************************
* Function Name: synth_core
****************************************************************************//**
* @par Summary
*    Reads LPC coefficient indexes from bitstream, looks up coefficient values
* and updates global current coefficients for DMA/ISR audio sample generator.
*******************************************************************************/
static void synth_core()
{
   /* Enable hardware */
    Timer_Synth_Start();

    /* Read LPC data bitstream until end frame */
    uint8_t energy;
    uint8_t b_repeat_flag;
	do
    {
		energy = get_bits(4);

		if (0 == energy)
        {
			/* Energy = 0: silent frame */
			g_synth_energy = 0;
		}

        else if (0xF == energy)
        {
			/* Energy = 15: end frame, stop synthesizer */
			g_synth_energy = 0;
			g_synth_k1     = 0;
			g_synth_k2     = 0;
			g_synth_k3     = 0;
			g_synth_k4     = 0;
			g_synth_k5     = 0;
			g_synth_k6     = 0;
			g_synth_k7     = 0;
			g_synth_k8     = 0;
			g_synth_k9     = 0;
			g_synth_k10    = 0;
		}

        else
        {
			b_repeat_flag = get_bits(1);

            g_synth_energy = ENERGY[energy];
			g_synth_pitch  = PITCH[get_bits(6)];

			/* A repeat frame would reuse previous coefficients */
            /* otherwise read filter parameter values from input data */
			if (!b_repeat_flag)
            {
				/* All frames use the first 4 coefficients */
				g_synth_k1 = K1[get_bits(5)];
				g_synth_k2 = K2[get_bits(5)];
				g_synth_k3 = K3[get_bits(4)];
				g_synth_k4 = K4[get_bits(4)];

				if (g_synth_pitch)
                {
					/* Voiced frames use 6 extra coefficients. */
					g_synth_k5  = K5[get_bits(4)];
					g_synth_k6  = K6[get_bits(4)];
					g_synth_k7  = K7[get_bits(4)];
					g_synth_k8  = K8[get_bits(3)];
					g_synth_k9  = K9[get_bits(3)];
					g_synth_k10 = K10[get_bits(3)];
				}
			}
		}
        
        /* Generate 25 ms (23 + 2) worth of audio frames via ISR/DMA */
        /* It takes approx 2 ms to update coefficients */
        CyDelay(23);
	}
    while (0xF != energy);
    
    Timer_Synth_Stop();
}

/*******************************************************************************
* Function Name: synth_isr
****************************************************************************//**
* @par Summary
*    Synth ISR routine generates audio samples at defined sample rate.
*******************************************************************************/
CY_ISR(synth_isr)
{
    /* Output clamp */
    VDAC_Synth_SetValue(generate_sample());

    /* Clear the interrupt (enable) */
   Timer_Synth_ReadStatusRegister();   
}

/*******************************************************************************
* Function Name: reverse_bit_order
****************************************************************************//**
* @par Summary
*    Reverses bit order of given byte from LSB to MSB for easier processing.
*******************************************************************************/
static uint8_t reverse_bit_order(uint8_t byte)
{
	/* 76543210 */
	byte = (byte >> 4) | (byte << 4); // Swap in groups of 4
	/* 32107654 */
	byte = ((byte & 0xCC) >> 2) | ((byte & 0x33) << 2); // Swap in groups of 2
	/* 10325476 */
	byte = ((byte & 0xAA) >> 1) | ((byte & 0x55) << 1); // Swap bit pairs
	/* 01234567 */
	return byte;
}

/*******************************************************************************
* Function Name: get_bits
****************************************************************************//**
* @par Summary
*    Reads given number of bits from current source byte. If there's not enough
* bits available, more bits are added from next bitstream source byte.
*******************************************************************************/
static uint8_t get_bits(uint8_t bit_count)
{
	uint16_t data = (g_byte_1 << 8) | g_byte_2;
	data = data << g_bit_pointer;
	uint8_t value = data >> (16 - bit_count);

    g_bit_pointer = g_bit_pointer + bit_count;
	if (8 <= g_bit_pointer)
    {
		g_bit_pointer = g_bit_pointer - 8;
        g_byte_1 = g_byte_2;
        
        if (!gb_using_fram)
        {
    		gp_byte++;
            g_byte_2 = reverse_bit_order(*(gp_byte + 1));
        }
	}

	return value;
}

/*******************************************************************************
* Function Name: generate_sample
****************************************************************************//**
* @par Summary
*    LPC decoder procedure. Calculates audio sample value from current global
* LPC coefficients. Resulting 9-bit value is bitshifted and saturated in order
* to fully use 13-bit VDAC.
*******************************************************************************/
static int16_t generate_sample(void)
{    
    static uint8_t period_counter;

    static int16_t x0;
    static int16_t x1;
    static int16_t x2;
    static int16_t x3;
    static int16_t x4;
    static int16_t x5;
    static int16_t x6;
    static int16_t x7;
    static int16_t x8;
    static int16_t x9;
    
    int16_t u0;
    int16_t u1;
    int16_t u2;
    int16_t u3;
    int16_t u4;
    int16_t u5;
    int16_t u6;
    int16_t u7;
    int16_t u8;
    int16_t u9;
    int16_t u10;

    if (g_synth_pitch)
    {
        /* Voiced source */
        if (period_counter < g_synth_pitch)
        {
            period_counter++;
        }
        else
        {
            period_counter = 0;
        }

        if (period_counter < CHIRP_SIZE)
        {
            u10 = ((CHIRP[period_counter]) * (uint32_t)g_synth_energy) >> 8;
        }
        else
        {
            u10 = 0;
        }
    }
    else
    {
        /* Unvoiced source */
        static uint16_t synth_rand = 1;
        synth_rand = (synth_rand >> 1) ^ ((synth_rand & 1) ? 0xB800 : 0);
        u10 = (synth_rand & 1) ? g_synth_energy : -g_synth_energy;
    }

    /* Lattice filter forward path */
    u9 = u10 - ((g_synth_k10 * x9) >> 7);
    u8 = u9  - ((g_synth_k9  * x8) >> 7);
    u7 = u8  - ((g_synth_k8  * x7) >> 7);
    u6 = u7  - ((g_synth_k7  * x6) >> 7);
    u5 = u6  - ((g_synth_k6  * x5) >> 7);
    u4 = u5  - ((g_synth_k5  * x4) >> 7);
    u3 = u4  - ((g_synth_k4  * x3) >> 7);
    u2 = u3  - ((g_synth_k3  * x2) >> 7);
    u1 = u2  - ((g_synth_k2  * x1) >> 15);
    u0 = u1  - ((g_synth_k1  * x0) >> 15);
    
    /* Lattice filter reverse path */
    x9 = x8 + ((g_synth_k9 * u8) >> 7);
    x8 = x7 + ((g_synth_k8 * u7) >> 7);
    x7 = x6 + ((g_synth_k7 * u6) >> 7);
    x6 = x5 + ((g_synth_k6 * u5) >> 7);
    x5 = x4 + ((g_synth_k5 * u4) >> 7);
    x4 = x3 + ((g_synth_k4 * u3) >> 7);
    x3 = x2 + ((g_synth_k3 * u2) >> 7);
    x2 = x1 + ((g_synth_k2 * u1) >> 15);
    x1 = x0 + ((g_synth_k1 * u0) >> 15);
    x0 = u0;
    
    uint16_t sample_out = u0 + 0b100000000; // temporary variable
    
    return sample_out;
}

/* [] END OF FILE */
