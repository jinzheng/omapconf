/*
 *
 * @Component			OMAPCONF
 * @Filename			temp54xx.c
 * @Description			OMAP5 Temperature Sensors Functions
 * @Author			Patrick Titiano (p-titiano@ti.com)
 * @Date			2011
 * @Copyright			Texas Instruments Incorporated
 *
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <temp54xx.h>
#include <hwtemp54xx.h>
#include <adctable.h>
#include <ctrlmod_core54xx-defs.h>
#include <cpuinfo.h>
#include <lib.h>
#include <autoadjust_table.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <emif.h>

/* #define TEMP54XX_DEBUG */
#ifdef TEMP54XX_DEBUG
#define dprintf(format, ...)	 printf(format, ## __VA_ARGS__)
#else
#define dprintf(format, ...)
#endif



/* ------------------------------------------------------------------------*//**
 * @FUNCTION		hwtemp54xx_get
 * @BRIEF		return temperature measured by selected sensor's register buffer
 *			(in degrees celcius)
 * @RETURNS		measured temperature in case of success
 *			TEMP_ABSOLUTE_ZERO (-273) in case of error
 * @param[in]		id: ADC temperature sensor id
 * @param[in, out]	temp: temperature (Celcius, min) (returned)
 * @DESCRIPTION		return temperature measured by selected sensor
 *			(in degrees celcius)
 *//*------------------------------------------------------------------------ */
#define OMAP54_DTEMP_REG(dom,reg_id)  &omap5430_control_dtemp_##dom##_##reg_id
int hwtemp54xx_get(hwtemp54xx_sensor_id id)
{

	/* Initialize register pointer */
	reg *temperature_register_address[5];

	/* Use id to choose which register to read from */
	switch (id)
	{
	case HWTEMP54XX_MPU:
		/*			temperature_register_address = &omap5430_control_dtemp_mpu_0;  */
		temperature_register_address[4] = OMAP54_DTEMP_REG(mpu, 4);
		temperature_register_address[3] = OMAP54_DTEMP_REG(mpu, 3);
		temperature_register_address[2] = OMAP54_DTEMP_REG(mpu, 2);
		temperature_register_address[1] = OMAP54_DTEMP_REG(mpu, 1);
		temperature_register_address[0] = OMAP54_DTEMP_REG(mpu, 0);
		break;
	case HWTEMP54XX_GPU:
		/*	temperature_register_address = &omap5430_control_dtemp_mm_0;  */
		temperature_register_address[4] = OMAP54_DTEMP_REG(mm, 4);
		temperature_register_address[3] = OMAP54_DTEMP_REG(mm, 3);
		temperature_register_address[2] = OMAP54_DTEMP_REG(mm, 2);
		temperature_register_address[1] = OMAP54_DTEMP_REG(mm, 1);
		temperature_register_address[0] = OMAP54_DTEMP_REG(mm, 0);
		break;
	case HWTEMP54XX_CORE:
		/*			temperature_register_address = &omap5430_control_dtemp_core_0;  */
		temperature_register_address[4] = OMAP54_DTEMP_REG(core, 4);
		temperature_register_address[3] = OMAP54_DTEMP_REG(core, 3);
		temperature_register_address[2] = OMAP54_DTEMP_REG(core, 2);
		temperature_register_address[1] = OMAP54_DTEMP_REG(core, 1);
		temperature_register_address[0] = OMAP54_DTEMP_REG(core, 0);
		break;
	default:
		/*debug output*/
		printf("ran through the default of hwtemp54xx_get");
		break;
	}/* END of  Use id to choose which register to read from */

	int i;
	unsigned int tempFIFOsum = 0;
	/*for loop reads all of the ADC code from the FIFO temperature buffer
	 * and stores the sum in 'adc'
	 */
	for (i = 4; i >= 0; i--)
	{
		/*/read the register and return the register
		only the lower 10 bits are valid */
		tempFIFOsum += 0x3ff & reg_read(temperature_register_address[i]);

		/*
		 * Formula equivalent to the table
		 *calculated = -0.0000313338516389067 * adc * adc
				+ 0.460098087416116 * adc - 280.964285567881;
				*/
	} /*End of for loop that adds 5 ADC values */
	return readADCtable(tempFIFOsum/5);
}

/* ------------------------------------------------------------------------*//**
 * @FUNCTION		readADCtable
 * @BRIEF		convert ADC code from ADC code versus temperature  table
 * @RETURNS		temperature (int)
 *			-999 in case of error
 * @param[in]		ADC code as read from the register
 * @DESCRIPTION		convert ADC code from ADC code versus temperature  table.
 *//*------------------------------------------------------------------------ */
int readADCtable(unsigned int adcValueReadFromRegister)
{
	return numericTemperature[adcValueReadFromRegister-ADC_TABLE_BEGIN];
}


