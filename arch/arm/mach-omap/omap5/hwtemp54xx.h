/*
 *
 * @Component			OMAPCONF
 * @Filename			hwtemp54xx.h
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


#ifndef __HWTEMP54XX_H__
#define __HWTEMP54XX_H__


#include <stdio.h>
#include <voltdm54xx.h>


typedef enum {
	HWTEMP54XX_MPU,
	HWTEMP54XX_HOTSPOT_MPU,
	HWTEMP54XX_GPU,
	HWTEMP54XX_HOTSPOT_GPU,
	HWTEMP54XX_CORE,
	HWTEMP54XX_EMIF1,
	HWTEMP54XX_EMIF2,
	HWTEMP54XX_PCB,
	HWTEMP54XX_CASE,
	HWTEMP54XX_CHARGER,
	HWTEMP54XX_ID_MAX
} hwtemp54xx_sensor_id;


/**///FIXME make sure this is only executed on ES2.0 54xx and not any other version because ADC_TABLE_BEGIN is version indepedent
#define ADC_TABLE_BEGIN 540
#define ADC_TABLE_END 945


const char *hwtemp54xx_name_get(hwtemp54xx_sensor_id id);
/*hwtemp54xx_sensor_id voltdm2sensor_id(voltdm54xx_id vdd_id);
 * based on the temp54xx template*/
int hwtemp54xx_get(hwtemp54xx_sensor_id id);


#endif
