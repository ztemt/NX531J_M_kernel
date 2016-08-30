/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MFD_AK4376_PDATA_H__

#define __MFD_AK4376_PDATA_H__

enum {
	PLL_OFF,
	PLL_BICK_MODE,
	PLL_MCKI_MODE,
	XTAL_MODE,
};

enum {
	FS_32,
	FS_48,
	FS_64,
};

enum {
	PDN_OFF,
	PDN_ON,
};

enum {
    PLL_CLK_9P6MHZ,
    PLL_CLK_11P289MHZ,
    PLL_CLK_12P288MHZ,
    PLL_CLK_19P2MHZ,
};


struct ak4376_platform_data {
	int nPllMode;	//0:PLL OFF, 1:PLL_BICK (Slave), 2:PLL_MCKI (Master), 3:PLL_XTAL (Master)
	unsigned int pdn_en;		//set GPIO pin from AP
};

#endif
