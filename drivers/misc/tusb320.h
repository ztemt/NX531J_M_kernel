/*

 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __TUSB320__
#define __TUSB320__

typedef unsigned char BYTE;

enum tusb320_type{
	TUSB320_TYPE_DEFAULT = 0,
	TUSB320_TYPE_UFP = 1,
	TUSB320_TYPE_DFP = 2,
	TUSB320_TYPE_DRP = 3,
};


struct tusb320_platform_data{
	void (*audio_cb)(bool attach);
	void (*debug_cb)(bool attach);
	void (*power_acc_cb)(bool attach);
	void (*source_cb)(bool attach, int bc_lvl);	
	void (*sink_cb)(bool attach);
};


#endif

