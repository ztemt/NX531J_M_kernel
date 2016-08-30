/*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __PI5U_H__
#define __PI5U_H__

typedef unsigned char BYTE;

enum pi5u_type{
	PI5U_TYPE_UFP = 0,
	PI5U_TYPE_DFP = 1,
	PI5U_TYPE_DRP = 2,
};


struct pi5u_platform_data{
	void (*audio_cb)(bool attach);
	void (*debug_cb)(bool attach);
	void (*power_acc_cb)(bool attach);
	void (*source_cb)(bool attach, int bc_lvl);	
	void (*sink_cb)(bool attach);
};


#endif

