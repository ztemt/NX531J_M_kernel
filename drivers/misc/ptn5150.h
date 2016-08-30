/*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __PTN5150_H__
#define __PTN5150_H__

typedef unsigned char BYTE;

enum ptn5150_type{
	PTN5150_TYPE_UFP = 0,
	PTN5150_TYPE_DFP = 2,
	PTN5150_TYPE_DRP = 4,
};


struct ptn5150_platform_data{
	void (*audio_cb)(bool attach);
	void (*debug_cb)(bool attach);
	void (*power_acc_cb)(bool attach);
	void (*source_cb)(bool attach, int bc_lvl);	
	void (*sink_cb)(bool attach);
};


#endif

