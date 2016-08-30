#ifndef __NUBIA_FUELGAUGE_CNTL_H__
#define __NUBIA_FUELGAUGE_CNTL_H__

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/wakelock.h>
#include <linux/delay.h>

struct nubia_fg_params {
	int 	usb_in;
	int     batt_temp;
	int     chg_status;
	int     batt_ma;
	int     batt_mv;
	int     new_soc;
	int     fg_soc;
	int     batt_soc;
	int     batt_status;
	int     new_batt_status;
	int     batt_health;
	int     new_batt_health;
	int     chg_vol;
	int     soc_chg_done;
};


struct nubia_fg_cntl {
	struct device		*dev;
	void   *fg_pri_data;
	
	/* psy */
	struct power_supply	    *usb_psy;
	struct power_supply	    *batt_psy;
	
	struct delayed_work		batt_worker;
	struct nubia_fg_params  batt_data;
	int resume_status;
	int usbin_in_suspend;
	int low_vol_thrhld;
	struct wake_lock *soc_wlock;

	int (*nubia_update_fg_data)(struct nubia_fg_cntl * );
	int (*nubia_print_info)(struct nubia_fg_cntl * );
	int (*nubia_get_soc)(struct nubia_fg_cntl * ,int *);
	int  support_backup;
	int (*nubia_backup_soc)(struct nubia_fg_cntl * );
};

int nubia_batt_cntl_init(struct nubia_fg_cntl *batt_cntl);
int nubia_batt_cntl_deinit(struct nubia_fg_cntl *batt_cntl);
int nubia_batt_suspend(struct nubia_fg_cntl *batt_cntl);
int nubia_batt_resume(struct nubia_fg_cntl *batt_cntl);
int nubia_report_batt_capacity(struct nubia_fg_cntl *batt_cntl);

#endif

