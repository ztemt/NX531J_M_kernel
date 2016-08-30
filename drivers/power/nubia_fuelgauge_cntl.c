/* Copyright (c) 2015 The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "NXDG:%s: " fmt, __func__

#include "nubia_fuelgauge_cntl.h"

enum nx_resume_status {
	NX_RESUME_IDLE_STAT = 0,
	NX_RESUME_WAKE_STAT,
	NX_RESUME_PROP_STAT,
};

#define NXFG_DEFAULT_SOC    100
#define NXFG_FULL_SOC    100
#define NXFG_CUTOFF_VOLTAGE    3400
#define NXFG_EMPTY_VOLTAGE    3100
#define NXFG_FULL_VOLTAGE    4400
#define NXFG_BATT_MAX_VOLTAGE    4500


#define FG_INFO 1
#define FG_DEBUG 4
//log level < bqfg_log_level will show
static int nxfg_log_level = 3;  
module_param(nxfg_log_level, int, 0644);

#define NXFG_DEBUG(x...) do {if (FG_DEBUG < nxfg_log_level) pr_info(x); } while (0)
#define NXFG_INFO(x...)  do {if (FG_INFO  < nxfg_log_level) pr_info(x); } while (0)

static int nubia_is_resume_soc_invalid(struct nubia_fg_cntl *batt_cntl, int new_soc)
{
    int soc_invalid = 0;

    //if  the resume soc > suspend soc when there is no charger, the new soc is invalid
	if(!batt_cntl->usbin_in_suspend && (new_soc > batt_cntl->batt_data.batt_soc))
		soc_invalid = 1;

	NXFG_INFO("soc_invalid=%d sus_usb=%d\n",soc_invalid,batt_cntl->usbin_in_suspend);

	return soc_invalid;
}

static bool nubia_is_charger_present(struct nubia_fg_cntl *batt_cntl)
{
	union power_supply_propval ret = {0,};

	if (batt_cntl->usb_psy == NULL)
		batt_cntl->usb_psy = power_supply_get_by_name("usb");
	
	if (batt_cntl->usb_psy) {
		batt_cntl->usb_psy->get_property(batt_cntl->usb_psy,
					POWER_SUPPLY_PROP_PRESENT, &ret);
		return ret.intval;
	}

	return false;
}

void nubia_update_power_supply(struct nubia_fg_cntl *batt_cntl)
{
#ifdef  CONFIG_ZTEMT_CHARGER
	if (batt_cntl->usb_psy == NULL)
		batt_cntl->usb_psy = power_supply_get_by_name("usb");

	if (batt_cntl->usb_psy)
		power_supply_changed(batt_cntl->usb_psy);
#else
	if (batt_cntl->batt_psy == NULL || batt_cntl->batt_psy < 0)
		batt_cntl->batt_psy = power_supply_get_by_name("battery");

	if (batt_cntl->batt_psy > 0)
		power_supply_changed(batt_cntl->batt_psy);
#endif
}

static int  nubia_update_batt_params(struct nubia_fg_cntl *batt_cntl)
{
    union power_supply_propval ret = {0,};
	struct nubia_fg_params *batt_param = &batt_cntl->batt_data;
	int fg_batt_soc;
	int rc;

	rc = batt_cntl->nubia_get_soc(batt_cntl, &fg_batt_soc);
	if(rc >= 0){
		batt_param->new_soc = fg_batt_soc;
		batt_param->fg_soc = fg_batt_soc;
	}
		
	if (batt_cntl->batt_psy == NULL || batt_cntl->batt_psy < 0)
		batt_cntl->batt_psy = power_supply_get_by_name("battery");

	if(!batt_cntl->batt_psy)
		return -EINVAL;

    batt_cntl->batt_psy->get_property(batt_cntl->batt_psy,POWER_SUPPLY_PROP_CURRENT_NOW,&ret);
    batt_param->batt_ma = ret.intval/1000;

    batt_cntl->batt_psy->get_property(batt_cntl->batt_psy,POWER_SUPPLY_PROP_VOLTAGE_NOW,&ret);
    batt_param->batt_mv = ret.intval/1000;
   
    batt_cntl->batt_psy->get_property(batt_cntl->batt_psy,POWER_SUPPLY_PROP_STATUS,&ret);
    batt_param->new_batt_status = ret.intval;

    batt_cntl->batt_psy->get_property(batt_cntl->batt_psy,POWER_SUPPLY_PROP_HEALTH,&ret);
    batt_param->new_batt_health = ret.intval;

    batt_cntl->batt_psy->get_property(batt_cntl->batt_psy,POWER_SUPPLY_PROP_TEMP,&ret);
	batt_param->batt_temp = ret.intval;
	
	batt_param->usb_in = nubia_is_charger_present(batt_cntl);

	return 0;
}

static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(100, soc);
	return soc;
}

//!!!To check if the discharging current > hrd_ma
static int batt_is_dischging(struct nubia_fg_params *pbatt, int hrd_ma)
{
    /************************************************************
       * !!!--  Note  --!!!
       *   batt_ma > 0,means it is discharging, batt_ma <= 0,means it is charging
       *   the discharging current's +/- maybe different  for different projects
       *************************************************************/
       
    return pbatt->batt_ma > hrd_ma ? 1 : 0;
}

#define BATT_LOW_MONITOR_MS	    10000
#define BATT_NORMAL_MONITOR_MS	20000
#define BATT_SLOW_MONITOR_MS	60000
static int nubia_get_delay_time(struct nubia_fg_params *pbatt)
{
   if(pbatt->batt_status == POWER_SUPPLY_STATUS_CHARGING)
	   return BATT_NORMAL_MONITOR_MS;

    if(pbatt->batt_soc > 90)
		return BATT_SLOW_MONITOR_MS;
    else if(pbatt->batt_soc > 20)
		return BATT_NORMAL_MONITOR_MS;
	else 
		return BATT_LOW_MONITOR_MS;
}

static void nubia_batt_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct nubia_fg_cntl *batt_cntl = container_of(dwork, struct nubia_fg_cntl, batt_worker);
	struct nubia_fg_params *pbatt = &batt_cntl->batt_data;
	int power_supply_change = 0;
	int last_batt_soc;
	static int batt_low_count = 0;
	int rc;
	
    //update batt data
    last_batt_soc = pbatt->batt_soc;
	rc = nubia_update_batt_params(batt_cntl);
	if(rc < 0){
		pr_err("update batt params fail\n");
		goto NEXT;
	}

	//after charging full for a long time, soc < 100 and charger is online, so set soc to 100%
	if( pbatt->usb_in && last_batt_soc==100 && (pbatt->new_soc==99 || pbatt->new_soc==98) ){	
		if(!batt_is_dischging(pbatt,10)){
			pbatt->new_soc = 100;
			NXFG_INFO("set soc to 100\n");
		}
	}

	//after charging done and stop charging,but  soc < 100 and charger is online, so set soc  to 100%
	if( pbatt->new_batt_status == POWER_SUPPLY_STATUS_FULL
	    && !batt_is_dischging(pbatt,10)
	    && (pbatt->new_soc>=95 && pbatt->new_soc<100)){
              pbatt->new_soc = 100;
		NXFG_INFO("chg done set soc to 100\n");
	}

    //To avoid the power off voltage is too high,make that power off voltage < 3400
	if(last_batt_soc != 0 && pbatt->new_soc == 0 && !pbatt->usb_in){
		if(pbatt->batt_mv > NXFG_CUTOFF_VOLTAGE && pbatt->batt_mv < NXFG_BATT_MAX_VOLTAGE){
			pbatt->new_soc = 1;
			batt_low_count = 0;
			pr_info("batt_vol is above 3400,set soc =1 \n");
		}else if(pbatt->batt_mv > NXFG_EMPTY_VOLTAGE && pbatt->batt_mv < NXFG_BATT_MAX_VOLTAGE
		            && batt_low_count < 2){
			batt_low_count ++;
			pbatt->new_soc = 1;
			pr_info("batt_vol is above 3200,set soc =1 for %d times\n",batt_low_count);
		}
	}else{
		batt_low_count = 0;
	}

    //To update the new soc
	if(last_batt_soc >= 0){
		if(batt_cntl->resume_status){
			if(batt_cntl->resume_status == NX_RESUME_WAKE_STAT
				  && !nubia_is_resume_soc_invalid(batt_cntl, pbatt->new_soc)){
				last_batt_soc = pbatt->new_soc;
			}   
			power_supply_change = 1;
			batt_cntl->resume_status = NX_RESUME_IDLE_STAT;
	    }else if( (last_batt_soc < pbatt->new_soc) && pbatt->usb_in && !batt_is_dischging(pbatt,0) ) 
			last_batt_soc++;
		else if( (last_batt_soc > pbatt->new_soc) && batt_is_dischging(pbatt,0) ) 
			last_batt_soc--;
	}else{
	    last_batt_soc = pbatt->new_soc;
	}

    //To check if it need update power supply
	if(pbatt->batt_soc != last_batt_soc){
		pbatt->batt_soc = bound_soc(last_batt_soc);
		power_supply_change = 1;
	}

	if(pbatt->batt_status != pbatt->new_batt_status){
		pbatt->batt_status = pbatt->new_batt_status;
		power_supply_change = 1;
	}

	if(pbatt->batt_health != pbatt->new_batt_health){
		pbatt->batt_health = pbatt->new_batt_health;
		power_supply_change = 1;
	}

	if(power_supply_change)
	{
		pr_err("BATT:CHG nubia_update_power_supply batt_soc=%d, batt_ma=%d, batt_mv=%d,batt_temp=%d, batt_status=%d, usb_in=%d \n",
		      pbatt->batt_soc,pbatt->batt_ma,pbatt->batt_mv, pbatt->batt_temp,pbatt->batt_status,pbatt->usb_in );
		nubia_update_power_supply(batt_cntl);
	}

	batt_cntl->nubia_print_info(batt_cntl);

NEXT:
	schedule_delayed_work(&batt_cntl->batt_worker,
			  round_jiffies_relative(msecs_to_jiffies(nubia_get_delay_time(pbatt))));
}

int nubia_batt_suspend(struct nubia_fg_cntl *batt_cntl)
{
	if (!batt_cntl) {
		pr_err("called before init\n");
		return -1;
	}

	batt_cntl->usbin_in_suspend = nubia_is_charger_present(batt_cntl);
	cancel_delayed_work_sync(&batt_cntl->batt_worker);
	return 0;
}

int nubia_batt_resume(struct nubia_fg_cntl *batt_cntl)
{
	if (!batt_cntl) {
		pr_err("called before init\n");
		return -1;
	}

	batt_cntl->resume_status = NX_RESUME_WAKE_STAT;
	schedule_delayed_work(&batt_cntl->batt_worker,
				  round_jiffies_relative(msecs_to_jiffies(100)));
	return 0;
}

int nubia_report_batt_capacity(struct nubia_fg_cntl *batt_cntl)
{
    struct nubia_fg_params *pbatt;
	int resume_soc_invalid = 0;
	int batt_soc;
	int ret;

	if (!batt_cntl) {
		pr_err("called before init\n");
		return NXFG_DEFAULT_SOC;
	}

	pbatt = &batt_cntl->batt_data;

	if(batt_cntl->resume_status == NX_RESUME_WAKE_STAT){
		ret = batt_cntl->nubia_get_soc(batt_cntl, &batt_soc);
		if( ret < 0 || nubia_is_resume_soc_invalid(batt_cntl, batt_soc) )
			resume_soc_invalid = 1;

		//if resume_soc_invalid is true or batt_soc=0 after first resume,we don't update the soc here
		if( !resume_soc_invalid && batt_soc )
			pbatt->batt_soc = batt_soc;

		batt_cntl->resume_status = NX_RESUME_PROP_STAT;
	}

	if(batt_cntl->support_backup)
		batt_cntl->nubia_backup_soc(batt_cntl);

	NXFG_DEBUG("report soc=%d\n",pbatt->batt_soc);
	
	return pbatt->batt_soc;
}

int nubia_batt_cntl_init(struct nubia_fg_cntl *batt_cntl)
{
	struct nubia_fg_params *pbatt;

	//These pointers must be inited
    if(!batt_cntl
		 || !batt_cntl->nubia_print_info 
		 || !batt_cntl->nubia_get_soc
		 || !batt_cntl->fg_pri_data){
		pr_err("Not init nubia batt cntl!\n");
		return -EINVAL;
	}

	if(batt_cntl->support_backup && !batt_cntl->nubia_backup_soc){
		pr_err("Not init backup_soc!\n");
		return -EINVAL;
	}

	pbatt = &batt_cntl->batt_data;
	
    INIT_DELAYED_WORK(&batt_cntl->batt_worker,nubia_batt_worker);
	pbatt->new_soc = pbatt->batt_soc;
	pbatt->batt_status = POWER_SUPPLY_STATUS_UNKNOWN;
	pbatt->batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
	batt_cntl->resume_status = NX_RESUME_IDLE_STAT;

	schedule_delayed_work(&batt_cntl->batt_worker,
		   round_jiffies_relative(msecs_to_jiffies(2 * BATT_NORMAL_MONITOR_MS)));

	return 0;

}

int nubia_batt_cntl_deinit(struct nubia_fg_cntl *batt_cntl)
{
	if(!batt_cntl){
		pr_err("Deinit nubia batt fail!\n");
		return -EINVAL;
	}

	cancel_delayed_work_sync(&batt_cntl->batt_worker);
	return 0;
}


