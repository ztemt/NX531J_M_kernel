/*
*Copyright (C) 2013-2014 Silicon Image, Inc.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation version 2.
* This program is distributed AS-IS WITHOUT ANY WARRANTY of any
* kind, whether express or implied; INCLUDING without the implied warranty
* of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
* See the GNU General Public License for more details at
* http://www.gnu.org/licenses/gpl-2.0.html.
*/
#include "Wrap.h"
#include "si_time.h"
#include "si_usbpd_core.h"
#include "si_usbpd_main.h"
#include "si_usbpd_regs.h"

#define DRP 0
#define DFP 1
#define UFP 2

extern int drp_mode;
struct kmem_cache *usbpd_cache;


int sii_drv_set_exit_mode(struct sii70xx_drv_context *drv_context,
		uint8_t portnum)
{
	/*return sii_usbpd_req_alt_mode(drv_context);*/
	return sii_usbpd_req_exit_mode(drv_context, portnum);
}

int sii_drv_set_alt_mode(struct sii70xx_drv_context *drv_context,
		uint8_t portnum,
		uint8_t svid_mode)
{
	/*return sii_usbpd_req_alt_mode(drv_context);*/
	return sii_usbpd_req_alt_mode(drv_context, portnum, svid_mode);
}

int sii_drv_set_pr_swap(struct sii70xx_drv_context *drv_context,
		uint8_t portnum)
{
	return sii_usbpd_req_power_swap(drv_context, portnum);
}

int sii_drv_set_dr_swap(struct sii70xx_drv_context *drv_context,
		uint8_t portnum)
{
	return sii_usbpd_req_data_swap(drv_context, portnum);
}

int sii_drv_get_sr_cap(struct sii70xx_drv_context *drv_context,
		uint8_t portnum)
{
	/*return sii_usbpd_req_alt_mode(drv_context);*/
	return sii_usbpd_req_src_cap(drv_context, portnum);
}

int sii_drv_set_vconn_swap(struct sii70xx_drv_context *drv_context,
		uint8_t portnum)
{
	return sii_usbpd_req_vconn_swap(drv_context, portnum);
}

void sii70xx_vbus_enable(struct sii70xx_drv_context *drv_context,
		uint8_t is_src)
{
	sii_platform_vbus_control(drv_context, is_src);
}

static bool sii_start_xmit(struct sii70xx_drv_context *drv_context,
		struct pd_cb_params *sm_inputs)
{
	if (sii_platform_rd_reg8(REG_ADDR__PDTXCS) != 0x06) {
		sii_platform_block_write8(REG_ADDR__PDTXBUF0,
				(uint8_t *)sm_inputs->send_data,
				sm_inputs->count);
		sii_platform_wr_reg8(REG_ADDR__PDTXBC,
				sm_inputs->count);
		sii_platform_wr_reg8(REG_ADDR__PDTXCS,
				BIT_MSK__PDTXCS__RI_PDTXTRANSMIT_WP);
	} else {
		pr_debug("Channel is busy\n");
		return false;
	}

	return true;
}

bool xmit_message(void *context,
		struct pd_cb_params *sm_inputs)
{
	struct sii70xx_drv_context *drv_context =
		(struct sii70xx_drv_context *)context;
	uint8_t result = 0;

	pr_debug("xmit_message :%x %x\n",
			sm_inputs->send_data[0],
			sm_inputs->send_data[1]);

	if (!sm_inputs->send_data || !sm_inputs->count) {
		/*TODO: need to check count here*/
		pr_warning("%s: Error data object",
				__func__);
		return false;
	}
	result = sii_start_xmit(drv_context, sm_inputs);

	if (!result) {
		pr_err("Data is not being written");
		pr_err("cc busy\n");
		return false;
	}
	return true;
}

bool sii_process_pd_status(void *context, uint32_t events)
{
	struct sii70xx_drv_context *drv_context =
		(struct sii70xx_drv_context *)context;
	struct sii_typec *ptypec_dev = (struct sii_typec *)
		drv_context->ptypec;
	struct sii_usbp_policy_engine *pUsbpd =
		(struct sii_usbp_policy_engine *)
		drv_context->pusbpd_policy;

	if (events != 0) {
		sii_typec_events(ptypec_dev, events);
		if (events == FEAT_PR_SWP) {
			if (pUsbpd->drv_context->
					drp_config == USBPD_DFP) {
				pr_debug("start UFP");
				usbpd_set_ufp_swap_init(pUsbpd);
			} else {
				pr_debug("start DFP");
				usbpd_set_dfp_swap_init(pUsbpd);
			}
		}
	}

	return true;
}

void send_device_softreset(struct sii70xx_drv_context *drv_context)
{
	sii_platform_wr_reg8(REG_ADDR__PDCCSRST, 0x01);
	sii_platform_wr_reg8(REG_ADDR__PDCCSRST, 0x0C);
}

static void si70xx_driver_reset(struct sii70xx_drv_context *drv_context)
{
	struct sii_usbp_policy_engine *pUsbpd =
		(struct sii_usbp_policy_engine *)
		drv_context->pusbpd_policy;
	pUsbpd->drv_context->drp_config = pUsbpd->drv_context->
		old_drp_config;
	sii70xx_pd_reset_variables(pUsbpd);
	sii70xx_pd_sm_reset(pUsbpd);
}

void sii_update_70xx_mode(struct sii70xx_drv_context *drv_context,
		enum phy_drp_config drp_mode)
{
	/*if (drp_mode == DFP)
		set_70xx_mode(drv_context, TYPEC_DRP_DFP);
	else if (drp_mode == UFP)
		set_70xx_mode(drv_context, TYPEC_DRP_UFP);
	else if (drp_mode == DRP)
		set_70xx_mode(drv_context, TYPEC_DRP_TOGGLE_RP);
	else*/
		set_70xx_mode(drv_context, drp_mode);
}

static void confirm_conn_debounce(struct sii70xx_drv_context *drv_context,
		bool is_dfp)
{
	struct sii_typec *ptypec_dev = (struct sii_typec *)drv_context->
		ptypec;
	uint8_t vol_thres = 0;
	u8 rcd_data1, rcd_data2;

	/*msleep(20);*/

	rcd_data1 = sii_platform_rd_reg8(REG_ADDR__CC1VOL) & 0x3F;
	rcd_data2 = sii_platform_rd_reg8(REG_ADDR__CC2VOL) & 0x3F;

	pr_info(" !!! Disconnect Reason :  %d %d \t", rcd_data1, rcd_data2);

	if (is_dfp) {
		vol_thres = sii_platform_rd_reg8(REG_ADDR__CCCTR8) & 0x3F;

		if (rcd_data1 >= vol_thres && rcd_data2 >= vol_thres) {
			pr_info("DFP : CC in Disconnect Range ==> Done !!!\n");
			si70xx_driver_reset(drv_context);
			update_typec_status(ptypec_dev, is_dfp, false);
			sii_70xx_enable_interrupts(drv_context,
				drv_context->drp_mode);
		} else {
			pr_info("DFP : CC is not in range - Spurious !!!\n");
		}
	} else {
		if (rcd_data1 == 0x00 && rcd_data2 == 0x00) {
			pr_info("UFP : CC Vol in Disconnect Range ==> Done!!!\n");
			si70xx_driver_reset(drv_context);
			update_typec_status(ptypec_dev, is_dfp, false);
			sii_70xx_enable_interrupts(drv_context,
				drv_context->drp_mode);
		} else {
			pr_info("UFP : CC is not in range - Spurious !!!\n");
		}
	}
}

void sii_dfp_read_vbus_status(struct sii70xx_drv_context
			*drv_context)
{
	uint8_t read_data = 0;

	read_data = sii_platform_rd_reg8(REG_ADDR__CC24STAT1);
	pr_info(" read data = %x", read_data);

	if ((read_data & 0x03) == 0x03) {/*ykim1: check bit 2:1*/
		drv_context->vbus_status->ccctr22_reg =
			sii_platform_rd_reg8(REG_ADDR__CCCTR22);
		drv_context->vbus_status->ccctr21_reg =
			sii_platform_rd_reg8(REG_ADDR__CCCTR21);
		drv_context->vbus_status->ccctr03_reg =
			sii_platform_rd_reg8(REG_ADDR__CCCTR3);
		drv_context->vbus_status->ccctr02_reg =
			sii_platform_rd_reg8(REG_ADDR__CCCTR2);

		sii_platform_wr_reg8(REG_ADDR__CCCTR22, 0x00);
		sii_platform_wr_reg8(REG_ADDR__CCCTR21, 0x12);
		sii_platform_wr_reg8(REG_ADDR__CCCTR3, 0xFF);
		sii_platform_wr_reg8(REG_ADDR__CCCTR2, 0x00);

		drv_context->vbus_status->work = true;
		pr_info("\ndfp setting WA enabled\n");
	} else {
		drv_context->vbus_status->work = false;
	}

}

void sii_dfp_restore_vbus_status(struct sii70xx_drv_context
			*drv_context)
{
	if (drv_context->vbus_status->work) {
		sii_platform_wr_reg8(REG_ADDR__CCCTR22,
			drv_context->vbus_status->ccctr22_reg);
		sii_platform_wr_reg8(REG_ADDR__CCCTR21,
			drv_context->vbus_status->ccctr21_reg);
		sii_platform_wr_reg8(REG_ADDR__CCCTR3,
			drv_context->vbus_status->ccctr03_reg);
		sii_platform_wr_reg8(REG_ADDR__CCCTR2,
			drv_context->vbus_status->ccctr02_reg);
		drv_context->vbus_status->work = false;
		pr_info("\ndfp setting WA completed\n");
	}
}

void set_70xx_mode(struct sii70xx_drv_context *drv_context,
		enum phy_drp_config drp_role)
{
	uint8_t temp = 0;

	drv_context->phy_mode = drp_role;
	if (drp_role == TYPEC_DRP_TOGGLE_RD) {
		pr_info("DRP ROLE: TOGGLE_RD\n");
		sii_platform_wr_reg8(REG_ADDR__CCCTR12, 0x02);
	} else if (drp_role == TYPEC_DRP_TOGGLE_RP) {
		pr_info("DRP ROLE: TOGGLE_RP\n");
		sii_platform_wr_reg8(REG_ADDR__CCCTR12, 0x03);
		temp = sii_platform_rd_reg8(REG_ADDR__CCCTR12);
		pr_info("Toggle RP value = %x\n", temp);
	} else if (drp_role == TYPEC_DRP_DFP) {
		pr_info("DRP ROLE: DFP(F)\n");
#ifdef SII_SABRE_ES0
		sii_dfp_read_vbus_status(drv_context);
		sii_platform_wr_reg8(REG_ADDR__CCCTR12, 0x01);
		usleep_range(1500, 1500);
		if (sii_platform_rd_reg8(REG_ADDR__CCCTR12)
			== 0x01) {/*ykim1: compare full byte*/
			pr_info("!!!DFP is configured successfully!!!)\n");
		} else {
			pr_info("!!!DFP configuration error!!!)\n");
			/*ykim1: error report*/
		}
		sii_dfp_restore_vbus_status(drv_context);
#else
		sii_platform_wr_reg8(REG_ADDR__CCCTR12, 0x01);
#endif

	} else {
		pr_info("DRP ROLE: UFP(F)\n");
		sii_platform_wr_reg8(REG_ADDR__CCCTR12, 0x00);
	}
}

void sii_mask_detach_interrupts(struct sii70xx_drv_context *drv_context)
{
	sii_platform_clr_bit8(REG_ADDR__PDCC24INTM3,
		BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK25 |
		BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK29);
	if (sii_platform_rd_reg8(REG_ADDR__PDCC24INT3) &
		BIT_MSK__PDCC24INT3__REG_PDCC24_INTR25) {
		sii_platform_set_bit8(REG_ADDR__PDCC24INT3,
			BIT_MSK__PDCC24INT3__REG_PDCC24_INTR25);
	}

	if (sii_platform_rd_reg8(REG_ADDR__PDCC24INT3) &
		BIT_MSK__PDCC24INT3__REG_PDCC24_INTR29) {
		sii_platform_set_bit8(REG_ADDR__PDCC24INT3,
			BIT_MSK__PDCC24INT3__REG_PDCC24_INTR29);
	}
}

void sii_mask_attach_interrupts(struct sii70xx_drv_context *drv_context)
{
	sii_platform_set_bit8(REG_ADDR__PDCC24INTM3,
		BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK25 |
		BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK29);
}

void sii_timer_disconnect_handler(void *context)
{
	struct sii70xx_drv_context *drv_context =
		(struct sii70xx_drv_context *)context;

	sii_timer_stop(&
		(drv_context->usbpd_inst_disconnect_tmr));
	pr_info("\n**confirm conection debounce**\n");
	confirm_conn_debounce(drv_context,
		drv_context->connection_status);
	complete(&drv_context->disconnect_done_complete);
}

void check_drp_status(struct sii70xx_drv_context *drv_context)
{
	struct sii_typec *ptypec_dev = (struct sii_typec *)drv_context->
		ptypec;
	uint8_t dev_conn_status = 0;

	if (test_bit(DFP_ATTACHED, &ptypec_dev->inputs)
			|| test_bit(DFP_DETACHED, &ptypec_dev->inputs)) {
		clear_bit(DFP_ATTACHED, &ptypec_dev->inputs);
		clear_bit(DFP_DETACHED, &ptypec_dev->inputs);

		dev_conn_status = sii_platform_rd_reg8(
				REG_ADDR__CC_CONN_STAT) & 0x03;
		pr_info("\n DFP Related interrupts ");

	} else if (test_bit(UFP_DETACHED, &ptypec_dev->inputs)
			|| test_bit(UFP_ATTACHED, &ptypec_dev->inputs)) {
		clear_bit(UFP_ATTACHED, &ptypec_dev->inputs);
		clear_bit(UFP_DETACHED, &ptypec_dev->inputs);

		dev_conn_status = sii_platform_rd_reg8(
				REG_ADDR__CC_CONN_STAT) & 0x30;
		pr_info("\n UFP Related interrupts ");
	} else
		pr_debug("Invalid Connection\n");

	pr_info("\ndev_conn_status:%x\n", dev_conn_status);

	switch (dev_conn_status) {
	case BIT_MSK__CC_CONN_STAT__RO_CC_DFP_ATTACHED:
		set_bit(DFP_ATTACHED, &ptypec_dev->inputs);
		/* dfp attached */
		/*if (!ptypec_dev->dfp_attached)*/
			update_typec_status(ptypec_dev, true, true);

		break;
	case BIT_MSK__CC_CONN_STAT__RO_CC_DFP_DETACHED:
		/* dfp dettached */
		ptypec_dev->dfp_attached = 0;
		drv_context->connection_status = false;
		sii_timer_start(&
			(drv_context->usbpd_inst_disconnect_tmr));
		init_completion(&drv_context->disconnect_done_complete);
		wait_for_completion_timeout(
			&drv_context->disconnect_done_complete,
			msecs_to_jiffies(50));
		break;
	case BIT_MSK__CC_CONN_STAT__RO_CC_UFP_ADC_ATTACHED:
		set_bit(UFP_ATTACHED, &ptypec_dev->inputs);
		/* ufp attached */
		/*if (!ptypec_dev->ufp_attached)*/
		update_typec_status(ptypec_dev,
			false, true);

		break;

	case BIT_MSK__CC_CONN_STAT__RO_CC_UFP_ADC_DETACHED:
		/* ufp dettached */
		drv_context->connection_status = true;
		ptypec_dev->dfp_attached = 0;
		sii_timer_start(&
			(drv_context->usbpd_inst_disconnect_tmr));
		init_completion(&drv_context->disconnect_done_complete);
		wait_for_completion_timeout(
			&drv_context->disconnect_done_complete,
			msecs_to_jiffies(50));
		break;

	default:
		pr_info("Nothing connected\n");
		break;
	};
}

void sii_platform_read_registers(
	struct sii70xx_drv_context *drv_context)
{
	uint8_t buff[256];
	int i = 0, j = 0;
	sii_platform_block_read8(REG_ADDR__SABER_ID0,
			buff,
			255);
	for (i = 0; i < 256; i++) {
		pr_debug("%x ", buff[i]);
		j++;
		if (j == 16) {
			pr_debug("\n");
			j = 0;
		}
	}
}

irqreturn_t usbpd_irq_handler(int irq, void *data)
{
	struct sii_usbp_policy_engine *pusbpd_policy;
	u8 xfer_sts[5];
	struct sii70xx_drv_context *drv_context =
		(struct sii70xx_drv_context *)data;
	struct sii_typec *ptypec_dev = (struct sii_typec *)
		drv_context->ptypec;

	pusbpd_policy = (struct sii_usbp_policy_engine *)
		drv_context->pusbpd_policy;
	/*bool result;*/

	pr_info("%s():%d", __func__, __LINE__);
	if (!drv_context) {
		pr_err("Error in getting context...\n");
		return IRQ_NONE;
	}
	pr_debug(" IRQ -1\t");
	/*sii_platform_read_70xx_gpio(drv_context);*/
	if (!down_interruptible(&drv_context->isr_lock)) {
		if (drv_context->dev_flags & DEV_FLAG_SHUTDOWN)
			goto irq_done;
		if (drv_context->dev_flags & DEV_FLAG_COMM_MODE)
			goto irq_done;
		if (!(sii_platform_rd_reg8(REG_ADDR__CALIB_CTRL) & 0x02)) {
			/*if the code enters in this if condition,
			the chip must have got reset*/
			pr_info("\n !!!!!!!!!!!!!!!!!!!!!!!!!\n");
			pr_info("\n !!!!!BOARD RESET !!!!!!!!\n");
			pr_info("\n !!!!!!!!!!!!!!!!!!!!!!!!!\n");
			sii_platform_read_registers(drv_context);
			sii_platform_wr_reg8(REG_ADDR__CALIB_CTRL, 0x02);
			/*this is not a solution, we are just making
			interrupt line low again.If chip got reset once
			again, the if loop will again exercise*/
			goto irq_done;
		}
		xfer_sts[0] = sii_platform_rd_reg8(REG_ADDR__PDCC24INT0);
		sii_platform_wr_reg8(REG_ADDR__PDCC24INT0, xfer_sts[0]);
		xfer_sts[1] = sii_platform_rd_reg8(REG_ADDR__PDCC24INT1);
		sii_platform_wr_reg8(REG_ADDR__PDCC24INT1, xfer_sts[1]);
		xfer_sts[2] = sii_platform_rd_reg8(REG_ADDR__PDCC24INT2);
		sii_platform_wr_reg8(REG_ADDR__PDCC24INT2, xfer_sts[2]);
		xfer_sts[3] = sii_platform_rd_reg8(REG_ADDR__PDCC24INT3);
		sii_platform_wr_reg8(REG_ADDR__PDCC24INT3, xfer_sts[3]);
		xfer_sts[4] = sii_platform_rd_reg8(REG_ADDR__PDCC24INT4);
		sii_platform_wr_reg8(REG_ADDR__PDCC24INT4, xfer_sts[4]);

		pr_info(" %x %x %x %x %x\t", xfer_sts[0], xfer_sts[1],
				xfer_sts[2], xfer_sts[3], xfer_sts[4]);


		if (xfer_sts[0] & BIT_MSK__PDCC24INT0__REG_PDCC24_INTR1) {
			/*PD Protocol Layer Transmitter Transmission retry
			  error interrupt*/
			pr_info(" nretry - no action\n");
			pusbpd_policy->busy_flag = true;
			/*usbipd_send_soft_reset(drv_context,
			CTRL_MSG__SOFT_RESET);*/
			wakeup_pd_queues(drv_context);
		}

		if (xfer_sts[0] & BIT_MSK__PDCC24INT0__REG_PDCC24_INTR0) {
			/*PD Protocol Layer Transmitter Transmission done
			  interrupt*/
			pr_info(": Transmission Done :\n");
			pusbpd_policy->tx_good_crc_received = true;
			wakeup_pd_queues(drv_context);
		}

		if (xfer_sts[0] & BIT_MSK__PDCC24INT0__REG_PDCC24_INTR2) {
			/*PD Protocol Layer Transmitter CRCReceiveTimer
			  timeout interrupt*/
			/*pr_info("\nCRCReceiveTimer timeout interrupt");*/
		}

		if (xfer_sts[4] & BIT_MSK__PDCC24INT4__REG_PDCC24_INTR32) {
			/*PD Protocol Layer Receiver Hard Reset/Cable Recept
			  Reception interrupt*/
			if (!pusbpd_policy->busy_flag) {
				pr_info("\n received HR/CB Reset\n");
				process_hard_reset(drv_context);
			} else {
				pr_info("busy has processed\n");
				pusbpd_policy->hard_reset_in_progress = true;
				sii_platform_set_bit8(REG_ADDR__PDCTR0,
						0x06);
				pusbpd_policy->busy_flag = false;
			}
		}

		if (xfer_sts[0] & BIT_MSK__PDCC24INT0__REG_PDCC24_INTR5) {
			/*PD Protocol Layer Receiver Message Reception done
			  interrupt*/
			if (!pusbpd_policy->hard_reset_in_progress) {
				if (!pusbpd_policy->busy_flag)
					sii_rcv_usbpd_data(drv_context);
			} else {
				sii_platform_wr_reg8(REG_ADDR__PDCTR0,
				BIT_MSK__PDCTR0__RI_PRL_RX_MSG_READ_DONE_WP);
				pusbpd_policy->hard_reset_in_progress = false;
			}
		}

		if (xfer_sts[1] & BIT_MSK__PDCC24INT1__REG_PDCC24_INTR8) {
			/*This bit is set if message request from Policy
			  Engine is discarded since Protocol Layer Tx is busy*/
			pr_info("\n: Tx is busy\n");
			sii_platform_wr_reg8(REG_ADDR__PDCTR0,
					0x04);
			wakeup_pd_queues(drv_context);
		}

		if (xfer_sts[1] & BIT_MSK__PDCC24INT1__REG_PDCC24_INTR11) {
			sii_platform_set_bit8(REG_ADDR__PDCTR0,
				BIT_MSK__PDCTR0__RI_PRL_RX_MSG_READ_DONE_WP);
		}
		if (sii_platform_rd_reg8(REG_ADDR__PDCC24INTM3) &
				BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK24) {

			if (xfer_sts[3] &
				BIT_MSK__PDCC24INT3__REG_PDCC24_INTR24) {
				/*ro_intr_cc_dfp_attached*/
				if (pusbpd_policy->pr_swap.in_progress)
					pr_debug("PR_SWAP  - dfp attached\n");
				else {
					if (ptypec_dev->dfp_attached) {
						;
					} else {
						pr_info(" DFP ATTACHED INTR: %x\n",
						sii_platform_rd_reg8(
						REG_ADDR__CC24STAT1));
						set_bit(DFP_ATTACHED,
							&ptypec_dev->
							inputs);
						check_drp_status(drv_context);
					}
				}
			}
		}

		if (sii_platform_rd_reg8(REG_ADDR__PDCC24INTM3) &
			BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK25) {
			if (xfer_sts[3] &
				BIT_MSK__PDCC24INT3__REG_PDCC24_INTR25) {
				/*ro_intr_cc_dfp_detached:*/
				if (pusbpd_policy->pr_swap.in_progress)
					pr_debug("PR_SWAP-dfp detached\n");
				else {
					pr_info(" DFP DETACHED INTR:\n");
					set_bit(DFP_DETACHED,
						&ptypec_dev->inputs);
					check_drp_status(drv_context);
				}
			}
		}

		if (sii_platform_rd_reg8(REG_ADDR__PDCC24INTM3) &
				BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK28) {
			if (xfer_sts[3] &
				BIT_MSK__PDCC24INT3__REG_PDCC24_INTR28) {
				/*ro_intr_cc_ufp_adc_attached:*/
				if (pusbpd_policy->pr_swap.in_progress) {
					pr_debug("PR_SWAP - ufp attached\n");
				} else {
					if (ptypec_dev->ufp_attached) {
						;
					} else {
						pr_info(" UFP ATTACHED INTR:\n");
						set_bit(UFP_ATTACHED,
							&ptypec_dev->inputs);
						check_drp_status(drv_context);
					}
				}
			}
		}

		if (sii_platform_rd_reg8(REG_ADDR__PDCC24INTM3) &
				BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK29) {
			if (xfer_sts[3] &
				BIT_MSK__PDCC24INT3__REG_PDCC24_INTR29) {
				/*ro_intr_cc_ufp_adc_detached:*/
				if (pusbpd_policy->pr_swap.in_progress)
					pr_debug("PR_SWAP In Prog-ufp detached\n");
				else {
					pr_info(" UFP DETACHED INTR:\n");
					set_bit(UFP_DETACHED, &ptypec_dev->
						inputs);
					check_drp_status(drv_context);
				}
			}
		}

		if (xfer_sts[4] & BIT_MSK__PDCC24INT4__REG_PDCC24_INTR33) {
			/*This bit is set if there is internal error in
			  hardware-assisted connection state*/
			/*pr_debug("\nCC Logic Interrupt-- HW Error.\n");*/
			/*send_device_softreset(drv_context);*/
		}
irq_done:
		if (!pusbpd_policy->hard_reset_in_progress)
			pusbpd_policy->busy_flag = false;

		up(&drv_context->isr_lock);
	}
	return  IRQ_HANDLED;
}

void validate_2_4mhz(struct sii70xx_drv_context *drv_context)
{
	if ((sii_platform_rd_reg8(REG_ADDR__CALIB_DATA) & 0x0E)) {
		pr_debug(" Already in 2.4Mhz Mode\n");
	} else {
		sii_platform_wr_reg8(REG_ADDR__OTP_WR_DATA0, 0x0E);
		sii_platform_wr_reg8(REG_ADDR__CALIB_CTRL, 0x02);
	}
}

void sii_70xx_enable_interrupts(struct sii70xx_drv_context *drv_context,
	int drp_mode)
{
	/*if (drp_mode == DFP) {
		sii_platform_set_bit8(REG_ADDR__PDCC24INTM3,
			BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK24 |
			BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK25);
	}
	else if (drp_mode == UFP) {
		sii_platform_set_bit8(REG_ADDR__PDCC24INTM3,
			BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK28 |
			BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK29);
	}
	else if (drp_mode == DRP){*/
		sii_platform_wr_reg8(REG_ADDR__PDCC24INTM3,
			BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK24 |
			BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK25 |
			BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK28 |
			BIT_MSK__PDCC24INTM3__REG_PDCC24_INTRMASK29);
	/*} else {
		pr_err("unknown mode selected\n");
		return;
	}*/
}

void sii70xx_drv_init(struct sii70xx_drv_context *drv_context, int drp_mode)
{
	sii_platform_read_70xx_gpio(drv_context);
	set_cc_reset(drv_context, true);
	set_pd_reset(drv_context, true);
	set_cc_reset(drv_context, false);

	sii_platform_wr_reg8(REG_ADDR__SABER_INTR_MASK, 0x3);
	sii_platform_wr_reg8(REG_ADDR__CALIB_CTRL, 0x02);
	sii_platform_wr_reg8(REG_ADDR__CCCTR8, 0x2F);

	sii_platform_wr_reg8(REG_ADDR__PDCC24INT0, 0xFF);
	sii_platform_wr_reg8(REG_ADDR__PDCC24INT1, 0xFF);
	sii_platform_wr_reg8(REG_ADDR__PDCC24INT2, 0xFF);
	sii_platform_wr_reg8(REG_ADDR__PDCC24INT3, 0xFF);
	sii_platform_wr_reg8(REG_ADDR__PDCC24INT4, 0xFF);

	sii_platform_wr_reg8(REG_ADDR__PDCC24INTM0, 0x23);
	sii_platform_wr_reg8(REG_ADDR__PDCC24INTM1, 0x0B);
	sii_platform_wr_reg8(REG_ADDR__PDCC24INTM2, 0x00);
	sii_platform_wr_reg8(REG_ADDR__PDCC24INTM4, 0x01);

	validate_2_4mhz(drv_context);

	sii70xx_vbus_enable(drv_context, VBUS_DEFAULT);

	sii_platform_wr_reg8(REG_ADDR__PDCTR3, 0x39);

	sii_platform_wr_reg8(REG_ADDR__CCCTR2, 0x90);
	sii_platform_wr_reg8(REG_ADDR__CCCTR3, 0x01);

	/*pr_info(" Vconn => %X\n",
	sii_platform_rd_reg8(REG_ADDR__ANA_CCPD_MODE_CTRL));

	pr_info(" Defaule current  => %X\n",
	sii_platform_rd_reg8(REG_ADDR__ANA_CCPD_RES_CTRL));

	sii_platform_set_bit8(REG_ADDR__ANA_CCPD_MODE_CTRL,
		BIT_MSK__ANA_CCPD_MODE_CTRL__RI_MANUAL_AUTOMODEB);

	sii_platform_set_bit8(REG_ADDR__ANA_CCPD_RES_CTRL, 0x01);

	sii_platform_set_bit8(REG_ADDR__ANA_CCPD_RES_CTRL, 0x02);

	pr_info(" Defaule current - 1 => %X\n",
	sii_platform_rd_reg8(REG_ADDR__ANA_CCPD_RES_CTRL));

	pr_info(" After Vconn => %X\n",
	sii_platform_rd_reg8(REG_ADDR__ANA_CCPD_MODE_CTRL));*/
	drv_context->drp_mode = drp_mode;

	if (drp_mode == DFP) {
		sii_update_70xx_mode(drv_context,
		TYPEC_DRP_DFP);
	} else if (drp_mode == UFP) {
		sii_update_70xx_mode(drv_context,
		TYPEC_DRP_UFP);
	} else if (drp_mode == DRP) {
		sii_update_70xx_mode(drv_context,
		TYPEC_DRP_TOGGLE_RD);
	} else {
		pr_err("unknown mode selected\n");
		return;
	}
	sii_70xx_enable_interrupts(drv_context, drp_mode);
	sii70xx_vbus_enable(drv_context, VBUS_SNK);

	sii_platform_wr_reg8(REG_ADDR__PDCTR19, 0x66);

	sii_platform_wr_reg8(REG_ADDR__CCCTR10, 0x01);
	sii_platform_set_bit8(REG_ADDR__CCCTR0,
		BIT_MSK__CCCTR0__RI_CC_FINE_CLK_EN);

}

int sii70xx_device_init(struct device *dev, struct gpio *sk_gpios)
{
	struct sii70xx_drv_context *drv_context;
	struct sii_usbp_policy_engine *pusbpd_policy;
	struct sii_usbpd_protocol *pusbpd_prtLyr;
	bool ret = 0;

	drv_context = kzalloc(sizeof(struct sii70xx_drv_context),
			GFP_KERNEL);
	if (!drv_context) {
		dev_err(dev, "platform device allocation failed\n");
		return -ENOMEM;
	}
	drv_context->pUsbpd_dp_mngr = kzalloc(sizeof(
				struct sii_usbp_device_policy), GFP_KERNEL);

	if (!drv_context->pUsbpd_dp_mngr) {
		dev_err(dev, "platform dp manager alloc failed\n");
		goto exit;
	}
	drv_context->vbus_status = kzalloc(sizeof(
				struct vbus_status_reg), GFP_KERNEL);

	if (!drv_context->vbus_status) {
		dev_err(dev, "vbus struct allocation failed\n");
		goto exit;
	}
	sema_init(&drv_context->isr_lock, 1);
	ret = sii_drv_sysfs_init(drv_context, dev);
	if (!ret) {
		dev_err(dev, "sysfs initialization failed\n");
		goto exit;
	}

	drv_context->irq = gpio_to_irq(sk_gpios[INT_INDEX].gpio);
	drv_context->sii_gpio = sk_gpios;
	dev_set_drvdata(dev, drv_context);
	sii70xx_platform_reset(drv_context);
	sii70xx_drv_init(drv_context, drp_mode);

	drv_context->ptypec = sii_phy_init(drv_context);
	if (!drv_context->ptypec) {
		pr_debug("Phy is not set .\n");
		goto exit;
	}

	pusbpd_policy = usbpd_init(drv_context, sii_process_pd_status);
	drv_context->pusbpd_policy = pusbpd_policy;

	if (!drv_context->pusbpd_policy) {
		pr_debug("%s: Error in device creation.\n", __func__);
		goto exit;
	}

	pusbpd_prtLyr = usbpd_core_init(drv_context,
			&pusbpd_policy->cfg, xmit_message);

	drv_context->pUsbpd_prot = pusbpd_prtLyr;

	if (!drv_context->pUsbpd_prot) {
		pr_warning("%s: pd_core failed...\n", __func__);
		goto exit;
	}

	ret = sii_timer_create(sii_timer_disconnect_handler,
			drv_context, &drv_context->usbpd_inst_disconnect_tmr,
			20, false);

	if (ret != 0) {
		pr_err("Failed to register disconnect timer!\n");
		goto exit;
	}


	ret = request_threaded_irq(drv_context->irq, NULL, usbpd_irq_handler,
			IRQF_TRIGGER_HIGH | IRQF_ONESHOT, SII_DRIVER_NAME,
			drv_context);
	if (ret < 0) {
		dev_err(dev, "request_threaded_irq failed, status\n");
		goto exit;
	}
	pr_debug("Driver is intialised properly\n");
	goto done;
exit:
	dev_set_drvdata(dev, drv_context);
	usbpd_device_exit(dev);
done:	return ret;
}

void usbpd_device_exit(struct device *dev)
{
	uint8_t ret = 0;
	struct sii70xx_drv_context *drv_context =
		(struct sii70xx_drv_context *)dev_get_drvdata(dev);

	ret = down_interruptible(&drv_context->isr_lock);
	if (!ret) {
		up(&drv_context->isr_lock);
		free_irq(drv_context->irq,
				drv_context);
		pr_debug("IRQ freed\n");
	}
	if (drv_context->usbpd_inst_disconnect_tmr) {
		sii_timer_stop(&drv_context->
			usbpd_inst_disconnect_tmr);
		ret = sii_timer_delete(&drv_context->
			usbpd_inst_disconnect_tmr);
	}

	usbpd_core_exit(drv_context->pUsbpd_prot);
	pr_debug("usbpd_core_exit exit\n");

	usbpd_exit(drv_context->pusbpd_policy);
	pr_debug("usbpd exit done\n");
	sii_phy_exit(drv_context->ptypec);
	pr_debug("phy exit done\n");
	sii_drv_sysfs_exit(drv_context);
	pr_debug("sysfs exit done\n");

	if (!drv_context->vbus_status)
		kfree(drv_context->vbus_status);
	if (!drv_context->pUsbpd_dp_mngr)
		kfree(drv_context->pUsbpd_dp_mngr);

	pr_debug("pUsbpd_dp_mngr exit\n");
	sii70xx_platform_reset(drv_context);
	kfree(drv_context);
	pr_debug("usbpd_device_exit\n");
}
