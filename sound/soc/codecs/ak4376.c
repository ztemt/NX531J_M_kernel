/*
 * ak4376.c  --  audio driver for AK4376
 *
 * Copyright (C) 2015 Asahi Kasei Microdevices Corporation
 *  Author                Date        Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      15/06/12	    1.0
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mfd/ak437x/pdata.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/of_gpio.h>
#include "ak4376.h"
#include <sound/q6afe-v2.h>
#include <sound/q6core.h>

//#define AK4376_DEBUG				//used at debug mode
//#define AK4376_CONTIF_DEBUG		//used at debug mode
#define CONFIG_DEBUG_FS_CODEC		//used at debug mode

#ifdef AK4376_DEBUG
#define akdbgprt printk
#else
#define akdbgprt(format, arg...) do {} while (0)
#endif

/* AK4376 Codec Private Data */
struct ak4376_priv {
    struct mutex mutex;
    unsigned int priv_pdn_en;			//PDN GPIO pin
    unsigned int priv_ldo_en;			//PDN GPIO pin
    int pdn1;							//PDN control, 0:Off, 1:On, 2:No use(assume always On)
    int pdn2;							//PDN control for kcontrol
    int fs1;
    int rclk;							//Master Clock
    int nBickFreq;						//0:32fs, 1:48fs, 2:64fs
    struct ak4376_platform_data *pdata;	//platform data
    int nPllMCKI;						//0:9.6MHz, 1:11.2896MHz, 2:12.288MHz, 3:19.2MHz
    int nDeviceID;						//0:AK4375, 1:AK4375A, 2:AK4376, 3:Other IC
    int lpmode;							//0:High Performance, 1:Low power mode
    int xtalfreq;						//0:12.288MHz, 1:11.2896MHz
    int nDACOn;
    struct i2c_client *i2c;
};

unsigned int ak4376_reg_read(struct snd_soc_codec *, unsigned int);
static int ak4376_write_register(struct snd_soc_codec *, unsigned int, unsigned int);
static struct snd_soc_codec *ak4376_codec;

#ifdef CONFIG_DEBUG_FS_CODEC
static int ak4376_reg_write(struct snd_soc_codec *, u16, u16);
#endif

#define USE_TERT_MI2S_CLK
#ifdef USE_TERT_MI2S_CLK
struct afe_clk_set mi2s_mclk = {
    AFE_API_VERSION_I2S_CONFIG,
    Q6AFE_LPASS_CLK_ID_MCLK_3, /* TBD */
    0,
    Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO,
    Q6AFE_LPASS_CLK_ROOT_DEFAULT,
    0,
};
static int enable_i2s_tert_mclk(int on)
{
    int ret = 0;
    if (on) {
        /* TBD: Need confirmation from HW team */
        mi2s_mclk.enable = 1;
        mi2s_mclk.clk_freq_in_hz = 9600000;

        pr_err("%s: Enabling mclk, clk_freq_in_hz = %u\n",
                __func__, mi2s_mclk.clk_freq_in_hz);

        ret = afe_set_lpass_clock_v2(AFE_PORT_ID_TERTIARY_MI2S_RX,&mi2s_mclk);
        if (ret < 0) {
            pr_err("%s: afe lpass mclk failed, err:%d\n",__func__, ret);
        }
    } else  {
        mi2s_mclk.enable = 0;
        pr_info("%s: Disabling mclk\n", __func__);
        ret = afe_set_lpass_clock_v2(AFE_PORT_ID_TERTIARY_MI2S_RX,&mi2s_mclk);
        if (ret < 0)
            pr_err("%s: afe lpass clock failed, err:%d\n",__func__, ret);
    }
    return ret;
}
#endif

static inline void ak4376_update_register(struct snd_soc_codec *codec)
{
    u8 cur_cache;
    u8 cur_register;
    u8 *cache = codec->reg_cache;
    int i;

    akdbgprt("\t[AK4376] %s(%d)\n", __FUNCTION__,__LINE__);

    for (i = 0; i < AK4376_16_DUMMY; i++) {
        cur_register = ak4376_reg_read(codec, i);
        cur_cache = cache[i];

        akdbgprt("\t[AK4376] %s(%d) reg:0x%x (I2C, cache)=(0x%x,0x%x)\n", __FUNCTION__,__LINE__,i,cur_register,cur_cache);

        if (cur_register != cur_cache){
            ak4376_write_register(codec, i, cur_cache);
            akdbgprt("\t[AK4376] %s(%d) write cache to register\n", __FUNCTION__,__LINE__);
        }
    }

    cur_register = ak4376_reg_read(codec, AK4376_24_MODE_CONTROL);
    cur_cache = cache[AK4376_24_MODE_CONTROL];

    akdbgprt("\t[AK4376] %s(%d) (reg:0x24)cur_register=%x, cur_cache=%x\n", __FUNCTION__,__LINE__,cur_register,cur_cache);

    if (cur_register != cur_cache)
        ak4376_write_register(codec, AK4376_24_MODE_CONTROL, cur_cache);
}

/* GPIO control for PDN */
static int ak4376_pdn_control(struct snd_soc_codec *codec, int pdn)
{
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    akdbgprt("\t[AK4376] %s(%d) pdn=%d\n",__FUNCTION__,__LINE__,pdn);

    if ((ak4376->pdn1 == 0) && (pdn == 1)) {
        gpio_direction_output(ak4376->priv_pdn_en, 1);
        akdbgprt("\t[AK4376] %s(%d) Turn on priv_pdn_en\n", __FUNCTION__,__LINE__);
        ak4376->pdn1 = 1;
        ak4376->pdn2 = 1;
        udelay(800);
        ak4376_update_register(codec);

    } else if ((ak4376->pdn1 == 1) && (pdn == 0)) {
        gpio_direction_output(ak4376->priv_pdn_en, 0);
        akdbgprt("\t[AK4376] %s(%d) Turn off priv_pdn_en\n", __FUNCTION__,__LINE__);
        ak4376->pdn1 = 0;
        ak4376->pdn2 = 0;
        snd_soc_cache_init(codec);
    }

    return 0;
}

#ifdef CONFIG_DEBUG_FS_CODEC
static struct mutex io_lock;

static ssize_t reg_data_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret, i, fpt;
    int rx[22];

    mutex_lock(&io_lock);
    for (i = 0; i < AK4376_16_DUMMY; i++) {
        ret = ak4376_reg_read(ak4376_codec, i);
        if (ret < 0) {
            pr_err("%s: read register error.\n", __func__);
            break;
        } else {
            rx[i] = ret;
        }
    }

    rx[22] = ak4376_reg_read(ak4376_codec, AK4376_24_MODE_CONTROL);
    mutex_unlock(&io_lock);


    if (i == 22) {
        ret = fpt = 0;
        for (i = 0; i < AK4376_16_DUMMY; i++, fpt += 6) {
            ret += sprintf(buf + fpt, "%02x,%02x\n", i, rx[i]);
        }

        ret += sprintf(buf + i * 6, "24,%02x\n", rx[22]);
        return ret;
    } else {
        return sprintf(buf, "read error");
    }
}

static ssize_t reg_data_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    char	*ptr_data = (char *)buf;
    char	*p;
    int		i, pt_count = 0;
    unsigned short val[20];

    while ((p = strsep(&ptr_data, ","))) {
        if (!*p)
            break;
        if (pt_count >= 20)
            break;
        val[pt_count] = simple_strtoul(p, NULL, 16);
        pt_count++;
    }

    mutex_lock(&io_lock);
    for (i = 0; i < pt_count; i+=2) {
        ak4376_reg_write(ak4376_codec, val[i], val[i+1]);
        pr_debug("%s: write add=%d, val=%d.\n", __func__, val[i], val[i+1]);
    }
    mutex_unlock(&io_lock);

    return count;
}

static DEVICE_ATTR(reg_data, 0640, reg_data_show, reg_data_store);

#endif

/* ak4376 register cache & default register settings */
static const u8 ak4376_reg[AK4376_MAX_REGISTERS] = {
    0x00,	/*	0x00	AK4376_00_POWER_MANAGEMENT1		*/
    0x00,	/*	0x01	AK4376_01_POWER_MANAGEMENT2		*/
    0x00,	/*	0x02	AK4376_02_POWER_MANAGEMENT3		*/
    0x00,	/*	0x03	AK4376_03_POWER_MANAGEMENT4		*/
    0x00,	/*	0x04	AK4376_04_OUTPUT_MODE_SETTING	*/
    0x00,	/*	0x05	AK4376_05_CLOCK_MODE_SELECT		*/
    0x00,	/*	0x06	AK4376_06_DIGITAL_FILTER_SELECT	*/
    0x00,	/*	0x07	AK4376_07_DAC_MONO_MIXING		*/
    0x00,	/*	0x08	AK4376_08_RESERVED				*/
    0x00,	/*	0x09	AK4376_09_RESERVED				*/
    0x00,	/*	0x0A	AK4376_0A_RESERVED				*/
    0x19,	/*	0x0B	AK4376_0B_LCH_OUTPUT_VOLUME		*/
    0x19,	/*	0x0C	AK4376_0C_RCH_OUTPUT_VOLUME		*/
    0x0B,	/*	0x0D	AK4376_0D_HP_VOLUME_CONTROL		*/
    0x00,	/*	0x0E	AK4376_0E_PLL_CLK_SOURCE_SELECT	*/
    0x00,	/*	0x0F	AK4376_0F_PLL_REF_CLK_DIVIDER1	*/
    0x00,	/*	0x10	AK4376_10_PLL_REF_CLK_DIVIDER2	*/
    0x00,	/*	0x11	AK4376_11_PLL_FB_CLK_DIVIDER1	*/
    0x00,	/*	0x12	AK4376_12_PLL_FB_CLK_DIVIDER2	*/
    0x00,	/*	0x13	AK4376_13_DAC_CLK_SOURCE		*/
    0x00,	/*	0x14	AK4376_14_DAC_CLK_DIVIDER		*/
    0x40,	/*	0x15	AK4376_15_AUDIO_IF_FORMAT		*/
    0x00,	/*	0x16	AK4376_16_DUMMY					*/
    0x00,	/*	0x17	AK4376_17_DUMMY					*/
    0x00,	/*	0x18	AK4376_18_DUMMY					*/
    0x00,	/*	0x19	AK4376_19_DUMMY					*/
    0x00,	/*	0x1A	AK4376_1A_DUMMY					*/
    0x00,	/*	0x1B	AK4376_1B_DUMMY					*/
    0x00,	/*	0x1C	AK4376_1C_DUMMY					*/
    0x00,	/*	0x1D	AK4376_1D_DUMMY					*/
    0x00,	/*	0x1E	AK4376_1E_DUMMY					*/
    0x00,	/*	0x1F	AK4376_1F_DUMMY					*/
    0x00,	/*	0x20	AK4376_20_DUMMY					*/
    0x00,	/*	0x21	AK4376_21_DUMMY					*/
    0x00,	/*	0x22	AK4376_22_DUMMY					*/
    0x00,	/*	0x23	AK4376_23_DUMMY					*/
    0x00,	/*	0x24	AK4376_24_MODE_CONTROL			*/
};

static const struct {
    int readable;   /* Mask of readable bits */
    int writable;   /* Mask of writable bits */
} ak4376_access_masks[] = {
    { 0xFF, 0x11 },	//0x00
    { 0xFF, 0x33 },	//0x01
    { 0xFF, 0x11 },	//0x02
    { 0xFF, 0x7F },	//0x03
    { 0xFF, 0x3F },	//0x04
    { 0xFF, 0xFF },	//0x05
    { 0xFF, 0xCB },	//0x06
    { 0xFF, 0xFF },	//0x07
    { 0xFF, 0xFF },	//0x08
    { 0xFF, 0xFF },	//0x09
    { 0xFF, 0xFF },	//0x0A
    { 0xFF, 0x9F },	//0x0B
    { 0xFF, 0x1F },	//0x0C
    { 0xFF, 0x0F },	//0x0D
    { 0xFF, 0x21 },	//0x0E
    { 0xFF, 0xFF },	//0x0F
    { 0xFF, 0xFF },	//0x10
    { 0xFF, 0xFF },	//0x11
    { 0xFF, 0xFF },	//0x12
    { 0xFF, 0x01 },	//0x13
    { 0xFF, 0xFF },	//0x14
    { 0xFF, 0x1F },	//0x15
    { 0x00, 0x00 },	//0x16	//DUMMY
    { 0x00, 0x00 },	//0x17	//DUMMY
    { 0x00, 0x00 },	//0x18	//DUMMY
    { 0x00, 0x00 },	//0x19	//DUMMY
    { 0x00, 0x00 },	//0x1A	//DUMMY
    { 0x00, 0x00 },	//0x1B	//DUMMY
    { 0x00, 0x00 },	//0x1C	//DUMMY
    { 0x00, 0x00 },	//0x1D	//DUMMY
    { 0x00, 0x00 },	//0x1E	//DUMMY
    { 0x00, 0x00 },	//0x1F	//DUMMY
    { 0x00, 0x00 },	//0x20	//DUMMY
    { 0x00, 0x00 },	//0x21	//DUMMY
    { 0x00, 0x00 },	//0x22	//DUMMY
    { 0x00, 0x00 },	//0x23	//DUMMY
    { 0xFF, 0x50 },	//0x24
};

/* Output Digital volume control:
 * from -12.5 to 3 dB in 0.5 dB steps (mute instead of -12.5 dB) */
static DECLARE_TLV_DB_SCALE(ovl_tlv, -1250, 50, 0);
static DECLARE_TLV_DB_SCALE(ovr_tlv, -1250, 50, 0);

/* HP-Amp Analog volume control:
 * from -22 to 6 dB in 2 dB steps (mute instead of -42 dB) */
static DECLARE_TLV_DB_SCALE(hpg_tlv, -2200, 200, 0);

static const char *ak4376_ovolcn_select_texts[] = {"Dependent", "Independent"};
static const char *ak4376_mdacl_select_texts[] = {"x1", "x1/2"};
static const char *ak4376_mdacr_select_texts[] = {"x1", "x1/2"};
static const char *ak4376_invl_select_texts[] = {"Normal", "Inverting"};
static const char *ak4376_invr_select_texts[] = {"Normal", "Inverting"};
static const char *ak4376_cpmod_select_texts[] =
{"Automatic Switching", "+-VDD Operation", "+-1/2VDD Operation"};
static const char *ak4376_hphl_select_texts[] = {"9ohm", "Hi-Z"};
static const char *ak4376_hphr_select_texts[] = {"9ohm", "Hi-Z"};
static const char *ak4376_dacfil_select_texts[]  =
{"Sharp Roll-Off", "Slow Roll-Off", "Short Delay Sharp Roll-Off", "Short Delay Slow Roll-Off"};
static const char *ak4376_bcko_select_texts[] = {"64fs", "32fs"};
static const char *ak4376_dfthr_select_texts[] = {"Digital Filter", "Bypass"};
static const char *ak4376_ngate_select_texts[] = {"On", "Off"};
static const char *ak4376_ngatet_select_texts[] = {"Short", "Long"};

static const struct soc_enum ak4376_dac_enum[] = {
    SOC_ENUM_SINGLE(AK4376_0B_LCH_OUTPUT_VOLUME, 7,
            ARRAY_SIZE(ak4376_ovolcn_select_texts), ak4376_ovolcn_select_texts),
    SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 2,
            ARRAY_SIZE(ak4376_mdacl_select_texts), ak4376_mdacl_select_texts),
    SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 6,
            ARRAY_SIZE(ak4376_mdacr_select_texts), ak4376_mdacr_select_texts),
    SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 3,
            ARRAY_SIZE(ak4376_invl_select_texts), ak4376_invl_select_texts),
    SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 7,
            ARRAY_SIZE(ak4376_invr_select_texts), ak4376_invr_select_texts),
    SOC_ENUM_SINGLE(AK4376_03_POWER_MANAGEMENT4, 2,
            ARRAY_SIZE(ak4376_cpmod_select_texts), ak4376_cpmod_select_texts),
    SOC_ENUM_SINGLE(AK4376_04_OUTPUT_MODE_SETTING, 0,
            ARRAY_SIZE(ak4376_hphl_select_texts), ak4376_hphl_select_texts),
    SOC_ENUM_SINGLE(AK4376_04_OUTPUT_MODE_SETTING, 1,
            ARRAY_SIZE(ak4376_hphr_select_texts), ak4376_hphr_select_texts),
    SOC_ENUM_SINGLE(AK4376_06_DIGITAL_FILTER_SELECT, 6,
            ARRAY_SIZE(ak4376_dacfil_select_texts), ak4376_dacfil_select_texts), 
    SOC_ENUM_SINGLE(AK4376_15_AUDIO_IF_FORMAT, 3,
            ARRAY_SIZE(ak4376_bcko_select_texts), ak4376_bcko_select_texts),
    SOC_ENUM_SINGLE(AK4376_06_DIGITAL_FILTER_SELECT, 3,
            ARRAY_SIZE(ak4376_dfthr_select_texts), ak4376_dfthr_select_texts),
    SOC_ENUM_SINGLE(AK4376_06_DIGITAL_FILTER_SELECT, 0,
            ARRAY_SIZE(ak4376_ngate_select_texts), ak4376_ngate_select_texts),
    SOC_ENUM_SINGLE(AK4376_06_DIGITAL_FILTER_SELECT, 1,
            ARRAY_SIZE(ak4376_ngatet_select_texts), ak4376_ngatet_select_texts),
};

static const char *bickfreq_on_select[] = {"32fs", "48fs", "64fs"};
static const char *pllmcki_on_select[] = {"9.6MHz", "11.2896MHz", "12.288MHz", "19.2MHz"};
static const char *lpmode_on_select[] = {"High Performance", "Low Power"};
static const char *xtalfreq_on_select[] = {"12.288MHz", "11.2896MHz"};
static const char *pdn_on_select[] = {"Off", "On"};

static const struct soc_enum ak4376_bitset_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(bickfreq_on_select), bickfreq_on_select), 
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(pllmcki_on_select), pllmcki_on_select),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(lpmode_on_select), lpmode_on_select),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(xtalfreq_on_select), xtalfreq_on_select),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(pdn_on_select), pdn_on_select),
};

static int get_bickfs(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }
    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);

    ucontrol->value.enumerated.item[0] = ak4376->nBickFreq;

    return 0;
}

static int ak4376_set_bickfs(struct snd_soc_codec *codec)
{
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }
    akdbgprt("\t[AK4376] %s(%d) nBickFreq=%d\n",__FUNCTION__,__LINE__,ak4376->nBickFreq);

    if ( ak4376->nBickFreq == FS_32 ) { 	//32fs
        snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x03, 0x01);	//DL1-0=01(16bit, >=32fs)
    } else if( ak4376->nBickFreq == FS_48 ) {	//48fs
        snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x03, 0x00);	//DL1-0=00(24bit, >=48fs)
    } else {								//64fs
        snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x03, 0x02);	//DL1-0=1x(32bit, >=64fs)
    }

    return 0;
}

static int set_bickfs(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }
    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);
    ak4376->nBickFreq = ucontrol->value.enumerated.item[0];
    ak4376_set_bickfs(codec);

    return 0;
}

static int get_pllmcki(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }
    ucontrol->value.enumerated.item[0] = ak4376->nPllMCKI;
    return 0;
}

static int set_pllmcki(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }
    ak4376->nPllMCKI = ucontrol->value.enumerated.item[0];
    return 0;
}

static int get_lpmode(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }
    ucontrol->value.enumerated.item[0] = ak4376->lpmode;
    return 0;
}

static int ak4376_set_lpmode(struct snd_soc_codec *codec)
{
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }
    if ( ak4376->lpmode == 0 ) { 	//High Performance Mode
        snd_soc_update_bits(codec, AK4376_02_POWER_MANAGEMENT3, 0x10, 0x00);	//LPMODE=0(High Performance Mode)
        if ( ak4376->fs1 <= 12000 ) {
            snd_soc_update_bits(codec, AK4376_24_MODE_CONTROL, 0x40, 0x40);	//DSMLP=1
        } else {
            snd_soc_update_bits(codec, AK4376_24_MODE_CONTROL, 0x40, 0x00);	//DSMLP=0
        }
    } else {							//Low Power Mode
        snd_soc_update_bits(codec, AK4376_02_POWER_MANAGEMENT3, 0x10, 0x10);	//LPMODE=1(Low Power Mode)
        snd_soc_update_bits(codec, AK4376_24_MODE_CONTROL, 0x40, 0x40);			//DSMLP=1
    }

    return 0;
}

static int set_lpmode(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }
    ak4376->lpmode = ucontrol->value.enumerated.item[0];
    ak4376_set_lpmode(codec);

    return 0;
}

static int get_xtalfreq(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }
    ucontrol->value.enumerated.item[0] = ak4376->xtalfreq;

    return 0;
}

static int set_xtalfreq(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }

    ak4376->xtalfreq = ucontrol->value.enumerated.item[0];

    return 0;
}

static int get_pdn(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }
    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);

    ucontrol->value.enumerated.item[0] = ak4376->pdn2;

    return 0;
}

static int set_pdn(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    if(ak4376 == NULL)
    {
        pr_err("ak4376 is not initialization");
        return 0;
    }
    ak4376->pdn2 = ucontrol->value.enumerated.item[0];
    akdbgprt("\t[AK4376] %s(%d) pdn2=%d\n",__FUNCTION__,__LINE__,ak4376->pdn2);
    if (ak4376->pdn1 == 0)
        ak4376_pdn_control(codec, ak4376->pdn2);

    return 0;
}

#ifdef AK4376_DEBUG

static const char *test_reg_select[]   = 
{
    "read AK4376 Reg 00:24",
};

static const struct soc_enum ak4376_enum[] = 
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(test_reg_select), test_reg_select),
};

static int nTestRegNo = 0;

static int get_test_reg(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    /* Get the current output routing */
    ucontrol->value.enumerated.item[0] = nTestRegNo;

    return 0;
}

static int set_test_reg(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    u32    currMode = ucontrol->value.enumerated.item[0];
    int    i, value;
    int	   regs, rege;

    nTestRegNo = currMode;
    regs = 0x00;
    rege = 0x15;

    for ( i = regs ; i <= rege ; i++ ){
        value = snd_soc_read(codec, i);
        printk("***AK4376 Addr,Reg=(%x, %x)\n", i, value);
    }
    value = snd_soc_read(codec, 0x24);
    printk("***AK4376 Addr,Reg=(%x, %x)\n", 0x24, value);

    return 0;
}
#endif

static const struct snd_kcontrol_new ak4376_snd_controls[] = {
    SOC_SINGLE_TLV("AK4376 Digital Output VolumeL",
            AK4376_0B_LCH_OUTPUT_VOLUME, 0, 0x1F, 0, ovl_tlv),
    SOC_SINGLE_TLV("AK4376 Digital Output VolumeR",
            AK4376_0C_RCH_OUTPUT_VOLUME, 0, 0x1F, 0, ovr_tlv),
    SOC_SINGLE_TLV("AK4376 HP-Amp Analog Volume",
            AK4376_0D_HP_VOLUME_CONTROL, 0, 0x0F, 0, hpg_tlv),

    SOC_ENUM("AK4376 Digital Volume Control", ak4376_dac_enum[0]), 
    SOC_ENUM("AK4376 DACL Signal Level", ak4376_dac_enum[1]), 
    SOC_ENUM("AK4376 DACR Signal Level", ak4376_dac_enum[2]), 
    SOC_ENUM("AK4376 DACL Signal Invert", ak4376_dac_enum[3]), 
    SOC_ENUM("AK4376 DACR Signal Invert", ak4376_dac_enum[4]), 
    SOC_ENUM("AK4376 Charge Pump Mode", ak4376_dac_enum[5]), 
    SOC_ENUM("AK4376 HPL Power-down Resistor", ak4376_dac_enum[6]),
    SOC_ENUM("AK4376 HPR Power-down Resistor", ak4376_dac_enum[7]),
    SOC_ENUM("AK4376 DAC Digital Filter Mode", ak4376_dac_enum[8]),
    SOC_ENUM("AK4376 BICK Output Frequency", ak4376_dac_enum[9]),
    SOC_ENUM("AK4376 Digital Filter Mode", ak4376_dac_enum[10]),
    SOC_ENUM("AK4376 Noise Gate", ak4376_dac_enum[11]),
    SOC_ENUM("AK4376 Noise Gate Time", ak4376_dac_enum[12]),

    SOC_ENUM_EXT("AK4376 BICK Frequency Select", ak4376_bitset_enum[0], get_bickfs, set_bickfs),
    SOC_ENUM_EXT("AK4376 PLL MCKI Frequency", ak4376_bitset_enum[1], get_pllmcki, set_pllmcki),
    SOC_ENUM_EXT("AK4376 Low Power Mode", ak4376_bitset_enum[2], get_lpmode, set_lpmode),
    SOC_ENUM_EXT("AK4376 Xtal Frequency", ak4376_bitset_enum[3], get_xtalfreq, set_xtalfreq),
    SOC_ENUM_EXT("AK4376 PDN Control", ak4376_bitset_enum[4], get_pdn, set_pdn),

#ifdef AK4376_DEBUG
    SOC_ENUM_EXT("Reg Read", ak4376_enum[0], get_test_reg, set_test_reg),
#endif

};



/* DAC control */
static int ak4376_dac_event2(struct snd_soc_codec *codec, int event) 
{
    u8 MSmode;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    MSmode = snd_soc_read(codec, AK4376_15_AUDIO_IF_FORMAT);

    switch (event) {
        case SND_SOC_DAPM_PRE_PMU:	/* before widget power up */
            ak4376->nDACOn = 1;
            snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x01);		//PMCP1=1
            mdelay(6);																//wait 6ms
            udelay(500);															//wait 0.5ms
            snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x30);		//PMLDO1P/N=1
            mdelay(1);																//wait 1ms
            break;
        case SND_SOC_DAPM_POST_PMU:	/* after widget power up */
            snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x02);		//PMCP2=1
            mdelay(4);																//wait 4ms
            udelay(500);															//wait 0.5ms
            break;
        case SND_SOC_DAPM_PRE_PMD:	/* before widget power down */
            snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x00);		//PMCP2=0
            break;
        case SND_SOC_DAPM_POST_PMD:	/* after widget power down */
            snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x00);		//PMLDO1P/N=0
            snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x00);		//PMCP1=0
            if (ak4376->pdata->nPllMode == PLL_OFF) {
                if (MSmode & 0x10) {	//Master mode
                    snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x10,0x00);	//MS bit = 0
                }
            }
            ak4376->nDACOn = 0;
            break;
    }
    return 0;
}

static int ak4376_dac_event(struct snd_soc_dapm_widget *w,
        struct snd_kcontrol *kcontrol, int event) //CONFIG_LINF
{
    struct snd_soc_codec *codec = w->codec;
    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);
    ak4376_dac_event2(codec, event);
    return 0;
}

/* PLL control */
static int ak4376_pll_event2(struct snd_soc_codec *codec, int event) 
{
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    switch (event) {
        case SND_SOC_DAPM_PRE_PMU:	/* before widget power up */
        case SND_SOC_DAPM_POST_PMU:	/* after widget power up */
            if ((ak4376->pdata->nPllMode == PLL_BICK_MODE) || (ak4376->pdata->nPllMode == PLL_MCKI_MODE)) {
                snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x01,0x01);	//PMPLL=1
            } else if (ak4376->pdata->nPllMode == XTAL_MODE) {
                snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x10,0x10);	//PMOSC=1
            }
            break;
        case SND_SOC_DAPM_PRE_PMD:	/* before widget power down */
            break;
        case SND_SOC_DAPM_POST_PMD:	/* after widget power down */
            if ((ak4376->pdata->nPllMode == PLL_BICK_MODE) || (ak4376->pdata->nPllMode == PLL_MCKI_MODE)) {
                snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x01,0x00);	//PMPLL=0
#ifdef USE_TERT_MI2S_CLK
                enable_i2s_tert_mclk(0);
#endif
            } else if (ak4376->pdata->nPllMode == XTAL_MODE) {
                snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x10,0x00);	//PMOSC=0
            }
            break;
    }

    return 0;
}

static int ak4376_pll_event(struct snd_soc_dapm_widget *w,
        struct snd_kcontrol *kcontrol, int event) //CONFIG_LINF
{
    struct snd_soc_codec *codec = w->codec;

    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);

    ak4376_pll_event2(codec, event);

    return 0;
}

/* HPL Mixer */
static const struct snd_kcontrol_new ak4376_hpl_mixer_controls[] = {
    SOC_DAPM_SINGLE("LDACL", AK4376_07_DAC_MONO_MIXING, 0, 1, 0), 
    SOC_DAPM_SINGLE("RDACL", AK4376_07_DAC_MONO_MIXING, 1, 1, 0), 
};

/* HPR Mixer */
static const struct snd_kcontrol_new ak4376_hpr_mixer_controls[] = {
    SOC_DAPM_SINGLE("LDACR", AK4376_07_DAC_MONO_MIXING, 4, 1, 0), 
    SOC_DAPM_SINGLE("RDACR", AK4376_07_DAC_MONO_MIXING, 5, 1, 0), 
};


/* ak4376 dapm widgets */
static const struct snd_soc_dapm_widget ak4376_dapm_widgets[] = {
    // DAC
    SND_SOC_DAPM_DAC_E("AK4376 DAC", "NULL", AK4376_02_POWER_MANAGEMENT3, 0, 0, 
            ak4376_dac_event, (SND_SOC_DAPM_POST_PMU |SND_SOC_DAPM_PRE_PMD 
                |SND_SOC_DAPM_PRE_PMU |SND_SOC_DAPM_POST_PMD)),

    // PLL, OSC
    SND_SOC_DAPM_SUPPLY_S("AK4376 PLL", 0, SND_SOC_NOPM, 0, 0, 
            ak4376_pll_event, (SND_SOC_DAPM_POST_PMU |SND_SOC_DAPM_PRE_PMD 
                |SND_SOC_DAPM_PRE_PMU |SND_SOC_DAPM_POST_PMD)),

    SND_SOC_DAPM_AIF_IN("AK4376 SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),

    // Analog Output
    SND_SOC_DAPM_OUTPUT("AK4376 HPL"),
    SND_SOC_DAPM_OUTPUT("AK4376 HPR"),

    SND_SOC_DAPM_MIXER("AK4376 HPR Mixer", AK4376_03_POWER_MANAGEMENT4, 1, 0, 
            &ak4376_hpr_mixer_controls[0], ARRAY_SIZE(ak4376_hpr_mixer_controls)),

    SND_SOC_DAPM_MIXER("AK4376 HPL Mixer", AK4376_03_POWER_MANAGEMENT4, 0, 0, 
            &ak4376_hpl_mixer_controls[0], ARRAY_SIZE(ak4376_hpl_mixer_controls)),

};

static const struct snd_soc_dapm_route ak4376_intercon[] = 
{

    {"AK4376 DAC", "NULL", "AK4376 PLL"},
    {"AK4376 DAC", "NULL", "AK4376 SDTI"},

    {"AK4376 HPL Mixer", "LDACL", "AK4376 DAC"},
    {"AK4376 HPL Mixer", "RDACL", "AK4376 DAC"},
    {"AK4376 HPR Mixer", "LDACR", "AK4376 DAC"},
    {"AK4376 HPR Mixer", "RDACR", "AK4376 DAC"},

    {"AK4376 HPL", "NULL", "AK4376 HPL Mixer"},
    {"AK4376 HPR", "NULL", "AK4376 HPR Mixer"},

};

static int ak4376_set_mcki(struct snd_soc_codec *codec, int fs, int rclk)
{
    u8 mode;
    u8 mode2;
    int mcki_rate;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    akdbgprt("\t[AK4376] %s fs=%d rclk=%d\n",__FUNCTION__, fs, rclk);

    if ((fs != 0)&&(rclk != 0)) {
        if (rclk > 28800000) return -EINVAL;

        if (ak4376->pdata->nPllMode == PLL_OFF) {	//PLL_OFF
            mcki_rate = rclk/fs;
        } else {		//XTAL_MODE
            if ( ak4376->xtalfreq == 0 ) {		//12.288MHz
                mcki_rate = 12288000/fs;
            } else {	//11.2896MHz
                mcki_rate = 11289600/fs;
            }
        }

        mode = snd_soc_read(codec, AK4376_05_CLOCK_MODE_SELECT);
        mode &= ~AK4376_CM;

        if (ak4376->lpmode == 0) {				//High Performance Mode
            switch (mcki_rate) {
                case 32:
                    mode |= AK4376_CM_0;
                    break;
                case 64:
                    mode |= AK4376_CM_1;
                    break;
                case 128:
                    mode |= AK4376_CM_3;
                    break;
                case 256:
                    mode |= AK4376_CM_0;
                    mode2 = snd_soc_read(codec, AK4376_24_MODE_CONTROL);
                    if ( fs <= 12000 ) {
                        mode2 |= 0x40;	//DSMLP=1
                        snd_soc_write(codec, AK4376_24_MODE_CONTROL, mode2);
                    } else {
                        mode2 &= ~0x40;	//DSMLP=0
                        snd_soc_write(codec, AK4376_24_MODE_CONTROL, mode2);
                    }
                    break;
                case 512:
                    mode |= AK4376_CM_1;
                    break;
                case 1024:
                    mode |= AK4376_CM_2;
                    break;
                default:
                    return -EINVAL;
            }
        } else {					//Low Power Mode (LPMODE == DSMLP == 1)
            switch (mcki_rate) {
                case 32:
                    mode |= AK4376_CM_0;
                    break;
                case 64:
                    mode |= AK4376_CM_1;
                    break;
                case 128:
                    mode |= AK4376_CM_3;
                    break;
                case 256:
                    mode |= AK4376_CM_0;
                    break;
                case 512:
                    mode |= AK4376_CM_1;
                    break;
                case 1024:
                    mode |= AK4376_CM_2;
                    break;
                default:
                    return -EINVAL;
            }
        }

        snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, mode);
    }

    return 0;
}

static int ak4376_set_pllblock(struct snd_soc_codec *codec, int fs)
{
    u8 mode;
    int nMClk, nPLLClk, nRefClk;
    int PLDbit, PLMbit, MDIVbit;
    int PLLMCKI;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);

    mode = snd_soc_read(codec, AK4376_05_CLOCK_MODE_SELECT);
    mode &= ~AK4376_CM;

    if ( fs <= 24000 ) {
        mode |= AK4376_CM_1;
        nMClk = 512 * fs;
    } else if ( fs <= 96000 ) {
        mode |= AK4376_CM_0;
        nMClk = 256 * fs;
    } else if ( fs <= 192000 ) {
        mode |= AK4376_CM_3;
        nMClk = 128 * fs;
    } else {		//fs > 192kHz
        mode |= AK4376_CM_1;
        nMClk = 64 * fs;
    }

    snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, mode);

    if ( (fs % 8000) == 0 ) {
        nPLLClk = 122880000;
    } else if ( (fs == 11025 ) && ( ak4376->nBickFreq == FS_48 ) && ( ak4376->pdata->nPllMode == 1 )) {
        nPLLClk = 101606400;
    } else {
        nPLLClk = 112896000;
    }

    if ( ak4376->pdata->nPllMode == PLL_BICK_MODE) {		//BICK_PLL (Slave)
        if ( ak4376->nBickFreq == FS_32 ) {		//32fs
            if ( fs <= 96000 ) PLDbit = 1;
            else if ( fs <= 192000 ) PLDbit = 2;
            else PLDbit = 4;
            nRefClk = 32 * fs / PLDbit;
        } else if ( ak4376->nBickFreq == FS_48 ) {	//48fs
            if ( fs <= 16000 ) PLDbit = 1;
            else if ( fs <= 192000 ) PLDbit = 3;
            else PLDbit = 6;
            nRefClk = 48 * fs / PLDbit;
        } else {  									// 64fs
            if ( fs <= 48000 ) PLDbit = 1;
            else if ( fs <= 96000 ) PLDbit = 2;
            else if ( fs <= 192000 ) PLDbit = 4;
            else PLDbit = 8;
            nRefClk = 64 * fs / PLDbit;
        }
    } else {		//MCKI_PLL (Master)
        if ( ak4376->nPllMCKI == PLL_CLK_9P6MHZ ) { //9.6MHz
            PLLMCKI = 9600000;
            if ( (fs % 4000) == 0) nRefClk = 1920000;
            else nRefClk = 384000;
        } else if ( ak4376->nPllMCKI == PLL_CLK_11P289MHZ) { //11.2896MHz
            PLLMCKI = 11289600;
            if ( (fs % 4000) == 0) return -EINVAL;
            else nRefClk = 2822400;
        } else if ( ak4376->nPllMCKI == PLL_CLK_12P288MHZ ) { //12.288MHz
            PLLMCKI = 12288000;
            if ( (fs % 4000) == 0) nRefClk = 3072000;
            else nRefClk = 768000;
        } else {								//19.2MHz
            PLLMCKI = 19200000;
            if ( (fs % 4000) == 0) nRefClk = 1920000;
            else nRefClk = 384000;
        }
        PLDbit = PLLMCKI / nRefClk;
    }

    PLMbit = nPLLClk / nRefClk;
    MDIVbit = nPLLClk / nMClk;

    PLDbit--;
    PLMbit--;
    MDIVbit--;

    //PLD15-0
    snd_soc_write(codec, AK4376_0F_PLL_REF_CLK_DIVIDER1, ((PLDbit & 0xFF00) >> 8));
    snd_soc_write(codec, AK4376_10_PLL_REF_CLK_DIVIDER2, ((PLDbit & 0x00FF) >> 0));
    //PLM15-0
    snd_soc_write(codec, AK4376_11_PLL_FB_CLK_DIVIDER1, ((PLMbit & 0xFF00) >> 8));
    snd_soc_write(codec, AK4376_12_PLL_FB_CLK_DIVIDER2, ((PLMbit & 0x00FF) >> 0));

    if (ak4376->pdata->nPllMode == PLL_BICK_MODE) {	//BICK_PLL (Slave)
        snd_soc_update_bits(codec, AK4376_0E_PLL_CLK_SOURCE_SELECT, 0x03, 0x01);	//PLS=1(BICK)
    } else {										//MCKI PLL (Slave/Master)
        snd_soc_update_bits(codec, AK4376_0E_PLL_CLK_SOURCE_SELECT, 0x03, 0x00);	//PLS=0(MCKI)
    }

    //MDIV7-0
    snd_soc_write(codec, AK4376_14_DAC_CLK_DIVIDER, MDIVbit);

    return 0;
}

static int ak4376_set_timer(struct snd_soc_codec *codec)
{
    int ret, curdata;
    int count, tm, nfs;
    int lvdtm, vddtm, hptm;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    lvdtm = 0;
    vddtm = 0;
    hptm = 0;

    nfs = ak4376->fs1;

    //LVDTM2-0 bits set
    ret = snd_soc_read(codec, AK4376_03_POWER_MANAGEMENT4);
    curdata = (ret & 0x70) >> 4;	//Current data Save
    ret &= ~0x70;
    do {
        count = 1000 * (64 << lvdtm);
        tm = count / nfs;
        if ( tm > LVDTM_HOLD_TIME ) break;
        lvdtm++;
    } while ( lvdtm < 7 );			//LVDTM2-0 = 0~7
    if ( curdata != lvdtm) {
        snd_soc_write(codec, AK4376_03_POWER_MANAGEMENT4, (ret | (lvdtm << 4)));
    }

    //VDDTM3-0 bits set
    ret = snd_soc_read(codec, AK4376_04_OUTPUT_MODE_SETTING);
    curdata = (ret & 0x3C) >> 2;	//Current data Save
    ret &= ~0x3C;
    do {
        count = 1000 * (1024 << vddtm);
        tm = count / nfs;
        if ( tm > VDDTM_HOLD_TIME ) break;
        vddtm++;
    } while ( vddtm < 8 );			//VDDTM3-0 = 0~8
    if ( curdata != vddtm) {
        snd_soc_write(codec, AK4376_04_OUTPUT_MODE_SETTING, (ret | (vddtm<<2)));
    }

    return 0;
}

static int ak4376_hw_params_set(struct snd_soc_codec *codec, int nfs1)
{
    u8 fs;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);
    pr_info("%s:gpio value %d",__func__,gpio_get_value_cansleep(ak4376->priv_pdn_en));

    fs = snd_soc_read(codec, AK4376_05_CLOCK_MODE_SELECT);
    fs &= ~AK4376_FS;

    switch (nfs1) {
        case 8000:
            fs |= AK4376_FS_8KHZ;
            break;
        case 11025:
            fs |= AK4376_FS_11_025KHZ;
            break;
        case 16000:
            fs |= AK4376_FS_16KHZ;
            break;
        case 22050:
            fs |= AK4376_FS_22_05KHZ;
            break;
        case 32000:
            fs |= AK4376_FS_32KHZ;
            break;
        case 44100:
            fs |= AK4376_FS_44_1KHZ;
            break;
        case 48000:
            fs |= AK4376_FS_48KHZ;
            break;
        case 88200:
            fs |= AK4376_FS_88_2KHZ;
            break;
        case 96000:
            fs |= AK4376_FS_96KHZ;
            break;
        case 176400:
            fs |= AK4376_FS_176_4KHZ;
            break;
        case 192000:
            fs |= AK4376_FS_192KHZ;
            break;
        case 352800:
            fs |= AK4376_FS_352_8KHZ;
            break;
        case 384000:
            fs |= AK4376_FS_384KHZ;
            break;
        default:
            return -EINVAL;
    }
    snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, fs);

    if ( ak4376->pdata->nPllMode == PLL_OFF) {		//PLL Off
        snd_soc_update_bits(codec, AK4376_13_DAC_CLK_SOURCE, 0x03, 0x00);	//DACCKS=0
        ak4376_set_mcki(codec, nfs1, ak4376->rclk);
    } else if ( ak4376->pdata->nPllMode == XTAL_MODE) {	//XTAL MODE
        snd_soc_update_bits(codec, AK4376_13_DAC_CLK_SOURCE, 0x03, 0x02);	//DACCKS=2
        ak4376_set_mcki(codec, nfs1, ak4376->rclk);
    } else {											//PLL mode
        snd_soc_update_bits(codec, AK4376_13_DAC_CLK_SOURCE, 0x03, 0x01);	//DACCKS=1
        ak4376_set_pllblock(codec, nfs1);
    }

    ak4376_set_timer(codec);

    return 0;
}

static int ak4376_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params,
        struct snd_soc_dai *dai)
{
    struct snd_soc_codec *codec = dai->codec;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    ak4376->fs1 = params_rate(params);

    ak4376_hw_params_set(codec, ak4376->fs1);
    ak4376_pdn_control(codec, 1);
    pr_info("%s:pdn gpio gpio value %d",__func__,gpio_get_value_cansleep(ak4376->priv_pdn_en));
    pr_info("%s:ldo gpio gpio value %d",__func__,gpio_get_value_cansleep(ak4376->priv_ldo_en));
    return 0;
}

static int ak4376_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
        unsigned int freq, int dir)
{
    struct snd_soc_codec *codec = dai->codec;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    akdbgprt("\t[AK4376] %s freq=%dHz(%d)\n",__FUNCTION__,freq,__LINE__);
    pr_info("%s:gpio value %d",__func__,gpio_get_value_cansleep(ak4376->priv_pdn_en));

    ak4376_pdn_control(codec, 1);
    ak4376->rclk = freq;
    if ((ak4376->pdata->nPllMode == PLL_OFF) || (ak4376->pdata->nPllMode == XTAL_MODE)) {	//Not PLL mode
        ak4376_set_mcki(codec, ak4376->fs1, freq);
    }

    return 0;
}

static int ak4376_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{

    struct snd_soc_codec *codec = dai->codec;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    u8 format;

    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);

    ak4376_pdn_control(codec, 1);
    pr_info("%s:gpio value %d",__func__,gpio_get_value_cansleep(ak4376->priv_pdn_en));

    /* set master/slave audio interface */
    format = snd_soc_read(codec, AK4376_15_AUDIO_IF_FORMAT);
    format &= ~AK4376_DIF;

    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
        case SND_SOC_DAIFMT_CBS_CFS:
            format |= AK4376_SLAVE_MODE;
            break;
        case SND_SOC_DAIFMT_CBM_CFM:
            if (ak4376->nDeviceID == 2) {
                format |= AK4376_MASTER_MODE;
            }
            else return -EINVAL;
            break;
        case SND_SOC_DAIFMT_CBS_CFM:
        case SND_SOC_DAIFMT_CBM_CFS:
        default:
            dev_err(codec->dev, "Clock mode unsupported");
            return -EINVAL;
    }

    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
        case SND_SOC_DAIFMT_I2S:
            format |= AK4376_DIF_I2S_MODE;
            break;
        case SND_SOC_DAIFMT_LEFT_J:
            format |= AK4376_DIF_MSB_MODE;
            break;
        default:
            return -EINVAL;
    }

    /* set format */
    snd_soc_write(codec, AK4376_15_AUDIO_IF_FORMAT, format);

    return 0;
}

static int ak4376_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
    int	ret;

    switch (reg) {
        default:
            ret = 0;
            break;
    }
    return ret;
}

static int ak4376_readable(struct snd_soc_codec *codec, unsigned int reg)
{

    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);

    if (reg >= ARRAY_SIZE(ak4376_access_masks))
        return 0;
    return ak4376_access_masks[reg].readable != 0;
}


/* Read ak4376 register cache */
static inline u32 ak4376_read_reg_cache(struct snd_soc_codec *codec, u16 reg)
{
    u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4376_reg));
    return (u32)cache[reg];
}


/* Read ak4376 IC register */
unsigned int ak4376_i2c_read(u8 *reg, int reglen, u8 *data, int datalen)
{
    struct i2c_msg xfer[2];
    int ret;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(ak4376_codec);
    struct i2c_client *client = ak4376->i2c;

    // Write register 
    xfer[0].addr = client->addr;
    xfer[0].flags = 0;
    xfer[0].len = reglen;
    xfer[0].buf = reg;

    // Read data
    xfer[1].addr = client->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = datalen;
    xfer[1].buf = data;

    ret = i2c_transfer(client->adapter, xfer, 2);

    if (ret == 2)
        return 0;
    else if (ret < 0)
        return -ret;
    else 
        return -EIO;
}

unsigned int ak4376_reg_read(struct snd_soc_codec *codec, unsigned int reg)
{
    unsigned char tx[1], rx[1];
    int	wlen, rlen;
    int ret = 0;
    unsigned int rdata = 0;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    if (ak4376->pdn1 == 0) {
        rdata = ak4376_read_reg_cache(codec, reg);
        akdbgprt("\t[AK4376] %s Read cache\n",__FUNCTION__);
    } else if ((ak4376->pdn1 == 1) || (ak4376->pdn1 == 2)) {
        wlen = 1;
        rlen = 1;
        tx[0] = reg;

        ret = ak4376_i2c_read(tx, wlen, rx, rlen);
        akdbgprt("\t[AK4376] %s Read IC register\n",__FUNCTION__);

        if (ret < 0) {
            akdbgprt("\t[AK4376] %s error ret = %d\n",__FUNCTION__,ret);
            rdata = -EIO;
        } else {
            rdata = (unsigned int)rx[0];
        }
    }

    return rdata;
}

/* Write AK4376 register cache */
static inline void ak4376_write_reg_cache(
        struct snd_soc_codec *codec, 
        u16 reg,
        u16 value)
{
    u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4376_reg));
    cache[reg] = (u8)value;
}

static int ak4376_write_register(struct snd_soc_codec *codec, unsigned int reg,
        unsigned int value)
{
    int wlen;
    int rc = 0;
    unsigned char tx[2];
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    akdbgprt("\t[AK4376] %s(%d) (%x,%x)\n",__FUNCTION__,__LINE__,reg,value);

    wlen = 2;
    tx[0] = reg; 
    tx[1] = value;

    ak4376_write_reg_cache(codec, reg, value);	

    if ((ak4376->pdn1 == 1) || (ak4376->pdn1 ==2)) {
        rc = i2c_master_send(ak4376->i2c, tx, wlen);
    }

    return rc;
}


#ifdef CONFIG_DEBUG_FS_CODEC
static int ak4376_reg_write(
        struct snd_soc_codec *codec,
        u16 reg,
        u16 value)
{
    akdbgprt("\t[AK4376] %s(%d) (%x,%x)\n",__FUNCTION__,__LINE__,reg,value);

    snd_soc_write(codec, (unsigned int)reg, (unsigned int)value);

    return 0;
}
#endif

// * for AK4376
static int ak4376_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{
    int ret = 0;

    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);
    return ret;
}


static int ak4376_set_bias_level(struct snd_soc_codec *codec,
        enum snd_soc_bias_level level)
{
    int level1 = level;

    akdbgprt("\t[AK4376] %s(%d) level=%d\n",__FUNCTION__,__LINE__,level1);
    akdbgprt("\t[AK4376] %s(%d) codec->dapm.bias_level=%d\n",__FUNCTION__,__LINE__,codec->dapm.bias_level);

    switch (level1) {
        case SND_SOC_BIAS_ON:
            break;
        case SND_SOC_BIAS_PREPARE:
            if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY)
                akdbgprt("\t[AK4376] %s(%d) codec->dapm.bias_level == SND_SOC_BIAS_STANDBY\n",__FUNCTION__,__LINE__);
            if (codec->dapm.bias_level == SND_SOC_BIAS_ON)
                akdbgprt("\t[AK4376] %s(%d) codec->dapm.bias_level >= SND_SOC_BIAS_ON\n",__FUNCTION__,__LINE__);
            break;
        case SND_SOC_BIAS_STANDBY:
            if (codec->dapm.bias_level == SND_SOC_BIAS_PREPARE) {
                akdbgprt("\t[AK4376] %s(%d) codec->dapm.bias_level == SND_SOC_BIAS_PREPARE\n",__FUNCTION__,__LINE__);
                ak4376_pdn_control(codec, 0);
            } if (codec->dapm.bias_level == SND_SOC_BIAS_OFF)
            akdbgprt("\t[AK4376] %s(%d) codec->dapm.bias_level == SND_SOC_BIAS_OFF\n",__FUNCTION__,__LINE__);
            break;
        case SND_SOC_BIAS_OFF:
            if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY) {
                akdbgprt("\t[AK4376] %s(%d) codec->dapm.bias_level == SND_SOC_BIAS_STANDBY\n",__FUNCTION__,__LINE__);
                ak4376_pdn_control(codec, 0);
            } break;
    }
    codec->dapm.bias_level = level;

    return 0;
}

static int ak4376_set_dai_mute(struct snd_soc_dai *dai, int mute) 
{
    u8 MSmode;
    struct snd_soc_codec *codec = dai->codec;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    akdbgprt("\t[AK4376] %s(%d) codec->dapm.bias_level=%d\n",__FUNCTION__,__LINE__,codec->dapm.bias_level);

    if (ak4376->pdata->nPllMode == PLL_OFF) {
        if ( ak4376->nDACOn == 0 ) {
            MSmode = snd_soc_read(codec, AK4376_15_AUDIO_IF_FORMAT);
            if (MSmode & 0x10) {	//Master mode	
                snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x10,0x00);	//MS bit = 0
            }
        }
    }

    if (codec->dapm.bias_level <= SND_SOC_BIAS_STANDBY) {
        akdbgprt("\t[AK4376] %s(%d) codec->dapm.bias_level <= SND_SOC_BIAS_STANDBY\n",__FUNCTION__,__LINE__);
        ak4376_pdn_control(codec, 0);
    }

    return 0;
}

#define AK4376_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
        SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
        SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
        SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
        SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
        SNDRV_PCM_RATE_192000)

#define AK4376_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)


static int ak4376_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai) {
    pr_info("ak4376_startup .");
#ifdef USE_TERT_MI2S_CLK
    enable_i2s_tert_mclk(1);
#endif
    return 0;
}

static void ak4376_shutdown(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai) {

    pr_info("ak4376_shutdown.\n"); 
}

static struct snd_soc_dai_ops ak4376_dai_ops = {
    .hw_params	= ak4376_hw_params,
    .set_sysclk	= ak4376_set_dai_sysclk,
    .set_fmt	= ak4376_set_dai_fmt,
    .trigger = ak4376_trigger,
    .digital_mute = ak4376_set_dai_mute,
    .startup = ak4376_startup,
    .shutdown = ak4376_shutdown,
};

struct snd_soc_dai_driver ak4376_dai[] = {   
    {										 
        .name = "ak4376-AIF1",
        .playback = {
            .stream_name = "Playback",
            .channels_min = 1,
            .channels_max = 2,
            .rates = AK4376_RATES,
            .formats = AK4376_FORMATS,
        },
        .ops = &ak4376_dai_ops,
    },
};

static int ak4376_init_reg(struct snd_soc_codec *codec)
{
    u8 DeviceID;
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);

    ak4376_pdn_control(codec, 1);

    DeviceID = ak4376_reg_read(codec, AK4376_15_AUDIO_IF_FORMAT);

    switch (DeviceID >> 5) {
        case 0:
            ak4376->nDeviceID = 0;		//0:AK4375
            printk("AK4375 is connecting.\n");
            break;
        case 1:
            ak4376->nDeviceID = 1;		//1:AK4375A
            printk("AK4375A is connecting.\n");
            break;
        case 2:
            ak4376->nDeviceID = 2;		//2:AK4376
            printk("AK4376 is connecting.\n");
            break;
        default:
            ak4376->nDeviceID = 3;		//3:Other IC
            printk("This device are neither AK4375/A nor AK4376.\n");
    }

    ak4376_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
    akdbgprt("\t[AK4376 bias] %s(%d)\n",__FUNCTION__,__LINE__);

    return 0;
}
struct ak4376_platform_data ak4376_pdata;
static int ak4376_probe(struct snd_soc_codec *codec)
{
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    int ret = 0;
    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);

#ifdef AK4376_CONTIF_DEBUG
    codec->read = ak4376_reg_read;
    codec->write = ak4376_write_register;
#endif


#ifdef CONFIG_DEBUG_FS_CODEC
    mutex_init(&io_lock);
#endif

    ak4376_codec = codec;
    //ak4376->pdata = dev_get_platdata(codec->dev);
    ak4376->pdata = &ak4376_pdata;
    ak4376->pdata->nPllMode = PLL_MCKI_MODE;
    ak4376->pdata->pdn_en = ak4376->priv_pdn_en;

    ak4376_init_reg(codec);
    akdbgprt("\t[AK4376 Effect] %s(%d)\n",__FUNCTION__,__LINE__);

    ak4376->fs1 = 48000;
    ak4376->rclk = 0;
    ak4376->nBickFreq = FS_32;		//0:32fs, 1:48fs, 2:64fs
    ak4376->nPllMCKI = PLL_CLK_9P6MHZ;		//0:9.6MHz, 1:11.2896MHz, 2:12.288MHz, 3:19.2MHz
    ak4376->lpmode = 0;			//0:High Performance mode, 1:Low Power Mode
    ak4376->xtalfreq = 0;		//0:12.288MHz, 1:11.2896MHz
    ak4376->nDACOn = 0;
    return ret;
}

static int ak4376_remove(struct snd_soc_codec *codec)
{
    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);
    ak4376_set_bias_level(codec, SND_SOC_BIAS_OFF);
    return 0;
}

static int ak4376_suspend(struct snd_soc_codec *codec)
{
    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);
    ak4376_set_bias_level(codec, SND_SOC_BIAS_OFF);
    return 0;
}

static int ak4376_resume(struct snd_soc_codec *codec)
{
    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);
    ak4376_init_reg(codec);
    return 0;
}

struct snd_soc_codec_driver soc_codec_dev_ak4376 = {
    .probe = ak4376_probe,
    .remove = ak4376_remove,
    .suspend = ak4376_suspend,
    .resume = ak4376_resume,

    .controls = ak4376_snd_controls,
    .num_controls = ARRAY_SIZE(ak4376_snd_controls),

    .idle_bias_off = true,
    .set_bias_level = ak4376_set_bias_level,
    .reg_cache_size = ARRAY_SIZE(ak4376_reg),
    .reg_word_size = sizeof(u8),
    .reg_cache_default = ak4376_reg,
    .readable_register = ak4376_readable,
    .volatile_register = ak4376_volatile,	
    .dapm_widgets = ak4376_dapm_widgets,
    .num_dapm_widgets = ARRAY_SIZE(ak4376_dapm_widgets),
    .dapm_routes = ak4376_intercon,
    .num_dapm_routes = ARRAY_SIZE(ak4376_intercon),
    .read = ak4376_reg_read,
    .write = ak4376_write_register,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_ak4376);

static int ak4376_i2c_probe(struct i2c_client *i2c,
        const struct i2c_device_id *id)
{
    struct ak4376_priv *ak4376;
    int ret=0;
    printk("%s, [lhd] enter \n ",__func__);
    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);

    ak4376 = kzalloc(sizeof(struct ak4376_priv), GFP_KERNEL);

    if (ak4376 == NULL) return -ENOMEM;

#ifdef  CONFIG_DEBUG_FS_CODEC
    ret = device_create_file(&i2c->dev, &dev_attr_reg_data);
    if (ret) {
        pr_err("%s: Error to create reg_data\n", __FUNCTION__);
    }
#endif

    i2c_set_clientdata(i2c, ak4376);


    ak4376->i2c = i2c;
    ak4376->pdn1 = 0;
    ak4376->pdn2 = 0;
    ak4376->priv_pdn_en = 0;
    ak4376->priv_ldo_en = 0;

    ret = snd_soc_register_codec(&i2c->dev,
            &soc_codec_dev_ak4376, &ak4376_dai[0], ARRAY_SIZE(ak4376_dai));
    if (ret < 0){
        kfree(ak4376);
        akdbgprt("\t[AK4376 Error!] %s(%d)\n",__FUNCTION__,__LINE__);
    }

    /* GPIO request for PDN */
    ak4376->priv_pdn_en = of_get_named_gpio(i2c->dev.of_node,"en-gpio", 0);
    akdbgprt("\t[AK4376] %s(%d) pdata set is valid\n",__FUNCTION__,__LINE__);
    if (gpio_is_valid(ak4376->priv_pdn_en)) 
    {
        akdbgprt("\t[AK4376] %s(%d) pdn_en is valid\n",__FUNCTION__,__LINE__);
        ret = gpio_request(ak4376->priv_pdn_en, "pdn_en");
        if (ret) {
            akdbgprt("\t[AK4376] %s(%d) cannot get pdn_en gpio\n",__FUNCTION__,__LINE__);
            ak4376->pdn1 = 2;	//No use GPIO control
        } else {
            ret = gpio_direction_output(ak4376->priv_pdn_en, 0);
            if (ret) {
                akdbgprt("\t[AK4376] %s(%d) pdn_en=0 fail\n", __FUNCTION__,__LINE__);
                gpio_free(ak4376->priv_pdn_en);
                ak4376->pdn1 = 2;
            } else {
                dev_dbg(&i2c->dev, "vad_clock_en=0\n");
                akdbgprt("\t[AK4376] %s(%d) pdn_en=0\n", __FUNCTION__,__LINE__);
            }
        }
    } else {
        akdbgprt("\t[AK4376] %s(%d) pdn_en is invalid\n", __FUNCTION__,__LINE__);
        ak4376->pdn1 = 2;
    }

    ak4376->priv_ldo_en = of_get_named_gpio(i2c->dev.of_node,"ldo-en-gpio", 0);
    if(gpio_is_valid(ak4376->priv_ldo_en))
    {
        ret = gpio_request(ak4376->priv_ldo_en, "ldo_en");
        if (ret) {
            akdbgprt("\t[AK4376] %s(%d) cannot get ldo_en gpio\n",__FUNCTION__,__LINE__);
        } else {
            ret = gpio_direction_output(ak4376->priv_ldo_en, 1);
            printk("ldo_en_gpio value is %d\n",gpio_get_value_cansleep(ak4376->priv_ldo_en));
        }
    } else{
        akdbgprt("\t[AK4376] %s(%d) ldo_en_gpio is invalid\n", __FUNCTION__,__LINE__);
    }
    akdbgprt("\t[AK4376] %s(%d) pdn1=%d\n", __FUNCTION__,__LINE__,ak4376->pdn1);

    return ret;
}

static int ak4376_i2c_remove(struct i2c_client *client)
{

    akdbgprt("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);

#ifdef CONFIG_DEBUG_FS_CODEC
    device_remove_file(&client->dev, &dev_attr_reg_data);
#endif

    snd_soc_unregister_codec(&client->dev);
    kfree(i2c_get_clientdata(client));

    return 0;
}

static const struct i2c_device_id ak4376_i2c_id[] = {
    { "ak4376", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, ak4376_i2c_id);

static struct i2c_driver ak4376_i2c_driver = {
    .driver = {
        .name = "ak4376",
        .owner = THIS_MODULE,
    },
    .probe = ak4376_i2c_probe,
    .remove = ak4376_i2c_remove,
    .id_table = ak4376_i2c_id,
};

static int __init ak4376_modinit(void)
{

    akdbgprt("\t[AK4376] %s(%d)\n", __FUNCTION__,__LINE__);

    return i2c_add_driver(&ak4376_i2c_driver);
}

module_init(ak4376_modinit);

static void __exit ak4376_exit(void)
{
    i2c_del_driver(&ak4376_i2c_driver);
}
module_exit(ak4376_exit);

MODULE_DESCRIPTION("ASoC ak4376 codec driver");
MODULE_LICENSE("GPL");
