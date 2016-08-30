#ifndef __GF_LOG_H__
#define __GF_LOG_H__

#define GF_LOG_TAG "GF318M"
#define GF_LOG_ON
//#define GF_DEBUG_ON

#ifdef  GF_LOG_ON
#define GF_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] "  fmt, \
	GF_LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define GF_LOG_INFO(fmt, args...) printk(KERN_INFO "[%s] [%s: %d] "  fmt, \
	GF_LOG_TAG, __FUNCTION__, __LINE__, ##args)

    #ifdef  GF_DEBUG_ON
#define  GF_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt, \
	GF_LOG_TAG, __FUNCTION__, __LINE__, ##args)
    #else
#define GF_LOG_DEBUG(fmt, args...)
    #endif
#else
#define GF_LOG_ERROR(fmt, args...)
#define GF_LOG_INFO(fmt, args...)
#define GF_LOG_DEBUG(fmt, args...)
#endif

#endif
