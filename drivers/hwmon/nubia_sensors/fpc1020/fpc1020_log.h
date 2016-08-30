#ifndef __FPC1020_LOG__
#define __FPC1020_LOG__

#define FPC_LOG_TAG "FPC1020"
#define FPC_LOG_ON
//#define FPC_DEBUG_ON

#ifdef  FPC_LOG_ON
#define FPC_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] "  fmt, \
	FPC_LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define FPC_LOG_INFO(fmt, args...) printk(KERN_INFO "[%s] [%s: %d] "  fmt, \
	FPC_LOG_TAG, __FUNCTION__, __LINE__, ##args)

    #ifdef  FPC_DEBUG_ON
#define  FPC_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt, \
	FPC_LOG_TAG, __FUNCTION__, __LINE__, ##args)
    #else
#define FPC_LOG_DEBUG(fmt, args...)
    #endif
#else
#define FPC_LOG_ERROR(fmt, args...)
#define FPC_LOG_INFO(fmt, args...)
#define FPC_LOG_DEBUG(fmt, args...)
#endif

#endif
