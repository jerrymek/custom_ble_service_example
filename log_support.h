#ifndef __LOG_SUPPORT_H__
#define __LOG_SUPPORT_H__

#define GENERAL_FAILURE 0xffff  // Todo: no code for initiating err_code to unsuccessful.
#define __FILENAME__ (__builtin_strrchr(__FILE__, '/') ? __builtin_strrchr(__FILE__, '/') + 1 : __FILE__)

/**
 * @brief If err_code != 0 call NRF_LOG_DEBUG and APP_ERROR_CHECK.
 * @description If log level < NRF_LOG_DEBUG do APP_ERROR_CHECK w/o logging.
 */
#define MY_ERROR_CHECK(x) \
    if(x!=0) {\
    NRF_LOG_DEBUG("%s(%d) err_code = 0x%x", __FILENAME__, __LINE__, x);\
    APP_ERROR_CHECK(x);\
    }

/**
 * @brief If err_code != 0 call NRF_LOG_DEBUG.
 * @description If log level < NRF_LOG_DEBUG no logging is done.
 */
#define MY_ERROR_LOG(x) \
    if(x!=0) {\
    NRF_LOG_DEBUG("%s(%d) err_code = 0x%x", __FILENAME__, __LINE__, x);\
    }

#endif // __LOG_SUPPORT_H__
