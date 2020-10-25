#ifndef __LOG_SUPPORT_H__
#define  __LOG_SUPPORT_H__

#define __FILENAME__ (__builtin_strrchr(__FILE__, '/') ? __builtin_strrchr(__FILE__, '/') + 1 : __FILE__)

#define MY_ERROR_CHECK(x) \
    if(x!=0) {\
    NRF_LOG_DEBUG("%s(%d) err_code = 0x%x", __FILENAME__, __LINE__, x);\
    APP_ERROR_CHECK(x);\
    }

#endif
