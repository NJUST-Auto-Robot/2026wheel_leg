#pragma once
#include "cmsis_os.h"
#ifdef __cplusplus
extern "C"
{
#endif

    void tst_func(void);

#ifdef __cplusplus
}

class tst_class
{
public:
    void tst_class_func(void);
    uint8_t tst_class_flag = 0;
};
#endif // __cplusplus
