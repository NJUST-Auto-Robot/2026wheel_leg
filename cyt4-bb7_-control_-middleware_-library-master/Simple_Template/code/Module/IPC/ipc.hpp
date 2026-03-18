#pragma once
#include "zf_common_headfile.h"

// 接收的方位角范围是0-360
extern uint32_t ipc_get_data;

// 接收错误计数
extern uint32_t ipc_error_count;

#ifdef __cplusplus
extern "C" {
#endif

void my_ipc_callback(uint32 receive_data);

#ifdef __cplusplus
}
#endif