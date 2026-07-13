/**
 * @file   rcwl1655.h
 * @author zgs
 * @brief  超声波传感器驱动
 * @version 2.1
 * @date   2025-12-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __RCWL1655_REG_H__
#define __RCWL1655_REG_H__

#include <linux/types.h>

#define RCWL1655_READ_CMD       0x01

struct rcwl1655_sample {
    u32 distance_raw;
    u32 frame_id;
    u64 timestamp_ns;
};

#endif // __RCWL1655_REG_H__