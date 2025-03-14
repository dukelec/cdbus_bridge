/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CDCTL_PLL_CAL_H__
#define __CDCTL_PLL_CAL_H__

typedef struct {
    unsigned n;
    unsigned m;
    unsigned d;
    float error;
    float deviation;
} pllcfg_t;

pllcfg_t cdctl_pll_cal(unsigned input, unsigned output);
unsigned cdctl_pll_get(unsigned input, pllcfg_t cfg);

#endif
