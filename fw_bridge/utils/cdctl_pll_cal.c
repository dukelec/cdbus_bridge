/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include <math.h>
#include <float.h>
#include "cdctl_pll_cal.h"


pllcfg_t cdctl_pll_cal(unsigned input, unsigned output) {
    pllcfg_t best = {0, 0, 0, FLT_MAX, FLT_MAX};
    unsigned min_vco = 100e6, max_vco = 500e6, target_vco = 300e6;
    unsigned min_div_freq = 1e6, max_div_freq = 15e6, target_div_freq = 8e6;

    for (int d = 0; d <= 2; d++) {
        unsigned factor_d = 1 << d; // pow(2, d)

        for (int n = 0; n <= 31; n++) {
            float div_freq = (float)input / (n + 2);
            if (div_freq < min_div_freq || div_freq > max_div_freq)
                continue;

            for (int m = 0; m <= 511; m++) {
                float vco_freq = div_freq * (m + 2);
                if (vco_freq < min_vco || vco_freq > max_vco)
                    continue;

                float computed_output = vco_freq / factor_d;
                float error = fabsf(computed_output - output);

                // optimize div_freq and vco_freq
                float div_freq_deviation = fabsf(div_freq - target_div_freq) / target_div_freq;
                float vco_freq_deviation = fabsf(vco_freq - target_vco) / target_vco;
                float total_deviation = div_freq_deviation + vco_freq_deviation;

                if (error < best.error || (fabsf(error - best.error) < FLT_EPSILON && total_deviation < best.deviation)) {
                    best.n = n;
                    best.m = m;
                    best.d = d;
                    best.error = error;
                    best.deviation = total_deviation;
                }
            }
        }
    }

    if (best.d == 2)
        best.d = 3;
    return best;
}


unsigned cdctl_pll_get(unsigned input, pllcfg_t cfg)
{
    if (cfg.d == 3)
        cfg.d = 2;
    float div_freq = (float)input / (cfg.n + 2);
    float vco_freq = div_freq * (cfg.m + 2);
    return vco_freq / (1 << cfg.d);
}
