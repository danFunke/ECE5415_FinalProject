#include "kalman_filter.h"

void kalman_filter_init(kalman_filter_1d_t* filter)
{
  filter->kalman_value = 0;
  filter->kalman_uncertainty = 2 * 2;
}

void kalman_filter_update(kalman_filter_1d_t* filter, float kalman_rate, float kalman_disp) 
{
  filter->kalman_value = filter->kalman_value + 0.005 * kalman_rate;
  filter->kalman_uncertainty = filter->kalman_uncertainty + 0.005 * 0.005 * 4 * 4;
  float kalman_gain = filter->kalman_uncertainty * 1 / (1 * filter->kalman_uncertainty + 3 * 3);
  filter->kalman_value = filter->kalman_value + kalman_gain * (kalman_disp - filter->kalman_value);
  filter->kalman_uncertainty = (1 - kalman_gain) * filter->kalman_uncertainty;
}