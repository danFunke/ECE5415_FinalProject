/**
 *
 *
 *
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

typedef struct {
  float kalman_value;
  float kalman_uncertainty;
} kalman_filter_1d_t;

void kalman_filter_init(kalman_filter_1d_t* filter);

void kalman_filter_update(kalman_filter_1d_t* filter, float kalman_rate, float kalman_disp);

#endif /* KALMAN_FILTER_H_ */