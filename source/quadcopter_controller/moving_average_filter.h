/**
 *
 *
 *
 */

#ifndef MOVING_AVERAGE_FILTER_H_
#define MOVING_AVERAGE_FILTER_H_

#define WINDOW_SIZE 10

typedef struct {
  int index;
  int value;
  int sum;
  int window[WINDOW_SIZE];
  int averaged_value;
} moving_average_filter_t;

void moving_average_filter_init(moving_average_filter_t* filter);

void moving_average_filter_update(moving_average_filter_t* filter, int new_value);

#endif /* MOVING_AVERAGE_FILTER_H_ */