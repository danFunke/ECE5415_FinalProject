#include "moving_average_filter.h"

void moving_average_filter_init(moving_average_filter_t* filter)
{
  filter->index = 0;
  filter->value = 0;
  filter->sum = 0;
  filter->averaged_value = 0;
}

void moving_average_filter_update(moving_average_filter_t* filter, int new_value) 
{
  filter->sum = (filter->sum - filter->window[filter->index]);  // Remove the oldest entry from the sum
  filter->window[filter->index] = new_value;                    // Add new value to window
  filter->sum = filter->sum + new_value;                        // Add new value to sum
  filter->index = (filter->index + 1) % WINDOW_SIZE;            // Increment entry index
  filter->averaged_value = filter->sum / WINDOW_SIZE;           // Calculate averaged filter value
}