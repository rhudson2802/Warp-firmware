#include <stdlib.h>

typedef struct acc_measurement{
	int16_t x;
	int16_t y;
	int16_t z;
} acc_measurement;


int16_t compute_mean(int16_t data[], uint8_t N);
int32_t compute_variance(int16_t data[], int16_t mean, uint8_t N);
void generate_distribution(int16_t data[], uint8_t N, int16_t * mean, int32_t * variance);
acc_measurement read_accelerometer();
void read_acceleration_distribution(uint8_t N, int16_t * x_mean, int32_t * x_var, int16_t * y_mean, int32_t * y_var, int16_t * z_mean, int32_t * z_var);
void rotate_array_by_one_16bit(int16_t data[], uint8_t N);
void rotate_array_by_one_32bit(int32_t data[], uint8_t N);
void print_array(int16_t data[], uint8_t N);
void low_pass_filter(int16_t means[], int32_t vars[], uint8_t N, int32_t output[]);
void equate_arrays(int32_t input[], int32_t output[], uint8_t length);
uint32_t if_variance(int32_t var1[], int32_t var2[]);
int8_t pedometer();
