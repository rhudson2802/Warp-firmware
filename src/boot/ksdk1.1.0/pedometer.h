#include <stdlib.h>

typedef struct acc_measurement{
	int16_t x;
	int16_t y;
	int16_t z;
} acc_measurement;


int16_t compute_mean(int16_t data[], uint8_t N);
int16_t compute_variance(int16_t data[], int16_t mean, uint8_t N);
void generate_distribution(int16_t data[], uint8_t N, int16_t * mean, int16_t * variance);
acc_measurement read_accelerometer();
void read_acceleration_distribution(uint8_t N, int16_t * x_mean, int16_t * x_var, int16_t * y_mean, int16_t * y_var, int16_t * z_mean, int16_t * z_var);
void rotate_array_by_one(int16_t data[], uint8_t N);
void print_array(int16_t data[], uint8_t N);
void low_pass_filter(int16_t means[], int16_t vars[], uint8_t N, int16_t output[]);
void equate_arrays(int16_t input[], int16_t output[], uint8_t length);
uint16_t if_variance(int16_t var1[], int16_t var2[]);
int8_t pedometer();
