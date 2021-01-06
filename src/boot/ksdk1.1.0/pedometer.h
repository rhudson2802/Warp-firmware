#include <stdlib.h>

typedef struct distribution{
	int16_t mean;
	int16_t variance;
} distribution;

typedef struct acc_distribution{
	distribution x;
	distribution y;
	distribution z;
} acc_distribution;

typedef struct acc_measurement{
	int16_t x;
	int16_t y;
	int16_t z;
} acc_measurement;


int16_t compute_mean(int16_t data[], uint8_t N);
int16_t compute_variance(int16_t data[], int16_t mean, uint8_t N);
distribution generate_distribution(int16_t data[], uint8_t N);
acc_measurement read_accelerometer();
acc_distribution read_acceleration_distribution(uint8_t N);
void rotate_array_by_one(int16_t data[], uint8_t N);
void print_array(int16_t data[], uint8_t N);
void low_pass_filter(int16_t means[], int16_t vars[], uint8_t N, int16_t * output, int16_t * uncertainty);
void equate_arrays(int16_t input[], int16_t output[], uint8_t length);
void print_acc_distribution(acc_distribution dist);
int8_t pedometer();
