#include <stdlib.h>

typedef struct distribution{
	int32_t mean;
	int32_t variance;
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

int32_t compute_mean(int16_t data[], uint8_t N);
int32_t compute_variance(int16_t data[], int32_t mean, uint8_t N);
distribution generate_distribution(int16_t data[], uint8_t N);
acc_measurement read_accelerometer();
acc_distribution read_acceleration_distribution(uint8_t N);
void rotate_array_by_one(int32_t data[], uint8_t N);
void print_array(int32_t data[], uint8_t N);
void low_pass_filter(int32_t means[], int32_t vars[], uint8_t N, int32_t * output, int32_t * uncertainty);
void print_acc_distribution(acc_distribution dist);
int8_t pedometer();
