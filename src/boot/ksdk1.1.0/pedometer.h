#include <stdlib.h>

typedef struct distribution{
	int mean;
	int variance;
} distribution;

int compute_mean(int data[], int N);
int compute_variance(int data[], int mean, int N);
distribution generate_distribution(int data[], int N);
int pedometer();
