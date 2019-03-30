#include "filter2.h"

#include <math.h>
#include "Arduino.h"
#include "matrix.h"
#include "vector3.h"
#include "stdlib.h"

bool filter2::init(int buffer_size) {

    // Delete existing buffers
    this->deinit();

    this->_buffer_size = buffer_size;

    // 1+2+3+... = n(n+1)/2, averaging cancels n
    this->_average_n = (float)(buffer_size + 1) / 2;

    this->in_buf_x   = ringbuffer_new(buffer_size);
    this->in_buf_y   = ringbuffer_new(buffer_size);
    this->in_buf_z   = ringbuffer_new(buffer_size);
    this->var_buf_x  = ringbuffer_new(buffer_size);
    this->var_buf_y  = ringbuffer_new(buffer_size);
    this->var_buf_z  = ringbuffer_new(buffer_size);

    // If any buffer could not be allocated,
    // delete allocated buffers and return error
    if (   this->in_buf_x   == NULL
        || this->in_buf_y   == NULL
        || this->in_buf_z   == NULL
        || this->var_buf_x  == NULL
        || this->var_buf_y  == NULL
        || this->var_buf_z  == NULL) {
        this->deinit();
        return false;
    }

    // Fill buffers to fill capacity with zeros
    ringbuffer_fill(this->in_buf_x,   0.0f);
    ringbuffer_fill(this->in_buf_y,   0.0f);
    ringbuffer_fill(this->in_buf_z,   0.0f);
    ringbuffer_fill(this->var_buf_x,  0.0f);
    ringbuffer_fill(this->var_buf_y,  0.0f);
    ringbuffer_fill(this->var_buf_z,  0.0f);

    return true;
}

void filter2::deinit() {
    if (this->_buffer_size == 0) return;

    if (this->in_buf_x  != NULL) ringbuffer_destroy(this->in_buf_x);
    if (this->in_buf_y  != NULL) ringbuffer_destroy(this->in_buf_y);
    if (this->in_buf_z  != NULL) ringbuffer_destroy(this->in_buf_z);
    if (this->var_buf_x != NULL) ringbuffer_destroy(this->var_buf_x);
    if (this->var_buf_y != NULL) ringbuffer_destroy(this->var_buf_y);
    if (this->var_buf_z != NULL) ringbuffer_destroy(this->var_buf_z);

    this->in_buf_x   = NULL;
    this->in_buf_y   = NULL;
    this->in_buf_z   = NULL;
    this->var_buf_x  = NULL;
    this->var_buf_y  = NULL;
    this->var_buf_z  = NULL;

    this->_buffer_size = 0;
}

void filter2::add_sample(vector input) {

    ringbuffer_dequeue(this->in_buf_x);
    ringbuffer_dequeue(this->in_buf_y);
    ringbuffer_dequeue(this->in_buf_z);
    ringbuffer_enqueue(this->in_buf_x, input.x);
    ringbuffer_enqueue(this->in_buf_y, input.y);
    ringbuffer_enqueue(this->in_buf_z, input.z);

    calculate_mean();
    calculate_variance();
    calculate_covariance();
    calculate_standard_deviation();

    ringbuffer_dequeue(this->var_buf_x);
    ringbuffer_dequeue(this->var_buf_y);
    ringbuffer_dequeue(this->var_buf_z);
    ringbuffer_enqueue(this->var_buf_x, this->_variance.x);
    ringbuffer_enqueue(this->var_buf_y, this->_variance.y);
    ringbuffer_enqueue(this->var_buf_z, this->_variance.z);

}

//region Getters
int filter2::buffer_size() {
    return this->_buffer_size;
}

vector filter2::sample_mean() {
    return this->_mean;
}

vector filter2::sample_variance() {
    return this->_variance;
}

vector filter2::covariance() {
    return this->_covariance;
}

vector filter2::standard_deviation() {
    return this->_standardDeviation;
}
//endregion

vector filter2::least_squares_regression() {
    vector mean = this->_mean;
    vector variance = this->_variance;
    vector cov = this->_covariance;

    vector b;
    b.x = cov.x / variance.x;
    b.y = cov.y / variance.y;
    b.z = cov.z / variance.z;

    vector a;
    a.x = mean.x - (b.x * this->_average_n);
    a.y = mean.y - (b.y * this->_average_n);
    a.z = mean.z - (b.z * this->_average_n);

    vector output;
    output.x = a.x + b.x * (this->_buffer_size / 2);
    output.y = a.y + b.y * (this->_buffer_size / 2);
    output.z = a.z + b.z * (this->_buffer_size / 2);

    return output;
}

Matrix filter2::weighted_least_squares_regression(RingBuffer* yValues, RingBuffer* variances) {

    Matrix X_Matrix;
	X_Matrix = Matrix(this->_buffer_size, 2);

	Matrix Y_Matrix;
	Y_Matrix = Matrix(this->_buffer_size, 1);

	Matrix B_Matrix;
	B_Matrix = Matrix(2, 1);

	Matrix Error_Matrix = this->create_error_matrix(variances);
	Matrix Weight_Matrix = this->create_weight_matrix(variances);

    for (int i = 0; i < this->_buffer_size; i++) {
		X_Matrix.set(i, 0, i);
		X_Matrix.set(i, 1, i);
		Y_Matrix.set(i, 0, ringbuffer_at(yValues, i));
	}

    // Calculate slope
    Matrix temp = X_Matrix.transpose() * Weight_Matrix;
	B_Matrix = (temp * X_Matrix).inverse() * (temp * Y_Matrix);

	return B_Matrix;
}

vector filter2::lowess_smooth() {
    Matrix B_Matrix_x;
	B_Matrix_x = weighted_least_squares_regression(this->in_buf_x, this->var_buf_x);

	Matrix B_Matrix_y;
	B_Matrix_y = weighted_least_squares_regression(this->in_buf_y, this->var_buf_y);

	Matrix B_Matrix_z;
	B_Matrix_z = weighted_least_squares_regression(this->in_buf_z, this->var_buf_z);

    int n = this->_buffer_size;
    vector output;
	output.x = B_Matrix_x.get(0, 0) * ((float)n / 2) + B_Matrix_x.get(1, 0);
	output.y = B_Matrix_y.get(0, 0) * ((float)n / 2) + B_Matrix_y.get(1, 0);
	output.z = B_Matrix_z.get(0, 0) * ((float)n / 2) + B_Matrix_z.get(1, 0);

    return output;
}


//region Private Calculations
void filter2::calculate_mean() {
    vector sum;
    sum.clear();
    for (int i = 0; i < this->_buffer_size; i++) {
        sum.x += ringbuffer_at(this->in_buf_x, i);
        sum.y += ringbuffer_at(this->in_buf_y, i);
        sum.z += ringbuffer_at(this->in_buf_z, i);
    }

    this->_mean.x = sum.x / this->_buffer_size;
    this->_mean.y = sum.y / this->_buffer_size;
    this->_mean.z = sum.z / this->_buffer_size;
}

void filter2::calculate_variance() {
    vector sum;
	sum.clear();
	for (int i = 0; i < this->_buffer_size; i++) {
		sum.x += sq((ringbuffer_at(this->in_buf_x, i) - this->_mean.x));
		sum.y += sq((ringbuffer_at(this->in_buf_y, i) - this->_mean.y));
		sum.z += sq((ringbuffer_at(this->in_buf_z, i) - this->_mean.z));
	}

    this->_variance.x = sum.x / (this->_buffer_size - 1);
    this->_variance.y = sum.y / (this->_buffer_size - 1);
    this->_variance.z = sum.z / (this->_buffer_size - 1);
}

void filter2::calculate_variance2() {
	float n = _buffer_size;

	vector sum;
	sum.clear();
	for (int i = 0; i < this->_buffer_size; i++) {
		sum.x += sq(ringbuffer_at(this->in_buf_x, i));
		sum.y += sq(ringbuffer_at(this->in_buf_y, i));
		sum.z += sq(ringbuffer_at(this->in_buf_z, i));
	}

	this->_variance.x = (sum.x - (n * sq(this->_mean.x))) / (n - 1);
	this->_variance.y = (sum.x - (n * sq(this->_mean.y))) / (n - 1);
	this->_variance.z = (sum.x - (n * sq(this->_mean.z))) / (n - 1);

}

void filter2::calculate_covariance() {
    vector sum;
	sum.clear();
	for (int i = 0; i < this->_buffer_size; i++) {
		sum.x += sq((ringbuffer_at(this->in_buf_x, i) - this->_mean.x) * (i - this->_average_n));
		sum.y += sq((ringbuffer_at(this->in_buf_y, i) - this->_mean.y) * (i - this->_average_n));
		sum.z += sq((ringbuffer_at(this->in_buf_z, i) - this->_mean.z) * (i - this->_average_n));
	}

    this->_covariance.x = sum.x / (this->_buffer_size - 1);
    this->_covariance.y = sum.y / (this->_buffer_size - 1);
    this->_covariance.z = sum.z / (this->_buffer_size - 1);
}

void filter2::calculate_standard_deviation() {
    this->_standardDeviation.x = sqrt(this->_variance.x);
    this->_standardDeviation.y = sqrt(this->_variance.y);
    this->_standardDeviation.z = sqrt(this->_variance.z);
}
//endregion

Matrix filter2::create_error_matrix(RingBuffer* variances) {

    int n = variances->count;
	Matrix Error_matrix;
	Error_matrix = Matrix(n, 1);

	//Create Error Matrix
	for (int i = 0; i < n; i++) {
		//Create a column vector of length n with each position as the current variance at i
		Error_matrix.set(i, 0, ringbuffer_at(variances, i));
	}

	return Error_matrix;
}

Matrix filter2::create_weight_matrix(RingBuffer* variances) {

    int n = variances->count;
	Matrix Weight_matrix;
	Weight_matrix = Matrix(n, n);

	//Create Weight Matrix
	for (int row = 0; row < n; row++) {
		//row
		for (int col = 0; col < n; col++) {
			//column
			if (row == col) {
				//set diagonal positions to ( 1 / current variance ) at that row
				Weight_matrix.set(row, col, 1 / ringbuffer_at(variances, row));
			}
			else {
				//set non diagonal positions to 0
				Weight_matrix.set(row, col, 0);
			}
		}
	}

	return Weight_matrix;
}
