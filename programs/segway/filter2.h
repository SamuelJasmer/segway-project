#ifndef FILTER2_H
#define FILTER2_H

#include "matrix.h"
#include "ringbuffer.h"
#include "vector3.h"

class filter2 {

	public:

		/**
		 * @brief  Initializes this filter with buffers of the specified size
		 * @param  buffer_size: The size of the input buffers
		 * @retval True if the filter was created successfully or false if
         *         memory could not be allocated.
		 */
		bool init(int buffer_size);

        /**
         * @brief  Frees memory used by this filter and deletes the buffers.
         * @retval None
         */
        void deinit();

        /**
         * @brief  Adds the specified sample to the input buffers and
         *         recalculates the mean, variance, covariance, and standard
         *         deviation.
         * @param  input: The new input sample
         * @retval None
         */
        void add_sample(vector input);

        /**
         * @brief  Returns the size of the internal buffers.
         * @retval The size of the internal buffers
         */
        int buffer_size();

        /**
         * @brief  Returns the sample mean of the input buffers.
         * @retval The sample mean of the input buffers
         */
		vector sample_mean();

        /**
         * @brief  Returns the sample variance of the input buffers.
         * @retval The sample variance of the input buffers
         */
		vector sample_variance();

        /**
         * @brief  Returns the covariance of the input buffers.
         * @retval The covariance of the input buffers
         */
		vector covariance();

        /**
         * @brief  Returns the standard deviation of the input buffers.
         * @retval The standard deviation of the input buffers
         */
		vector standard_deviation();

		vector least_squares_regression();
		Matrix weighted_least_squares_regression(RingBuffer* yValues, RingBuffer* variances);
		vector lowess_smooth();


	private:
        int _buffer_size;
        float _average_n;

        // Saved Statistic Values
		vector _mean;
		vector _variance;
		vector _covariance;
		vector _standardDeviation;

        // Input Buffers
		RingBuffer* in_buf_x;
		RingBuffer* in_buf_y;
		RingBuffer* in_buf_z;

        // Variance Buffers
        RingBuffer* var_buf_x;
        RingBuffer* var_buf_y;
        RingBuffer* var_buf_z;

        /**
         * Calculates the mean value of each of the in_buf_* buffers and sets
         * _mean.
         */
        void calculate_mean();

        /**
         * Calculates the variance of each of the in_buf_* buffers and sets
         * _variance. This assumes that calculate_mean has already been called.
         */
        void calculate_variance();
		void calculate_variance2();

        /**
         * Calculates the covariance of each of the in_buf_* buffers and sets
         * _covariance. This assumes that calculate_mean has already been called.
         */
        void calculate_covariance();

        /**
         * Calculates the standard deviation of each of the in_buf_* buffers and
         * sets _standardDeviation. This assumes that calculate_variance has
         * already been called.
         */
        void calculate_standard_deviation();

        /**
         * @brief  Creates a column vector from the values in the specified buffer.
         * @param  variances: RingBuffer whose values should be copied
         * @retval An nx1 matrix
         */
		Matrix create_error_matrix(RingBuffer* variances);

        /**
         * @brief  Creates an nxn diagonal matrix from the values in the
         *         specified buffer.
         * @param  variances: RingBuffer whose values should be copied
         * @retval An nxn diagonal matrix
         */
        Matrix create_weight_matrix(RingBuffer* variances);
};

#endif
