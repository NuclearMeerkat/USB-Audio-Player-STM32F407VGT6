#include "arm_math.h"
#include "stm32f4xx.h"
/**
* @brief Instance structure for the floating-point transposed direct form II Biquad cascade filter.
*/
typedef struct
{
	  uint8_t numStages;         /**< number of 2nd order stages in the filter.  Overall order is 2*numStages. */
	  float32_t *pState;         /**< points to the array of state coefficients.  The array is of length 4*numStages. */
const float32_t *pCoeffs;        /**< points to the array of coefficients.  The array is of length 5*numStages. */
} arm_biquad_cascade_stereo_df2T_instance_f32;

void arm_biquad_cascade_stereo_df2T_f32(
  const arm_biquad_cascade_stereo_df2T_instance_f32 * S,
  const float32_t * pSrc,
        float32_t * pDst,
        uint32_t blockSize);

void arm_biquad_cascade_stereo_df2T_init_f32(
        arm_biquad_cascade_stereo_df2T_instance_f32 * S,
        uint8_t numStages,
  const float32_t * pCoeffs,
        float32_t * pState);
