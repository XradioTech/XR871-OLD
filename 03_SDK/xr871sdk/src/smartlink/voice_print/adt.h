#ifndef _ADT_H_
#define _ADT_H_


/* frequency type selector */
typedef enum
{
	FREQ_TYPE_LOW = 0,      /* low frequency, 2K~5K */
	FREQ_TYPE_MIDDLE,       /* middle frequency, 8K~12K */
	FREQ_TYPE_HIGH          /* high frequency, 16K~20K */
} freq_type_t;

/* macros of return valule of decoder */
#define RET_DEC_ERROR           -1      /* decoder error */
#define RET_DEC_NORMAL          0       /* decoder normal return */
#define RET_DEC_NOTREADY        1       /* cant get result for decoder has not finished */
#define RET_DEC_END             2       /* decoder end */


/* definition of decoder config paramters */
typedef struct
{
	int max_strlen;         /* max supported length of string */
	int sample_rate;        /* sample rate */
	freq_type_t freq_type;  /* freq type */
	int group_symbol_num;   /* symbol number of every group */
	int error_correct;      /* use error correcting code function */
	int error_correct_num;  /* error correcting code capabilty */
} config_decoder_t;

/**
 * @brief Create decoder .
 * @param decode_config:
 *        @decode config param.
 * @retval  decoder handler, faild if NULL.
 */
void *decoder_create(config_decoder_t *decode_config );

/**
 * @brief Reset decoder.
 * @param handle:
 *        @decoder handler.
 * @retval  None.
 */
void decoder_reset(void *handle);

/**
 * @brief Get sample number of every sample frame.
 * @param handle:
 *        @decoder handler.
 * @retval  sample number of every sample frame(16bit of every sample data).
 */
int decoder_getbsize(void *handle);

/**
 * @brief Fed decoder data.
 * @param handle:
 *        @decoder handler.
 * @param pcm:
 *        @pcm data, the sample number is equal to decoder_getbsize.
 * @retval  defined by RET_DEC_XX.
 */
int decoder_fedpcm(void *handle, short *pcm);

/**
 * @brief Get decode result.
 * @param handle:
 *        @decoder handler.
 * @param str:
 *        @decode result string.
 * @retval defined by RET_DEC_XX.
 */
int decoder_getstr(void *handle, unsigned char *str);

/**
 * @brief Release decoder handler.
 * @param handle:
 *        @decoder handler.
 * @retval  None.
 */
void decoder_destroy(void *handle);

#endif
