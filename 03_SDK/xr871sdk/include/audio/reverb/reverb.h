#ifndef REVERB_H
#define REVERB_H

typedef struct _reverb_info{
	unsigned short NumChannels;
	unsigned long  SampleRate;
	unsigned short BitsPerSample;
	unsigned int   delay_sample;     //ms
	short          feed_ratio;
	void*          reverb_buffer;
	unsigned short write_pos;
}reverb_info;

extern int reverb_init(reverb_info *info);
extern void reverb_comb(reverb_info *info,void* read_buffer,short buffer_length);
extern void reverb_release(reverb_info *info);
#endif