#ifndef _FFERRNO_H_
#define _FFERRNO_H_

const char * FR_Table[];

#define FF_STR_ERRNO(errno) (FR_Table[errno])

#endif