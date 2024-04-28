#ifndef ENCODER_THREAD_VALUES_h
#define ENCODER_THREAD_VALUES_h

#include "EncoderValues.h"

struct EncoderThreadValues
{
    EncoderValues shoulder;
    EncoderValues wrist;
    EncoderValues gripper;
};

#endif