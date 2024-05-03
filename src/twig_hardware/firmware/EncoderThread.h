#ifndef ENCODER_THREAD_h
#define ENCODER_THREAD_h

#include "Mutex.h"
#include "Thread.h"
#include "Encoder.h"
#include "FirmwareConfig.h"
#include "EncoderThreadValues.h"


// Manages a thread for the update method
class EncoderThread : public Thread
{
public:
    void run() override
    {
        EncoderThreadValues newValues;

        wristEncoder.update();
        shoulderEncoder.update();
        gripperEncoder.update();

        newValues.shoulder = shoulderEncoder.values;
        newValues.wrist = wristEncoder.values;
        newValues.gripper = gripperEncoder.values;

        if (!values.setValue(newValues))
        {
#ifdef DEBUG_ENCODERS
            Serial.println("ERROR: Unable to update encoder values: Resource busy");
#endif
        }
    }

    Mutex<EncoderThreadValues> values = Mutex<EncoderThreadValues>(EncoderThreadValues());

    Encoder shoulderEncoder = Encoder(SHOULDER_ENCODER_SDA_PIN, SHOULDER_ENCODER_SCL_PIN, INITIAL_MIN_ENCODER_MAGNITUDE, "Shoulder");
    Encoder wristEncoder = Encoder(WRIST_ENCODER_SDA_PIN, WRIST_ENCODER_SCL_PIN, INITIAL_MIN_ENCODER_MAGNITUDE, "Wrist");
    Encoder gripperEncoder = Encoder(GRIPPER_ENCODER_SDA_PIN, GRIPPER_ENCODER_SCL_PIN, INITIAL_MIN_ENCODER_MAGNITUDE, "Gripper");
    
    EncoderThread() : Thread("Encoder Thread", 2)
    {
    }

    ~EncoderThread()
    {
        end();
    }

    bool getValues(EncoderThreadValues &out)
    {
        return values.getValue(out);
    }

    void begin()
    {
        shoulderEncoder.begin();
        wristEncoder.begin();
        gripperEncoder.begin();
        start();
    }

    void end()
    {
        shoulderEncoder.end();
        wristEncoder.end();
        gripperEncoder.end();
        stop();
    }
};

#endif