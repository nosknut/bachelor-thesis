#ifndef THREAD_h
#define THREAD_h

#include "Arduino_FreeRTOS.h"

// https://github.com/feilipu/Arduino_FreeRTOS_Library/blob/master/examples/AnalogRead_DigitalRead/AnalogRead_DigitalRead.ino
struct Thread
{
    TaskHandle_t taskHandle;

public:
    int priority = 1;
    const String name;

    Thread(const String name) : name(name)
    {
    }

    ~Thread()
    {
        stop();
    }

    virtual void run() = 0;

    void start()
    {
        int stackSize = 128;

        // https://stackoverflow.com/questions/45831114/c-freertos-task-invalid-use-of-non-static-member-function
        TaskFunction_t taskFunction = [](void *taskParams)
        {
            Thread *thread = (Thread *)taskParams;
            while (true)
            {
                thread->run();
                yield();
            }
        };

        // https://www.freertos.org/a00125.html
        xTaskCreate(taskFunction, name.c_str(), stackSize, this, priority, &taskHandle);
    }

    void stop()
    {
        vTaskDelete(taskHandle);
    }
};

#endif