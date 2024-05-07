#ifndef THREAD_h
#define THREAD_h

#include "FreeRTOS.h"

// https://github.com/feilipu/Arduino_FreeRTOS_Library/blob/master/examples/AnalogRead_DigitalRead/AnalogRead_DigitalRead.ino
struct Thread
{
    TaskHandle_t taskHandle;

public:
    const int priority;
    const int stackSize;
    const String name;

    /**
     * Create a thread
     * @param priority The priority of the thread. 3 is highest, 0 is lowest
     * @param stackSize The size of the stack
     * @param name The name of the thread
     */
    Thread(const String name, int priority = 1, int stackSize = 128) : name(name), priority(priority), stackSize(stackSize)
    {
    }

    ~Thread()
    {
        stop();
    }

    virtual void run() = 0;

    static void taskFunction(void *taskParams)
    {
        Thread *thread = (Thread *)taskParams;
        while (true)
        {
            thread->run();
            yield();
        }
    };

    void start()
    {
        // https://www.freertos.org/a00125.html
        xTaskCreate(Thread::taskFunction, name.c_str(), stackSize, this, priority, &taskHandle);
    }

    void stop()
    {
        vTaskDelete(taskHandle);
    }
};

#endif