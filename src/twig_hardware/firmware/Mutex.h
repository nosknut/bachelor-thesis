#ifndef MUTEX_h
#define MUTEX_h


#include <FreeRTOS.h>
#include <semphr.h>

// https://github.com/feilipu/Arduino_FreeRTOS_Library/blob/master/examples/Mutex/Mutex.ino
template<typename T>
class Mutex {
private:
    T value;
    SemaphoreHandle_t mutex;

public:
    Mutex(T initialValue) : value(initialValue) {
        mutex = xSemaphoreCreateMutex();
    }

    ~Mutex() {
        vSemaphoreDelete(mutex);
    }

    bool getValue(T& result) {
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            result = value;
            xSemaphoreGive(mutex);
            return true;
        }
        return false;
    }

    bool setValue(const T& newValue) {
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            value = newValue;
            xSemaphoreGive(mutex);
            return true;
        }
        return false;
    }
};

#endif
