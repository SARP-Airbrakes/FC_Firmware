
#include <testing.h>

#include <sdk/mutex.h>

#include <FreeRTOS.h>
#include <semphr.h>

/**
 * A test to ensure that the mutex is dynamically allocated correctly.
 */
testing_result_t sdk_mutex_allocate(void)
{
    sdk::mutex mutex;
    SemaphoreHandle_t handle = static_cast<SemaphoreHandle_t>(mutex.unwrap());
    
    if (handle == nullptr) {

    }
}

const testing_test_t sdk_mutex_tests[] = {
    {"Allocate mutex", sdk_mutex_allocate},
    { nullptr, nullptr }
};
