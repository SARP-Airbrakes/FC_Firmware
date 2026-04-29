
#include <selfcheck.h>
#include <airbrakes.h>

#include <cstdio>
#include <cmsis_os.h>

#include <Eigen/Eigen>

struct selfcheck_test {
    enum status {
        OK,
        FAIL,
    };

    const char *name;
    status ( *test)(void);
    void ( *print_help)(void);
};

extern const selfcheck_test selfcheck_tests[];
int selfcheck_flag = -1;

void selfcheck_test_and_print(void)
{
    printf("Running diagnostic tests...\r\n");

    int running_total = 0;
    int success_total = 0;
    int fail_total = 0;

    for (const selfcheck_test *test = selfcheck_tests; test->name; test++) {
        if (test->test == nullptr) {
            printf("\r\n%s\r\n", test->name);
            continue;
        }
        printf("[ ... ] %s\r", test->name);

        osDelay(10);
        selfcheck_flag = -1;
        auto status = test->test();

        running_total++;

        if (status == selfcheck_test::OK) {
            success_total++;

            // extra space to clear the extra character
            printf("[ OK ] %s \r\n", test->name);
        } else if (status == selfcheck_test::FAIL) {
            fail_total++;
            printf("[ FAIL ] %s\r\n", test->name);
            if (test->print_help != nullptr)
                test->print_help();
        }
    }

    printf("\r\n");
    printf("%d test(s) ran: %d succeeded, %d failed\r\n", running_total,
            success_total, fail_total);
}

#define SELFCHECK_UNWRAP_BOOL(status) \
    do { \
        auto status_ = (status); \
        if (!status_.is_ok() || !status_.unwrap()) { \
            selfcheck_flag = static_cast<int>(status_.err); \
            return selfcheck_test::FAIL; \
        } \
        return selfcheck_test::OK; \
    } while (0)
#define SELFCHECK_PRINT_STATUS() \
    do { \
        if (selfcheck_flag != -1) \
            printf(" - received status: %d\r\n", selfcheck_flag); \
    } while(0)

const selfcheck_test selfcheck_tests[] = {
    { "General stability checks:", NULL, NULL },
    {
        "Interfaces loaded",
        []() {
            return (state_handle != nullptr) ?
                selfcheck_test::OK :
                selfcheck_test::FAIL;
        },
        []() {
            printf(" - this is very bad. this requires re-flashing\r\n");
            printf(" - the following tests will exhibit undefined behavior\r\n");
            printf(" - confirmed not ready for flight\r\n");
        }
    },
    {
        "Matrix multiplication",
        []() {
            Eigen::Matrix3f matrix0 = Eigen::Matrix3f::Constant(3, 3, 1);
            Eigen::Matrix3f matrix1 = Eigen::Matrix3f::Constant(3, 3, 2);
            Eigen::Matrix3f result = matrix0 * matrix1;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    if (result(i, j) != 6)
                        return selfcheck_test::FAIL;
            return selfcheck_test::OK;
        },
        []() {
            printf(" - flight-critical failure\r\n");
            printf(" - requires immediate reflashing\r\n");
            printf(" - confirmed not ready for flight\r\n");
        }
    },
    {
        "Small FreeRTOS memory allocation",
        []() {
            void *memory = pvPortMalloc(16);
            bool valid = memory != nullptr;
            vPortFree(memory);
            return valid ? selfcheck_test::OK : selfcheck_test::FAIL;
        },
        []() {
            printf(" - may be out of heap memory, rebooting may help\r\n");
            printf(" - possible memory leak\r\n");
        }
    },
    {
        "Small libc memory allocation",
        []() {
            char *memory = (char *) malloc(16);
            bool valid = memory != nullptr;
            free(memory);
            return valid ? selfcheck_test::OK : selfcheck_test::FAIL;
        },
        []() {
            printf(" - may be out of system heap (seperate from FreeRTOS heap), rebooting may help\r\n");
            printf(" - possible memory leak, probably related to Eigen\r\n");
        }
    },
    {
        "Small operator new allocation",
        []() {
            // std::bad_alloc possible here
            char *memory = new char[16]();
            bool valid = memory != nullptr;
            delete[] memory;
            return valid ? selfcheck_test::OK : selfcheck_test::FAIL;
        },
        []() {
            printf(" - this one is harder to diagnose\r\n");
        }
    },
    {
        "RTOS delay",
        []() {
            return (osDelay(10) == osOK) ? selfcheck_test::OK : selfcheck_test::FAIL;
        },
        []() {
            printf(" - rtos unstable, may need to reboot\r\n");
            printf(" - possible structural firmware issue\r\n");
        }
    },

    { "Connectivity checks:", NULL, NULL },
    {
        "IMU connection",
        []() {
            SELFCHECK_UNWRAP_BOOL(state_handle->imu.is_connected());
        },
        []() {
            printf(" - check that the imu is connected\r\n");
            printf(" - possible power problem\r\n");
            SELFCHECK_PRINT_STATUS();
        }
    },
    {
        "Barometer connection",
        []() {
            SELFCHECK_UNWRAP_BOOL(state_handle->baro.is_connected());
        },
        []() {
            printf(" - check that the baro is connected\r\n");
            printf(" - possible power problem\r\n");
            SELFCHECK_PRINT_STATUS();
        }
    },
    {
        "Flash connection",
        []() {
            SELFCHECK_UNWRAP_BOOL(state_handle->flash.is_connected());
        },
        []() {
            printf(" - check that the flash chip is connected\r\n");
            printf(" - possible power problem\r\n");
            SELFCHECK_PRINT_STATUS();
        }
    },
    { NULL, NULL, NULL },
};
