
#include <testing.h>

#include <usbd_cdc_if.h>
#include <cstdarg>
#include <cstdio>

struct test_suite {
    const char *name;
    const testing_test_t *tests;
};

static const test_suite tests[] = {
    { "sdk::w25q128jv", sdk_w25q128jv_tests, },
    { nullptr, nullptr, }
};

static void serial_print(const char *buf)
{
    int i = 0;
    const char *tmp;
    for (tmp = buf; *tmp; tmp++)
        i++;
    CDC_Transmit_FS((uint8_t *) buf, i);
}

static void serial_printf(const char *format, ...)
{
    char buf[128];
    std::va_list vargs;
    va_start(vargs, format);
    vsnprintf(buf, sizeof(buf), format, vargs);
    va_end(vargs);
    serial_print(buf);
}

extern "C" {

void testing_test_and_print()
{
    const test_suite *suite;
    
    serial_print("Tests:\r\n");
    for (suite = tests; suite->name; suite++) {
        const testing_test_t *test;
        int fail_count = 0;
        int success_count = 0;

        serial_printf("SUITE [%s]:\r\n", suite->name);
        for (test = suite->tests; test->name; test++) {
            serial_printf("  %s - ", test->name);
            
            testing_result_t result = test->test();
            switch (result) {
            case testing_result_ok:
                success_count++;
                serial_print("[ok]\r\n");
                break;
            case testing_result_fail:
                fail_count++;
                serial_print("[failed]\r\n");
                break;
            default:
                serial_print("[unknown]\r\n");
                break;
            }
        }

        serial_printf("\r\nSUITE [%s]: %d succeeded, %d failed\r\n\r\n",
                suite->name, success_count, fail_count);
    }
}

}
