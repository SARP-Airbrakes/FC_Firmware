
#ifndef TESTING_H_
#define TESTING_H_

#define TESTING_ASSERT(cond) if (!(cond)) { return testing_result_fail; }

typedef enum {
    testing_result_ok,
    testing_result_fail,
    testing_result_unknown
} testing_result_t;

typedef struct {
    const char *name;
    testing_result_t ( *test)(void);
} testing_test_t;

extern const testing_test_t sdk_mutex_tests[];
extern const testing_test_t sdk_w25q128jv_tests[];

#ifdef __cplusplus
extern "C" {
#endif

// run the test suite and then print
void testing_test_and_print(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // TESTING_H_
