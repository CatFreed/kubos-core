/*
 * KubOS Core Flight Services
 * Copyright (C) 2016 Kubos Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include "kubos-core/modules/klog.h"

#include "kubos-core/unity/unity.h"

#define LOG_PATH "/tmp/_klog"

inline bool _fstat(char *path, int *size) {
    struct stat st;
    if (stat(path, &st) != 0) {
        return false;
    }

    if (size) {
        *size = (int) st.st_size;
    }
    return true;
}

#define remove_if_exists(p) if (_fstat(p, NULL)) remove(p)

void setUp(void)
{
    klog_console_level = LOG_NONE;
}

void tearDown(void)
{
    klog_console_level = LOG_INFO;
    klog_file_level = LOG_DEBUG;

    klog_cleanup();
    remove_if_exists(LOG_PATH ".000");
    remove_if_exists(LOG_PATH ".001");
    remove_if_exists(LOG_PATH ".002");
    remove_if_exists(LOG_PATH ".003");
    remove_if_exists(LOG_PATH ".004");
}

void test_FileLog(void)
{
    klog_file_level = LOG_DEBUG;
    int result = klog_init_file(LOG_PATH, strlen(LOG_PATH), 256, 1);
    TEST_ASSERT_EQUAL_INT(result, 0);


    KLOG_INFO("test", "123:%d", 456);
    KLOG_WARN("logger", "hi");
    KLOG_ERR("o", "error");
    KLOG_DEBUG("b", "debug");
    klog_cleanup();

    FILE *log_file = fopen(LOG_PATH ".000", "r");
    TEST_ASSERT_NOT_NULL(log_file);

    char buffer[256];
    char lines[4][64];
    int i = 0;
    size_t bytes = fread(buffer, 1, 256, log_file);
    buffer[bytes] = '\0';

    char *token = strtok(buffer, "\n");
    while (NULL != token)
    {
        strcpy(lines[i++], token);
        token = strtok(NULL, "\n");
    }

    TEST_ASSERT_EQUAL_STRING_MESSAGE(&lines[0][14], " test:I 123:456", "Info log no good");
    TEST_ASSERT_EQUAL_STRING_MESSAGE(&lines[1][14], " logger:W hi", "Warn log no good");
    TEST_ASSERT_EQUAL_STRING_MESSAGE(&lines[2][14], " o:E error", "Error log no good");
    TEST_ASSERT_EQUAL_STRING_MESSAGE(&lines[3][14], " b:D debug", "Debug log no good");
    TEST_ASSERT_EQUAL_INT(i, 4);
}


static void test_RotateParts(void)
{
    int result = klog_init_file(LOG_PATH, strlen(LOG_PATH), 32, 3);
    int size = 0;

    TEST_ASSERT_EQUAL_INT(result, 0);

    // This should be a line length of 21
    KLOG_WARN("a", "b");
    KLOG_WARN("a", "b");
    klog_cleanup();

    TEST_ASSERT(_fstat(LOG_PATH ".000", &size));
    TEST_ASSERT_EQUAL_INT(size, 42);

    KLOG_WARN("a", "b");
    klog_cleanup();

    TEST_ASSERT(_fstat(LOG_PATH ".000", &size));
    TEST_ASSERT_EQUAL_INT(size, 42);
    TEST_ASSERT(_fstat(LOG_PATH ".001", &size));
    TEST_ASSERT_EQUAL_INT(size, 21);

    KLOG_WARN("a", "b");
    klog_cleanup();

    TEST_ASSERT(_fstat(LOG_PATH ".000", &size));
    TEST_ASSERT_EQUAL_INT(size, 42);
    TEST_ASSERT(_fstat(LOG_PATH ".001", &size));
    TEST_ASSERT_EQUAL_INT(size, 42);

    // force rotation
    KLOG_WARN("a", "b");
    KLOG_WARN("a", "b");
    klog_cleanup();

    TEST_ASSERT(_fstat(LOG_PATH ".000", &size));
    TEST_ASSERT_EQUAL_INT(size, 0);
    TEST_ASSERT(_fstat(LOG_PATH ".001", &size));
    TEST_ASSERT_EQUAL_INT(size, 42);
    TEST_ASSERT(_fstat(LOG_PATH ".002", &size));
    TEST_ASSERT_EQUAL_INT(size, 42);

    KLOG_WARN("aa", "bb");
    klog_cleanup();

    TEST_ASSERT(_fstat(LOG_PATH ".000", &size));
    TEST_ASSERT_EQUAL_INT(size, 23);
    TEST_ASSERT(_fstat(LOG_PATH ".001", &size));
    TEST_ASSERT_EQUAL_INT(size, 42);
    TEST_ASSERT(_fstat(LOG_PATH ".002", &size));
    TEST_ASSERT_EQUAL_INT(size, 42);
}

void resetTest(void)
{
    tearDown();
    setUp();
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_FileLog);
    RUN_TEST(test_RotateParts);
    return UNITY_END();
}
