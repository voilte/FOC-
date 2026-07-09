#ifndef FOC_TEST_H
#define FOC_TEST_H

/**
 * @file foc_test.h
 * @brief 测试 / 调试入口
 *
 * 当前模式：速度环联调（恒速 FOC_TEST_TARGET_RPM）
 * main 调用 FOC_Test_Init() + 循环 FOC_Test_Poll()
 */

#include <stdint.h>

void FOC_Test_Init(void);
void FOC_Test_Poll(void);

/** 格式化打印到调试 UART（非阻塞） */
void FOC_Test_Printf(const char *fmt, ...);

#endif /* FOC_TEST_H */
