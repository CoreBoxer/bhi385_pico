#ifndef _COINES_H_
#define _COINES_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 这是一个空壳替身头文件。
 * bhi385_parse.c 里所有真正调用 coines_set_pin_config() 的代码，
 * 都包裹在 #if defined(MCU_APP30) / #if defined(MCU_APP31) 条件编译块中。
 * 只要我们的PlatformIO构建不主动定义这些宏（默认就不会定义），
 * 这些调用会在预处理阶段被整体剔除，不会进入实际编译，
 * 因此这里不需要提供任何真实函数实现。
 */

#ifdef __cplusplus
}
#endif

#endif /* _COINES_H_ */