# RT1064 `zf_common` 通用层学习笔记

## 1. 我对 `zf_common` 的整体理解

`zf_common` 不是某一个单独外设的驱动，而是整个开源库的“公共基础层”。

它主要解决这几类问题：

1. 统一基础类型和编译器适配。
2. 提供系统初始化入口，例如时钟和中断分组。
3. 提供调试能力，例如 `zf_assert`、`zf_log`、`debug_init`。
4. 提供通用数据结构和工具函数，例如 FIFO、字符串转换、简化版 `sprintf`。
5. 提供异常和中断向量的兜底实现。

可以把整个库理解成这样一层层往上叠：

```text
zf_common        -> 公共基础层
zf_driver        -> 芯片外设驱动层
zf_device        -> 外接模块驱动层
zf_components    -> 组件层
user             -> 自己写的应用层
```

所以学习 `zf_common` 的目标，不是死记 API，而是先建立一个判断：

- 哪些能力是“所有模块都依赖”的。
- 哪些文件应该最先读。
- 写应用时哪些初始化顺序不能错。

---

## 2. 目录中文件分别做什么

`zf_common` 目录下当前主要文件：

- `zf_common_headfile.h`
  统一总头文件。它把 SDK、`zf_common`、`zf_driver`、`zf_device`、组件层头文件都串起来。大多数示例工程都直接 `#include "zf_common_headfile.h"`。
- `zf_common_typedef.h`
  基础类型、布尔宏、内联宏、弱定义宏、对齐和段属性宏。
- `zf_common_clock.h/.c`
  系统时钟初始化入口，内部还会顺带做延时系统和中断分组初始化。
- `zf_common_interrupt.h/.c`
  全局中断开关、指定中断开关、优先级设置、中断分组初始化。
- `zf_common_debug.h/.c`
  断言、日志、调试串口、`printf` 重定向、调试接收环形缓冲。
- `zf_common_fifo.h/.c`
  环形 FIFO 队列，很多串口、无线、相机模块都在用。
- `zf_common_function.h/.c`
  常见数学与字符串转换函数，外加一个轻量版 `zf_sprintf`。
- `zf_common_font.h/.c`
  字库和颜色定义，主要给显示类设备使用。
- `zf_common_vector.h/.c`
  异常处理函数和弱中断入口，属于底层兜底层。

---

## 3. 建议的阅读顺序

我建议按下面顺序看：

1. `zf_common_headfile.h`
   先看它把哪些层串起来，理解这个库的总入口。
2. `zf_common_typedef.h`
   先认清常用类型、宏和段属性，不然后面很多定义会看不顺。
3. `zf_common_clock.c`
   明白系统是怎么启动起来的。
4. `zf_common_interrupt.c`
   明白中断优先级和全局中断控制。
5. `zf_common_debug.c`
   这是排错最重要的模块，值得细看。
6. `zf_common_fifo.c`
   这是最常复用的数据结构，很多中断收数都靠它。
7. `zf_common_function.c`
   作为工具箱看，按需查。
8. `zf_common_vector.c`
   最后看，理解异常和默认中断入口机制。

---

## 4. 从启动流程理解通用层

很多示例 `main` 的开头都差不多：

```c
clock_init(SYSTEM_CLOCK_600M);
debug_init();
```

这两句非常关键。

### 4.1 `clock_init()` 实际做了什么

从 `zf_common_clock.c` 可以看到，`clock_init()` 主要做了几件事：

1. 记录全局变量 `system_clock`。
2. 调用 `BOARD_ConfigMPU()`。
3. 调用 `BOARD_InitBootClocks()`。
4. 调用 `system_delay_init()`。
5. 调用 `interrupt_init()`。

这说明一个重要事实：

`zf_common` 虽然提供了统一入口，但真正的时钟树配置仍然依赖 NXP SDK 和板级文件，例如 `board.h`、`clock_config.h`。

也就是说：

- `clock_init()` 是你应用层视角的统一入口。
- 真正的底层时钟细节，是由板级配置文件决定的。

### 4.2 为什么 `clock_init()` 必须最先调用

因为它后面连着两类基础能力：

- 延时系统初始化。
- 中断分组初始化。

如果你还没初始化时钟和延时系统，就去跑外设初始化、定时器配置、串口波特率配置，后面行为很容易不稳定。

---

## 5. `zf_common_headfile.h` 应该怎么理解

这个文件本质上是“总入口头文件”。

它按层次把头文件组织成几段：

1. 标准 C 头文件。
2. NXP SDK 底层头文件。
3. `zf_common` 公共层头文件。
4. `zf_driver` 外设驱动层头文件。
5. `zf_device` 外接设备层头文件。
6. 组件层头文件。

这给我一个非常重要的阅读思路：

- 写应用时，优先从 `zf_common_headfile.h` 入手，减少头文件管理负担。
- 真正深入学习某个模块时，再去看它自己的 `.h/.c`。

实际开发中，大多数 `main.c` 直接包含这个头文件就够用了。

---

## 6. `zf_common_typedef.h` 的重点

这个文件看起来很长，但核心只有三类内容。

### 6.1 基础类型别名

例如：

- `uint8` / `uint16` / `uint32`
- `int8` / `int16` / `int32`
- `vuint8` / `vuint16` / `vuint32`

虽然标准库已经有 `uint32_t` 这类类型，但这个库统一用自己的命名风格，目的是让整套代码风格一致。

### 6.2 常用宏

例如：

- `ZF_ENABLE` / `ZF_DISABLE`
- `ZF_TRUE` / `ZF_FALSE`
- `ZF_INLINE`
- `ZF_WEAK`
- `ZF_PACKED`

这些宏经常出现在驱动代码里，特别是结构体打包、弱函数覆盖、内联函数定义。

### 6.3 段属性和内存区域宏

例如：

- `AT_ITCM_SECTION_INIT`
- `AT_DTCM_SECTION`
- `AT_OCRAM_SECTION`
- `AT_SDRAM_SECTION`
- `AT_SDRAM_NONCACHE_SECTION`

这部分非常重要，因为 RT1064 这类 MCU 很强调：

- 代码放在哪块存储器。
- 变量是否可缓存。
- DMA 用的数据是否要放在非缓存区。

所以这个头文件不是“普通 typedef 文件”，它还承担了内存布局抽象层的作用。

---

## 7. `zf_common_interrupt` 怎么用

这个模块比较直观，主要 API 有：

- `interrupt_global_disable()`
- `interrupt_global_enable()`
- `interrupt_enable(IRQn_Type irqn)`
- `interrupt_disable(IRQn_Type irqn)`
- `interrupt_set_priority(IRQn_Type irqn, uint8 priority)`
- `interrupt_init()`

### 7.1 `interrupt_init()` 做了什么

它内部调用：

```c
NVIC_SetPriorityGrouping(3);
```

注释说明是：

- 抢占优先级 0 到 15
- 无子优先级

也就是说，这套库更偏向“只关心抢占优先级，不细分响应优先级”的使用方式。

### 7.2 中断优先级的理解方式

在这个库里：

- 数值越小，优先级越高。
- 示例里经常用 `interrupt_set_priority(PIT_IRQn, 1);`
- 外部中断优先级设成 `0` 时，可以打断优先级为 `1` 的 PIT 中断。

### 7.3 `interrupt_global_disable()` / `interrupt_global_enable()`

这两个函数不只是简单封装，还加了一个 `interrupt_nest_count` 计数器。

这表示作者希望支持“嵌套式临界区”的用法，大意是：

```c
uint32 state = interrupt_global_disable();
// 临界区
interrupt_global_enable(state);
```

要注意：

- `enable` 时传入的是之前保存的 `primask`。
- 不要随便传常量，最好总是成对使用。

---

## 8. `zf_common_debug` 是最值得掌握的模块

这个模块非常关键，因为在嵌入式里，能不能快速定位错误，决定了开发效率。

### 8.1 最常用的接口

- `debug_init()`
- `zf_assert(x)`
- `zf_log(x, str)`
- `debug_assert_enable()`
- `debug_assert_disable()`
- `debug_read_ring_buffer()`

### 8.2 `debug_init()` 做了什么

默认配置在 `zf_common_debug.h`：

- `DEBUG_UART_INDEX` 默认是 `UART_1`
- `DEBUG_UART_BAUDRATE` 默认是 `115200`
- `DEBUG_UART_TX_PIN`
- `DEBUG_UART_RX_PIN`
- `DEBUG_UART_USE_INTERRUPT`

`debug_init()` 内部会：

1. 初始化调试输出结构体。
2. 默认把输出绑定到 UART。
3. 调用 `uart_init()` 初始化调试串口。
4. 如果启用了 `DEBUG_UART_USE_INTERRUPT`，还会初始化一个 FIFO，并打开串口接收中断。

### 8.3 `zf_assert(x)` 怎么理解

宏定义最终会调用：

```c
debug_assert_handler((x), __FILE__, __LINE__)
```

这意味着断言失败时，会自动带上：

- 当前文件名
- 当前行号

断言失败后，代码会：

1. 关闭全局中断。
2. 执行保护动作，把一堆 PWM 输出占空比拉成 0。
3. 进入死循环，持续输出错误位置。

这个设计很适合车模、电机类项目，因为出错时先“刹车”，避免硬件继续失控。

### 8.4 `zf_log(x, str)` 怎么理解

逻辑和断言类似，但它不会卡死程序：

- `x == 0` 时输出日志。
- `x != 0` 时不输出。

这种写法很适合和条件判断联动，例如：

```c
zf_log(fifo_write_buffer(&fifo, data, len) == FIFO_SUCCESS, "fifo write error");
```

意思是：成功就不打印，失败才打印。

### 8.5 为什么 `debug` 还能输出到屏幕

`debug_output_struct` 里有两类输出回调：

- `output_uart`
- `output_screen`

也就是说，调试模块本质上做的是“调试输出抽象”。

默认是串口。
如果某个显示设备驱动做了绑定，也可以把断言和日志打到屏幕上。

### 8.6 使用 debug 时最容易踩的坑

1. 改了 `DEBUG_UART_INDEX`，但没有把 `debug_interrupr_handler()` 挪到对应 UART 的中断函数里。
2. 开了 `DEBUG_UART_USE_INTERRUPT`，却没有处理对应串口中断。
3. 程序还没 `debug_init()`，就以为 `printf` 一定有输出。
4. 断言卡死后只看“程序不动了”，没去看串口输出的文件和行号。

---

## 9. FIFO 是通用层里最实用的数据结构

`zf_common_fifo` 用的是环形缓冲区思想。

核心结构体 `fifo_struct` 里最关键的成员有：

- `buffer`：底层缓冲区地址
- `head`：写指针
- `end`：读指针
- `size`：剩余空间
- `max`：总容量
- `type`：8/16/32 位数据类型

### 9.1 常用 API

- `fifo_init()`
- `fifo_clear()`
- `fifo_used()`
- `fifo_write_element()`
- `fifo_write_buffer()`
- `fifo_read_element()`
- `fifo_read_buffer()`
- `fifo_read_tail_buffer()`

### 9.2 最典型的使用场景

最经典的场景就是“中断接收，主循环处理”：

```c
// 1. 定义底层缓冲区
uint8 uart_get_data[64];
fifo_struct uart_data_fifo;

// 2. 初始化 FIFO
fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);

// 3. 在中断里写
uart_query_byte(UART_INDEX, &get_data);
fifo_write_buffer(&uart_data_fifo, &get_data, 1);

// 4. 在主循环里读
fifo_data_count = fifo_used(&uart_data_fifo);
fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
```

这套模式的优点：

- 中断里尽量短，只负责搬运数据。
- 复杂解析留到主循环做。
- 不容易阻塞中断。

### 9.3 `FIFO_READ_ONLY` 和 `FIFO_READ_AND_CLEAN`

这是使用 FIFO 时必须分清的两个模式：

- `FIFO_READ_ONLY`
  只读，不释放数据。
- `FIFO_READ_AND_CLEAN`
  读取后释放对应数据空间。

如果你只是想窥视当前数据内容，不想真正消费它，就用 `FIFO_READ_ONLY`。

### 9.4 `fifo_read_tail_buffer()` 的含义

这是从“尾部最近写入的数据”读。

适合这种需求：

- 我不关心全部历史数据。
- 我只想拿最近的一段数据做判断。

但要注意源码注释已经明确提醒：

- 如果用 `FIFO_READ_AND_CLEAN`，会直接清空整个 FIFO。

### 9.5 FIFO 的并发保护怎么理解

代码里有一个 `execution` 状态位，用于标记：

- 当前在 reset
- 当前在 clear
- 当前在 write
- 当前在 read

这不是严格意义上的线程安全实现，但它至少提供了“避免明显重入冲突”的保护思路。

在单片机场景下，这种设计已经比较实用。

---

## 10. `zf_common_function` 应该怎么学

这个模块我更建议当成“工具箱”。

### 10.1 三类常用内容

1. 简单宏函数
   - `func_abs`
   - `func_limit`
   - `func_limit_ab`

2. 数据转换函数
   - `func_str_to_int`
   - `func_int_to_str`
   - `func_str_to_uint`
   - `func_uint_to_str`
   - `func_str_to_float`
   - `func_float_to_str`
   - `func_str_to_hex`
   - `func_hex_to_str`

3. 其他工具函数
   - `func_get_sin_amplitude_table`
   - `func_get_greatest_common_divisor`
   - `func_soft_delay`
   - `zf_sprintf`

### 10.2 学这个模块时要注意什么

这些函数大多是“够用版”，不是完整标准库替代品。

例如：

- `func_float_to_str()` 更偏轻量实现。
- `zf_sprintf()` 只支持有限格式。
- 很多函数没有做特别强的边界保护。

所以正确心态是：

- 在资源受限、依赖标准库不方便时使用。
- 不要把它当成 PC 平台那种完整 libc。

### 10.3 我最常用的几个

- `func_limit(x, y)`：做控制量限幅很方便。
- `func_limit_ab(x, a, b)`：区间裁剪非常直接。
- `func_str_to_int()`：串口指令解析时常用。
- `zf_sprintf()`：不用完整 `sprintf` 时比较轻量。

---

## 11. `zf_common_vector` 应该怎么理解

这个模块初看会比较吓人，因为里面有很多中断函数名。

但它的核心思想其实很简单：

### 11.1 它是“默认中断入口 + 弱定义兜底”

例如很多 IRQHandler 都写成了弱定义：

- 如果用户自己实现了同名中断函数，就会覆盖默认实现。
- 如果用户没实现，默认实现通常会进 `while(1)`。

这能带来两个效果：

1. 没接好的中断会及时暴露，不会悄悄失效。
2. 用户只需要重写自己要用的中断函数。

### 11.2 异常处理函数为什么要看

例如：

- `HardFault_Handler()`
- `MemManage_Handler()`
- `BusFault_Handler()`

尤其 `HardFault_Handler()` 里的注释很值得重视，核心意思是：

很多人不是“外设没初始化”，而是“中断先开了，结果中断里用到的外设还没初始化”。

这个提醒非常实战。

### 11.3 用户真正要做什么

通常不是直接改 `zf_common_vector.c`，而是：

- 在自己工程的 `isr.c` 中实现对应的 `IRQHandler`。
- 在中断里清标志位、调用驱动回调、最后 `__DSB()`。

这也是示例工程的做法。

---

## 12. `zf_common_font` 的定位

这个模块不是通用层的核心逻辑模块，但它很常被显示驱动复用。

主要内容包括：

- RGB565 颜色枚举。
- ASCII 字库。
- 中文字库。
- 一些图片资源。

所以它更像“显示相关公共资源库”。

如果当前主要目标是学底层框架，可以先略读。

---

## 13. 我总结出的初始化主线

如果以后自己写一个基于这套库的新工程，我会先按这个顺序组织：

```c
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    // GPIO / UART / PIT / PWM / ADC 等驱动初始化
    // 外接设备初始化
    // 中断优先级设置

    while(1)
    {
        // 主循环逻辑
    }
}
```

如果某个外设用了接收中断，那么再补上：

1. 初始化外设。
2. 使能接收中断。
3. 设置优先级。
4. 在 `isr.c` 中实现对应 `IRQHandler`。
5. 中断里只收数，主循环里处理数据。

---

## 14. 如何阅读这套代码才不容易乱

我建议用“自顶向下 + 带着问题读”的方式。

### 14.1 第一遍：只看接口和职责

只看每个 `.h` 文件，回答三个问题：

1. 这个模块负责什么。
2. 这个模块暴露哪些 API。
3. 这个模块被谁依赖。

### 14.2 第二遍：只看关键流程

例如只盯这几条主线：

- `clock_init()` 调了谁。
- `debug_init()` 调了谁。
- FIFO 的写指针和读指针怎么移动。
- 中断入口最终跳到哪里。

### 14.3 第三遍：结合示例工程看调用

推荐结合这几个示例看：

- `E01_gpio_demo`
  看最小工程骨架。
- `E02_uart_demo`
  看 `FIFO + UART中断 + 主循环处理`。
- `E10_printf_debug_log_demo`
  看 `debug_init + zf_log + zf_assert`。
- `E11_interrupt_priority_set_demo`
  看中断优先级的设置方式。

### 14.4 第四遍：自己画图

建议自己手画两张图：

1. 模块依赖图
   `zf_common -> zf_driver -> zf_device -> user`
2. 串口接收链路图
   `UART中断 -> IRQHandler -> uart_rx_interrupt_handler -> fifo_write -> main读取`

只要这两张图画出来，理解会快很多。

---

## 15. 实际使用时的几个高频模板

### 15.1 模板一：最小工程模板

```c
#include "zf_common_headfile.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    while(1)
    {
    }
}
```

### 15.2 模板二：用 `zf_log` 做错误提示

```c
zf_log(init_result == 0, "device init error");
```

理解方式：

- 条件成立，不打印。
- 条件失败，打印错误。

### 15.3 模板三：用 FIFO 做串口接收缓存

```c
fifo_init(&uart_fifo, FIFO_DATA_8BIT, uart_buffer, 64);
uart_rx_interrupt(UART_INDEX, ZF_ENABLE);
interrupt_set_priority(LPUART1_IRQn, 0);
```

中断里：

```c
uart_query_byte(UART_INDEX, &rx_data);
fifo_write_buffer(&uart_fifo, &rx_data, 1);
```

主循环里：

```c
len = fifo_used(&uart_fifo);
if(len)
{
    fifo_read_buffer(&uart_fifo, data, &len, FIFO_READ_AND_CLEAN);
}
```

### 15.4 模板四：参数检查

```c
zf_assert(ptr != NULL);
zf_assert(len <= MAX_LEN);
```

这类写法在驱动开发里非常推荐，因为出错位置能直接定位到文件和行号。

---

## 16. 学完 `zf_common` 后，我认为最重要的认知

### 16.1 通用层的价值不是“功能多”，而是“统一”

它统一了：

- 类型风格
- 初始化入口
- 调试方法
- 数据缓冲方式
- 中断处理思路

### 16.2 真正要掌握的是设计习惯

从这套代码里能学到的，不只是 API，还有一套嵌入式写法：

- 初始化集中在开头。
- 中断里只做最少的事。
- 用 FIFO 解耦中断和主循环。
- 用断言尽早暴露参数错误。
- 用统一日志接口排错。

### 16.3 以后看驱动层会轻松很多

一旦 `zf_common` 理顺，再看 `zf_driver_uart.c`、`zf_driver_pwm.c`、`zf_device_xxx.c` 会顺很多，因为它们大量复用了这里的基础设施。

---

## 17. 我自己整理的学习结论

一句话总结：

`zf_common` 是这套 RT1064 开源库的“系统底座 + 调试底座 + 通用工具箱”。

如果只抓最核心的知识点，我认为是这五个：

1. `clock_init()` 是系统入口，必须最先调用。
2. `debug_init()` + `zf_assert()` + `zf_log()` 是排错主力。
3. `fifo` 是中断收数和主循环处理之间的桥梁。
4. `interrupt_set_priority()` 决定中断抢占关系。
5. `vector/isr` 体系决定中断最终怎么落到你的代码里。

---

## 18. 下一步学习建议

学完这层后，建议按下面顺序继续：

1. `zf_driver_uart`
   因为它最容易和 `debug`、`fifo`、中断机制串起来。
2. `zf_driver_gpio` / `zf_driver_pit`
   容易理解输入输出和定时中断。
3. `zf_driver_pwm`
   适合结合 `debug_assert_handler()` 里的保护逻辑理解。
4. 之后再看一个具体设备驱动，例如无线模块或屏幕。

如果后面继续学，我建议每学一个驱动，都回答下面四个问题：

1. 它依赖了哪些 `zf_common` 能力。
2. 它初始化时需要哪些前置条件。
3. 它有没有中断。
4. 它的数据是直接处理还是先入 FIFO。

---

## 19. 附：我认为最值得记住的源码观察点

- `clock_init()` 不是真正重写时钟树，而是调用板级 SDK 配置。
- `debug_assert_handler()` 出错后会先做电机 PWM 保护。
- `debug` 支持 UART 和屏幕两种输出抽象。
- `debug` 的串口接收本质也是 FIFO。
- `HardFault_Handler()` 的注释点出了“先开中断后初始化外设”的常见坑。
- `vector` 里的大量弱定义，是为了让用户只重写自己需要的中断。

---

## 20. 最后一句话

学习这套通用层，最好的方法不是背函数，而是抓住三条线：

- 启动线：`clock_init -> delay/init -> interrupt_init`
- 调试线：`debug_init -> zf_log / zf_assert`
- 数据线：`中断收数 -> FIFO缓存 -> 主循环处理`

这三条线理顺了，`zf_common` 就算真正入门了。
