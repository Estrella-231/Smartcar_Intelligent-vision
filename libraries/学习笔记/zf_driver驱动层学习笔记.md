# RT1064 `zf_driver` 驱动层学习笔记

## 1. 我对 `zf_driver` 的整体理解

如果说 `zf_common` 是底座，那么 `zf_driver` 就是“芯片片上外设驱动层”。

它主要负责两件事：

1. 把 NXP SDK 的底层外设接口二次封装成统一风格。
2. 把引脚、通道、模块号做成枚举，减少用户直接操作寄存器和复杂配置的负担。

从代码风格看，`zf_driver` 不是为了做成高度抽象框架，而是为了让你在单片机项目里可以直接写：

```c
gpio_init(B9, GPO, GPIO_LOW, GPO_PUSH_PULL);
uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);
pit_ms_init(PIT_CH0, 1000);
pwm_init(PWM1_MODULE3_CHA_D0, 17000, 0);
```

也就是说，它追求的是：

- 上手快
- 例程一致
- 引脚关系清楚
- 出错时能被 `zf_assert` 快速抓出来

---

## 2. `zf_driver` 在整套库中的位置

我把这套代码大致理解成四层：

```text
zf_common   -> 公共基础层
zf_driver   -> 片上外设驱动层
zf_device   -> 外接模块驱动层
user        -> 自己的应用层
```

依赖方向一般是：

`user -> zf_device -> zf_driver -> zf_common -> SDK`

因此学习 `zf_driver` 的关键不是只会调函数，而是要理解：

- 它和 `zf_common` 的关系。
- 它对 SDK 做了哪些封装。
- 为什么设备层几乎都建立在它之上。

---

## 3. `zf_driver` 目录里主要有哪些模块

当前目录里主要驱动有：

- `zf_driver_gpio`
  通用 GPIO、快速 GPIO、IOMUX 复用。
- `zf_driver_uart`
  串口初始化、收发、中断开关。
- `zf_driver_adc`
  ADC 初始化、单次采样、均值滤波采样。
- `zf_driver_pwm`
  PWM 初始化和占空比更新。
- `zf_driver_pit`
  周期中断定时器。
- `zf_driver_timer`
  通用计时器，适合测时间。
- `zf_driver_exti`
  外部中断输入。
- `zf_driver_encoder`
  编码器接口。
- `zf_driver_flash`
  Flash 页读写。
- `zf_driver_iic` / `zf_driver_soft_iic`
  硬件 IIC / 软件 IIC。
- `zf_driver_spi` / `zf_driver_soft_spi`
  硬件 SPI / 软件 SPI。
- `zf_driver_delay`
  微秒/毫秒延时。
- `zf_driver_csi` / `zf_driver_flexio_csi`
  摄像头采集相关驱动。
- `zf_driver_sdio`
  SD/TF 卡相关。
- `zf_driver_usb_cdc`
  USB CDC 串口类接口。
- `zf_driver_romapi`
  ROM API 相关封装。

如果你是第一次学，我建议先把前 8 个学透，后面的按项目需求再看。

---

## 4. 读 `zf_driver` 代码前要先建立的认知

### 4.1 这是“统一封装层”，不是完全重写底层

很多驱动实现里都能直接看到 SDK 调用，比如：

- `LPUART_Init`
- `PIT_Init`
- `PWM_Init`
- `GPIO_PinInit`

所以它的思路不是绕开 SDK，而是：

- 用 SDK 做底层初始化和寄存器控制。
- 用自己的一层接口做简化和统一。

### 4.2 引脚和模块强绑定

这套驱动非常依赖枚举值表达“引脚属于哪个外设”。

例如 UART：

- `UART1_TX_B12`
- `UART1_RX_B13`

例如 PWM：

- `PWM1_MODULE3_CHA_D0`
- `PWM2_MODULE3_CHB_D3`

这意味着你使用驱动时，核心不是“传一个普通 pin 号”，而是“选一个已经绑定好功能的外设引脚枚举”。

### 4.3 `zf_assert` 是驱动层的重要保护

驱动层很多地方会检查：

- 引脚和模块是否匹配
- 参数是否超范围
- 占空比是否合法
- 分频是否可计算

所以一旦程序卡在断言，不要先怀疑驱动错了，先检查：

- 枚举选错没
- 一个模块的两路引脚是否来自同一外设
- 参数范围是否越界

---

## 5. 学 `zf_driver` 最好的阅读方法

我建议每个驱动按同一个顺序读：

1. 先看 `.h`
   看枚举、API、注释。
2. 再看 `.c`
   只抓初始化函数和最常用 API。
3. 再看对应 `Example`
   看用户实际怎么写。

比如学 UART，我会这样看：

1. `zf_driver_uart.h`
   先看有哪些 `UARTx_TX_xxx`、`UARTx_RX_xxx` 枚举。
2. `zf_driver_uart.c`
   重点看 `uart_iomuxc()`、`uart_init()`、`uart_rx_interrupt()`。
3. `E02_uart_demo`
   看中断和 FIFO 怎么配合。

这个方法比一口气啃完整个 `.c` 文件效率高很多。

---

## 6. `zf_driver` 的统一代码风格

我总结出几条规律：

### 6.1 接口命名非常统一

通常都是这几类：

- `xxx_init`
- `xxx_enable` / `xxx_disable`
- `xxx_set_xxx`
- `xxx_get_xxx`
- `xxx_write_xxx`
- `xxx_read_xxx`

### 6.2 先枚举，再初始化

几乎所有驱动都是：

- 先用枚举表达资源
- 再用 `init()` 绑定资源

例如：

```c
adc_init(ADC1_CH3_B14, ADC_12BIT);
```

这里把：

- ADC 模块
- 通道号
- 对应引脚

都打包进了一个枚举。

### 6.3 很多驱动都先做 IOMUX

例如 UART、PWM、GPIO，本质上都要先做引脚复用配置。

所以看到 `xxx_iomuxc()` 这一类内部函数时，你要知道：

- 这部分是“引脚功能路由”
- 真正外设初始化在后面

---

## 7. 我建议的学习顺序

### 7.1 第一阶段：最基础的输入输出

1. `gpio`
2. `delay`
3. `pit`
4. `timer`

这一阶段目标：

- 会点灯
- 会读按键
- 会做延时
- 会做周期中断
- 会测量一段代码运行时间

### 7.2 第二阶段：数据输入输出

1. `uart`
2. `adc`
3. `exti`

这一阶段目标：

- 会串口打印
- 会串口收数
- 会采模拟量
- 会做引脚中断

### 7.3 第三阶段：控制类外设

1. `pwm`
2. `encoder`

这一阶段目标：

- 会输出 PWM
- 会做电机或舵机控制基础实验
- 会读编码器反馈

### 7.4 第四阶段：通信与存储

1. `spi`
2. `iic`
3. `flash`

这一阶段目标：

- 会和传感器通信
- 会做寄存器读写
- 会保存参数到 Flash

### 7.5 第五阶段：专项模块

- `csi`
- `flexio_csi`
- `sdio`
- `usb_cdc`
- `romapi`

这些通常和相机、存储、USB、量产等项目场景有关，建议在具体项目里边用边学。

---

## 8. 核心驱动应该怎么理解

## 8.1 GPIO

### 作用

- 配通用输入输出
- 设置高低电平
- 翻转引脚
- 读取引脚状态

### 最常用接口

- `gpio_init()`
- `gpio_set_level()`
- `gpio_get_level()`
- `gpio_toggle_level()`

### 关键理解

`gpio_init()` 本质上做了两件事：

1. 配 IOMUX，让这个引脚切到 GPIO 功能。
2. 调 SDK 的 `GPIO_PinInit()` 完成方向和电平配置。

### 使用模板

```c
gpio_init(B9, GPO, GPIO_LOW, GPO_PUSH_PULL);
gpio_init(C4, GPI, 0, GPI_PULL_UP);
```

### 要注意的点

- `GPO_PUSH_PULL`、`GPI_PULL_UP` 这类模式非常重要，不只是“输入输出”，还带上下拉和驱动能力配置。
- `fast_gpio_*` 是性能强化版，不是普通 GPIO 的简单别名。

### 如何学

先看 `E01_gpio_demo`，这是最适合入门的驱动示例。

---

## 8.2 UART

### 作用

- 串口收发
- 打日志
- 接收上位机数据
- 给无线模块/蓝牙模块/GNSS 做底层通信

### 最常用接口

- `uart_init()`
- `uart_write_byte()`
- `uart_write_buffer()`
- `uart_write_string()`
- `uart_read_byte()`
- `uart_query_byte()`
- `uart_rx_interrupt()`

### 关键理解

`uart_init()` 的关键逻辑是：

1. 检查 `tx_pin` 和 `rx_pin` 是否和 UART 模块匹配。
2. 通过 `uart_iomuxc()` 配置引脚复用。
3. 调 SDK 初始化 LPUART。

### 推荐使用方式

如果是调试输出，直接配合 `debug_init()`。

如果是业务收数，推荐这样做：

1. 中断里只收 1 字节。
2. 放进 FIFO。
3. 主循环里统一解析。

这也是 `E02_uart_demo` 的标准做法。

### 要注意的点

- 引脚必须和 UART 号匹配，否则会进断言。
- 开启接收中断后，还必须在 `isr.c` 写对应的 `LPUARTx_IRQHandler`。
- 串口模块很常和 `zf_common_fifo` 配套出现。

---

## 8.3 ADC

### 作用

- 采集模拟电压
- 读取电位器、模拟传感器、电池电压等

### 最常用接口

- `adc_init()`
- `adc_convert()`
- `adc_mean_filter_convert()`

### 关键理解

ADC 不是“每个引脚都完全独立”，同一 ADC 模块会共享某些配置。

`E03_adc_demo` 已经明确点出一个很重要的现象：

- 如果多个引脚属于同一个 ADC 模块
- 最后一次 `adc_init()` 的分辨率设置会对这个模块生效

这点必须记住。

### 使用模板

```c
adc_init(ADC1_CH3_B14, ADC_12BIT);
value = adc_convert(ADC1_CH3_B14);
mean = adc_mean_filter_convert(ADC1_CH3_B14, 10);
```

### 要注意的点

- 分辨率不是每个 pin 独立记忆。
- 读出来的是数字量，不是电压值，电压还要自己换算。

---

## 8.4 PWM

### 作用

- 驱动电机
- 控制舵机
- 输出占空比可调波形

### 最常用接口

- `pwm_init()`
- `pwm_set_duty()`

### 关键理解

PWM 枚举名里已经包含了：

- PWM 模块号
- 子模块号
- A/B 通道
- 引脚

例如：

`PWM1_MODULE3_CHA_D0`

### 必须理解的一条规则

同一个子模块的 A/B 通道频率必须一致，占空比可以不同。

这在头文件注释里写得很清楚，是学习 PWM 时最容易忽略的地方。

### 使用模板

```c
pwm_init(PWM1_MODULE3_CHA_D0, 17000, 0);
pwm_set_duty(PWM1_MODULE3_CHA_D0, PWM_DUTY_MAX / 2);
```

### 要注意的点

- `duty` 范围不能超过 `PWM_DUTY_MAX`。
- 频率太低或分频无法满足时会断言。
- 如果几个引脚在同一子模块，后初始化的频率会覆盖前一个。

---

## 8.5 PIT

### 作用

- 做周期中断
- 定期采样
- 定期刷新状态

### 最常用接口

- `pit_ms_init()`
- `pit_us_init()`
- `pit_enable()`
- `pit_disable()`
- `pit_flag_get()`
- `pit_flag_clear()`

### 关键理解

PIT 更像“节拍器”。

使用方式通常是：

1. `pit_ms_init(PIT_CH0, 1000);`
2. 在 `PIT_IRQHandler()` 里清标志。
3. 调你自己的 `pit_handler()` 或置一个标志位。

### 推荐习惯

中断里尽量只：

- 清标志位
- 置状态标志

主循环里再做耗时逻辑。

---

## 8.6 TIMER

### 作用

- 测量一段代码运行多久
- 做简单计时统计

### 最常用接口

- `timer_init()`
- `timer_start()`
- `timer_stop()`
- `timer_get()`
- `timer_clear()`

### 关键理解

它和 PIT 的区别是：

- PIT 更适合“定期打断我”
- TIMER 更适合“帮我计时”

### 三种计时模式

- `TIMER_IPG1_2_CLOCK`
- `TIMER_US`
- `TIMER_MS`

建议先用 `TIMER_US` 和 `TIMER_MS`，最直观。

---

## 8.7 EXTI

### 作用

- 引脚触发中断
- 按键、触发信号、边沿检测

### 最常用接口

- `exti_init()`
- `exti_enable()`
- `exti_disable()`
- `exti_flag_get()`
- `exti_flag_clear()`

### 关键理解

一个 GPIO 中断组往往对应多个引脚，所以你不能只写“有中断”，还要在中断里判断到底是谁触发了。

`E06_exti_demo` 的写法很典型：

```c
if(exti_flag_get(KEY1))
{
    exti_flag_clear(KEY1);
    exti_state[0] = 1;
}
```

### 要注意的点

- `exti_init()` 只是配置引脚触发方式。
- 你还要在对应 `IRQHandler` 里手动判断和清标志位。

---

## 8.8 ENCODER

### 作用

- 读正交编码器
- 读带方向信号的增量编码器

### 最常用接口

- `encoder_quad_init()`
- `encoder_dir_init()`
- `encoder_get_count()`
- `encoder_clear_count()`

### 关键理解

编码器驱动已经把：

- 哪个 QTIMER
- 哪个编码器通道
- 哪个引脚组合

都映射进枚举了。

所以本质上还是“资源枚举 + 初始化绑定”的同一套思想。

### 推荐用法

用 PIT 每隔固定周期采样一次编码器计数，然后清零，得到周期增量。

这正是 `E07_encoder_demo` 的思路。

---

## 8.9 FLASH

### 作用

- 保存参数
- 保存标定数据
- 保存掉电不丢失信息

### 最常用接口

- `flash_init()`
- `flash_check()`
- `flash_erase_page()`
- `flash_read_page_to_buffer()`
- `flash_write_page_from_buffer()`
- `flash_buffer_clear()`

### 关键理解

这个驱动不是让你随便按字节读写，而是围绕：

- sector
- page
- 缓冲区

来组织的。

还定义了一个 `flash_data_union`，方便把不同类型映射到统一的 32 位存储单元里。

### 最要注意的点

同一个 `flash_union_buffer[i]` 位置，只能按一种类型理解。

如果你写进去 `float_type`，再按 `int8_type` 去读，结果当然是错的。

### 另一个重要点

头文件明确提醒：

RT 系列有 cache，读 Flash 前往往要注意 cache 一致性问题。

这不是普通 MCU 上经常遇到的点，但在 RT1064 上必须有这个意识。

---

## 9. SPI / IIC 该怎么理解

这两个驱动我建议作为“寄存器通信类驱动”一起学。

### 共同特点

- 先选模块号和引脚
- 再初始化波特率/模式
- 再做 `read/write/transfer`
- 都提供了“寄存器读写”接口

例如 SPI：

- `spi_write_8bit_register()`
- `spi_read_8bit_register()`

例如 IIC：

- `iic_write_8bit_register()`
- `iic_read_8bit_register()`

### 学习建议

不要先从纯总线时序入手，而要结合一个设备驱动看。

比如你以后看 IMU、OLED、摄像头配置时，会发现这些函数瞬间变得很自然。

---

## 10. 如何真正学会“使用”驱动层

我建议你每学一个驱动，都回答下面五个问题：

1. 这个驱动的资源是怎么枚举的？
2. 初始化函数最少要传哪些参数？
3. 有没有必须匹配的引脚组合？
4. 有没有中断？
5. 对应示例工程怎么写？

只要这五个问题能答出来，这个驱动基本就入门了。

---

## 11. 我总结出的常见坑

### 11.1 引脚枚举和模块号不匹配

最常见于 UART、IIC、SPI、ENCODER。

例如：

- 你选的是 `UART_1`
- 却传了 `UART2_TX_xxx`

这类错误通常会直接断言。

### 11.2 只初始化外设，不写中断服务函数

常见于：

- UART 接收中断
- PIT 周期中断
- EXTI 外部中断

初始化只是“开门”，真正动作还要靠 `isr.c`。

### 11.3 同一模块共享配置没注意

最典型的是：

- ADC 分辨率共享
- PWM 同一子模块频率共享

### 11.4 中断里做了太多事

这会导致：

- 丢数据
- 卡顿
- 优先级问题

推荐做法始终是：

- 中断里收数/置标志
- 主循环里处理

### 11.5 忘记初始化顺序

正确顺序通常应该是：

```c
clock_init(SYSTEM_CLOCK_600M);
debug_init();
// 驱动初始化
// 中断优先级设置
// 主循环
```

---

## 12. 我推荐的实战学习路线

如果你想真正把 `zf_driver` 学扎实，我建议按这个路线做：

### 第 1 步

先跑通这几个例程：

- `E01_gpio_demo`
- `E02_uart_demo`
- `E03_adc_demo`
- `E04_pwm_demo`
- `E05_pit_demo`
- `E06_exti_demo`
- `E07_encoder_demo`
- `E08_flash_demo`
- `E09_timer_demo`

### 第 2 步

每个例程只回答三个问题：

1. 初始化了哪些驱动？
2. 有没有中断？
3. 主循环干什么？

### 第 3 步

自己做一个综合小项目，例如：

- 按键触发中断
- PWM 输出占空比变化
- 编码器测速度
- UART 打印结果

只要你能把这四个串起来，驱动层就算真正掌握一大半了。

---

## 13. 我自己整理的理解结论

一句话总结：

`zf_driver` 是一套“把 RT1064 片上外设变得更容易调用”的统一驱动层。

它最值得学的不是某一个函数，而是这一套写法：

- 用枚举绑定资源
- 用统一 `init/read/write/set` 接口操作外设
- 用 `zf_assert` 防止错误配置
- 用示例工程给出标准使用姿势

如果只记最核心的六点，我会记这六条：

1. `zf_driver` 大多是 SDK 的二次封装，不是从零造轮子。
2. 外设资源几乎都通过枚举表达。
3. 很多驱动都先做 IOMUX，再做外设初始化。
4. 中断驱动的正确姿势是“中断里最少化，主循环里处理”。
5. 同一模块下某些配置是共享的，不是每个引脚独立。
6. 最快的学习方法永远是“头文件 + 实现 + 对应示例”三件套一起看。

---

## 14. 学完 `zf_driver` 后下一步该学什么

建议继续往上学 `zf_device`。

因为一旦你理解了：

- UART / SPI / IIC
- GPIO / EXTI
- PWM / PIT / TIMER

再看外设模块，比如：

- OLED
- IPS 屏
- IMU
- WiFi
- 摄像头

就会很顺，你会明显看到这些设备驱动只是把 `zf_driver` 组合起来用了。

---

## 15. 最后一句话

学习 `zf_driver`，不要按“一个函数一个函数背”，而要抓住四条主线：

- 引脚复用线：这个外设能用哪些引脚
- 初始化线：这个驱动怎么从未配置状态进入可用状态
- 中断线：这个驱动有没有 ISR 配套要求
- 数据线：数据是直接读写，还是经 FIFO/状态位转交主循环

这四条线理顺了，`zf_driver` 就算真正入门了。
