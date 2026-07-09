# UART 协议规范

## 物理层

- **外设**: USART1
- **引脚**: PA9 (TX) / PA10 (RX)
- **波特率**: 115200
- **数据格式**: 8N1（8 数据位，无校验，1 停止位）
- **USB 转串口芯片**: CH340 或 CP2102
- **Linux 设备**: `/dev/ttyUSB0` (CH340) 或 `/dev/ttyACM0` (CP2102)

## 命令格式（主机 → STM32）

所有命令必须以 `\r\n` (0x0D 0x0A) 结尾。固件 IRQ 中断处理程序在收到 0x0A 后才置位接收完成标志。

### 模式切换命令

| 命令 | 效果 |
|------|------|
| `face\r\n` | 切换到表情显示模式（默认） |
| `sensor\r\n` | 切换到传感器数据显示模式 |

### 表情命令（8 个，与固件 emote_lookup_by_name 一一对应）

| 命令 | 效果 |
|------|------|
| `loving\r\n` | loving 表情（默认循环播放） |
| `happy\r\n` | happy 表情 |
| `sad\r\n` | sad 表情 |
| `thinking\r\n` | thinking 表情 |
| `surprised\r\n` | surprised 表情 |
| `sleepy\r\n` | sleepy 表情 |
| `angry\r\n` | angry 表情 |
| `neutral\r\n` | neutral 表情 |

### 传感器自动模式控制

| 命令 | 效果 |
|------|------|
| `sensoron\r\n` | 启用传感器自动检测（每 3 秒检测一次，异常时自动切表情） |
| `sensoroff\r\n` | 禁用传感器自动检测（恢复手动控制） |

### 调试 / 保留

| 命令 | 效果 |
|------|------|
| `blink\r\n` | 触发一次眨眼（roboeyes 模式） |
| `look N/S/E/W/center\r\n` | 设置注视方向 |
| `LIST/PLAY/STOP/DEL\r\n` | W25Q64 动画控制（FA_ProcessUART） |

## 响应格式（STM32 → 主机）

### 启动横幅
```
=== BOOT ===
USART1 BRR=0xXXXX HCLK=XXXX Hz
STAGE 1: after uart_init OK
STAGE 2: after OLED_Init OK
STAGE 3: sensors initialized (disabled by default)
STAGE 4: W25Q init skipped (under #if 0)
=== Robot Face + W25Q64 Anim Player ===
```

### 模式切换确认
```
[MODE] Face expression
[MODE] Sensor display
```

### 表情命令确认
```
[emote] happy
[emote] angry
[emote] sad
[emote] surprised
[emote] sleepy
[emote] thinking
[emote] neutral
[emote] loving
[sensor] ON
[sensor] OFF
```

### 未知命令
```
[?] unknown: '<cmd>'
    face/sensor/happy/angry/tired/default/blink/look N|S|E|W|center
    LIST / PLAY <name> / STOP / DEL <name>
```

### 传感器模式输出（仅在 `sensor` 模式下）
```
Env Monitor
T:25.3 H:60.2%
Smk:1234 1.00V
Lx:456.7 lx
```

## 时序说明

- 表情模式帧率：~30fps（33ms/帧，每帧 delay_ms(33)）
- 表情动画：10 帧 @ 4fps = 2.5s 循环（loving）或 播一次 + 5s 暂停
- 传感器自动检测：每 3 秒一次
- 发送命令后建议等待 200ms 再读取响应

## 重要变更（v2 固件）

- 默认表情改为 `loving`（之前是 happy）
- 表情名精确匹配：`loving`/`surprised`/`thinking`（不是 `love`/`surprise`/`think`）
- 传感器模式默认**禁用**，需发 `sensoron` 启用
- 不再有 `idle` 和 `scan` 表情
