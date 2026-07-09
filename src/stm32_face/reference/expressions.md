# 表情词汇表

## 表情列表（与固件 emote_lookup_by_name 一一对应）

| 表情 | 命令 | 情感语境 | 适用场景 |
|------|------|----------|----------|
| loving | `loving` | 喜爱、温柔 | 默认状态、感谢、问候 |
| happy | `happy` | 喜悦、满意 | 好消息、任务成功 |
| sad | `sad` | 失望、共情 | 坏消息、失败 |
| thinking | `thinking` | 思考、分析 | 正在查询、湿度异常 |
| surprised | `surprised` | 震惊、发现 | 意外事件、温度极端 |
| sleepy | `sleepy` | 困倦、放松 | 深夜、光线很暗 |
| angry | `angry` | 愤怒、警告 | 错误、烟雾报警 |
| neutral | `neutral` | 平静、中性 | 中性显示 |

**注意**：固件命令是 `loving` 不是 `love`，`surprised` 不是 `surprise`，`thinking` 不是 `think`。

## 表情切换

- 每个表情是 96×48 的 1bpp 动画，10 帧 @ 4fps = 2.5s 循环
- 切换命令后立即生效，旧动画被覆盖
- loving 是唯一 `loop=1` 的表情（一直循环），其他都是播放一次后停 5 秒

## 情感映射建议

当用户没有明确指定表情时，根据上下文选择：

| 场景 | 推荐表情 |
|------|----------|
| 用户打招呼 | happy |
| 用户说再见 | sad |
| 传感器数据正常（无异常） | loving |
| 烟雾报警 | angry |
| 温度过高 / 过低 | surprised |
| 湿度过高 / 过低 | thinking |
| 用户表达感谢 | loving |
| 深夜时段 / 很暗 | sleepy |
| 收到意外数据 | surprised |
| 中性状态 | neutral |

## 传感器自动模式（需发 `sensor on`）

启用后每 3 秒自动检测：

| 异常 | 阈值 | 自动表情 |
|------|------|----------|
| 烟雾 MQ-2 ADC | > 2000 | angry |
| 温度 | > 40°C 或 < 0°C | surprised |
| 湿度 | > 90% 或 < 20% | thinking |
| 光照 | < 10 lux | sleepy |
| 无异常 | — | 恢复 loving |
