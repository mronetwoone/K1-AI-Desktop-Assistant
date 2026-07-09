#!/usr/bin/env python3
"""
stm32_bridge.py - STM32 表情屏串口调试工具

与 Integrated_Sensors 固件通信，控制表情 + 读取传感器数据。
所有输出为 JSON 格式，便于调试程序解析。

用法:
    stm32_bridge.py status          探测串口状态
    stm32_bridge.py send <expr>     发送表情命令 (idle/happy/sad/surprise/sleepy/angry/love/scan)
    stm32_bridge.py read            读取一组完整传感器数据
    stm32_bridge.py monitor <秒>    持续读取 N 秒，返回 min/max/avg

依赖: pyserial (pip3 install pyserial)
"""

import argparse
import json
import re
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print(json.dumps({"ok": False, "error": "缺少 pyserial，请运行: pip3 install pyserial"}))
    sys.exit(1)

# 串口候选列表
PORT_CANDIDATES = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1", "/dev/ttyACM1"]
# 固件 HCLK=108MHz, USART1 BRR=0x03A9 → 115200 baud
# CH9102/CH340 在 115200 下 TX 正常但 RX 异常 (auto-detect 锁到 76800)
BAUD_RATE = 115200

# 有效表情命令（与固件 emote_lookup_by_name 一一对应）
VALID_EXPRESSIONS = ["loving", "happy", "sad", "thinking", "surprised", "sleepy", "angry", "neutral"]

# 传感器解析正则
RE_TEMP_HUM = re.compile(r"T:(\d+)\.(\d+)\s+H:(\d+)\.(\d+)")
RE_SMOKE = re.compile(r"Smoke:(\d+)")
RE_LIGHT = re.compile(r"Light:(\d+)\.(\d+)")
RE_DHT11_ERR = re.compile(r"DHT11 error")


def output(data):
    """输出 JSON 到 stdout"""
    print(json.dumps(data, ensure_ascii=False))


def find_serial_port():
    """自动检测可用串口"""
    # 先尝试候选列表
    for port in PORT_CANDIDATES:
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
            ser.close()
            return port
        except (serial.SerialException, OSError):
            continue

    # 再扫描所有可用串口
    for info in serial.tools.list_ports.comports():
        try:
            ser = serial.Serial(info.device, BAUD_RATE, timeout=0.1)
            ser.close()
            return info.device
        except (serial.SerialException, OSError):
            continue

    return None


def open_serial(port):
    """打开串口，失败重试一次"""
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=2)
        time.sleep(0.1)
        ser.reset_input_buffer()
        return ser
    except (serial.SerialException, OSError):
        time.sleep(0.5)
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=2)
            time.sleep(0.1)
            ser.reset_input_buffer()
            return ser
        except (serial.SerialException, OSError):
            return None


def parse_sensor_line(line):
    """解析一行传感器数据，返回 (类型, 值) 或 None"""
    line = line.strip()

    m = RE_TEMP_HUM.search(line)
    if m:
        temp = float(f"{m.group(1)}.{m.group(2)}")
        hum = float(f"{m.group(3)}.{m.group(4)}")
        return ("temp_hum", {"temperature": temp, "humidity": hum})

    m = RE_SMOKE.search(line)
    if m:
        adc = int(m.group(1))
        return ("smoke", {"smoke_adc": adc})

    m = RE_LIGHT.search(line)
    if m:
        lux = float(f"{m.group(1)}.{m.group(2)}")
        return ("light", {"light": lux})

    if RE_DHT11_ERR.search(line):
        return ("dht11_error", None)

    return None


def collect_sensor_set(ser, timeout=5):
    """读取直到收集到完整的传感器数据集（温湿度+烟雾+光照）"""
    result = {}
    start = time.time()

    while time.time() - start < timeout:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            parsed = parse_sensor_line(line)
            if parsed is None:
                continue

            kind, data = parsed
            if kind == "temp_hum":
                result["temperature"] = data["temperature"]
                result["humidity"] = data["humidity"]
            elif kind == "smoke":
                result["smoke_adc"] = data["smoke_adc"]
            elif kind == "light":
                result["light"] = data["light"]
            elif kind == "dht11_error":
                result["temperature"] = None
                result["humidity"] = None

            # 检查是否收集完整
            has_temp = "temperature" in result
            has_smoke = "smoke_adc" in result
            has_light = "light" in result
            if has_temp and has_smoke and has_light:
                return result, False  # complete

        except (serial.SerialException, OSError):
            return result, True  # partial on error

    return result, True  # partial on timeout


# === 子命令实现 ===

def cmd_status(args):
    """探测串口状态"""
    port = args.port or find_serial_port()
    if not port:
        output({"ok": False, "error": f"未找到串口，已尝试: {', '.join(PORT_CANDIDATES)}"})
        return 1

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        ser.close()
        output({"ok": True, "port": port, "baud": BAUD_RATE, "accessible": True})
        return 0
    except serial.SerialException as e:
        if "Permission" in str(e) or "权限" in str(e):
            output({"ok": False, "port": port, "accessible": False, "error": "权限不足，请运行: sudo usermod -aG dialout $USER"})
        else:
            output({"ok": False, "port": port, "accessible": False, "error": str(e)})
        return 1


def cmd_send(args):
    """发送表情命令"""
    expr = args.expression.lower()
    if expr not in VALID_EXPRESSIONS:
        output({"ok": False, "error": f"无效表情: {expr}，有效值: {', '.join(VALID_EXPRESSIONS)}"})
        return 1

    port = args.port or find_serial_port()
    if not port:
        output({"ok": False, "error": f"未找到串口，已尝试: {', '.join(PORT_CANDIDATES)}"})
        return 1

    ser = open_serial(port)
    if not ser:
        output({"ok": False, "error": f"无法打开串口: {port}"})
        return 1

    try:
        # 发送命令（\r\n 结尾，固件 IRQ 要求 0x0D+0x0A）
        cmd = f"{expr}\r\n".encode("utf-8")
        ser.write(cmd)
        ser.flush()

        # 等待可能的响应
        time.sleep(0.2)
        response_lines = []
        while ser.in_waiting:
            raw = ser.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    response_lines.append(line)

        output({"ok": True, "command": expr, "response": response_lines})
        return 0
    except (serial.SerialException, OSError) as e:
        output({"ok": False, "error": f"串口通信失败: {e}"})
        return 1
    finally:
        ser.close()


def cmd_sensor(args):
    """启用/禁用传感器自动检测"""
    state = args.state.lower()
    if state not in ("on", "off"):
        output({"ok": False, "error": "参数必须是 on 或 off"})
        return 1

    # 固件命令: sensoron / sensoroff
    cmd_word = "sensoron" if state == "on" else "sensoroff"

    port = args.port or find_serial_port()
    if not port:
        output({"ok": False, "error": f"未找到串口，已尝试: {', '.join(PORT_CANDIDATES)}"})
        return 1

    ser = open_serial(port)
    if not ser:
        output({"ok": False, "error": f"无法打开串口: {port}"})
        return 1

    try:
        ser.write(f"{cmd_word}\r\n".encode("utf-8"))
        ser.flush()
        time.sleep(0.2)
        response_lines = []
        while ser.in_waiting:
            raw = ser.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    response_lines.append(line)
        output({"ok": True, "command": cmd_word, "response": response_lines})
        return 0
    except (serial.SerialException, OSError) as e:
        output({"ok": False, "error": f"串口通信失败: {e}"})
        return 1
    finally:
        ser.close()


def cmd_read(args):
    """读取一组完整传感器数据"""
    port = args.port or find_serial_port()
    if not port:
        output({"ok": False, "error": f"未找到串口，已尝试: {', '.join(PORT_CANDIDATES)}"})
        return 1

    ser = open_serial(port)
    if not ser:
        output({"ok": False, "error": f"无法打开串口: {port}"})
        return 1

    try:
        data, partial = collect_sensor_set(ser, timeout=5)
        if not data:
            output({"ok": False, "error": "未收到任何传感器数据，超时 5 秒"})
            return 1

        # 格式化输出
        sensors = {}
        if "temperature" in data:
            sensors["temperature"] = {"value": data["temperature"], "unit": "C"}
        if "humidity" in data:
            sensors["humidity"] = {"value": data["humidity"], "unit": "%"}
        if "smoke_adc" in data:
            adc = data["smoke_adc"]
            voltage = round(adc * 3.3 / 4095, 2)
            sensors["smoke"] = {"raw_adc": adc, "voltage": voltage}
        if "light" in data:
            sensors["light"] = {"value": data["light"], "unit": "lux"}

        output({"ok": True, "sensors": sensors, "partial": partial})
        return 0
    except (serial.SerialException, OSError) as e:
        output({"ok": False, "error": f"串口通信失败: {e}"})
        return 1
    finally:
        ser.close()


def cmd_monitor(args):
    """持续读取传感器数据并聚合"""
    duration = args.seconds
    if duration < 1 or duration > 300:
        output({"ok": False, "error": "监控时长必须在 1-300 秒之间"})
        return 1

    port = args.port or find_serial_port()
    if not port:
        output({"ok": False, "error": f"未找到串口，已尝试: {', '.join(PORT_CANDIDATES)}"})
        return 1

    ser = open_serial(port)
    if not ser:
        output({"ok": False, "error": f"无法打开串口: {port}"})
        return 1

    try:
        temps = []
        hums = []
        smokes = []
        lights = []
        start = time.time()

        while time.time() - start < duration:
            data, _ = collect_sensor_set(ser, timeout=3)
            if data:
                if data.get("temperature") is not None:
                    temps.append(data["temperature"])
                if data.get("humidity") is not None:
                    hums.append(data["humidity"])
                if "smoke_adc" in data:
                    smokes.append(data["smoke_adc"])
                if "light" in data:
                    lights.append(data["light"])

        if not temps and not smokes and not lights:
            output({"ok": False, "error": f"在 {duration} 秒内未收到任何传感器数据"})
            return 1

        result = {"ok": True, "duration": duration, "samples": len(temps)}

        def agg(vals):
            if not vals:
                return None
            return {"min": round(min(vals), 1), "max": round(max(vals), 1), "avg": round(sum(vals) / len(vals), 1)}

        if temps:
            result["temperature"] = agg(temps)
        if hums:
            result["humidity"] = agg(hums)
        if smokes:
            result["smoke_adc"] = agg(smokes)
        if lights:
            result["light"] = agg(lights)

        output(result)
        return 0
    except (serial.SerialException, OSError) as e:
        output({"ok": False, "error": f"串口通信失败: {e}"})
        return 1
    finally:
        ser.close()


def main():
    parser = argparse.ArgumentParser(
        description="OpenClaw <-> STM32 串口桥接脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", help="指定串口 (默认自动检测)")

    sub = parser.add_subparsers(dest="command")

    sub.add_parser("status", help="探测串口状态")

    p_send = sub.add_parser("send", help="发送表情命令")
    p_send.add_argument("expression", help="表情名称: loving/happy/sad/thinking/surprised/sleepy/angry/neutral")

    p_sensor = sub.add_parser("sensor", help="启用/禁用传感器自动检测")
    p_sensor.add_argument("state", help="on 或 off")

    sub.add_parser("read", help="读取一组传感器数据")

    p_mon = sub.add_parser("monitor", help="持续监控传感器")
    p_mon.add_argument("seconds", type=int, help="监控时长 (秒)")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        output({"ok": False, "error": "请指定子命令: status/send/read/monitor"})
        sys.exit(1)

    handlers = {
        "status": cmd_status,
        "send": cmd_send,
        "sensor": cmd_sensor,
        "read": cmd_read,
        "monitor": cmd_monitor,
    }

    sys.exit(handlers[args.command](args))


if __name__ == "__main__":
    main()
