#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_leap_ext_print.py
简单测试 leap_ext C 扩展：初始化、轮询 next_event 并打印返回内容
"""

import time
import sys
import pprint

TIMEOUT_MS = 10      # 传给 leap_ext.next_event 的 poll timeout（毫秒）
SLEEP_WHEN_NONE = 0.01  # 当没有事件时短暂休眠，避免忙等

def main():
    try:
        import leap_ext
    except Exception as e:
        print("无法导入 leap_ext。请确认 C 扩展已编译并安装。\n错误：", e)
        sys.exit(1)

    pp = pprint.PrettyPrinter(indent=2, compact=False)
    print("初始化 leap_ext...")
    try:
        # 如果你的 py 扩展接收参数，比如 optimize_hmd，可以传入 True/False
        leap_ext.init()
    except TypeError:
        # 若 init() 没有参数
        leap_ext.init()
    except Exception as e:
        print("leap_ext.init() 报错：", e)
        sys.exit(1)

    print("开始轮询 next_event（按 Ctrl-C 退出）")
    try:
        while True:
            try:
                # 有些版本 next_event() 没参数或参数名不同：这里以 timeout_ms 作为常见签名
                ev = None
                try:
                    ev = leap_ext.next_event(TIMEOUT_MS)
                except TypeError:
                    # 退回到无参调用
                    ev = leap_ext.next_event()
                except Exception as e:
                    # 如果调用出现其他异常，打印并继续
                    print("[ERROR] next_event 调用异常：", e)
                    time.sleep(0.1)
                    continue

                t = time.time()
                if ev is None:
                    # 没有新事件，短暂停一下
                    # 你可以把这行注释掉以减少输出
                    # print(f"[{t:.3f}] no event")
                    time.sleep(SLEEP_WHEN_NONE)
                    continue

                # 打印事件类型和值（优雅格式）
                print("="*60)
                print(f"[{t:.3f}] Received event:")
                # 如果是 dict，pretty print；否则直接打印 repr
                if isinstance(ev, dict):
                    pp.pprint(ev)
                else:
                    # 可能是 tuple/list/other -> 打印 repr
                    print(repr(ev))
                print("="*60)
            except Exception as e:
                # 捕获单次循环内任何异常，确保不会退出
                print("[LOOP ERROR]", e)
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n用户中断，准备退出...")
    finally:
        try:
            leap_ext.shutdown()
            print("已调用 leap_ext.shutdown()")
        except Exception as e:
            print("调用 leap_ext.shutdown() 时出错：", e)

if __name__ == "__main__":
    main()
