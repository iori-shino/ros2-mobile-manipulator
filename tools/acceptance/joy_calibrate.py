#!/usr/bin/env python3
"""Label real /joy events interactively; never publishes motion."""
import json
import statistics
import time
from pathlib import Path
import rclpy
import yaml
from sensor_msgs.msg import Joy
from rclpy.qos import qos_profile_sensor_data

ROOT = Path(__file__).resolve().parents[2]

def main():
    rclpy.init()
    node = rclpy.create_node('joy_calibration')
    samples = []
    sub = node.create_subscription(Joy, '/joy', lambda m: samples.append(
        {'t': time.monotonic(), 'axes': list(m.axes), 'buttons': list(m.buttons)}), qos_profile_sensor_data)
    out = ROOT / 'validation/stage6'
    out.mkdir(parents=True, exist_ok=True)
    evidence = {'source': '/joy', 'driver': 'joy_node', 'steps': {}}
    def status(label):
        print('\n' + label, flush=True)
        (out / 'calibration_status.json').write_text(json.dumps({'prompt': label, 'done': list(evidence['steps'])}, ensure_ascii=False))
    def collect(seconds):
        samples.clear()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        return list(samples)
    def wait_for(test, label):
        status(label)
        deadline = time.monotonic() + 1800
        last_warning = 0.
        while time.monotonic() < deadline:
            group = collect(.7)
            if not group and time.monotonic()-last_warning > 5:
                print('尚未收到 /joy 数据：请确认 joy_node 正在运行、USB 已连接；这不是未松开手柄。', flush=True)
                last_warning = time.monotonic()
            if len(group) >= 5 and all(test(s) for s in group):
                return group
        raise RuntimeError('等待操作或 /joy 超时，请重新运行。')
    status('手柄标定：此程序只读 /joy，不会让机器人运动。请松开所有按钮、摇杆。')
    group = wait_for(lambda s: not any(s['buttons']) and all(abs(v) < .2 for v in s['axes']), '准备：全部松开，保持约 1 秒')
    baseline = [statistics.median(s['axes'][i] for s in group) for i in range(len(group[0]['axes']))]
    evidence['neutral'] = group
    evidence['axes_count'] = len(baseline)
    evidence['buttons_count'] = len(group[0]['buttons'])
    neutral = lambda s: not any(s['buttons']) and all(abs(v-b) < .2 for v,b in zip(s['axes'], baseline))
    labels = [('left_up','左摇杆 推到上'), ('left_down','左摇杆 推到下'),
              ('left_left','左摇杆 推到左'), ('left_right','左摇杆 推到右'),
              ('right_up','右摇杆 推到上'), ('right_down','右摇杆 推到下'),
              ('right_left','右摇杆 推到左'), ('right_right','右摇杆 推到右')]
    mapping = {}
    for key, label in labels:
        def active(s):
            return not any(s['buttons']) and sum(abs(v-b) > .65 for v,b in zip(s['axes'],baseline)) == 1
        group = wait_for(active, label + '，保持直到出现“松开”提示')
        indices = [max(range(len(baseline)), key=lambda i: abs(s['axes'][i]-baseline[i])) for s in group]
        if len(set(indices)) != 1:
            raise RuntimeError('摇杆轴不稳定，请重试')
        i = indices[0]
        value = statistics.median(s['axes'][i] for s in group)
        mapping[key] = {'axis': i, 'sign': 1 if value > baseline[i] else -1, 'observed': value}
        evidence['steps'][key] = group
        wait_for(neutral, '已记录 ' + key + '；松开所有摇杆和按钮')
    buttons = {}
    for label in ['A', 'B', 'X', 'Y', 'L', 'R', 'ZL', 'ZR', 'minus', 'plus']:
        display = {'minus': '－（减号）', 'plus': '＋（加号）'}.get(label, label)
        group = wait_for(lambda s: sum(s['buttons']) == 1 and all(abs(v-b) < .2 for v,b in zip(s['axes'],baseline)),
                         '按住手柄外壳标注的 ' + display + ' 按钮（不要按摇杆），直到提示松开')
        indices = [s['buttons'].index(1) for s in group]
        if len(set(indices)) != 1 or indices[0] in buttons.values():
            raise RuntimeError('检测到重复或不稳定按钮，请重试')
        buttons[label] = indices[0]
        evidence['steps'][label] = group
        wait_for(neutral, '已记录 ' + label + '；松开所有按钮和摇杆')
    for a,b in [('left_up','left_down'),('left_left','left_right'),('right_up','right_down'),('right_left','right_right')]:
        assert mapping[a]['axis'] == mapping[b]['axis'] and mapping[a]['sign'] == -mapping[b]['sign'], '相反方向不匹配'
    assert len({mapping[k]['axis'] for k in ['left_up','left_left','right_up','right_left']}) == 4
    config = {'device_name': 'Nintendo Co., Ltd. Pro Controller', 'driver_deadzone': .05,
              'axes_count': evidence['axes_count'], 'buttons_count': evidence['buttons_count'],
              'axes': mapping, 'buttons': buttons, 'enable_button': buttons['L'], 'stop_button': buttons['B'],
              'deadzone': .15, 'max_linear': .05, 'max_angular': .4, 'joy_timeout': .3, 'keyboard_timeout': .3}
    (ROOT / 'src/mobile_manipulator_gazebo/config/teleop_mapping.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    evidence['mapping'] = config
    (out / 'joy_calibration.json').write_text(json.dumps(evidence, indent=2))
    status('标定完成！已生成 YAML。L 为使能键，B 为停止键。现在可关闭此终端。')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('FAIL:', exc, flush=True)
        (ROOT/'validation/stage6/calibration_status.json').write_text(json.dumps({'error':str(exc),'completed':False},ensure_ascii=False))
    input('\n按 Enter 关闭窗口...')
