"""Pure wall-time arbitration and fail-closed teleoperation policy."""
import math


class TeleopCore:
    def __init__(self, config=None):
        self.joy_enabled = config is not None
        if config is None:
            config = {'max_linear': .05, 'max_angular': .4, 'deadzone': .15,
                      'joy_timeout': .3, 'keyboard_timeout': .3}
        self.c = config
        assert 0 < config['max_linear'] <= .05
        assert 0 < config['max_angular'] <= .4
        assert 0 <= config['deadzone'] < 1
        assert 0 < config['joy_timeout'] <= .5
        assert 0 < config['keyboard_timeout'] <= .5
        if self.joy_enabled:
            for key in ['left_up', 'left_left']:
                assert 0 <= config['axes'][key]['axis'] < config['axes_count']
                assert config['axes'][key]['sign'] in [-1, 1]
            assert 0 <= config['enable_button'] < config['buttons_count']
            assert 0 <= config['stop_button'] < config['buttons_count']
            assert config['enable_button'] != config['stop_button']
        self.last_joy = -math.inf
        self.last_key = -math.inf
        self.joy_command = (0., 0.)
        self.key_command = (0., 0.)
        self.armed = False
        self.released = False
        self.enable = False
        self.stop = False

    def axis(self, axes, name):
        spec = self.c['axes'][name]
        value = axes[spec['axis']] * spec['sign']
        dz = self.c['deadzone']
        return math.copysign(max(0., (min(1., abs(value))-dz)/(1-dz)), value)

    def joy(self, axes, buttons, now):
        if not self.joy_enabled:
            return
        if (len(axes) != self.c['axes_count'] or len(buttons) != self.c['buttons_count']
                or not all(math.isfinite(v) and abs(v) <= 1.01 for v in axes)
                or not all(v in (0, 1) for v in buttons)):
            self.armed = self.released = False
            self.last_joy = -math.inf
            self.last_key = -math.inf
            return
        if now-self.last_joy > self.c['joy_timeout']:
            self.armed = self.released = False
        self.last_joy = now
        self.enable = bool(buttons[self.c['enable_button']])
        self.stop = bool(buttons[self.c['stop_button']])
        x, z = self.axis(axes, 'left_up'), self.axis(axes, 'left_left')
        if self.stop:
            self.armed = self.released = False
            self.last_key = -math.inf
        elif not self.enable:
            if self.armed:
                self.last_key = -math.inf
            self.armed = False
            self.released = True
        elif self.released and not x and not z:
            self.armed = True
        if self.enable:
            self.last_key = -math.inf
        self.joy_command = (x*self.c['max_linear'], z*self.c['max_angular']) if self.armed else (0., 0.)

    def key(self, key, now):
        if key in (' ', 'q', 'stop'):
            self.last_key = -math.inf
            self.armed = self.released = False
            return
        # A held enable owns the input; keyboard cannot create a delayed command.
        if now-self.last_joy < self.c['joy_timeout'] and (self.enable or self.stop):
            return
        commands = {'w': (.04, 0.), 's': (-.04, 0.), 'a': (0., .3), 'd': (0., -.3)}
        if key in commands:
            self.key_command = commands[key]
            self.last_key = now

    def output(self, now):
        if now-self.last_joy >= self.c['joy_timeout']:
            self.armed = self.released = False
        elif self.stop or self.enable:
            return (self.joy_command if self.armed and not self.stop else (0., 0.)), 'joy'
        if now-self.last_key < self.c['keyboard_timeout']:
            return self.key_command, 'keyboard'
        return (0., 0.), 'idle'
