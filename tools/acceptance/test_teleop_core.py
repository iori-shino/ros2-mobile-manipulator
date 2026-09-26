"""Safety regressions; synthetic mapping is only a test fixture, not device configuration."""
import copy
import math
from teleop_core import TeleopCore


CONFIG = {'max_linear': .05, 'max_angular': .4, 'deadzone': .15, 'joy_timeout': .3,
          'keyboard_timeout': .3, 'axes_count': 6, 'buttons_count': 14,
          'axes': {'left_up': {'axis': 1, 'sign': 1}, 'left_left': {'axis': 0, 'sign': -1}},
          'enable_button': 4, 'stop_button': 0}

def joy(c, t, x=0., z=0., enable=False, stop=False):
    axes=[0.]*6; buttons=[0]*14
    axes[1]=x; axes[0]=-z
    buttons[4]=int(enable); buttons[0]=int(stop)
    c.joy(axes,buttons,t)

def armed():
    c=TeleopCore(copy.deepcopy(CONFIG))
    joy(c,0)
    joy(c,.01,enable=True)
    assert c.armed
    return c

def test_requires_release_and_center():
    c=TeleopCore(CONFIG)
    joy(c,0,x=1,enable=True)
    assert c.output(.01)[0]==(0.,0.)
    joy(c,.02)
    joy(c,.03,x=1,enable=True)
    assert c.output(.04)[0]==(0.,0.)
    joy(c,.05,enable=True)
    joy(c,.06,x=1,enable=True)
    assert c.output(.07)[0]==(.05,0.)

def test_deadzone_and_signs_and_limits():
    c=armed()
    joy(c,.02,x=.149,z=-.149,enable=True)
    assert c.output(.03)[0]==(0.,0.)
    joy(c,.04,x=1,z=-1,enable=True)
    assert c.output(.05)[0]==(.05,-.4)
    joy(c,.06,x=-1,z=1,enable=True)
    assert c.output(.07)[0]==(-.05,.4)

def test_release_stops_immediately():
    c=armed()
    joy(c,.02,x=1,enable=True)
    joy(c,.03,x=1)
    assert c.output(.03)[0]==(0.,0.)

def test_lost_joy_requires_rearm():
    c=armed()
    joy(c,.02,x=1,enable=True)
    assert c.output(.33)[0]==(0.,0.)
    joy(c,.34,enable=True)
    joy(c,.35,x=1,enable=True)
    assert c.output(.36)[0]==(0.,0.)
    joy(c,.37)
    joy(c,.38,enable=True)
    joy(c,.39,x=1,enable=True)
    assert c.output(.4)[0]==(.05,0.)

def test_stop_button_requires_release():
    c=armed()
    joy(c,.02,x=1,enable=True,stop=True)
    assert c.output(.02)[0]==(0.,0.)
    joy(c,.03,enable=True)
    assert not c.armed
    joy(c,.04)
    joy(c,.05,enable=True)
    assert c.armed

def test_keyboard_timeout_and_stop():
    c=TeleopCore(CONFIG)
    c.key('w',0)
    assert c.output(.1)[0]==(.04,0.)
    assert c.output(.31)[0]==(0.,0.)
    c.key('a',.4)
    assert c.output(.4)[0]==(0.,.3)
    c.key(' ',.41)
    assert c.output(.41)[0]==(0.,0.)

def test_no_stale_keyboard_resumes():
    c=TeleopCore(CONFIG)
    c.key('w',0)
    joy(c,.01)
    joy(c,.02,enable=True)
    c.key('s',.03)
    joy(c,.04)
    assert c.output(.05)[0]==(0.,0.)
    c.key('s',.06)
    assert c.output(.07)[0]==(-.04,0.)

def test_keyboard_stop_preempts_joy():
    c=armed()
    joy(c,.02,x=1,enable=True)
    c.key(' ',.03)
    joy(c,.04,x=1,enable=True)
    assert c.output(.05)[0]==(0.,0.)

def test_invalid_input_fails_closed():
    for axes,buttons in [([0.], [0]*14), ([math.nan]*6,[0]*14), ([2.]*6,[0]*14), ([0.]*6,[2]*14)]:
        c=armed()
        joy(c,.02,x=1,enable=True)
        c.joy(axes,buttons,.03)
        assert c.output(.04)[0]==(0.,0.)

def test_idle_joystick_does_not_own_keyboard():
    c=TeleopCore(CONFIG)
    joy(c,0)
    assert c.output(.01)[1]=='idle'
    c.key('d',.02)
    joy(c,.03)
    assert c.output(.04)==((0.,-.3),'keyboard')

def test_keyboard_only_needs_no_mapping():
    c=TeleopCore()
    c.joy([1.]*6,[1]*14,0)
    assert c.output(.01)[1]=='idle'
    c.key('w',.02)
    assert c.output(.03)[0]==(.04,0.)
