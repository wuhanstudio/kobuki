from building import *
import rtconfig

# get current directory
cwd     = GetCurrentDir()

# The set of source files associated with this SConscript file.
src     = Glob('*.c')

if GetDepend('KOBUKI_USING_GET_ODOMETRY_EXAMPLE'):
	src    += Glob('examples/kobuki_get_odometry_example.c')

if GetDepend('KOBUKI_USING_GET_VERSION_EXAMPLE'):
	src    += Glob('examples/kobuki_get_version_example.c')

if GetDepend('KOBUKI_USING_LED_EXAMPLE'):
	src    += Glob('examples/kobuki_led_example.c')

if GetDepend('KOBUKI_USING_PLAY_SOUND_EXAMPLE'):
	src    += Glob('examples/kobuki_play_sound_example.c')

if GetDepend('KOBUKI_USING_POWER_EXAMPLE'):
	src    += Glob('examples/kobuki_power_example.c')

if GetDepend('KOBUKI_USING_SET_SPEED_EXAMPLE'):
	src    += Glob('examples/kobuki_set_speed_example.c')

path   =  [cwd]

LOCAL_CCFLAGS = ''

group = DefineGroup('kobuki', src, depend = ['PKG_USING_KOBUKI'], CPPPATH = path, LOCAL_CCFLAGS = LOCAL_CCFLAGS)

Return('group')
