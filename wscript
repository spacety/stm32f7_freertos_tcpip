#!/usr/bin/env python
# encoding: utf-8

import os
import time

from waflib.Build import BuildContext

APPNAME = 'stm32f7_tcpip'
VERSION = '0.1'

top = '.'
out = 'build'
cwd = os.getcwd()

def down_libopencm3():
    if not os.path.exists('libopencm3'):
        print('begin downloading libopencm3 library...\n')
        os.system('git clone https://github.com/libopencm3/libopencm3.git')

def update_libopencm3():
    os.system('cd libopencm3 && git pull && make TARGETS=stm32/f7')

def options(ctx):
    ctx.load('gcc')

    down_libopencm3()

    ctx.add_option('--arch', action='store', default='cortex-m4', help='MCU arch')
    ctx.add_option('--toolchain', action='store', default='arm-none-eabi-', help='Set toolchain prefix')
    ctx.add_option('--update', action='store_true', help='update libopencm3 source')

def configure(ctx):
    ctx.env.CC = ctx.options.toolchain + "gcc"
    ctx.env.AR = ctx.options.toolchain + "ar"
    ctx.load('gcc')

    # Locate programs
    ctx.find_program('st-flash', var='STFLASH')
    ctx.find_program(ctx.options.toolchain + 'size', var='SIZE')
    ctx.find_program(ctx.options.toolchain + 'objcopy', var='OBJCOPY')

    # Generate build arguments
    ctx.env.append_unique('CFLAGS', ['-Wall', '-DSTM32F7', '-DSTM32F7xx', '-fno-common', '-Os', '-mthumb', '-mcpu=cortex-m7', '-mhard-float', '-mfpu=fpv5-d16', '-fno-exceptions', '-ffunction-sections', '-fdata-sections', '-Wempty-body', '-Wtype-limits', '-Wmissing-parameter-type', '-Wuninitialized', '-fno-strict-aliasing', '-Wno-unused-function', '-Wno-stringop-truncation', '-fsingle-precision-constant'])

    ctx.env.append_unique('LINKFLAGS', ['--static', '-nostartfiles', '-Wl,--gc-sections', '-mthumb', '-mcpu=cortex-m7', '-mhard-float', '-mfpu=fpv5-d16'])

    ctx.env.append_unique('LDFLAGS', ['--specs=nano.specs', '-Wl,--start-group', '-lc', '-lgcc', '-lnosys', '-Wl,--end-group', '-lm'])

    # FreeRTOS
    ctx.env.append_unique('FILES', ['rtos/*.c', 'src/*.c'])

    # TCP/IP
    ctx.env.append_unique('FILES', ['tcpip/*.c', 'tcpip/portable/*.c'])

    ctx.env.append_unique('LINKFLAGS', ['-T' + cwd + '/stm32f7.ld', '-Wl,-Map=tcpip.map'])

    ctx.env.append_unique('INCLUDES', ['../libopencm3/include', '../rtos/include', '../src', '../tcpip/include', '../tcpip/portable/include'])

    if ctx.options.update == True or not os.path.exists('libopencm3/lib/libopencm3_stm32f7.a'):
        update_libopencm3()

def build(ctx):
    ctx(export_includes=['include', '.', 'rtos/include', '../src', '../utils', '../tcpip/include', '../tcpip/portable/include', 'libopencm3/include'], name='include')
    # Linker script
    bin_target = 'tcpip.bin'
    hex_target = 'tcpip.hex'

    ctx.program(
        source=ctx.path.ant_glob(ctx.env.FILES),
        target='tcpip.elf',
        #linkflags = ctx.env.LINKFLAGS,
        stlib=['opencm3_stm32f7'],
        stlibpath=[cwd + '/libopencm3/lib']
    )
    ctx(rule='${OBJCOPY} -O binary ${SRC} ${TGT}', source='tcpip.elf', target=bin_target, name='objcopy', always=True)
    ctx(rule='${OBJCOPY} -O ihex ${SRC} ${TGT}', source='tcpip.elf', target=hex_target, name='objcopy', always=True)
    ctx(name="size", rule='${SIZE} ${SRC}', source='tcpip.elf', always=True)

def flash(ctx):
    ctx(name='flash', rule='${STFLASH} write ${SRC} 0x8000000', source='tcpip.bin', always=True)

class Program(BuildContext):
    cmd = 'flash'
    fun = 'flash'
