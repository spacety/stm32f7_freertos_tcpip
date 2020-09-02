# stm32f7_freertos_tcpip
stm32f7 FreeRTOS TCP/IP with libopencm3 for STM32F767Zi Nucleo Board.

## install arm gcc cross compiler
```
  download arm gcc toolchain from https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
  put arm-none-eabi-gcc in PATH
```
## download libopencm3
```
   git clone https://github.com/libopencm3/libopencm3.git
```
## install stlink utility
   read https://github.com/texane/stlink/blob/master/doc/compiling.md

## build program
```
   ./waf configure
   ./waf build
```

## flash program
```
   ./waf flash
```

## connect to STM32F767Zi Nucleo board
```
tio /dev/ttyACM0 
[tio 11:53:18] tio v1.32
[tio 11:53:18] Press ctrl-t q to quit
[tio 11:53:18] Connected

[UP | 192.168.16.50] # reset
PHY ID 7C130
xPhyReset: phyBMCR_RESET 0 ready
[DOWN] # +TCP: advertise: 01E1 config 3100
prvEthernetUpdateConfig: LS mask 00 Force 1
Autonego ready: 00000004: full duplex 100 mbit high status
Network buffers: 55 lowest 55
Link Status is high
Network buffers: 55 lowest 54
vDHCPProcess: offer c0a81032ip
vDHCPProcess: offer c0a81032ip

[DOWN] # rng
NEW RNG NUMBER 0xb6edb4f
[UP | 192.168.16.50] # 
[UP | 192.168.16.50] # 

[tio 11:55:38] Disconnected
[t61p:~ $] ping 192.168.16.50
PING 192.168.16.50 (192.168.16.50) 56(84) bytes of data.
64 bytes from 192.168.16.50: icmp_seq=1 ttl=64 time=0.175 ms
64 bytes from 192.168.16.50: icmp_seq=2 ttl=64 time=0.154 ms
64 bytes from 192.168.16.50: icmp_seq=3 ttl=64 time=0.235 ms
^C
--- 192.168.16.50 ping statistics ---
3 packets transmitted, 3 received, 0% packet loss, time 2033ms
rtt min/avg/max/mdev = 0.154/0.188/0.235/0.034 ms

```
