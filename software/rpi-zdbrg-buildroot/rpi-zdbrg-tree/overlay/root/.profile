alias rpi-ledon='echo default-on > /sys/devices/platform/leds/leds/led0/trigger'
alias rpi-ledoff='echo none > /sys/devices/platform/leds/leds/led0/trigger'
alias datarw='mount -o remount,rw /data'
alias dataro='mount -o remount,ro /data'
