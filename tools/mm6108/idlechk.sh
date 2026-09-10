#!/bin/sh
# idlechk.sh -- state1 (cpu-sleep, 220 us exit) disable flag + usage counter per CPU.
# usage MUST NOT ADVANCE across an arm that claims cpuidle=off; the flag alone is a claim,
# the counter is the evidence.
for c in /sys/devices/system/cpu/cpu*/cpuidle/state1; do
  n=$(basename $(dirname $(dirname $c)))
  printf "%s disable=%s usage=%s name=%s\n" $n "$(cat $c/disable)" "$(cat $c/usage)" "$(cat $c/name)"
done
