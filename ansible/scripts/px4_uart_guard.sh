#!/bin/bash
# Keeps disk-completion work away from the interrupt of PX4's serial link, and has the disk
# written steadily.
#
#   px4_uart_guard.sh apply     after the micro_xrce agent is started (drones_preflight.yml,
#                               drones_relaunch.yml)
#   px4_uart_guard.sh status    any time; drones_postflight.yml prints the flight's overruns
#
# Run as root (ansible become). Reads /proc and /sys; "apply" writes three settings there.
#
# WHY. PX4 streams to the Jetson over /dev/ttyTHS1 (3100000.serial, 3 Mbaud). The port has no
# receive DMA: its interrupt handler has to empty the 32-byte FIFO within about 50-100 us, or
# bytes are lost (an overrun, the oe count of /proc/tty/driver/tegra_hsuart; the frame then fails
# its CRC and the agent drops the whole PX4 message). Every device interrupt is taken on CPU0,
# the NVMe's included, and a disk write submitted from CPUs 0-3 (CPU0's cluster) is completed
# inside CPU0's NVMe interrupt: about 0.19 ms per 1.2 MB request, with interrupts off. Bench on
# nxt6, 19 Oct 2026, with a 29 MB/s writer like the camera recorder: 1566 overruns and about 870
# lost PX4 samples in 3 min, in windows of 0.8 s every 31 s (the kernel's writeback), as in the
# flights. 0 lost with the UART's interrupt on CPU1; 1 with the writeback workers on CPUs 4-7.
# Both, by this script, 20 Oct, with the boot jobs paused as preflight does: 0 overruns and 0
# lost in 2 x 70 s.
#
# WHAT "apply" DOES
#   1. The writeback workers (the kernel threads that write the page cache to disk) go to the
#      CPUs outside CPU0's cluster (4-7 on an Orin NX): the requests they submit are then
#      completed on those CPUs, not in CPU0's interrupt.
#   2. The UART's interrupt goes to CPU1. The kernel registers it only while a program has the
#      port open, so this waits up to 15 s for the agent, and finds it by name (3100000.serial in
#      /sys/kernel/irq/*/actions). It then stays on CPU1 when the agent closes and reopens the
#      port, until the next boot. CPU1 because it never completes disk requests: with
#      rq_affinity 1, CPU0 completes those of CPUs 1-3, which share its cache.
#   3. The kernel writes cached data to the disk from 64 MB on (vm.dirty_background_bytes; by
#      default from 10 % of the memory, which the camera recorder never reaches, so its 29 MB/s
#      left ~900 MB at a time, every ~31 s). nxt10's 128 GB drive takes 11 s for those 900 MB
#      (84 MB/s; nxt6's 0.8 s), and the camera stream stopped for 4-5 s at each, which failed its
#      self-calibration of 20 Oct 2026 (ATE 62 cm, PLATFORM-DEFECT). On nxt10, 80 s at 29 MB/s:
#      at most 65 MB cached and 80 MB being written, against 711 MB and 13.7 s of writing.
#      Only with item 1 or 2 done: without both, steady flushes from CPUs 0-3 would reach PX4's
#      link all the time instead of every 31 s.
#   4. Says so if the NVMe's rq_affinity is not 1, and leaves it: with 2, CPU1 would complete
#      its own requests, part of that with interrupts off (the IOMMU's TLB sync).
#   5. Notes the counters that "status" compares with.
# A reboot undoes items 1 to 3; the next "apply" does them again.
#
# Exit status: 0; 2 when item 1, 2 or 3 could not be done (named in the output); 1 on a wrong call.

set -u
UART_DEV=3100000.serial          # PX4's port, /dev/ttyTHS1
UART_MMIO=0x03100000
UART_CPU=1
NVME=nvme0n1
WB=/sys/devices/virtual/workqueue/writeback/cpumask
DIRTY_BG=67108864                # bytes of cached data from which the kernel writes it out (64 MB)
STATE=/run/swarmnxt_px4_uart     # "apply": time, overruns, disk interrupts, BLOCK softirqs off CPU0's cluster

MODE=${1:-}
[ "$(id -u)" = 0 ] || { echo "px4_uart_guard.sh: run as root"; exit 1; }

# "0-3,6" -> 0 1 2 3 6, one per line
cpus() { local IFS=, r; for r in $1; do [ -n "$r" ] && seq "${r%-*}" "${r#*-}"; done; }

# the online CPUs outside CPU0's cluster (the kernel's cache domain: package_cpus_list)
others() {
  local c0 c
  c0=" $(cpus "$(cat /sys/devices/system/cpu/cpu0/topology/package_cpus_list 2>/dev/null || echo 0-3)" | tr '\n' ' ')"
  for c in $(cpus "$(cat /sys/devices/system/cpu/online)"); do
    case $c0 in *" $c "*) ;; *) echo "$c" ;; esac
  done
}

hexmask() { local m=0 c; for c in "$@"; do m=$(( m | (1 << c) )); done; printf '%x' "$m"; }

# the interrupt number of the UART, while a program has the port open
uart_irq() {
  local f a
  for f in /sys/kernel/irq/*/actions; do
    read -r a 2>/dev/null < "$f" || continue
    [ "$a" = "$UART_DEV" ] && { f=${f%/actions}; echo "${f##*/}"; return 0; }
  done
  return 1
}

# "overruns disk_interrupts block_softirqs_outside_cpu0_cluster"; overruns -1 when the port is
# not listed. oe is printed by the driver only once it is not 0.
snapshot() {
  local l oe=-1 irqs=0 blk=0 f a c n=()
  l=$(grep -F "mmio:$UART_MMIO" /proc/tty/driver/tegra_hsuart 2>/dev/null)
  if [ -n "$l" ]; then
    oe=0
    case $l in *" oe:"*) oe=${l##* oe:}; oe=${oe%% *} ;; esac
  fi
  for f in /sys/kernel/irq/*/actions; do
    read -r a 2>/dev/null < "$f" || continue
    case $a in
      nvme0q*) IFS=, read -r -a n < "${f%/actions}/per_cpu_count"
               for c in "${n[@]}"; do irqs=$(( irqs + c )); done ;;
    esac
  done
  read -r -a n < <(grep '^ *BLOCK:' /proc/softirqs)
  for c in $(others); do blk=$(( blk + ${n[c + 1]:-0} )); done
  echo "$oe $irqs $blk"
}

apply() {
  local rc=0 o=() m v irq="" eff rq oe irqs blk wb_ok=0 uart_ok=0
  echo "PX4 serial link ($UART_DEV) kept clear of disk interrupts:"

  # 1. writeback workers
  mapfile -t o < <(others)
  if [ ${#o[@]} -eq 0 ]; then
    echo "  writeback workers: NOT MOVED, no CPU online outside CPU0's cluster"; rc=2
  else
    m=$(hexmask "${o[@]}")
    { echo "$m" > $WB; } 2>/dev/null
    v=$(tr -d ',\n' < $WB 2>/dev/null)
    if [ -n "$v" ] && [ $(( 16#$v )) = $(( 16#$m )) ]; then
      echo "  writeback workers: on CPUs ${o[*]} (cpumask $m)"; wb_ok=1
    else
      echo "  writeback workers: NOT MOVED, $WB reads '${v:-nothing}'"; rc=2
    fi
  fi

  # 2. the UART's interrupt
  for _ in $(seq 30); do irq=$(uart_irq) && break; sleep 0.5; done
  if [ -z "$irq" ]; then
    echo "  UART interrupt: NOT MOVED, no handler named $UART_DEV after 15 s (is the micro_xrce agent running?)"; rc=2
  else
    { echo $UART_CPU > /proc/irq/"$irq"/smp_affinity_list; } 2>/dev/null
    read -r eff < /proc/irq/"$irq"/effective_affinity_list
    if [ "$eff" = "$UART_CPU" ]; then
      echo "  UART interrupt $irq: on CPU$UART_CPU"; uart_ok=1
    else
      echo "  UART interrupt $irq: NOT MOVED, it is on CPU '$eff'"; rc=2
    fi
  fi

  # 3. steady disk writes, only with item 1 or 2 done
  if [ $wb_ok = 1 ] || [ $uart_ok = 1 ]; then
    { echo $DIRTY_BG > /proc/sys/vm/dirty_background_bytes; } 2>/dev/null
    read -r v 2>/dev/null < /proc/sys/vm/dirty_background_bytes
    if [ "${v:-}" = "$DIRTY_BG" ]; then
      echo "  disk writes: steady, from $(( DIRTY_BG >> 20 )) MB of cached data on"
    else
      echo "  disk writes: NOT SET, vm.dirty_background_bytes reads '${v:-nothing}'"; rc=2
    fi
  else
    echo "  disk writes: left as they are (bursts every ~31 s), since neither of the above was done"
  fi

  # 4. rq_affinity, looked at only
  read -r rq 2>/dev/null < /sys/block/$NVME/queue/rq_affinity
  [ "${rq:-}" = 1 ] || echo "  $NVME rq_affinity is '${rq:-?}', not 1: then CPU1 can complete disk requests (left as it is)"

  # 5. counters for "status"
  echo "$(date +%s) $(snapshot)" > $STATE
  read -r _ oe irqs blk < $STATE
  echo "  receive overruns since boot: $( [ "$oe" -ge 0 ] && echo "$oe" || echo "unknown, $UART_MMIO not in /proc/tty/driver/tegra_hsuart")"
  return $rc
}

status() {
  local m v irq eff rq oe irqs blk t0 oe0 irqs0 blk0
  read -r m 2>/dev/null < $WB
  echo "writeback workers: cpumask ${m:-unknown} (CPUs outside CPU0's cluster: $(others | tr '\n' ' '))"
  if irq=$(uart_irq); then
    read -r eff < /proc/irq/"$irq"/effective_affinity_list
    echo "UART interrupt $irq ($UART_DEV): on CPU $eff; count per CPU $(cat /sys/kernel/irq/"$irq"/per_cpu_count)"
  else
    echo "UART interrupt: no handler named $UART_DEV, the port is not open"
  fi
  read -r rq 2>/dev/null < /sys/block/$NVME/queue/rq_affinity
  echo "$NVME rq_affinity: ${rq:-unknown}"
  read -r v 2>/dev/null < /proc/sys/vm/dirty_background_bytes
  if [ "${v:-0}" -gt 0 ]; then
    echo "disk writes: from $(( v >> 20 )) MB of cached data on"
  else
    echo "disk writes: from $(cat /proc/sys/vm/dirty_background_ratio) % of the memory on (the default: bursts every ~31 s)"
  fi
  read -r oe irqs blk <<< "$(snapshot)"
  echo "receive overruns since boot: $( [ "$oe" -ge 0 ] && echo "$oe" || echo unknown)"
  if [ -s $STATE ]; then
    read -r t0 oe0 irqs0 blk0 < $STATE
    echo "since the last apply ($(date -d @"$t0" +%H:%M:%S), $(( $(date +%s) - t0 )) s ago): $(( oe - oe0 )) overruns;" \
         "$(( irqs - irqs0 )) disk interrupts, $(( blk - blk0 )) BLOCK softirqs outside CPU0's cluster" \
         "(about one per request completed there; the others were completed inside CPU0's interrupt)"
  else
    echo "no apply since the boot"
  fi
}

case $MODE in
  apply) apply ;;
  status) status ;;
  *) echo "usage: px4_uart_guard.sh apply|status"; exit 1 ;;
esac
