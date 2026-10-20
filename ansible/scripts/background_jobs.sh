#!/bin/bash
# Ubuntu's background jobs on a drone: paused for a flight, resumed after it.
#
#   background_jobs.sh off      before a flight (drones_preflight.yml, drones_relaunch.yml)
#   background_jobs.sh on       after it (drones_postflight.yml, drones_selfcalib.yml)
#   background_jobs.sh status
#
# Run as root (ansible become). The desktop-session part runs as the user who called sudo,
# or as the user given as the second argument.
#
# WHY. Ubuntu updates and maintains itself in the background, and that takes CPU, disk and
# WiFi from the flight stack. Measured on nxt3 on 15 Oct 2026: in the 3 minutes after a boot,
# PackageKit (fed by gnome-software, which downloads updates) used 57 s of CPU, update-notifier
# (apt-check) 19 s and gnome-software 5 s. A flight 78 s after a boot had its VIO poses up to
# 1.5 s late for 40 s. On nxt6 on 20 Oct, the update-manager window that update-notifier opened
# 1 min after a boot took a whole core until it was frozen 10 min later. Later, timers start
# more: update-notifier-download 5 min after a boot, apt-daily about 13 min, tmpfiles-clean
# 15 min, fstrim 33 min, then daily or weekly.
#
# WHAT IS PAUSED, AND HOW
#   - the timers in TIMERS: stopped, and their job stopped too if it is running. A timer whose
#     time passed in the meantime runs its job once when it is started again, after the flight.
#   - PackageKit: masked until "on" (a runtime mask, in /run) so that gnome-software or anything
#     else cannot start it again; masking stops it, which ends a running update check.
#   - cron (sysstat, e2scrub, anacron) and apport (it writes a core dump of any process that
#     crashes, in flight as well): stopped.
#   - update-notifier (with the update-manager it opens, which runs in its scope), update-manager
#     started any other way, gnome-software in the desktop session, and the tracker file indexer:
#     frozen (systemctl freeze: the processes stay, without CPU) and thawed after.
# What a flight needs is not touched: chrony, NetworkManager and wpa_supplicant, avahi, ssh,
# the micro_xrce agent, nvfancontrol, nvpmodel, jetson_clocks.
#
# What "off" paused is listed in $STATE, in /run, which a boot clears (and a boot starts all
# of it again by itself). "on" resumes what the list names and nothing else, so a second
# "off", or an "on" without an "off", leaves alone what this script did not pause.
#
# Exit status: 0; 2 when something of the lists is still running after "off" or still
# paused after "on" (named in the output); 1 on a wrong call.

set -u
STATE=/run/swarmnxt_background_jobs
TIMERS=(apt-daily.timer apt-daily-upgrade.timer update-notifier-download.timer
        update-notifier-motd.timer motd-news.timer fwupd-refresh.timer man-db.timer fstrim.timer
        e2scrub_all.timer dpkg-db-backup.timer anacron.timer ua-timer.timer
        apport-autoreport.timer snapd.snap-repair.timer systemd-tmpfiles-clean.timer)
DAEMONS=(cron.service apport.service)
MASKED=packagekit.service
SESSION_UNITS='update\\x2dnotifier|update\\x2dmanager|gnome\\x2dsoftware|^tracker-miner-fs-3\.service|^tracker-extract-3\.service'

MODE=${1:-}
U=${2:-${SUDO_USER:-}}
[ "$(id -u)" = 0 ] || { echo "background_jobs.sh: run as root"; exit 1; }

state() { systemctl show -p ActiveState --value "$1" 2>/dev/null; }
running() { case "$(state "$1")" in active|activating|reloading) return 0 ;; *) return 1 ;; esac; }
note() { grep -qxF "$1" "$STATE" 2>/dev/null || echo "$1" >> "$STATE"; }

# systemctl --user for the desktop session, whose manager only that user's bus reaches
SESSION=0
if [ -n "$U" ] && id -u "$U" >/dev/null 2>&1; then
  RUNDIR=/run/user/$(id -u "$U")
  [ -S "$RUNDIR/systemd/private" ] && SESSION=1
fi
usr() { runuser -u "$U" -- env XDG_RUNTIME_DIR="$RUNDIR" systemctl --user "$@"; }
session_units() {
  [ $SESSION = 1 ] || return 0
  usr list-units --type=service,scope --state=running --no-legend --plain 2>/dev/null \
    | awk '{print $1}' | grep -E "$SESSION_UNITS"
}
short() { local u=${1//\\x2d/-}; u=${u#app-gnome-}; echo "${u%%-[0-9]*.scope}" | sed 's/\.service$//;s/\.scope$//'; }

dpkg_busy() { fuser /var/lib/dpkg/lock-frontend /var/lib/dpkg/lock >/dev/null 2>&1; }

off() {
  local stopped=() jobs=() daemons=() frozen=() left=() masked="" before
  before=$( [ -s "$STATE" ] && wc -l < "$STATE" || echo 0)
  for t in "${TIMERS[@]}"; do
    running "$t" && systemctl stop "$t" && note "timer $t" && stopped+=("${t%.timer}")
  done
  # A job that installs packages is never cut short: dpkg would be left half-way.
  for i in $(seq 150); do dpkg_busy || break; [ "$i" = 1 ] && echo "waiting for a package installation to finish"; sleep 2; done
  dpkg_busy && left+=("a package installation (dpkg) still running after 5 min")
  for t in "${TIMERS[@]}"; do
    s=${t%.timer}.service
    if running "$s" && ! dpkg_busy; then systemctl stop "$s" && jobs+=("${s%.service}"); fi
  done
  for d in "${DAEMONS[@]}"; do
    running "$d" && systemctl stop "$d" && note "daemon $d" && daemons+=("${d%.service}")
  done
  if [ "$(systemctl show -p LoadState --value $MASKED)" != masked ]; then
    systemctl mask --runtime --now $MASKED >/dev/null 2>&1 && note "mask $MASKED" && masked="PackageKit (masked until 'on')"
  elif grep -qxF "mask $MASKED" "$STATE" 2>/dev/null; then
    systemctl stop $MASKED 2>/dev/null
  fi
  for u in $(session_units); do
    [ "$(usr show -p FreezerState --value "$u")" = running ] && usr freeze "$u" && note "freeze $u" && frozen+=("$(short "$u")")
  done

  # check: nothing of the lists still running
  for t in "${TIMERS[@]}"; do running "$t" && left+=("$t"); running "${t%.timer}.service" && left+=("${t%.timer}.service"); done
  for d in "${DAEMONS[@]}"; do running "$d" && left+=("$d"); done
  running $MASKED && left+=("$MASKED")
  for u in $(session_units); do [ "$(usr show -p FreezerState --value "$u")" = frozen ] || left+=("$u (not frozen)"); done

  echo "background jobs paused for the flight:"
  [ ${#stopped[@]} -gt 0 ] && echo "  timers stopped: ${stopped[*]}"
  [ ${#jobs[@]} -gt 0 ] && echo "  their running jobs stopped: ${jobs[*]}"
  [ ${#daemons[@]} -gt 0 ] || [ -n "$masked" ] && echo "  stopped: ${daemons[*]}${daemons:+ }$masked"
  [ ${#frozen[@]} -gt 0 ] && echo "  frozen: ${frozen[*]}"
  [ "$before" -gt 0 ] && echo "  already paused by an earlier 'off' since the boot: $before of them"
  [ $SESSION = 1 ] || echo "  no desktop session: update-notifier, gnome-software, tracker not looked at"
  if [ ${#left[@]} -gt 0 ]; then echo "  STILL RUNNING: ${left[*]}"; return 2; fi
  echo "  nothing of the lists is running now"
}

on() {
  if [ ! -s "$STATE" ]; then
    echo "background jobs: nothing to resume, none is paused"
    return 0
  fi
  local resumed=() left=()
  # PackageKit first: gnome-software, once thawed, goes straight to it
  while read -r kind unit; do
    [ "$kind" = mask ] && systemctl unmask --runtime "$unit" >/dev/null 2>&1 && resumed+=(PackageKit)
  done < "$STATE"
  while read -r kind unit; do
    case $kind in
      daemon|timer) systemctl start "$unit" && resumed+=("${unit%.*}") ;;
    esac
  done < "$STATE"
  while read -r kind unit; do
    if [ "$kind" = freeze ] && [ $SESSION = 1 ] && usr cat "$unit" >/dev/null 2>&1; then
      [ "$(usr show -p FreezerState --value "$unit")" = running ] || usr thaw "$unit"
      resumed+=("$(short "$unit")")
    fi
  done < "$STATE"

  # check: everything of the list running again
  while read -r kind unit; do
    case $kind in
      mask) [ "$(systemctl show -p LoadState --value "$unit")" = masked ] && left+=("$unit (masked)") ;;
      daemon|timer) running "$unit" || left+=("$unit") ;;
      freeze) [ $SESSION = 1 ] && usr cat "$unit" >/dev/null 2>&1 \
                && [ "$(usr show -p FreezerState --value "$unit")" != running ] && left+=("$unit (frozen)") ;;
    esac
  done < "$STATE"
  echo "background jobs resumed: ${resumed[*]}"
  if [ ${#left[@]} -gt 0 ]; then echo "  STILL PAUSED: ${left[*]}"; return 2; fi
  rm -f "$STATE"
}

status() {
  echo "paused by this script: $( [ -s "$STATE" ] && tr '\n' ';' < "$STATE" || echo nothing)"
  for t in "${TIMERS[@]}"; do printf '  %-34s %s\n' "$t" "$(state "$t")"; done
  for d in "${DAEMONS[@]}" $MASKED; do
    printf '  %-34s %s (%s)\n' "$d" "$(state "$d")" "$(systemctl show -p LoadState --value "$d")"
  done
  for u in $(session_units); do printf '  user %-29s %s\n' "$(short "$u")" "$(usr show -p FreezerState --value "$u")"; done
}

case $MODE in
  off) off ;;
  on) on ;;
  status) status ;;
  *) echo "usage: background_jobs.sh off|on|status [user]"; exit 1 ;;
esac
