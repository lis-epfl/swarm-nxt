#!/usr/bin/env python3
"""Render one table from the report.json files a self-calibration run collected.

Run on the CONTROLLER, once, over <collect-dir>/<drone>/report.json. Same shape as
analyze_logs.py in drones_postflight.yml: real code rather than Jinja, so it can be
tested outside ansible and the playbook only has to print its stdout.

Two tiers, because 11 drones of full detail is unreadable:
  1. one line per drone, every metric against its threshold, so the fleet fits a screen
  2. a detail block ONLY for drones that did not come back HEALTHY

Markers: '!' the metric that failed, '~' past 75% of its budget. The '~' matters more
than it looks -- it is the early warning that a vehicle is drifting toward a failure
several flights before it actually fails one.
"""
import argparse
import json
import os
import sys

# Thresholds mirror selfcalib/tool/diagnose.py. Duplicated deliberately: this script
# runs on the controller, which need not have the tool importable. If they ever drift,
# the numbers printed here are only labels -- pass/fail always comes from report.json.
THR = {
    'sc': ('self-consistency', 0.10),
    'focal': ('focal %', 2.0),
    'k1': ('k1', 0.06),
    'cxcy': ('c vs circle px', 120.0),
    'rot': ('extr rot deg', 8.0),
    'trans': ('extr trans cm', 15.0),
    'toff': ('t_d ms', 20.0),
    'ate': ('ATE m', 0.20),
}


def mark(value, limit, failed):
    """'!' if this metric is the one that failed, '~' if past 75% of budget."""
    if value is None:
        return ' '
    if failed:
        return '!'
    return '~' if value > 0.75 * limit else ' '


def load(collect_dir):
    rows = []
    for drone in sorted(os.listdir(collect_dir)):
        rp = os.path.join(collect_dir, drone, 'report.json')
        if not os.path.isfile(rp):
            # A drone that produced no report at all is a result too -- it must appear
            # in the table rather than silently vanishing from the fleet view.
            rows.append({'drone': drone, 'missing': True})
            continue
        try:
            r = json.load(open(rp))
        except Exception as e:
            rows.append({'drone': drone, 'missing': True, 'err': str(e)[:40]})
            continue
        d = r.get('diagnosis', {})
        dist = d.get('in_distribution', {})
        ate = d.get('ate', {})
        rows.append({
            'drone': r.get('drone', drone),
            'verdict': (d.get('verdict') or r.get('verdict') or '?'),
            'passes': r.get('passes_run') or r.get('converged_at_pass'),
            'sc': d.get('self_consistency', {}).get('worst_resid'),
            'focal': dist.get('focal_vs_fleet_pct'),
            'k1': dist.get('k1_vs_fleet'),
            'cxcy': dist.get('cxcy_vs_circlefit_px'),
            'rot': dist.get('extr_rot_vs_fleet_deg'),
            'trans': dist.get('extr_trans_vs_fleet_cm'),
            'toff': dist.get('toff_vs_fleet_ms'),
            'ate': ate.get('postg_m') if ate.get('postg_m') is not None else ate.get('global_m'),
            'ate_skipped': ate.get('skipped', False),
            'fails': dist.get('fails', []),
            'ate_source': r.get('ate_source'),
            'deploy_err': r.get('deploy_check_error'),
            'mount': r.get('mount', {}),
            'gates': {k: r.get('gate_' + k, {}).get('pass') for k in ('static', 'timing', 'image')},
            'gate_reason': r.get('gate_static', {}).get('reason', ''),
            'raw': r,
        })
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--collect-dir', required=True)
    ap.add_argument('--json-out', default=None)
    a = ap.parse_args()
    rows = load(a.collect_dir)
    if not rows:
        print('no drones in %s' % a.collect_dir)
        return 0

    print('')
    print('SELF-CALIBRATION — %s' % a.collect_dir)
    hdr = ('%-8s %-17s %4s %7s %7s %7s %8s %7s %7s %8s  %s'
           % ('DRONE', 'VERDICT', 'PASS', 'SC', 'FOCAL%', 'ROT°', 'TRANS cm', 't_d ms', 'ATE m', 'SOURCE', 'FAILS'))
    sub = ('%-8s %-17s %4s %7s %7s %7s %8s %7s %7s %8s'
           % ('', '', '', '/0.10', '/2.0', '/8.0', '/15.0', '/20.0', '/0.20', ''))
    print(hdr)
    print(sub)
    print('-' * len(hdr))

    def fmt(v, spec, m):
        return ('%s%s' % (spec % v, m)) if v is not None else '-'

    for r in rows:
        if r.get('missing'):
            print('%-8s %-17s  (no report.json — the run did not complete)' % (r['drone'], 'NO RESULT'))
            continue
        f = r['fails']
        short = r['verdict'].split(':')[0]
        print('%-8s %-17s %4s %7s %7s %7s %8s %7s %7s %8s  %s' % (
            r['drone'], short, r['passes'] if r['passes'] else '-',
            fmt(r['sc'], '%.4f', mark(r['sc'], THR['sc'][1], r['sc'] is not None and r['sc'] >= THR['sc'][1])),
            fmt(r['focal'], '%.2f', mark(r['focal'], THR['focal'][1], 'focal' in f)),
            fmt(r['rot'], '%.2f', mark(r['rot'], THR['rot'][1], 'extrinsics' in f)),
            fmt(r['trans'], '%.2f', mark(r['trans'], THR['trans'][1], 'extrinsics' in f)),
            fmt(r['toff'], '%.2f', mark(r['toff'], THR['toff'][1], 'toff' in f)),
            ('skipped' if r['ate_skipped'] else
             fmt(r['ate'], '%.4f', mark(r['ate'], THR['ate'][1], r['ate'] is not None and r['ate'] >= THR['ate'][1]))),
            (r['ate_source'] or '-')[:8], ','.join(f) if f else '-'))

    healthy = [r for r in rows if not r.get('missing') and r['verdict'].startswith('HEALTHY')]
    print('')
    print('%d HEALTHY (deployed) · %d other · %d no result'
          % (len(healthy),
             len([r for r in rows if not r.get('missing') and not r['verdict'].startswith('HEALTHY')]),
             len([r for r in rows if r.get('missing')])))

    # ---- tier 2: detail only where something went wrong ----
    for r in rows:
        if r.get('missing') or r['verdict'].startswith('HEALTHY'):
            continue
        print('')
        print('%s  %s' % (r['drone'], r['verdict']))
        bad = [k for k, v in r['gates'].items() if v is False]
        if bad:
            print('   gates failed: %s%s' % (','.join(bad),
                                             ('  — ' + r['gate_reason']) if r['gate_reason'] else ''))
        if r['ate_skipped']:
            print('   ATE: skipped (no ground truth in the recording) — the verdict cannot')
            print('        distinguish a calibration problem from a platform one')
        elif r['ate'] is not None and r['ate'] >= THR['ate'][1]:
            print('   ATE %.3f m exceeds %.2f m — the trajectory, not necessarily the calibration'
                  % (r['ate'], THR['ate'][1]))
        if r['deploy_err']:
            print('   deployment check failed (%s) — ATE above is the CALIBRATION pass,' % r['deploy_err'][:60])
            print('        which reads WORSE than deployment')
        if 'skipped' in r.get('mount', {}):
            print('   mount: skipped — %s' % r['mount']['skipped'])
        if r['fails']:
            print('   out of fleet distribution: %s' % ','.join(r['fails']))
            print('   -> inspect that camera. If the change is intentional, re-run with')
            print('      selfcalib_force=true to accept it as the new fixed point.')

    if a.json_out:
        with open(a.json_out, 'w') as fh:
            json.dump([{k: v for k, v in r.items() if k != 'raw'} for r in rows], fh, indent=1, default=str)
    return 0


if __name__ == '__main__':
    sys.exit(main())
