#!/usr/bin/env python3
"""Audit all scheduled trials and generate paper tables, plots and statistics."""
from pathlib import Path
from collections import defaultdict
import csv
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT/'report'
OUT.mkdir(exist_ok=True)
GROUPS = [('pp', 5000), ('rmpc', 1000), ('rmpc', 2500), ('rmpc', 5000)]
LABELS = ['Pure Pursuit', 'RMPC 1000', 'RMPC 2500', 'RMPC 5000']
COLORS = ['#167d8d', '#d9a441', '#b65a3c', '#4353a5']

def load(folder, expected):
    path = ROOT/folder
    manifest = json.loads((path/'manifest.json').read_text())
    results = []
    assert len(manifest['schedule']) == expected
    for track, method, samples, seed in manifest['schedule']:
        trial_dir = path/f'{track}_{method}_{samples}_s{seed}'
        result = json.loads((trial_dir/'result.json').read_text())
        a = np.load(trial_dir/'trace.npz')['data']
        assert len(a) == result['steps'] and np.isfinite(a).all()
        assert np.all(np.diff(a[:, 0]) > 0)
        assert abs(np.sqrt(np.mean(a[:, 3]**2))-result['rms_d_m']) < 1e-10
        assert abs(a[-1, 0]-result['duration_s']) < 1e-9
        if result['outcome'] == 'completed':
            assert a[-1, 4] >= result['track_length_m'] and not a[:, 10].any()
        elif result['outcome'] == 'collision':
            assert a[:, 10].any()
        assert np.all(np.abs(a[:, 6]) <= .41891)
        assert np.all(a[:, 7] <= 7.0001)
        result['_folder'] = str(trial_dir)
        results.append(result)
    return results, manifest

def bootstrap_mean(values):
    a = np.asarray(values, float)
    rng = np.random.default_rng(20261112)
    boot = a[rng.integers(0, len(a), size=(10000, len(a)))].mean(axis=1)
    return dict(mean=float(a.mean()), lo=float(np.percentile(boot, 2.5)),
                hi=float(np.percentile(boot, 97.5)))

def stats(rows):
    metrics = ['lap_s', 'rms_d_m', 'max_abs_d_m', 'decision_median_ms',
               'decision_p99_ms', 'pipeline_p99_ms', 'decision_over_20ms_fraction']
    out = dict(n=len(rows), completed=sum(r['outcome']=='completed' for r in rows),
               collisions=sum(r['outcome']=='collision' for r in rows),
               timeouts=sum(r['outcome']=='timeout' for r in rows))
    for key in metrics:
        values = [r[key] for r in rows if r[key] is not None]
        out[key] = dict(mean=float(np.mean(values)), sd=float(np.std(values, ddof=1)),
                        min=float(min(values)), max=float(max(values))) if values else None
    return out

def mean_sd(s, key, precision=3):
    if s[key] is None:
        return '--'
    return f"{s[key]['mean']:.{precision}f} $\\pm$ {s[key]['sd']:.{precision}f}"

def savefig(fig, name):
    fig.tight_layout()
    fig.savefig(OUT/f'{name}.pdf', bbox_inches='tight')
    fig.savefig(OUT/f'{name}.png', dpi=180, bbox_inches='tight')
    plt.close(fig)

def main():
    nominal, nm = load('study_v1', 80)
    slippery, sm = load('slippery_v1', 20)
    assert nm['controller_binary_sha256'] == sm['controller_binary_sha256']
    assert nm['script_sha256'] == sm['script_sha256']
    all_rows = nominal+slippery
    keys = [k for k in all_rows[0] if not k.startswith('_')]
    with (OUT/'all_trials.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows({k:r[k] for k in keys} for r in all_rows)
    groups = defaultdict(list)
    for r in all_rows:
        groups[(r['scenario'], r['track'], r['method'], r['samples'])].append(r)
    summary = {'|'.join(map(str, k)):stats(v) for k,v in groups.items()}
    paired = {}
    for track in ('ims', 'f1'):
        pp = {r['seed']:r for r in groups[('nominal', track, 'pp', 5000)]}
        rr = {r['seed']:r for r in groups[('nominal', track, 'rmpc', 5000)]}
        assert pp.keys() == rr.keys()
        for key in ('rms_d_m', 'lap_s'):
            values = [rr[s][key]-pp[s][key] for s in sorted(pp)
                      if rr[s][key] is not None and pp[s][key] is not None]
            paired[f'{track}_{key}_rmpc_minus_pp'] = {**bootstrap_mean(values), 'paired_n':len(values)}
    (OUT/'summary.json').write_text(json.dumps(dict(groups=summary, paired=paired), indent=2)+'\n')
    tex = [r'\begin{table*}[t]', r'\centering\small',
           r'\caption{Fixed 50 Hz study, ten independent trials per row. Means $\pm$ sample SD are over trials; lap time is reported for completed trials. P99 is calculated within each trial and then averaged.}',
           r'\label{tab:controlled}', r'\begin{tabular}{llrrrr}', r'\toprule',
           r'Track & Controller & Completed & Lap (s) & RMS $d$ (m) & Pipeline P99 (ms)\\',r'\midrule']
    for track in ('ims', 'f1'):
        for label,(method,n) in zip(LABELS,GROUPS):
            s = stats(groups[('nominal',track,method,n)])
            tex.append(f"{'IMS' if track=='ims' else 'New circuit'} & {label} & {s['completed']}/10 & "
                       f"{mean_sd(s,'lap_s',2)} & {mean_sd(s,'rms_d_m')} & {mean_sd(s,'pipeline_p99_ms')} " + r"\\")
    tex += [r'\bottomrule',r'\end{tabular}',r'\end{table*}']
    (OUT/'controlled_table.tex').write_text('\n'.join(tex)+'\n')
    # Four panels show every trial, avoiding aggregation over correlated laps.
    fig, axs = plt.subplots(2,2,figsize=(10,6),sharex=True)
    for row,track in enumerate(('ims','f1')):
        for col,key in enumerate(('rms_d_m','lap_s')):
            ax=axs[row,col]
            for j,((method,n),color) in enumerate(zip(GROUPS,COLORS)):
                values=[r[key] for r in groups[('nominal',track,method,n)] if r[key] is not None]
                ax.scatter(j+np.linspace(-.12,.12,len(values)),values,s=15,color=color,alpha=.75)
                ax.errorbar(j,np.mean(values),yerr=np.std(values,ddof=1),fmt='k_',capsize=4)
            ax.set_title(('IMS' if track=='ims' else 'New circuit')+(' — tracking' if col==0 else ' — lap time'))
            ax.set_ylabel('RMS lateral offset (m)' if col==0 else 'Lap time (s)')
            ax.set_xticks(range(4),['PP','RMPC\n1000','RMPC\n2500','RMPC\n5000'])
            ax.grid(axis='y',alpha=.25)
    savefig(fig,'controlled_comparison')
    fig,axs=plt.subplots(1,2,figsize=(10,3.7))
    for track,color in [('ims','#167d8d'),('f1','#4353a5')]:
        ss=[stats(groups[('nominal',track,'rmpc',n)]) for n in (1000,2500,5000)]
        for ax,key in zip(axs,('decision_median_ms','rms_d_m')):
            ax.errorbar([1000,2500,5000],[s[key]['mean'] for s in ss],
                        yerr=[s[key]['sd'] for s in ss],marker='o',capsize=4,
                        label='IMS' if track=='ims' else 'New circuit',color=color)
            ax.set_xlabel('RMPC samples');ax.grid(alpha=.25);ax.legend()
    axs[0].set_ylabel('Median solve-call duration (ms)')
    axs[1].set_ylabel('RMS lateral offset (m)')
    savefig(fig,'sample_tradeoff')
    # Seed chosen before examining outcomes, with no trajectory selection.
    fig,axs=plt.subplots(1,2,figsize=(10,3.7))
    for ax,method,label in zip(axs,('pp','rmpc'),('Pure Pursuit','RMPC 5000')):
        for scenario,style in [('nominal','-'),('slippery_sector','--')]:
            r=next(r for r in groups[(scenario,'f1',method,5000)] if r['seed']==81000)
            a=np.load(Path(r['_folder'])/'trace.npz')['data']
            ax.plot(a[:,4]/r['track_length_m'],a[:,3],style,
                    label='Nominal' if scenario=='nominal' else 'Low-grip sector',lw=1.3)
            if r['outcome']!='completed':
                ax.scatter(a[-1,4]/r['track_length_m'],a[-1,3],marker='x',s=45,color='red')
        ax.axvspan(.15,.35,color='gray',alpha=.15,label='Reduced friction region')
        ax.axhline(1.45,color='gray',ls=':',lw=.8);ax.axhline(-1.45,color='gray',ls=':',lw=.8)
        ax.set_xlabel('Unwrapped progress / track length');ax.set_ylabel('Lateral offset (m)')
        ax.set_title(label+' — seed 81000');ax.grid(alpha=.2);ax.legend(fontsize=7)
    savefig(fig,'slippery_response')
    print(json.dumps({'trial_count':len(all_rows),'summary':summary,'paired':paired},indent=2))

if __name__=='__main__':
    main()
