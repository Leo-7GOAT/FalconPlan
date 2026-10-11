"""Plot real CARLA traces; data provenance and rejected runs remain visible."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
def main():
    p=argparse.ArgumentParser();p.add_argument('--folder',default='artifacts/carla_sensor_v2');p.add_argument('--output',default='artifacts/portfolio');a=p.parse_args();folder=Path(a.folder);out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    report=json.loads((folder/'regression.json').read_text());fig,axes=plt.subplots(2,2,figsize=(12,8))
    for run in report['runs']:
        name=f"{run['scenario']}_seed{run['seed']}";rows=[json.loads(q) for q in (folder/name/'state.jsonl').read_text().splitlines()];t=[r['t'] for r in rows];label=f"{run['scenario']} seed{run['seed']} ({run['status']})"
        axes[0,0].plot([r['x'] for r in rows],[r['y'] for r in rows],label=label)
        axes[0,1].plot(t,[r['v'] for r in rows],label=label)
        axes[1,0].plot(t,[r['acceleration'] for r in rows],label=label)
        runtime=np.sort([r['controller_ms'] for r in rows]);axes[1,1].plot(runtime,np.linspace(0,1,len(runtime)),label=label)
    axes[0,0].set(xlabel='World x [m]',ylabel='World y-left [m]',title='REAL CARLA trajectories')
    axes[0,1].set(xlabel='Simulation time [s]',ylabel='Speed [m/s]',title='Measured speed')
    axes[1,0].set(xlabel='Simulation time [s]',ylabel='Raw finite-difference acceleration [m/s²]',title='Command limits do not replace measured bounds')
    axes[1,1].set(xlabel='Host controller runtime [ms]',ylabel='Empirical CDF',title='Includes MPC solving ticks')
    for ax in axes.flat:ax.legend(fontsize=7);ax.grid(alpha=.3)
    fig.suptitle(f"CARLA {report['server_version']} / {report['map']} — full acceptance remains PARTIAL",fontsize=11);fig.tight_layout();fig.savefig(out/'carla_measured_benchmark.png',dpi=150)
    print(out/'carla_measured_benchmark.png')
if __name__=='__main__':main()
