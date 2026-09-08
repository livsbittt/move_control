"""Plot recorded Gazebo calibration evidence; never starts ROS or simulation."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Patch
from matplotlib.colors import ListedColormap, BoundaryNorm

PHASES={None:('#9ca3af','Waiting'), 'collecting':('#64748b','Sensor checks'),
        'validating_motion':('#2563eb','Translation test'),
        'relocating_calibration':('#d97706','Relocation'),
        'validating_rotation':('#7c3aed','Rotation test'),
        'returning_calibration':('#08916b','Return to origin'),
        'ready':('#15803d','Ready'), 'failed':('#dc2626','Failed'),
        'sensor_hold':('#be123c','Sensor hold')}


def phase(value):
    return PHASES.get(value,('#a16207',str(value).replace('_',' ')))


def read_case(folder):
    folder=Path(folder)
    names=('track_samples.json','track_identity.json','track_result.json','run_manifest.json')
    samples,identity,result,manifest=[json.loads((folder/name).read_text(encoding='utf-8')) for name in names]
    if result.get('run_id')!=manifest.get('run_id'):
        raise ValueError(f'Run identity mismatch: {folder}')
    data=np.load(folder/'track_map.npz',allow_pickle=False)
    rows=[row for row in samples if row.get('pose') is not None]
    if not rows:raise ValueError(f'No recorded trajectory: {folder}')
    positions=np.array([r['pose'][:2] for r in rows],dtype=float)
    times=np.array([r['sim_s'] for r in rows],dtype=float)
    if not np.isfinite(positions).all() or not np.isfinite(times).all() or np.any(np.diff(times)<0):
        raise ValueError('Invalid recorded trajectory')
    return folder,rows,identity,result,manifest,positions,times,data


def plot_cases(folders,out):
    cases=[read_case(folder) for folder in folders]
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,
                         'axes.spines.right':False,'axes.titlesize':12,'figure.facecolor':'#f8fafc'})
    fig,axes=plt.subplots(len(cases),3,figsize=(17,5.5*len(cases)),squeeze=False,
                          gridspec_kw={'width_ratios':[1,1,1.25]})
    provenance=[]
    used_phases=[]
    for index,case in enumerate(cases):
        folder,rows,identity,result,manifest,xy,times,saved=case
        world,map_ax,time_ax=axes[index]
        label=identity.get('case',folder.name)
        for pose,size in identity['walls']:
            corners=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*np.array(size[:2])/2
            c,s=np.cos(pose[5]),np.sin(pose[5])
            corners=corners@np.array([[c,s],[-s,c]])+pose[:2]
            world.add_patch(Polygon(corners,facecolor='#334155',edgecolor='#0f172a',lw=.7))
        for j in range(1,len(rows)):
            value=rows[j].get('calibration');color,_=phase(value)
            world.plot(xy[j-1:j+1,0]*100,xy[j-1:j+1,1]*100,color=color,lw=3)
            if value not in used_phases:used_phases.append(value)
        # Walls above use metres: convert vertices explicitly into centimetres.
        for patch in world.patches:
            patch.set_xy(patch.get_xy()*100)
        radius=identity['robot_radius_m']*100
        origin=np.array(identity['spawn'])*100
        world.add_patch(Circle(origin,radius,fill=False,color='#475569',ls='--',lw=1.2))
        world.add_patch(Circle(xy[-1]*100,radius,fill=False,color='#15803d',lw=1.2))
        world.scatter(*origin,s=90,marker='x',color='#0f172a',zorder=5,label='Recorded origin')
        world.scatter(*(xy[-1]*100),s=45,facecolor='white',edgecolor='#15803d',zorder=6,label='Final sample')
        lo=np.minimum(xy.min(axis=0)*100,origin)-radius-8
        hi=np.maximum(xy.max(axis=0)*100,origin)+radius+8
        span=max(hi-lo);center=(lo+hi)/2
        world.set(xlim=(center[0]-span/2,center[0]+span/2),ylim=(center[1]-span/2,center[1]+span/2),
                  xlabel='World x (cm)',ylabel='World y (cm)',title=f'{label} | Ground truth (local view)')
        world.set_aspect('equal');world.grid(alpha=.15)
        world.legend(loc='upper left',fontsize=8)
        world.text(.02,.02,'Circles: trusted body circumradius\nOrientation is not recorded in these samples',
                   transform=world.transAxes,fontsize=8,color='#475569',va='bottom')

        arr=saved['data'];res=float(saved['resolution']);ox,oy=saved['origin']
        palette=ListedColormap(['#94a3b8','#ffffff','#0f172a'])
        classified=np.where(arr<0,0,np.where(arr>=65,2,1))
        map_ax.imshow(classified,origin='lower',extent=[ox*100,(ox+arr.shape[1]*res)*100,
                      oy*100,(oy+arr.shape[0]*res)*100],cmap=palette,norm=BoundaryNorm([-.5,.5,1.5,2.5],3),interpolation='nearest')
        map_ax.set(title='Recorded SLAM occupancy raster',xlabel='Map x (cm)',ylabel='Map y (cm)')
        map_ax.set_aspect('equal')
        map_ax.legend(handles=[Patch(facecolor=c,label=l) for c,l in zip(palette.colors,['Unknown','Free','Occupied'])],
                      loc='lower left',fontsize=8)

        for j in range(len(rows)-1):
            time_ax.axvspan(times[j],times[j+1],color=phase(rows[j].get('calibration'))[0],alpha=.12,lw=0)
        delta=(xy-np.array(identity['spawn']))*100
        time_ax.plot(times,delta[:,0],color='#0f172a',lw=1.8,label='x - origin')
        time_ax.plot(times,delta[:,1],color='#64748b',lw=1.5,ls='--',label='y - origin')
        time_ax.axhline(0,color='#cbd5e1',lw=.8)
        time_ax.set(title='Recorded position and calibration phase',xlabel='Simulation time (s)',ylabel='Offset from origin (cm)')
        time_ax.grid(alpha=.15);time_ax.legend(loc='lower left',fontsize=8)
        end_error=np.linalg.norm(xy[-1]-identity['spawn'])*100
        map_status=('Full map completion: not assessed in calibration cases' if identity.get('case')
                    else f"Map completion: {result.get('mapping_complete')} (separate from calibration)")
        text=(f"Final: {result.get('calibration_phase','unknown')} | ready={result.get('calibration_ready')}\n"
              f"Origin error: {end_error:.2f} cm | sampled path: {result.get('path_m',float('nan'))*100:.1f} cm\n"
              f"{map_status}\n"
              f"Run: {manifest.get('run_id','?')[:12]} | source: {manifest.get('source_at_start','?')[:12]}")
        time_ax.text(.02,.98,text,transform=time_ax.transAxes,va='top',fontsize=9,
                     bbox={'facecolor':'white','edgecolor':'#e2e8f0','alpha':.94,'pad':6})
        provenance.append({'directory':str(folder),'run_id':manifest['run_id'],
            'input_sha256':{name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in
              ('track_samples.json','track_identity.json','track_map.npz','track_result.json','run_manifest.json')}})
    fig.suptitle('Recorded Gazebo calibration evidence',fontsize=19,x=.04,ha='left',y=.99)
    fig.legend(handles=[Patch(facecolor=phase(p)[0],label=phase(p)[1]) for p in used_phases],
               loc='lower center',bbox_to_anchor=(.5,.044),ncol=7,frameon=False,fontsize=9)
    fig.text(.04,.018,'Wheel-physics simulation only. Auxiliary sensors: synthetic camera/IR, ground-truth IMU, lidar-derived US. Physical robot NOT verified.',fontsize=9,color='#475569')
    fig.subplots_adjust(left=.055,right=.985,top=.87 if len(cases)==1 else .94,bottom=.17 if len(cases)==1 else .10,wspace=.32,hspace=.4)
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(out,dpi=170,facecolor=fig.get_facecolor());plt.close(fig)
    out.with_suffix('.inputs.json').write_text(json.dumps(provenance,indent=2),encoding='utf-8')
    return out


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cases',nargs='+',type=Path)
    parser.add_argument('--out',required=True,type=Path)
    args=parser.parse_args()
    print(plot_cases(args.cases,args.out))


if __name__=='__main__':main()
