"""Real CARLA throttle/brake sweep. Does not identify physical R550 hardware."""
import argparse,json,math
from pathlib import Path
import numpy as np
def main():
    import carla
    p=argparse.ArgumentParser();p.add_argument('--host',default='172.21.16.1');p.add_argument('--output',default='artifacts/carla_calibration');p.add_argument('--manual-gear',action='store_true');a=p.parse_args()
    c=carla.Client(a.host,2000);c.set_timeout(20);w=c.get_world();o=w.get_settings();s=w.get_settings();s.synchronous_mode=True;s.fixed_delta_seconds=.05;w.apply_settings(s);results=[];out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    try:
        for throttle in [.1,.15,.2,.25,.3]:
            v=w.spawn_actor(w.get_blueprint_library().find('vehicle.tesla.model3'),w.get_map().get_spawn_points()[0]);rows=[]
            try:
                for _ in range(20):w.tick()
                prev=v.get_velocity();prevspeed=math.hypot(prev.x,prev.y)
                for k in range(160):
                    v.apply_control(carla.VehicleControl(throttle=throttle if k<100 else 0,brake=0 if k<100 else .15,manual_gear_shift=a.manual_gear,gear=1 if a.manual_gear else 0));w.tick();vel=v.get_velocity();acc=v.get_acceleration();speed=math.hypot(vel.x,vel.y)
                    rows.append(dict(t=(k+1)*.05,speed=speed,finite_difference_acceleration=(speed-prevspeed)/.05,native_acceleration=math.hypot(acc.x,acc.y),throttle=throttle if k<100 else 0,brake=0 if k<100 else .15));prevspeed=speed
                report=dict(throttle=throttle,max_speed=max(q['speed'] for q in rows),max_finite_difference_acceleration=max(q['finite_difference_acceleration'] for q in rows[:100]),max_native_acceleration=max(q['native_acceleration'] for q in rows[:100]),final_speed=rows[-1]['speed']);results.append(report)
                (out/f'throttle_{throttle}.jsonl').write_text('\n'.join(json.dumps(q) for q in rows)+'\n');print(json.dumps(report),flush=True)
            finally:v.destroy()
        (out/'identification.json').write_text(json.dumps(dict(status='PASS_REAL_SIM',engine=c.get_server_version(),map=w.get_map().name,dt=.05,runs=results),indent=2))
    finally:w.apply_settings(o)
if __name__=='__main__':main()
