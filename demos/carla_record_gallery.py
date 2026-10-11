"""Record complete camera timelines around the existing real CARLA regression.

Only camera resolution and wall-time capture pacing change. Existing controllers,
simulation dt, safety gates and physical acceptance measurements are unchanged.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import threading
import time

import numpy as np
from demos.carla_regression import make_routes,run_one


class CameraTimeline:
    def __init__(self,start_frame=0):
        self.condition=threading.Condition()
        self.images=[]
        self.last_frame=-1
        self.start_frame=start_frame

    def receive(self,image):
        pixels=np.frombuffer(image.raw_data,dtype=np.uint8).reshape(image.height,image.width,4)[:,:,:3][:,:,::-1].copy()
        with self.condition:
            if image.frame<=self.last_frame:
                raise ValueError('Camera frames must be unique and ordered')
            self.images.append((int(image.frame),float(image.timestamp),pixels))
            self.last_frame=int(image.frame)
            self.condition.notify_all()

    def pace(self,world_frame,timeout=2.):
        # GPU camera callbacks may trail physics. Permit four 50 ms ticks of
        # pipeline latency, but never let fast control discard the whole video.
        if world_frame<=self.start_frame+4:return
        deadline=time.monotonic()+timeout
        with self.condition:
            while self.last_frame<world_frame-4:
                remaining=deadline-time.monotonic()
                if remaining<=0:
                    raise TimeoutError('Camera pipeline did not keep up; recording is incomplete')
                self.condition.wait(remaining)

    def write(self,folder,start_time,end_time,fps=5):
        from PIL import Image
        selected=[]
        with self.condition:
            for frame,stamp,pixels in self.images:
                if stamp<start_time-1e-6 or stamp>end_time+1e-6:
                    continue
                if not selected or stamp-selected[-1][1]>=1/fps-1e-6:
                    selected.append((frame,stamp,pixels))
        duration=end_time-start_time
        if len(selected)<duration*fps*.85 or selected[-1][1]-selected[0][1]<duration-.6:
            raise RuntimeError('Full-run camera coverage was not captured')
        pictures=[Image.fromarray(pixels).quantize(colors=96) for _,_,pixels in selected]
        durations=[max(10,round((b[1]-a[1])*1000)) for a,b in zip(selected,selected[1:])]
        durations.append(round(1000/fps))
        pictures[0].save(folder/'simulation.gif',save_all=True,append_images=pictures[1:],duration=durations,loop=0)
        Image.fromarray(selected[len(selected)//2][2]).save(folder/'screenshot.png')
        timeline=[dict(camera_frame=frame,sensor_time_s=stamp,run_time_s=stamp-start_time) for frame,stamp,_ in selected]
        (folder/'camera_timeline.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in timeline))
        metadata=dict(source='REAL_CARLA_RGB_CALLBACK',frames=len(selected),resolution=[480,270],target_display_fps=fps,
                      first_run_time_s=timeline[0]['run_time_s'],last_run_time_s=timeline[-1]['run_time_s'],
                      full_simulation_seconds=duration,playback='Sensor timestamp intervals, no artificial motion/interpolation',
                      capture_pacing='Wait for GPU callback within four physics frames; affects wall timing only',
                      gif_sha256=hashlib.sha256((folder/'simulation.gif').read_bytes()).hexdigest())
        (folder/'camera_recording.json').write_text(json.dumps(metadata,indent=2))
        return metadata


class CameraProxy:
    def __init__(self,actor,timeline):self.actor,self.timeline=actor,timeline
    def __getattr__(self,name):return getattr(self.actor,name)
    def listen(self,legacy_callback):
        def callback(image):
            self.timeline.receive(image)
            legacy_callback(image)
        self.actor.listen(callback)


class RecordingWorld:
    def __init__(self,world):self.world,self.timeline=world,None
    def __getattr__(self,name):return getattr(self.world,name)
    def spawn_actor(self,blueprint,*args,**kwargs):
        if blueprint.id=='sensor.camera.rgb':
            blueprint.set_attribute('image_size_x','480')
            blueprint.set_attribute('image_size_y','270')
            blueprint.set_attribute('sensor_tick','0.1')
            self.timeline=CameraTimeline(self.world.get_snapshot().frame)
            return CameraProxy(self.world.spawn_actor(blueprint,*args,**kwargs),self.timeline)
        return self.world.spawn_actor(blueprint,*args,**kwargs)
    def tick(self,*args,**kwargs):
        frame=self.world.tick(*args,**kwargs)
        if self.timeline is not None:self.timeline.pace(frame)
        return frame


def main():
    import carla
    p=argparse.ArgumentParser()
    p.add_argument('--host',default='172.21.16.1');p.add_argument('--port',type=int,default=2000)
    p.add_argument('--output',type=Path,default=Path('artifacts/readme_gallery'))
    p.add_argument('--cpp-lib');p.add_argument('--scenarios',default='static_obstacle,following,curve,actuator_delay,command_timeout,state_timeout')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    client=carla.Client(args.host,args.port);client.set_timeout(20)
    real=client.get_world();original=real.get_settings();baseline={a.id for a in real.get_actors()}
    settings=real.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
    settings.substepping=True;settings.max_substep_delta_time=.01;settings.max_substeps=10
    real.apply_settings(settings);world=RecordingWorld(real);results=[]
    try:
        routes=make_routes(world)
        for scenario in args.scenarios.split(','):
            if scenario not in ('static_obstacle','following','curve','actuator_delay','command_timeout','state_timeout'):
                raise ValueError('Unknown recording scenario')
            route=routes[min(len(routes)-1,len(routes)//2)][1] if scenario=='curve' else routes[0][1]
            full_stack=scenario in ('static_obstacle','following')
            steps=400 if full_stack else 240
            seed=7 if scenario in ('static_obstacle','following','curve') else 19
            summary=run_one(client,world,route,scenario,seed,steps,args.output,full_stack,full_stack,args.cpp_lib)
            end=real.get_snapshot().timestamp.elapsed_seconds
            folder=args.output/f'{scenario}_seed{seed}'
            summary['camera_recording']=world.timeline.write(folder,end-steps*.05,end)
            results.append(summary)
            world.timeline=None
            print(json.dumps(dict(recorded=scenario,frames=summary['camera_recording']['frames'],acceptance=summary['status'])),flush=True)
        report=dict(engine='REAL_CARLA',client_version=client.get_client_version(),server_version=client.get_server_version(),
                    map=real.get_map().name,platform=platform.platform(),purpose='Actual full-run README camera recordings',
                    acceptance_status='PASS_REAL_SIM' if all(r['status']=='PASS_REAL_SIM' for r in results) else 'PARTIAL',
                    capture_status='PASS_COMPLETE_CAMERA_RECORDINGS',runs=results,
                    timing_boundary='Camera pacing/render/encoding runs are excluded from headline performance measurements')
        (args.output/'regression.json').write_text(json.dumps(report,indent=2))
    finally:
        real.apply_settings(original)
        restored=real.get_settings();remaining={a.id for a in real.get_actors()}-baseline
        cleanup=dict(remaining_new_actor_ids=sorted(remaining),all_owned_actors_destroyed=not remaining,
                     settings_restored=restored.synchronous_mode==original.synchronous_mode and restored.fixed_delta_seconds==original.fixed_delta_seconds)
        (args.output/'cleanup.json').write_text(json.dumps(cleanup,indent=2))
    if report['acceptance_status']!='PASS_REAL_SIM':raise SystemExit(1)


if __name__=='__main__':main()
