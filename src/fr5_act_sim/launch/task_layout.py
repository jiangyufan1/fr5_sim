"""Shared, reproducible layout for Gazebo and the initial MoveIt scene."""
import random
import math

COLORS = ['0.85 0.12 0.08 1','0.08 0.35 0.9 1','0.12 0.7 0.25 1','0.95 0.75 0.05 1','0.65 0.18 0.8 1']


def bin_parts(scene):
    b=scene['task']['bin'];x,y=b['center_xy'];sx,sy=b['outer_size_xy'];h=b['height'];t=b['wall_thickness'];z=scene['table_top']
    if not 0<t<min(sx,sy,h)/2:raise ValueError('Invalid bin thickness')
    return [([x,y,z+t/2],[sx,sy,t]),
            ([x-sx/2+t/2,y,z+h/2],[t,sy,h]),([x+sx/2-t/2,y,z+h/2],[t,sy,h]),
            ([x,y-sy/2+t/2,z+h/2],[sx-2*t,t,h]),([x,y+sy/2-t/2,z+h/2],[sx-2*t,t,h])]


def objects(scene):
    task=scene['task'];rng=random.Random(task['seed']);size=task['cube_size'];xmin,xmax,ymin,ymax=task['spawn_region'];gap=task['min_center_distance']
    if size<=0 or gap<size*math.sqrt(2):raise ValueError('Insufficient object separation')
    bx,by=task['bin']['center_xy'];bw,bd=task['bin']['outer_size_xy'];cx=scene['table_center_x'];tw,td,_=scene['table_size']
    result=[]
    for _ in range(10000):
        x,y=rng.uniform(xmin,xmax),rng.uniform(ymin,ymax)
        if not cx-tw/2+size<x<cx+tw/2-size or not -td/2+size<y<td/2-size:continue
        if abs(x-bx)<bw/2+size or abs(y-by)<bd/2+size:
            if abs(x-bx)<bw/2+size and abs(y-by)<bd/2+size:continue
        if any(math.hypot(x-o['xyz'][0],y-o['xyz'][1])<gap for o in result):continue
        i=len(result);result.append({'name':f'object_{i+1}','xyz':[x,y,scene['table_top']+size/2+.002],'size':[size]*3,'color':COLORS[i%len(COLORS)]})
        if len(result)==task['object_count']:return result
    raise ValueError('Cannot place objects without overlap; enlarge spawn region')
