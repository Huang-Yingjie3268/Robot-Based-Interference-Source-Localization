#!/usr/bin/env python3
"""Independent verifier for Question 1 using Sutherland-Hodgman clipping.

This file deliberately does not import q1_bounded_error_localization.py.  It rebuilds bearing-cone
half-planes, clips a large convex box incrementally, enumerates the diameter,
and computes a minimum enclosing circle independently.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
import numpy as np

TOL = 1.0e-9
CLIP_RADIUS = 1.0e7


def cone_halfplanes(sensors: np.ndarray, bearings_deg: np.ndarray, delta_deg: float):
    rows, rhs = [], []
    d = math.radians(delta_deg)
    for s, bd in zip(sensors, bearings_deg):
        t = math.radians(float(bd))
        lo = np.array([math.cos(t-d), math.sin(t-d)])
        hi = np.array([math.cos(t+d), math.sin(t+d)])
        a1 = np.array([lo[1], -lo[0]])
        a2 = np.array([-hi[1], hi[0]])
        rows.extend((a1, a2)); rhs.extend((float(a1@s), float(a2@s)))
    return np.array(rows), np.array(rhs)


def segment_line_intersection(p: np.ndarray, q: np.ndarray, a: np.ndarray, b: float) -> np.ndarray:
    den = float(a @ (q-p))
    if abs(den) <= TOL:
        return p.copy()
    t = (b - float(a@p)) / den
    return p + t*(q-p)


def clip_polygon(poly: np.ndarray, a: np.ndarray, b: float) -> np.ndarray:
    if len(poly) == 0:
        return poly
    out = []
    for i in range(len(poly)):
        p, q = poly[i], poly[(i+1) % len(poly)]
        pin = float(a@p) <= b + 2e-7
        qin = float(a@q) <= b + 2e-7
        if pin and qin:
            out.append(q)
        elif pin and not qin:
            out.append(segment_line_intersection(p, q, a, b))
        elif (not pin) and qin:
            out.append(segment_line_intersection(p, q, a, b)); out.append(q)
    if not out:
        return np.empty((0,2))
    unique=[]
    for x in out:
        if not unique or np.linalg.norm(x-unique[-1]) > 1e-7:
            unique.append(x)
    if len(unique)>1 and np.linalg.norm(unique[0]-unique[-1]) <= 1e-7:
        unique.pop()
    return np.array(unique)


def sutherland_hodgman(a: np.ndarray, b: np.ndarray) -> tuple[str,np.ndarray]:
    r=CLIP_RADIUS
    poly=np.array([[-r,-r],[r,-r],[r,r],[-r,r]],dtype=float)
    for ai,bi in zip(a,b):
        poly=clip_polygon(poly,ai,float(bi))
        if len(poly)==0:
            return 'empty', poly
    if np.any(np.max(np.abs(poly),axis=1) > 0.999999*r):
        return 'unbounded', np.empty((0,2))
    # merge almost identical points
    unique=[]
    for p in poly:
        if not any(np.linalg.norm(p-q)<=1e-6 for q in unique): unique.append(p)
    poly=np.array(unique)
    if len(poly)==1:
        return 'point',poly
    area=0.0
    if len(poly)>=3:
        area=0.5*abs(float(np.dot(poly[:,0],np.roll(poly[:,1],-1))-np.dot(poly[:,1],np.roll(poly[:,0],-1))))
    if len(poly)==2 or area<=1e-8:
        # take farthest two endpoints if clipping yielded repeated collinear points
        if len(poly)>2:
            _,pair=diameter(poly); poly=poly[list(pair)]
        return 'segment',poly
    center=poly.mean(axis=0)
    poly=poly[np.argsort(np.arctan2(poly[:,1]-center[1],poly[:,0]-center[0]))]
    return 'bounded_polygon',poly


def diameter(points: np.ndarray):
    if len(points)<2:return 0.0,(0,0)
    best=-1.0; pair=(0,1)
    for i in range(len(points)):
        for j in range(i+1,len(points)):
            d=float(np.linalg.norm(points[i]-points[j]))
            if d>best: best=d;pair=(i,j)
    return best,pair

@dataclass
class Circle:
    center: np.ndarray
    radius: float

def c2(a,b):
    c=(a+b)/2;return Circle(c,float(np.linalg.norm(a-c)))
def c3(a,b,c):
    M=np.array([2*(b-a),2*(c-a)]); y=np.array([float(b@b-a@a),float(c@c-a@a)])
    if abs(float(np.linalg.det(M)))<=TOL:
        vals=[c2(a,b),c2(a,c),c2(b,c)]
        vals=[z for z in vals if all(np.linalg.norm(p-z.center)<=z.radius+1e-8 for p in (a,b,c))]
        return min(vals,key=lambda z:z.radius)
    o=np.linalg.solve(M,y);return Circle(o,float(np.linalg.norm(a-o)))
def mec(points):
    ids=list(range(len(points)));random.Random(2026).shuffle(ids);C=None
    for ii,i in enumerate(ids):
        p=points[i]
        if C is not None and np.linalg.norm(p-C.center)<=C.radius+1e-8: continue
        C=Circle(p.copy(),0.0)
        for jj in range(ii):
            q=points[ids[jj]]
            if np.linalg.norm(q-C.center)<=C.radius+1e-8: continue
            C=c2(p,q)
            for kk in range(jj):
                rr=points[ids[kk]]
                if np.linalg.norm(rr-C.center)>C.radius+1e-8:C=c3(p,q,rr)
    return C

def bearing(sensor, source, offset):
    t=math.degrees(math.atan2(source[1]-sensor[1],source[0]-sensor[0]));return (t+offset)%360

def cases():
    src=np.array([0.,0.]); tri=np.array([[1000.,0.],[-500.,500*math.sqrt(3)],[-500.,-500*math.sqrt(3)]])
    return {
      'two_point':(np.array([[0.,0.],[100.,0.]]),np.array([45.,135.]),1.),
      'three_point':(np.array([[0.,0.],[100.,0.],[50.,100.]]),np.array([45.,135.,270.]),1.),
      'equilateral_counterexample':(tri,np.array([bearing(s,src,.5) for s in tri]),1.),
      'point_degenerate':(np.array([[0.,0.],[0.,0.]]),np.array([0.,180.]),1.),
      'unbounded':(np.array([[0.,0.]]),np.array([0.]),1.),
    }

def main():
    expected={
      'two_point':('bounded_polygon',4,3.4920769491747663,1.7460384745873796),
      'three_point':('bounded_polygon',6,3.4920769491747663,1.7460384745873796),
      'equilateral_counterexample':('bounded_polygon',3,30.229605714,17.453070997),
      'point_degenerate':('point',1,0.0,0.0),
      'unbounded':('unbounded',0,math.inf,math.inf),
    }
    print('case,state,vertices,D_m,Rmin_m')
    for name,(s,b,dlt) in cases().items():
        a,bv=cone_halfplanes(s,b,dlt);state,v=sutherland_hodgman(a,bv)
        es,ev,ed,er=expected[name]
        assert state==es,(name,state,es)
        assert len(v)==ev,(name,len(v),ev)
        if state in ('empty','unbounded'):
            print(f'{name},{state},{len(v)},--,--');continue
        D,_=diameter(v);R=mec(v).radius
        assert abs(D-ed)<=1e-6,(name,D,ed)
        assert abs(R-er)<=1e-6,(name,R,er)
        print(f'{name},{state},{len(v)},{D:.9f},{R:.9f}')
    print('S-H independent verification: PASS (tolerance 1e-6)')

if __name__=='__main__':main()
