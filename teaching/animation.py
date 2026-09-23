"""Write a self-contained robot replay; coordinates remain in metres."""
import html
import json
from pathlib import Path


def save_replay(path, points, walls=(), circles=(), goals=(), bounds=(12, 12), title="Robot replay"):
    data = dict(points=[list(map(float, p[:4])) for p in points],
                walls=[list(map(float, w)) for w in walls],
                circles=[list(map(float, c)) for c in circles],
                goals=[list(map(float, g)) for g in goals], bounds=list(bounds))
    document = '''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Robot replay</title>
<style>body{font:17px system-ui;max-width:850px;margin:30px auto;padding:15px;color:#213247}canvas{width:100%;background:#f0f5f7}input{width:65%}button{padding:10px}</style>
<h1>TITLE</h1><p>This is a replay of computed states in the teaching simulator. It is not a Gazebo recording.</p>
<canvas width="800" height="600" aria-label="Top view of the robot trajectory"></canvas>
<p><button id="play">Pause</button> <input id="time" type="range" min="0" value="0"><output id="label"></output></p>
<script>const d=DATA,c=document.querySelector('canvas'),x=c.getContext('2d'),s=document.querySelector('#time');
s.max=d.points.length-1;let active=true,last=0;const X=v=>35+v/d.bounds[0]*730,Y=v=>565-v/d.bounds[1]*530;
function circle(a,b,r,col){x.beginPath();x.ellipse(X(a),Y(b),r/d.bounds[0]*730,r/d.bounds[1]*530,0,0,7);x.fillStyle=col;x.fill();}
function draw(){let i=+s.value,p=d.points[i];x.clearRect(0,0,800,600);x.strokeStyle='#34465b';x.lineWidth=3;
for(let w of d.walls){x.beginPath();x.moveTo(X(w[0]),Y(w[1]));x.lineTo(X(w[2]),Y(w[3]));x.stroke();}
for(let o of d.circles)circle(...o,'#8795a8');for(let g of d.goals)circle(g[0],g[1],.15,'#dd921a');
x.strokeStyle='#087e94';x.lineWidth=2;x.beginPath();for(let j=0;j<=i;j++){let q=d.points[j];j?x.lineTo(X(q[1]),Y(q[2])):x.moveTo(X(q[1]),Y(q[2]));}x.stroke();
circle(p[1],p[2],.22,'#087e94');x.strokeStyle='#fff';x.beginPath();x.moveTo(X(p[1]),Y(p[2]));x.lineTo(X(p[1]+.25*Math.cos(p[3])),Y(p[2]+.25*Math.sin(p[3])));x.stroke();
document.querySelector('#label').textContent=' t = '+p[0].toFixed(2)+' s';}
document.querySelector('#play').onclick=()=>{active=!active;document.querySelector('#play').textContent=active?'Pause':'Play';};s.oninput=draw;
function tick(t){if(active&&t-last>50){s.value=(+s.value+1)%d.points.length;last=t;draw();}requestAnimationFrame(tick);}draw();requestAnimationFrame(tick);</script></html>'''
    Path(path).write_text(document.replace('TITLE', html.escape(title)).replace('DATA', json.dumps(data)))
