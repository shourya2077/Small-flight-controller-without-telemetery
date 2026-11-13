#pragma once
#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><title>Rocket FC</title>
<style>
:root{--bg:#0d1117;--card:#161b22;--line:#262d36;--tx:#e6edf3;--mut:#8b949e;--ok:#3fb950;--wn:#d29922;--bad:#f85149;--bl:#58a6ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.35 system-ui,sans-serif;padding:10px;max-width:760px;margin:auto}
h2{font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:var(--mut);margin:18px 2px 6px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(112px,1fr));gap:8px}
.t{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:8px 10px}
.t b{display:block;font-size:20px;font-variant-numeric:tabular-nums}.t span{font-size:11px;color:var(--mut)}
#banner{border-radius:10px;padding:12px;text-align:center;font-weight:700;font-size:18px;background:#21262d}
#go{margin-top:6px;border-radius:10px;padding:8px;text-align:center;font-weight:700}
.go{background:#12301b;color:var(--ok)}.nogo{background:#3a1416;color:var(--bad)}.caut{background:#3a2a0c;color:var(--wn)}
table{width:100%;border-collapse:collapse}td{padding:5px 4px;border-bottom:1px solid var(--line)}td:last-child{text-align:right;font-variant-numeric:tabular-nums}
.chip{display:inline-block;padding:1px 8px;border-radius:99px;font-size:11px;font-weight:700;margin-left:6px}
.good{background:#12301b;color:var(--ok)}.ok{background:#3a2a0c;color:var(--wn)}.bad{background:#3a1416;color:var(--bad)}.na{background:#21262d;color:var(--mut)}
button{background:#21262d;color:var(--tx);border:1px solid #30363d;border-radius:8px;padding:10px 12px;font:inherit;font-weight:600;margin:3px 3px 3px 0}
button:disabled{opacity:.35}button.danger{background:#5b1a1d;border-color:#8e2a2e}button.pri{background:#17407a;border-color:#245ea8}
input{background:#0d1117;color:var(--tx);border:1px solid #30363d;border-radius:6px;padding:7px;width:90px;font:inherit}
label{display:inline-block;margin:4px 10px 4px 0;font-size:12px;color:var(--mut)}label input{display:block;margin-top:2px}
canvas{width:100%;height:120px;display:block}
.bar{position:relative;height:8px;background:#21262d;border-radius:4px;width:120px;display:inline-block;vertical-align:middle;margin-right:8px}.bar i{position:absolute;top:0;bottom:0;background:#58a6ff;border-radius:4px}.bar u{position:absolute;left:50%;top:-2px;bottom:-2px;width:1px;background:#484f58}#warn div{padding:3px 0}
</style></head><body>
<div id="banner">connecting…</div><div id="go"></div><div id="warn" class="card" style="margin-top:6px;display:none"></div>

<h2>Live</h2>
<div class="grid">
<div class="t"><b id="alt">–</b><span>altitude AGL (m)</span></div>
<div class="t"><b id="pk">–</b><span>peak altitude (m)</span></div>
<div class="t"><b id="cl">–</b><span>climb rate (m/s)</span></div>
<div class="t"><b id="vz">–</b><span>IMU vertical speed (m/s)</span></div>
<div class="t"><b id="pr">–</b><span>pressure (hPa)</span></div>
<div class="t"><b id="bt">–</b><span>baro temp (°C)</span></div>
<div class="t"><b id="mg">–</b><span>|accel| (g)</span></div>
<div class="t"><b id="tl">–</b><span>tilt, unsigned (°)</span></div>
</div>
<div class="card" style="margin-top:8px;display:flex;gap:10px;align-items:center">
<div style="text-align:center"><svg id="arrow" width="86" height="86" viewBox="-37 -37 74 74"><circle r="34" fill="none" stroke="#30363d"/><path d="M-30 0H30M0-30V30" stroke="#21262d"/><g id="ar"><path d="M0-28L9-6H3V24H-3V-6H-9Z" fill="#58a6ff"/></g></svg><div id="axtxt" style="font-size:12px;color:var(--mut)"></div><select id="axsel" onchange="axChanged()"><option value="yw">Yaw (Z)</option><option value="rl">Roll (X)</option><option value="pt">Pitch (Y)</option></select><label style="display:block;margin:2px 0"><input type="checkbox" id="axinv" onchange="axChanged()" style="width:auto;display:inline"> flip direction</label></div>
<div style="flex:1"><canvas id="cv" width="600" height="120"></canvas></div></div>

<h2>IMU live <span id="lat" style="text-transform:none;letter-spacing:0"></span></h2>
<div class="card"><table id="imu"></table></div>

<h2>Sensor health &amp; accuracy (measured while still on the pad)</h2>
<div class="card"><table id="acc"></table></div>

<h2>Peaks</h2>
<div class="card"><table id="peaks"></table></div>

<h2>Eject detectors (any ONE fires)</h2>
<div class="card"><table id="det"></table></div>

<h2>Ground tests &amp; controls</h2>
<div class="card" id="ctl">
<button id="barm" class="pri" onclick="cmd('arm')">ARM</button><button id="bdis" onclick="cmd('disarm')">DISARM</button>
<button id="bzero" onclick="cmd('zero')">Re-zero baro</button><button id="bgyro" onclick="cmd('gyrocal')">Re-cal gyro</button>
<button id="btz" onclick="cmd('tiltzero')">Zero angles</button><button onclick="cmd('peaks')">Clear peaks</button><button id="brst" onclick="if(confirm('Reset for next flight?'))cmd('reset')">Reset for next flight</button>
<hr style="border:0;border-top:1px solid var(--line)">
<button id="bclick" onclick="cmd('click')">Relay click test</button>
<button id="btest" class="danger" onclick="testFire()">TEST EJECTION (3 s countdown)</button>
<button id="bcan" class="pri" style="display:none" onclick="cmd('cancel')">CANCEL</button>
<div id="tcount" style="font-weight:700;font-size:18px;color:var(--bad)"></div>
<div style="color:var(--mut);font-size:12px">Test uses the exact flight code path &amp; pulse length. <b>Disconnect the igniter / charge first.</b> Only works on the pad (not in flight).</div>
<hr style="border:0;border-top:1px solid var(--line)">
<b>Blind timer</b> <span style="color:var(--mut);font-size:12px">(only for the no-working-sensor case: starts the flight clock after N s)</span><br>
<input id="blsec" type="number" value="120" min="5" max="3600"> s
<button id="bbl" onclick="cmd('blind',{a:$('blsec').value})">Start</button><button onclick="cmd('blindcancel')">Cancel</button> <span id="blrem"></span>
</div>

<h2>Settings (saved to flash, pad only)</h2>
<div class="card">
<label>Backup timer (s)<input id="s_ap" type="number" min="3" max="120"></label>
<label>Max timer (s)<input id="s_mx" type="number" min="5" max="180"></label>
<label>Tilt limit (°)<input id="s_td" type="number" min="60" max="170"></label>
<label>Launch accel (g)<input id="s_lg" type="number" step="0.1" min="1.5" max="12"></label>
<label>Relay pulse (ms)<input id="s_pm" type="number" min="300" max="5000" step="100"></label><br>
<button id="bsave" class="pri" onclick="saveSet()">Save settings</button>
<div style="color:var(--mut);font-size:12px">Set the <b>backup timer</b> a little AFTER your simulated apogee time, and max timer a few seconds later.</div>
</div>
<p style="color:var(--mut);font-size:11px;text-align:center">Radio switches OFF at liftoff and returns after ejection.</p>

<script>
const $=id=>document.getElementById(id);
const ST=["PAD · ARMED","PAD · DISARMED","LAUNCH CANDIDATE","IN FLIGHT","EJECTED"];
const SC=["#12301b","#3a2a0c","#3a2a0c","#17407a","#3a1416"];
const RS=["—","BARO APOGEE","TILT","FREE-FALL","BACKUP TIMER","MAX TIMER","IMU VELOCITY"];
const LS=["—","ACCEL","BARO","LAUNCH PIN","BLIND TIMER"];
const WN=[[1,"Barometer not responding",1],[2,"IMU not responding",1],[4,"Being moved — hold the rocket still",0],[8,"Barometer noise high",0],[16,"|accel| not ≈ 1 g (IMU sensing/calibration problem)",1],[32,"Gyro calibration failed — press Re-cal gyro while still",0],[64,"DISARMED — will NOT auto-eject",1],[128,"Barometer settling…",0],[256,"A launch candidate was rejected earlier (bump)",0],[512,"Flight record restored after a reset — press Reset for next flight",1],[1024,"Orientation reference lost",0]];
let hist=[],setLoaded=false,last=null;
function rate(v,g,o){return v<=g?'good':(v<=o?'ok':'bad')}
function chip(c,t){return '<span class="chip '+c+'">'+t+'</span>'}
function row(a,b,c){return '<tr><td>'+a+'</td><td>'+b+(c||'')+'</td></tr>'}
function f(v,d){return (isFinite(v)?v:0).toFixed(d)}
function t(v){return v<0?'–':f(v,2)+' s'}
async function cmd(c,a){a=a||{};a.c=c;try{await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(a)})}catch(e){}}
function testFire(){if(confirm('Igniter / charge DISCONNECTED?\nThe relay will fire after a 3 s countdown.'))cmd('test')}
function saveSet(){cmd('save',{a:$('s_ap').value,b:$('s_mx').value,t:$('s_td').value,d:Math.round($('s_lg').value*10),e:$('s_pm').value});setLoaded=false}
function draw(){const c=$('cv'),x=c.getContext('2d');x.clearRect(0,0,c.width,c.height);if(hist.length<2)return;
let mn=Math.min(...hist,0),mx=Math.max(...hist,2);x.strokeStyle='#58a6ff';x.lineWidth=2;x.beginPath();
hist.forEach((v,i)=>{const px=i/(150-1)*c.width,py=c.height-6-(v-mn)/(mx-mn)*(c.height-12);i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke();
x.fillStyle='#8b949e';x.font='16px sans-serif';x.fillText(f(mx,1)+' m',4,16)}
function render(j){last=j;
const b=$('banner');b.textContent=ST[j.st]+(j.st==4?'  ·  '+RS[j.rs]:'');b.style.background=SC[j.st];
let fatal=0,ws=[];WN.forEach(w=>{if(j.wn&w[0]){ws.push('<div style="color:'+(w[2]?'var(--bad)':'var(--wn)')+'">'+(w[2]?'✖ ':'⚠ ')+w[1]+'</div>');if(w[2])fatal=1}});
const g=$('go');if(j.st>=2){g.style.display='none'}else{g.style.display='block';
 if(fatal){g.className='nogo';g.textContent='NO-GO'}else if(ws.length){g.className='caut';g.textContent='GO (with cautions)'}else{g.className='go';g.textContent='GO — all checks good'}}
const w=$('warn');w.style.display=ws.length&&j.st<2?'block':'none';w.innerHTML=ws.join('');
$('pk').textContent=f(j.pk,1);
$('pr').textContent=f(j.pr,2);$('bt').textContent=f(j.bt,1);const A=[];
A.push(row('Barometer',j.bo?'OK':'FAILED',chip(j.bo?'good':'bad',j.bo?'OK':'FAIL')));
A.push(row('IMU (MPU6500)',j.io?'OK':'FAILED',chip(j.io?'good':'bad',j.io?'OK':'FAIL')));
A.push(row('Baro noise (σ of altitude)',f(j.an,0)+' cm',chip(rate(j.an,40,100),rate(j.an,40,100).toUpperCase())));
A.push(row('Baro pressure σ',f(j.pn,1)+' Pa'));
A.push(row('Accel noise (σ of |a|)',f(j.acn,1)+' mg',chip(rate(j.acn,20,50),rate(j.acn,20,50).toUpperCase())));
A.push(row('|accel| error vs 1 g (still)',f(Math.abs(j.mg-1)*1000,0)+' mg',j.sl>1?chip(rate(Math.abs(j.mg-1),.03,.08),rate(Math.abs(j.mg-1),.03,.08).toUpperCase()):''));
A.push(row('Gyro noise σ / residual bias',f(j.gn,2)+' / '+f(j.gb,2)+' °/s',chip(rate(Math.max(j.gn,j.gb),.4,1.2),rate(Math.max(j.gn,j.gb),.4,1.2).toUpperCase())));
A.push(row('Still for',f(j.sl,0)+' s'));
A.push(row('Sample rates (IMU / baro)',f(j.ihz,0)+' / '+f(j.bhz,0)+' Hz'));
A.push(row('I2C errors (IMU / baro)',j.ie+' / '+j.be));
A.push(row('Sensor recoveries / bus resets',j.rc+' / '+j.ir));
A.push(row('Pad reference pressure',f(j.pp,2)+' hPa'));
A.push(row('IMU temp',f(j.it,1)+' °C'));
$('acc').innerHTML=A.join('');
$('peaks').innerHTML=[row('Peak altitude',f(j.pk,1)+' m'),row('Peak climb rate',f(j.pcl,1)+' m/s'),row('Max acceleration',f(j.mx,2)+' g'),row('Max rotation rate',f(j.mgy,0)+' °/s'),row('Max tilt',f(j.mtl,0)+' °'),row('Time to apogee',t(j.at)),row('Launch source',LS[j.ls]),row('Ejection time / reason',(j.et<0?'–':f(j.et,2)+' s')+' / '+RS[j.rs]),row('Relay fire count',j.fc)].join('');
const fl=j.ft>=0,rem=(x)=>fl?Math.max(0,x-j.ft).toFixed(1)+' s left':'T+'+x+' s';
const alive=(ok,okTxt)=>ok?chip('good',okTxt||'ACTIVE'):chip('bad','UNAVAILABLE');
$('det').innerHTML=[
row('Launch: accel &gt; '+f(j.lg/10,1)+' g',j.io?'watching':'—',alive(j.io)),
row('Launch: barometer climb',j.bo?'watching':'—',alive(j.bo)),
row('Launch: pin',j.pe?(j.ps?'TRIPPED':'armed'):'not fitted',chip(j.pe?'good':'na',j.pe?'ACTIVE':'OFF')),
row('A · Barometer apogee',j.bo?'drop &gt; 3 m':'—',alive(j.bo)),
row('B · Tilt &gt; '+j.td+'°',j.io&&j.tv?'now '+f(j.tl,0)+'°':'—',alive(j.io&&j.tv)),
row('C · IMU speed &lt; −3 m/s',j.io&&j.tv?'now '+f(j.vz,1):'—',alive(j.io&&j.tv)),
row('C2 · Free-fall (&lt; 0.3 g)',j.io?'now '+f(j.mg,2)+' g':'—',alive(j.io)),
row('D · Backup timer',rem(j.ap),chip('good','ALWAYS')),
row('E · Max timer',rem(j.mxt),chip('good','ALWAYS'))].join('');
const pad=j.st<=1;
$('barm').disabled=!(j.st==1);$('bdis').disabled=!(j.st==0);$('bzero').disabled=!pad;$('bgyro').disabled=!pad;
$('brst').disabled=!(pad||j.st==4);$('bclick').disabled=!pad||j.tr>=0;$('btest').disabled=!pad||j.tr>=0;$('bsave').disabled=!pad;$('bbl').disabled=j.st!=0;
$('bcan').style.display=j.tr>=0?'inline-block':'none';$('tcount').textContent=j.tr>=0?('FIRING IN '+f(j.tr,1)+' s'):(j.ry?'RELAY ON':'');
$('blrem').textContent=j.bl>=0?('fires clock in '+f(j.bl,0)+' s'):'';
if(!setLoaded&&document.activeElement.tagName!='INPUT'){$('s_ap').value=j.ap;$('s_mx').value=j.mxt;$('s_td').value=j.td;$('s_lg').value=(j.lg/10).toFixed(1);$('s_pm').value=j.pm;setLoaded=true}}
const AXN={yw:'Yaw',rl:'Roll',pt:'Pitch'};
function axChanged(){try{localStorage.setItem('ax',$('axsel').value);localStorage.setItem('inv',$('axinv').checked?'1':'0')}catch(e){}}
(function(){try{const a=localStorage.getItem('ax');if(a)$('axsel').value=a;$('axinv').checked=localStorage.getItem('inv')=='1'}catch(e){}})();
function drawArrow(j){const k=$('axsel').value,inv=$('axinv').checked?-1:1;let v=(j[k]||0)*inv;
 $('ar').setAttribute('transform','rotate('+(-v)+')');   // SVG rotate(+) is clockwise, so negate: anticlockwise turns the arrow anticlockwise
 $('axtxt').textContent=AXN[k]+' '+(v>=0?'+':'')+v.toFixed(1)+'° '+(v>0.5?'(anticlockwise)':v<-0.5?'(clockwise)':'')}
function bar(v,r){const p=Math.max(-1,Math.min(1,v/r))*50;return '<span class="bar"><u></u><i style="left:'+(p<0?50+p:50)+'%;width:'+Math.abs(p)+'%"></i></span>'}
let fN=0,fT=performance.now(),fHz=0,lastHist=0;
function renderFast(j){
 $('alt').textContent=f(j.alt,1);$('cl').textContent=f(j.cl,1);$('vz').textContent=f(j.vz,1);$('mg').textContent=f(j.mg,2);$('tl').textContent=f(j.tl,0);
 drawArrow(j);
 const now=performance.now();if(now-lastHist>100){lastHist=now;hist.push(j.alt);if(hist.length>150)hist.shift();draw()}
 const R=(n,v,r,u,d)=>'<tr><td>'+n+'</td><td>'+bar(v,r)+f(v,d)+' '+u+'</td></tr>';
 $('imu').innerHTML=[R('Accel X',j.ax,2,'g',3),R('Accel Y',j.ay,2,'g',3),R('Accel Z',j.az,2,'g',3),
  R('Roll  (about X, anticlockwise +)',j.rl,180,'°',1),R('Pitch (about Y, anticlockwise +)',j.pt,90,'°',1),R('Yaw   (about Z, anticlockwise +)',j.yw,180,'°',1),
  R('Gyro X',j.gx,250,'°/s',1),R('Gyro Y',j.gy,250,'°/s',1),R('Gyro Z',j.gz,250,'°/s',1),
  '<tr><td>|accel| · tilt · IMU speed</td><td>'+f(j.mg,3)+' g · '+f(j.tl,1)+'° · '+f(j.vz,1)+' m/s</td></tr>'].join('');
 fN++;if(now-fT>=1000){fHz=fN*1000/(now-fT);fN=0;fT=now}
 $('lat').textContent=' · data age '+j.age+' ms · display '+f(fHz,0)+' Hz'+(j.io?'':' · IMU OFFLINE');
}
async function fastPoll(){
 if(document.hidden){setTimeout(fastPoll,400);return}
 try{const ac=new AbortController();const to=setTimeout(()=>ac.abort(),600);
  const r=await fetch('/fast',{cache:'no-store',signal:ac.signal});clearTimeout(to);renderFast(await r.json());setTimeout(fastPoll,0)}
 catch(e){$('lat').textContent=' · link lost';setTimeout(fastPoll,300)}}
async function poll(){try{const r=await fetch('/api',{cache:'no-store'});render(await r.json())}catch(e){$('banner').textContent='LINK LOST';$('banner').style.background='#3a1416'}setTimeout(poll,500)}
poll();fastPoll();
</script></body></html>)HTML";
