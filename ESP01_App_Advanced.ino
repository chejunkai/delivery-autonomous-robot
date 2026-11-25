/*
 * =====================================================================
 *  ESP-01 (ESP8266) WiFi App - Delivery Robot ADVANCED
 * =====================================================================
 *  Upload THIS sketch to the ESP-01 (not to the Mega).
 *    Arduino IDE: File > Preferences > Additional boards URL:
 *      http://arduino.esp8266.com/stable/package_esp8266com_index.json
 *    Tools > Board > ESP8266 Boards > "Generic ESP8266 Module"
 *    Flash it with a USB ESP-01 programmer (GPIO0 to GND while powering up).
 *
 *  Use
 *    - The ESP-01 makes its own WiFi:  "AirostRobot"  password "robot1234"
 *    - Phone: connect to it and open  http://192.168.4.1
 *    - The page shows the live robot status (mode, step, colour 1/2/3 -> ball / station / base,
 *      junction count, line sensors L C R, obstacle, speed, volume, last message)
 *      and has the AUTO / MANUAL switch, the drive pad, arm + gripper buttons,
 *      colour override and speed / volume.
 *    - Buttons send "#<cmd>" to the Mega (Serial, 115200). The Mega sends status lines "S:...".
 * =====================================================================
 */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* AP_SSID = "CT_Honor";
const char* AP_PASS = "Ct240304";      // min 8 characters

ESP8266WebServer server(80);
String lastStatus = "";
unsigned long lastStatusMs = 0;
String rxLine;

const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Delivery Robot</title>
<style>
:root{--bg:#0f1115;--card:#1a1d24;--line:#2a2f3a;--tx:#e8eaf0;--mut:#8b93a5;--blue:#2d6cdf;--grn:#1f9d55;--org:#d97706;--red:#c53030;--pur:#7c3aed}
*{box-sizing:border-box}body{font-family:system-ui,sans-serif;background:var(--bg);color:var(--tx);margin:0;padding:12px;max-width:520px;margin:auto;user-select:none;-webkit-user-select:none}
h1{font-size:18px;margin:4px 0 10px;display:flex;justify-content:space-between;align-items:center}
#conn{font-size:12px;padding:3px 8px;border-radius:20px;background:#444}#conn.on{background:var(--grn)}#conn.off{background:var(--red)}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:10px;margin-bottom:10px}
.row{display:flex;gap:8px;align-items:center;justify-content:space-between}
.lbl{color:var(--mut);font-size:12px}.big{font-size:20px;font-weight:700}
#mode.AUTO{color:#4ade80}#mode.MANUAL{color:#fbbf24}
.mis{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:8px}
.mis div{border-radius:8px;padding:6px;text-align:center;background:#252a34;font-size:13px}
.mis b{display:block;font-size:15px;margin-top:2px}
.sens{display:flex;gap:6px}.dot{width:26px;height:26px;border-radius:6px;background:#e5e7eb;color:#111;font-size:12px;display:flex;align-items:center;justify-content:center;font-weight:700}
.dot.k{background:#111;color:#eee;border:1px solid #555}
#obs{padding:3px 8px;border-radius:6px;background:#333;font-size:12px}#obs.y{background:var(--red)}
#msg{font-size:14px;margin-top:8px;min-height:18px}
.g{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
button{font-size:16px;padding:14px 4px;border:0;border-radius:10px;background:var(--blue);color:#fff;touch-action:manipulation;font-weight:600}
button:active{filter:brightness(1.4)}.a{background:var(--grn)}.m{background:var(--org)}.x{background:var(--red)}.s{background:#4b5563}.k{background:var(--pur)}
.cr{background:#c53030}.cb{background:#2b6cb0}.cg{background:#2f855a}
h2{font-size:13px;color:var(--mut);margin:2px 0 8px;font-weight:600;text-transform:uppercase;letter-spacing:.05em}
.hint{font-size:12px;color:var(--mut);margin-top:6px}
#toast{position:fixed;left:50%;bottom:16px;transform:translateX(-50%);background:#333;padding:8px 14px;border-radius:8px;font-size:13px;opacity:0;transition:opacity .3s}
</style></head><body>
<h1>Delivery Robot <span id="conn">connecting...</span></h1>

<div class="card">
 <div class="row"><div><div class="lbl">Mode</div><div id="mode" class="big">-</div></div>
  <div style="text-align:right"><div class="lbl">Step</div><div id="ph" class="big">-</div></div></div>
 <div class="mis">
  <div>Colour 1<b id="c1">-</b><span id="t1" class="lbl">Ball -</span></div>
  <div>Colour 2<b id="c2">-</b><span id="t2" class="lbl">Station -</span></div>
  <div>Colour 3<b id="c3">-</b><span id="t3" class="lbl">Base -</span></div>
 </div>
 <div class="row" style="margin-top:10px">
  <div><div class="lbl">Line sensors</div><div class="sens"><div id="sL" class="dot">L</div><div id="sC" class="dot">C</div><div id="sR" class="dot">R</div></div></div>
  <div><div class="lbl">Junctions</div><div id="jn" class="big">0</div></div>
  <div><div class="lbl">Obstacle</div><span id="obs">clear</span></div>
  <div><div class="lbl">Spd / Vol</div><div id="sv">-</div></div>
 </div>
 <div id="msg">waiting for robot...</div>
</div>

<div class="card"><h2>Mode</h2>
 <div class="g"><button class="a" onclick="c('A')">AUTO</button><button class="m" onclick="c('M')">MANUAL</button><button class="x" onclick="c('X')">STOP</button></div>
 <div class="hint">AUTO at base 1 starts the mission (same as a double clap). AUTO later = continue from the current step.</div>
</div>

<div class="card"><h2>Drive (manual - hold)</h2>
 <div class="g">
  <span></span><button data-h="F">&#9650;</button><span></span>
  <button data-h="L">&#9664;</button><button class="s" onclick="c('S')">&#9632;</button><button data-h="R">&#9654;</button>
  <span></span><button data-h="B">&#9660;</button><span></span></div>
</div>

<div class="card"><h2>Arm + gripper (manual)</h2>
 <div class="g">
  <button data-h="u">Arm Up</button><button class="s" onclick="c('h')">Home</button><button data-h="d">Arm Down</button>
  <button onclick="c('o')">Grip Open</button><span></span><button onclick="c('c')">Grip Close</button>
  <button class="k" onclick="c('G')">GRAB</button><span></span><button class="k" onclick="c('P')">UNLOAD</button></div>
</div>

<div class="card"><h2>Colour override (if detection failed)</h2>
 <div class="g">
  <button class="cr" onclick="c('C1R')">1 Red</button><button class="cb" onclick="c('C1B')">1 Blue</button><button class="cg" onclick="c('C1G')">1 Green</button>
  <button class="cr" onclick="c('C2R')">2 Red</button><button class="cb" onclick="c('C2B')">2 Blue</button><button class="cg" onclick="c('C2G')">2 Green</button>
  <button class="cr" onclick="c('C3R')">3 Red</button><button class="cb" onclick="c('C3B')">3 Blue</button><span></span></div>
</div>

<div class="card"><h2>Settings</h2>
 <div class="g">
  <button class="s" onclick="c('-')">Speed -</button><button class="s" onclick="c('Z')">Restart</button><button class="s" onclick="c('+')">Speed +</button>
  <button class="s" onclick="c('v')">Vol -</button><button class="s" onclick="c('?')">Refresh</button><button class="s" onclick="c('V')">Vol +</button></div>
</div>
<div id="toast"></div>

<script>
const $=id=>document.getElementById(id);
function toast(t){$('toast').textContent=t;$('toast').style.opacity=1;clearTimeout(toast.h);toast.h=setTimeout(()=>$('toast').style.opacity=0,1500);}
function c(k){fetch('/c?k='+encodeURIComponent(k)).then(r=>{if(!r.ok)throw 0;}).catch(()=>toast('Not sent - check WiFi'));}
let t=null;
document.querySelectorAll('[data-h]').forEach(b=>{
 const k=b.dataset.h,dn=e=>{e.preventDefault();c(k);clearInterval(t);t=setInterval(()=>c(k),150);},
 up=e=>{e.preventDefault();clearInterval(t);t=null;if('FBLR'.includes(k))c('S');};
 b.addEventListener('touchstart',dn);b.addEventListener('touchend',up);
 b.addEventListener('mousedown',dn);b.addEventListener('mouseup',up);b.addEventListener('mouseleave',e=>{if(t)up(e);});
});
const NUM={RED:1,BLUE:2,GREEN:3};
function setConn(ok,txt){const e=$('conn');e.className=ok?'on':'off';e.textContent=txt;}
function show(x){
 // x = "age|mode|phase|c1|c2|c3|junctions|LCR|obstacle|speed|volume|message"
 const f=x.split('|');if(f.length<12){setConn(false,'no robot data');return;}
 const age=+f[0];if(age>3000)setConn(false,'robot silent');else setConn(true,'online');
 $('mode').textContent=f[1];$('mode').className='big '+f[1];$('ph').textContent=f[2];
 [3,4,5].forEach((i,n)=>{$('c'+(n+1)).textContent=f[i];});
 $('t1').textContent='Ball '+(NUM[f[3]]||'-');$('t2').textContent='Station '+(NUM[f[4]]||'-');
 $('t3').textContent='Base '+(f[5]=='RED'?1:f[5]=='BLUE'?2:'-');
 $('jn').textContent=f[6];
 ['sL','sC','sR'].forEach((id,i)=>$(id).className='dot'+(f[7][i]=='1'?' k':''));
 const o=f[8]=='1';$('obs').textContent=o?'BLOCKED':'clear';$('obs').className=o?'y':'';
 $('sv').textContent=f[9]+' / '+f[10];$('msg').textContent=f.slice(11).join('|');
}
function poll(){fetch('/s',{cache:'no-store'}).then(r=>r.text()).then(show).catch(()=>setConn(false,'offline - reconnecting')).finally(()=>setTimeout(poll,700));}
poll();
</script></body></html>)HTML";

void handleRoot()   { server.send_P(200, "text/html", PAGE); }
void handleStatus() {
  if (lastStatus.length() == 0) { server.send(200, "text/plain", "x"); return; }
  server.send(200, "text/plain", String(millis() - lastStatusMs) + "|" + lastStatus);
}
void handleCmd() {
  String k = server.arg("k");
  if (k.length() >= 1 && k.length() <= 3) { Serial.write('#'); Serial.print(k); }   // '#' = frame start
  server.send(200, "text/plain", "ok");
}

void setup() {
  Serial.begin(115200);               // must match ESP.begin(115200) on the Mega
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);      // IP = 192.168.4.1
  server.on("/", handleRoot);
  server.on("/s", handleStatus);
  server.on("/c", handleCmd);
  server.begin();
}

void loop() {
  server.handleClient();
  while (Serial.available()) {        // status lines from the Mega
    char ch = Serial.read();
    if (ch == '\n') {
      rxLine.trim();
      if (rxLine.startsWith("S:")) { lastStatus = rxLine.substring(2); lastStatusMs = millis(); }
      rxLine = "";
    } else if (rxLine.length() < 200) rxLine += ch;
  }
}
