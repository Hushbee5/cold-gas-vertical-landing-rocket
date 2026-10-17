// Source-native SVG diagram, rasterised with the bundled sharp runtime.
const fs = require('fs');
const path = require('path');
const sharp = require('sharp');
const out = path.resolve(__dirname, '../figures/paper/F05_gnc_architecture');
const parts = [];
function box(x,y,w,h,lines,fill='#fff') {
  parts.push(`<rect x="${x}" y="${y}" width="${w}" height="${h}" rx="7" fill="${fill}" stroke="#263b47" stroke-width="2"/>`);
  lines.forEach((t,i)=>parts.push(`<text x="${x+w/2}" y="${y+h/2-(lines.length-1)*14+i*28}" text-anchor="middle" dominant-baseline="central">${t}</text>`));
}
function arrow(points) { parts.push(`<polyline points="${points}" fill="none" stroke="#263b47" stroke-width="2" marker-end="url(#a)"/>`); }
function label(x,y,text) {parts.push(`<text x="${x}" y="${y}" text-anchor="middle" font-size="18">${text}</text>`);}
parts.push('<rect width="1500" height="600" fill="white"/>');
// Interfaces, not a claim that the Simulink NDI runs on the flight computer.
parts.push('<rect x="300" y="155" width="650" height="220" rx="10" fill="none" stroke="#825326" stroke-width="2" stroke-dasharray="10 7"/>');
box(20,195,220,115,['Guidance','vertical reference']);
box(330,185,260,140,['Controller','Model: PI + NDI','HIL: PI + PD'],'#f3f6f8');
box(710,195,210,115,['Actuators','needle valve','2 TVC servos']);
box(1050,195,290,115,['6-DOF plant','Simscape Multibody'],'#f1f6f2');
arrow('240,252 330,252'); label(282,231,'v_ref');
arrow('590,252 710,252'); label(650,231,'T_cmd, δ');
arrow('920,252 1050,252'); label(985,231,'F, M');
label(625,354,'HIL uses the ESP32-S3 and physical actuator interfaces');
box(640,25,240,100,['Variable gas','mass model'],'#faf6ee');
box(1045,25,300,100,['ISA atmosphere','equivalent lateral gust'],'#faf6ee');
arrow('850,125 1120,195'); arrow('1195,125 1195,195');
box(1050,455,290,100,['Landing-gear','ground-contact model'],'#f1f6f2');
arrow('1275,310 1275,455'); arrow('1115,455 1115,310');
box(335,445,510,120,['Feedback','Field: onboard sensor fusion','HIL: simulated state; fusion bypassed'],'#f3f6f8');
arrow('1340,252 1450,252 1450,590 850,590 850,515 845,515');
label(1388,233,'state');
arrow('335,505 125,505 125,310'); label(210,484,'h, v_z, attitude');
const svg=`<svg xmlns="http://www.w3.org/2000/svg" width="1500" height="600" viewBox="0 0 1500 600"><defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#263b47"/></marker></defs><g font-family="Times New Roman, serif" font-size="24" fill="#182630">${parts.join('')}</g></svg>`;
fs.writeFileSync(out+'.svg',svg);
sharp(Buffer.from(svg)).resize(3000,1200).png().toFile(out+'.png').then(()=>console.log(out+'.png'));
