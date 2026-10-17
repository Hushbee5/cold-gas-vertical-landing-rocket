// Tethered-drop telemetry figure generated directly from the supplied CSV.
const fs = require('fs');
const path = require('path');
const sharp = require('sharp');

const csvPath = path.resolve(__dirname, '../data/flight/tethered_drop_2026-08-21.csv');
const outPath = path.resolve(__dirname, '../figures/paper/F_tethered_drop_telemetry.png');
const rows = fs.readFileSync(csvPath, 'utf8').trim().split(/\r?\n/).slice(1)
  .map(line => line.split(',').map(Number));

const tStart = 142.001;
const tEnd = 159.0;
const data = rows.filter(r => r[0] >= tStart && r[0] <= tEnd);
const W = 1600, H = 900;
const left = 118, right = 42;
const plotW = W - left - right;
const panels = [
  {top: 88, bottom: 408, min: 0, max: 10.5, ticks: [0,2,4,6,8,10], label: 'Altitude [m]'},
  {top: 520, bottom: 820, min: -20, max: 22, ticks: [-20,-10,0,10,20], label: 'Attitude angle [deg]'},
];
const x = t => left + (t - tStart) / (tEnd - tStart) * plotW;
const y = (v,p) => p.bottom - (v-p.min)/(p.max-p.min)*(p.bottom-p.top);
const esc = s => String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;');
const pathFor = (col,p) => data.map((r,i)=>`${i?'L':'M'}${x(r[0]).toFixed(1)},${y(r[col],p).toFixed(1)}`).join(' ');
let s=[];
s.push(`<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">`);
s.push(`<rect width="100%" height="100%" fill="white"/>`);
s.push(`<style>text{font-family:Arial,Helvetica,sans-serif;fill:#222}.tick{font-size:23px}.axis{font-size:27px}.title{font-size:30px;font-weight:700}.note{font-size:22px}.grid{stroke:#d9d9d9;stroke-width:1}.frame{stroke:#222;stroke-width:2;fill:none}</style>`);
s.push(`<text x="${W/2}" y="40" text-anchor="middle" class="title">Unpowered tethered-drop telemetry, 21 August 2026</text>`);
for (const p of panels) {
  for (const v of p.ticks) {
    const yy=y(v,p); s.push(`<line x1="${left}" y1="${yy}" x2="${W-right}" y2="${yy}" class="grid"/>`);
    s.push(`<text x="${left-18}" y="${yy+8}" text-anchor="end" class="tick">${v}</text>`);
  }
  s.push(`<rect x="${left}" y="${p.top}" width="${plotW}" height="${p.bottom-p.top}" class="frame"/>`);
  s.push(`<text transform="translate(35 ${(p.top+p.bottom)/2}) rotate(-90)" text-anchor="middle" class="axis">${esc(p.label)}</text>`);
}
for (let sec=0; sec<=16; sec+=2) {
  const xx=x(tStart+sec);
  for (const p of panels) s.push(`<line x1="${xx}" y1="${p.top}" x2="${xx}" y2="${p.bottom}" class="grid"/>`);
  s.push(`<text x="${xx}" y="852" text-anchor="middle" class="tick">${sec}</text>`);
}
s.push(`<text x="${W/2}" y="890" text-anchor="middle" class="axis">Time from start of plotted descent interval [s]</text>`);
s.push(`<path d="${pathFor(1,panels[0])}" fill="none" stroke="#1769aa" stroke-width="4"/>`);
s.push(`<path d="${pathFor(3,panels[1])}" fill="none" stroke="#d1495b" stroke-width="3"/>`);
s.push(`<path d="${pathFor(4,panels[1])}" fill="none" stroke="#2a9d55" stroke-width="3"/>`);
const peaks = [[145.117,3.39],[146.989,2.43],[149.258,1.99],[151.688,1.86],[154.113,1.78]];
for (const [tt,v] of peaks) {
  const xx=x(tt), yy=y(v,panels[0]);
  s.push(`<circle cx="${xx}" cy="${yy}" r="6" fill="#1769aa"/>`);
  s.push(`<text x="${xx}" y="${yy-13}" text-anchor="middle" class="note">${v.toFixed(2)} m</text>`);
}
s.push(`<line x1="${left+24}" y1="${panels[1].top+28}" x2="${left+82}" y2="${panels[1].top+28}" stroke="#d1495b" stroke-width="5"/><text x="${left+94}" y="${panels[1].top+36}" class="note">Roll (max |angle| 16.25°)</text>`);
s.push(`<line x1="${left+470}" y1="${panels[1].top+28}" x2="${left+528}" y2="${panels[1].top+28}" stroke="#2a9d55" stroke-width="5"/><text x="${left+540}" y="${panels[1].top+36}" class="note">Pitch (max |angle| 19.875°)</text>`);
s.push(`<text x="${W-right-8}" y="${panels[0].top+30}" text-anchor="end" class="note">No propulsion; descent arrested by elastic tether</text>`);
s.push('</svg>');

sharp(Buffer.from(s.join('')), {density: 180}).png().toFile(outPath)
  .then(()=>console.log(outPath))
  .catch(e=>{console.error(e);process.exit(1)});
