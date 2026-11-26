// Executes the actual coordinator against deterministic controller/navigation doubles.
// This is software failure-path testing, not motor physics or scheduler validation.
const fs=require('fs'),path=require('path'),os=require('os'),cp=require('child_process'),assert=require('assert');
const root=process.cwd(),tmp=fs.mkdtempSync(path.join(os.tmpdir(),'2474v-pinnacle-'));
const header=fs.readFileSync('include/pinnacle_auton.hpp','utf8');
assert(header.includes('claw_toggle_overlap_verified = false'));
assert(fs.readFileSync('include/lift_calibration.hpp','utf8').includes('hold_validated = false'));
// Runtime test switches substitute ONLY the qualification constants in the doubles.
fs.writeFileSync(path.join(tmp,'pinnacle_auton.hpp'),header.replace('inline constexpr bool claw_toggle_overlap_verified','inline bool claw_toggle_overlap_verified'));
fs.mkdirSync(path.join(tmp,'EZ-Template'));
for(const f of ['subsystems.hpp','lift_calibration.hpp','nav_sequence.hpp','nav_diagnostics.hpp','preload_motion.hpp','EZ-Template/navigation_hooks.hpp'])
  fs.writeFileSync(path.join(tmp,f),'#include "mock.hpp"\n');
fs.copyFileSync('tests/pinnacle_auton_mock.hpp',path.join(tmp,'mock.hpp'));
const source=fs.readFileSync('src/subsystems.cpp','utf8');
// Exercise the real configurable pulse too, not an independently implemented timer.
const a=source.indexOf('bool pulse_toggle(unsigned pulse_ms)'),b=source.indexOf('bool toggle_run()',a);
assert(a>=0 && b>a);
fs.writeFileSync(path.join(tmp,'pulse.inc'),source.slice(a,b));
const test=fs.readFileSync('tests/pinnacle_auton_test.cpp','utf8').replace('ACTUAL_COORDINATOR',root.replaceAll('\\','/')+'/src/pinnacle_auton.cpp');
fs.writeFileSync(path.join(tmp,'test.cpp'),test);
const zig=process.env.ZIG || 'C:/Users/vadla/AppData/Local/Temp/2474v-phase1-tools/zig-windows-x86_64-0.13.0/zig.exe';
const exe=path.join(tmp,'test.exe');
cp.execFileSync(zig,['c++','-std=c++17','-Wall','-Wextra','-Werror','-I',tmp,path.join(tmp,'test.cpp'),'-o',exe],{stdio:'inherit'});
const output=cp.execFileSync(exe,[],{encoding:'utf8'});
console.log(output.split(/\r?\n/).filter(l=>l.startsWith('PASS:')).join('\n'));
console.log('PASS: actual Pinnacle helpers and pulse overload; production qualification defaults remain false.');
