// Run after pros make: node tests/verify_phase1_link.cjs <ARM bin directory> [baseline archive]
const fs=require('fs'),path=require('path'),cp=require('child_process'),crypto=require('crypto');
const bin=process.argv[2]; if(!bin)throw Error('Supply ARM toolchain bin directory');
function tool(name,args){return cp.execFileSync(path.join(bin,'arm-none-eabi-'+name+'.exe'),args,{maxBuffer:64*1024*1024});}
function check(v,m){if(!v)throw Error(m);}
const archive='firmware/EZ-Template.a',elf='bin/monolith.elf';
const temp=fs.mkdtempSync(path.join(require('os').tmpdir(),'phase1-link-'));
function extract(a,name){const dir=path.join(temp,name);fs.mkdirSync(dir);tool('ar',['x','--output='+dir,a]);return dir;}
const current=extract(archive,'current');
// Windows ar p translates LF bytes on stdout. Compare extracted binary files instead.

const members=tool('ar',['t',archive]).toString().trim().split(/\r?\n/);
const replacements=['drive.cpp.o','set_odom_pid.cpp.o','slew.cpp.o','pid_tasks.cpp.o','PID.cpp.o','exit_conditions.cpp.o'];
for(const member of replacements){
 check(members.filter(x=>x===member).length===1,'Duplicate/missing archive member '+member);
 const archived=fs.readFileSync(path.join(current,member)),built=fs.readFileSync('bin/ez_pid_patch/'+member);
 check(archived.equals(built),'Archive differs from built object '+member);
 console.log('PASS member '+member+' sha256='+crypto.createHash('sha256').update(archived).digest('hex'));
}
const anm=tool('nm',['-A','-C','--defined-only',archive]).toString();
const enm=tool('nm',['-C','--defined-only',elf]).toString();
const map=fs.readFileSync('bin/phase1-link.map','utf8');
const symbols=[['ez::Drive::private_drive_set(int, int)','drive.cpp.o'],['ez::Drive::raw_pid_odom_pp_set(std::vector<ez::odom, std::allocator<ez::odom> >, bool)','set_odom_pid.cpp.o'],['ez::Drive::ptp_task()','pid_tasks.cpp.o'],['ez::Drive::boomerang_task()','pid_tasks.cpp.o'],['ez::driver_control_ready()','pid_tasks.cpp.o'],['ez::driver_output_authorized()','pid_tasks.cpp.o'],['ez::slew::initialize(bool, double, double, double)','slew.cpp.o'],['ez::slew::iterate(double)','slew.cpp.o'],['ez::motion_output_fault(ez::output_safety::Fault)','pid_tasks.cpp.o']];
for(const [symbol,member] of symbols){
 const defs=anm.split(/\r?\n/).filter(x=>x.endsWith(' T '+symbol));
 check(defs.length===1&&defs[0].includes(':'+member+':'),'Archive symbol provenance '+symbol);
 const linked=enm.split(/\r?\n/).filter(x=>x.endsWith(' T '+symbol));
 check(linked.length===1,'ELF duplicate/missing '+symbol);
 check(map.split(/\r?\n/).some(x=>x.startsWith(symbol+' ')&&x.includes('EZ-Template.a('+member+')')),'Map provenance '+symbol);
 console.log('PASS ELF '+linked[0]+' <- '+member);
}
const dis=tool('objdump',['-d','-C',elf]).toString().replace(/\r/g,'');
for(const name of ['ez::Drive::ptp_task()','ez::Drive::boomerang_task()','ez::slew::iterate(double)','ez::Drive::private_drive_set(int, int)']) {
 const start=dis.indexOf('<'+name+'>:'); check(start>=0,'No disassembly '+name);
 const next=dis.indexOf('\n\n',start); const body=dis.slice(start,next);
 check(body.includes('ez::motion_output_fault'),'Linked guard absent '+name);
 console.log('PASS disassembly fault call '+name);
}
if(process.argv[3]) {
 const baseline=extract(process.argv[3],'baseline');
 const before=tool('ar',['t',process.argv[3]]).toString().trim().split(/\r?\n/);
 check(before.join('\n')===members.join('\n'),'Archive member set/order changed');
 let count=0;
 for(const member of members)if(!replacements.includes(member)){
  check(fs.readFileSync(path.join(current,member)).equals(fs.readFileSync(path.join(baseline,member))),'Unrelated member changed '+member);count++;
 }
 console.log('PASS '+count+' untouched archive members byte-identical');
}
const gateStart=dis.indexOf('<ez::Drive::private_drive_set(int, int)>:');
const gate=dis.slice(gateStart,dis.indexOf('\n\n',gateStart));
check(gate.includes('ez::driver_output_authorized()'),'Final motor boundary lacks lifecycle permission read');
const mainStart=dis.indexOf('<opcontrol>:'),cppMainStart=dis.indexOf('<opcontrol()>:');
const entry=mainStart>=0?mainStart:cppMainStart;check(entry>=0,'Missing opcontrol');
const control=dis.slice(entry,dis.indexOf('\n\n',entry));
const cleanup=control.indexOf('auton::lifecycle_stop'),ready=control.indexOf('ez::driver_control_ready');
check(cleanup>=0&&ready>cleanup,'Driver permission must follow cleanup in linked opcontrol');
console.log('PASS final boundary lifecycle read and opcontrol cleanup-before-permission ordering');
console.log('PASS native archive -> compiled member -> final ELF/map/disassembly verification');
