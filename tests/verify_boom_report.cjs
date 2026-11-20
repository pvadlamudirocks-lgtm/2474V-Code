'use strict';
const fs=require('fs'),assert=require('assert');
function validate(text) {
  const lines=text.replace(/\r\n/g,'\n').split('\n');
  const begin=lines.findIndex(l=>l.startsWith('BOOM_TRACE_BEGIN '));
  const end=lines.findIndex((l,i)=>i>begin&&l.startsWith('BOOM_TRACE_END '));
  assert(begin>=0&&end>begin,'Missing BEGIN/END: incomplete capture');
  const block=lines.slice(begin,end+1);
  const header=block.find(l=>l.startsWith('BOOM_CSV,row_index,'));
  assert(header,'Missing CSV header');
  const columns=header.split(',');
  const rows=block.filter(l=>/^BOOM_CSV,\d+,/.test(l));
  const validation=block.find(l=>l.startsWith('BOOM_VALIDATION '));
  assert(validation,'Missing validation footer');
  const field=(line,k)=>{const m=line.match(new RegExp('\\b'+k+'=([^ ]+)'));assert(m,'Missing '+k);return m[1];};
  assert(field(lines[begin],'id')===field(lines[end],'id'),'Trial mismatch');
  assert(+field(validation,'recorded')===rows.length,'Recorded/received row mismatch');
  assert(+field(validation,'emitted')===rows.length,'Emitted/received row mismatch');
  assert(+field(lines[end],'emitted')===rows.length,'END count mismatch');
  assert(field(lines[end],'local_write_ok')==='1','Local write failed');
  let hash=2166136261,previous=-1;const phases=[0,0,0];
  rows.forEach((line,i)=>{
    const cells=line.split(',');assert(cells.length===columns.length,'Column mismatch row '+i);
    assert(+cells[1]===i,'Missing, duplicate, or out-of-order row '+i);
    const ms=+cells[columns.indexOf('ms')];assert(Number.isFinite(ms),'Invalid timestamp');assert(ms>=previous,'Backward timestamp');previous=ms;
    const phase=+cells[columns.indexOf('phase')];assert(phase>=0&&phase<=2,'Bad phase');phases[phase]++;
    for(const b of Buffer.from(line+'\n','ascii')){hash^=b;hash=Math.imul(hash,16777619)>>>0;}
  });
  assert(hash.toString(16).padStart(8,'0')===field(validation,'csv_fnv1a'),'CSV checksum mismatch');
  assert(phases.join(',')===field(validation,'pre_motion_post'),'Phase count mismatch');
  return {rows:rows.length,columns:columns.length,phases,integrity:'PASS',captureHealth:'Inspect truncated/dropped/invalid/missed-slot fields separately'};
}
function validateAll(text) {
 const starts=[...text.matchAll(/^BOOM_TRACE_BEGIN /gm)].map(m=>m.index);
 assert(starts.length,'No reports');
 return starts.map((start,i)=>{
   try{return {report:i,...validate(text.slice(start,starts[i+1]??text.length))};}
   catch(e){return {report:i,integrity:'FAIL',error:e.message};}
 });
}function selfTest(){
  const source=fs.readFileSync('src/boomerang_motion_trace.cpp','utf8');
  const strings=call=>[...call.matchAll(/"((?:\\.|[^"\\])*)"/g)].map(m=>JSON.parse('"'+m[1]+'"')).join('');
  const calls=[...source.matchAll(/report\.print\(([\s\S]*?)\);/g)].map(m=>strings(m[1]));
  const h=calls.find(s=>s.startsWith('BOOM_CSV,row_index'));
  const f=calls.find(s=>s.startsWith('BOOM_CSV,%'));
  const extra=calls.find(s=>s.startsWith(',signed_target_native'));
  const extraFmt=calls.find(s=>s.startsWith(',%.5f,%d,%d,'));
  const formats=s=>[...s.matchAll(/%(?:\.\d+)?[udf]/g)].length;
  assert(h&&f&&extra&&extraFmt,'Missing formatter fragments');
  const header=h+Array.from({length:2},(_,i)=>['port','installed','config_ok','valid','raw_cd','in','errno'].map(k=>',T'+i+'_'+k).join('')).join('')+
    Array.from({length:4},(_,i)=>['rpm','mV','mA'].map(k=>',M'+i+'_'+k).join('')).join('')+extra.trimEnd();
  assert(header.split(',').length===1+formats(f)+2*7+4*3+formats(extraFmt),'Source format/header arity mismatch');
  const cols=header.split(',');
  const make=(count)=>{
    let hash=2166136261;const phase=[0,0,0];
    const rows=Array.from({length:count},(_,i)=>{
      const a=cols.map((_,j)=>j%7===0?'nan':'12345.67890');a[0]='BOOM_CSV';a[1]=String(i);a[cols.indexOf('ms')]=String(i?30*(i-1):0);
      const p=i===0?0:i===count-1?2:1;phase[p]++;a[cols.indexOf('phase')]=String(p);
      const line=a.join(',');for(const b of Buffer.from(line+'\n')){hash^=b;hash=Math.imul(hash,16777619)>>>0;}return line;
    });
    return ['BOOM_TRACE_BEGIN id=10001',header,...rows,
      'BOOM_VALIDATION recorded='+count+' emitted='+count+' pre_motion_post='+phase+' csv_fnv1a='+hash.toString(16).padStart(8,'0'),
      'BOOM_TRACE_END id=10001 emitted='+count+' local_write_ok=1'].join('\n');
  };
  const good=make(450);assert(validate(good).rows===450);
  for(const bad of [good.replace(/^BOOM_CSV,0,.*\n/m,''),good.replace('BOOM_CSV,2,','BOOM_CSV,1,'),
    good.replace('BOOM_CSV,2,30,','BOOM_CSV,2,-1,'),good.replace('BOOM_TRACE_END','MISSING_END'),good.replace(/csv_fnv1a=\w+/,'csv_fnv1a=00000000')]) {
    assert.throws(()=>validate(bad));
  }
    assert(validate(make(3)).phases.join(',')==='1,1,1');
  const os=require('os'),path=require('path');
  const folder=fs.mkdtempSync(path.join(os.tmpdir(),'boom-report-'));
  const file=path.join(folder,'synthetic-71.txt');
  fs.writeFileSync(file,make(71));
  assert(validate(fs.readFileSync(file,'utf8')).rows===71);
  assert(validateAll(make(71)+'\n'+make(71)).every(r=>r.integrity==='PASS'));
  assert(validateAll('BOOM_TRACE_BEGIN incomplete\n'+make(71))[0].integrity==='FAIL');
  console.log('Generated/verified offline fixture: '+file);
  console.log('PASS: source CSV arity; 450-row and nan fixtures; missing/duplicate/corrupt/truncated report detection');
}
if(process.argv[2]==='--self-test')selfTest();
else {assert(process.argv[2],'Usage: node tests/verify_boom_report.cjs report.txt');const results=validateAll(fs.readFileSync(process.argv[2],'utf8'));console.log(results);if(results.some(r=>r.integrity!=='PASS'))process.exitCode=1;}
