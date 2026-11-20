'use strict';
// Offline COBS boundary/failure model, NOT a hardware transport qualification.
const assert=require('assert');
function encode(input) {
 const out=[0];let code=1,pos=0;
 for(const b of input) {
  if(b===0){out[pos]=code;pos=out.length;out.push(0);code=1;}
  else {out.push(b);if(++code===255){out[pos]=code;pos=out.length;out.push(0);code=1;}}
 }
 out[pos]=code;return Buffer.from(out);
}
function decode(input) {
 const out=[];let i=0;
 while(i<input.length) {
  const code=input[i++];assert(code!==0&&i+code-1<=input.length,'COBS decode error');
  for(let j=1;j<code;++j)out.push(input[i++]);
  if(code!==255&&i<input.length)out.push(0);
 }
 return Buffer.from(out);
}
for(const n of [0,1,192,250,254,255,512,1024,2047,4095]) {
 const payload=Buffer.concat([Buffer.from('sout'),Buffer.alloc(n,65)]);
 const frame=encode(payload);assert.deepStrictEqual(decode(frame),payload);
 assert(!frame.includes(0));
}
const frame=encode(Buffer.concat([Buffer.from('sout'),Buffer.alloc(192,65)]));
assert.throws(()=>decode(frame.subarray(0,frame.length-1)));
// Newlines and UTF-8 are payload bytes. A correct frame decodes either.
for(const text of ['BOOM_CSV,0,nan\n','Unicode \u00b0\n']) {
 const p=Buffer.from('sout'+text);assert.deepStrictEqual(decode(encode(p)),p);
}
console.log('PASS: COBS boundary model, ASCII/UTF-8 payloads, and dropped-byte failure; not actual serial transmission');
