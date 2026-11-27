/** Single local geometry worker. Qt falls back to its normal child process
 * while this runtime starts or is busy. A cancelled client releases its worker. */
import {createServer} from 'node:http';
import {fork} from 'node:child_process';
import {readFile,lstat,mkdir,chmod,realpath,unlink,rm} from 'node:fs/promises';
import {basename,dirname,join} from 'node:path';
import {qtRequest,runtimeDir,socketPath} from './press-runtime-client.mjs';

process.umask(0o077);
await mkdir(runtimeDir,{recursive:true,mode:0o700});
const owned=await lstat(runtimeDir);
if(!owned.isDirectory()||owned.uid!==process.getuid()||(owned.mode&0o077))throw Error('Private runtime directory required');
const socket=await socketPath();
let worker=null,ready=false,job=null,closing=false;
async function start(){
  if(closing||worker)return;
  const available=Number(/^MemAvailable:\s+(\d+)/m.exec(await readFile('/proc/meminfo','utf8'))?.[1]??0);
  if(available<750*1024){setTimeout(start,15000).unref();return;}
  if(closing||worker)return;
  const child=fork(new URL('./press-runtime-worker.mjs',import.meta.url),[],{execArgv:['--expose-gc'],stdio:['ignore','ignore','inherit','ipc']});
  worker=child;
  child.on('message',msg=>{
    if(msg.ready){ready=true;console.log('Planner runtime ready');return;}
    if(!job)return;
    if(Number.isInteger(msg.code)){
      const res=job;job=null;ready=false;
      res.end(JSON.stringify({code:msg.code})+'\n');
    }else if(typeof msg.stdout==='string'||typeof msg.stderr==='string')job.write(JSON.stringify(msg)+'\n');
  });
  child.on('exit',()=>{
    worker=null;ready=false;
    if(job){const failed=job;job=null;failed.destroy();}
    if(!closing)setTimeout(start,1500);
  });
  child.on('error',error=>console.error(error.message));
}
async function accept(req,res){
  if(req.method!=='POST'||req.url!=='/plan'||!ready||job){res.writeHead(503).end();return;}
  let body='';
  for await(const chunk of req){
    if(body.length+chunk.length>8192){res.writeHead(400).end();return;}
    body+=chunk;
  }
  const {args,temp}=JSON.parse(body);
  const qt=Array.isArray(args)?qtRequest(args):null;
  if(!qt||typeof temp!=='string'||dirname(temp)!==dirname(qt.model)||!/^\.runtime-[a-zA-Z0-9]+$/.test(basename(temp))
     ||await realpath(temp)!==temp){res.writeHead(400).end();return;}
  if(!ready||job||res.destroyed){res.writeHead(503).end();return;}
  const files=await Promise.all([qt.model,qt.options,temp].map(p=>lstat(p)));
  if(files.some(f=>f.uid!==process.getuid())||!files[0].isFile()||!files[1].isFile()||!files[2].isDirectory()){res.writeHead(400).end();return;}
  if(!ready||job||res.destroyed){res.writeHead(503).end();return;}
  job=res;ready=false;
  res.on('close',()=>{
    if(job===res){job=null;worker?.kill('SIGKILL');}
    if(!res.writableEnded)rm(temp,{recursive:true,force:true}).catch(()=>{});
  });
  res.writeHead(200,{'content-type':'application/x-ndjson'});res.flushHeaders();
  console.log('Recomputing',basename(qt.model));
  worker.send({...qt,file:join(temp,'result.json')});
}
const server=createServer((req,res)=>{accept(req,res).catch(()=>{if(!res.headersSent)res.writeHead(400);res.end();});});
server.headersTimeout=5000;
server.requestTimeout=5000;
// The service launcher holds an exclusive flock across this process lifetime.
await unlink(socket).catch(e=>{if(e.code!=='ENOENT')throw e;});
await new Promise((resolve,reject)=>{server.once('error',reject);server.listen(socket,resolve);});
await chmod(socket,0o600);
start();
function stop(){
  closing=true;
  worker?.kill('SIGKILL');
  if(job){job.destroy();job=null;}
  server.close(()=>process.exit(0));
}
process.on('SIGTERM',stop);process.on('SIGINT',stop);
// The release launcher starts the replacement only after the current job ends.
setInterval(async ()=>{
  try{if(!job&&!closing&&await socketPath()!==socket)stop();}catch{if(!job&&!closing)stop();}
},15000).unref();

