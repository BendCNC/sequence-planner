/** Optional local runtime for Qt's standard replan. All other callers use the CLI. */
import {createHash} from 'node:crypto';
import {existsSync} from 'node:fs';
import {readFile,realpath,mkdtemp,rename,rm} from 'node:fs/promises';
import {request} from 'node:http';
import {dirname,basename,join,isAbsolute} from 'node:path';
import {fileURLToPath} from 'node:url';

export const runtimeDir='/data/bendcnc-qt-state/planner-runtime';
export async function socketPath(){
  const engine=await realpath(fileURLToPath(new URL('../',import.meta.url)));
  const revision=(await readFile(join(engine,'ENGINE_REVISION'),'utf8')).trim();
  if(!/^v6-sha256:[a-f0-9]{64}$/.test(revision))throw Error('Engine has no content stamp');
  const id=createHash('sha256').update(JSON.stringify([engine,revision,process.version])).digest('hex').slice(0,24);
  return join(runtimeDir,id+'.sock');
}
export function qtRequest(args){
  if(args.length!==6||args[0]!=='--model'||args[2]!=='--options'||args[4]!=='--output')return null;
  const [model,options,output]=[args[1],args[3],args[5]];
  if(![model,options,output].every(p=>typeof p==='string'&&isAbsolute(p)))return null;
  const dir=dirname(model);
  if(dirname(options)!==dir||dirname(output)!==dir||basename(dir)!=='unfold-plans'||basename(options)!=='options.json')return null;
  const match=/^([0-9a-f]{24})(?:\.spans)?\.model\.json$/.exec(basename(model));
  if(!match||!new RegExp('^'+match[1]+'(?:-[a-f0-9]{8})?\\.plan\\.json\\.tmp$').test(basename(output)))return null;
  return {model,options,output};
}
function submit(socketPath,body){
  return new Promise((resolve,reject)=>{
    const req=request({socketPath,path:'/plan',method:'POST',agent:false},res=>{
      clearTimeout(handshake);
      if(res.statusCode!==200){res.resume();resolve(null);return;}
      let pending='',code=null,summary='';
      res.setEncoding('utf8');
      res.on('data',data=>{
        pending+=data;
        let n;
        while((n=pending.indexOf('\n'))>=0){
          const line=pending.slice(0,n);pending=pending.slice(n+1);
          try{
            const msg=JSON.parse(line);
            if(typeof msg.stderr==='string')process.stderr.write(msg.stderr);
            if(typeof msg.stdout==='string')summary+=msg.stdout;
            if(Number.isInteger(msg.code))code=msg.code;
          }catch(error){req.destroy(error);}
        }
      });
      res.on('error',reject);
      res.on('end',()=>pending||![0,1,2,3].includes(code)?reject(Error('Incomplete planner reply')):resolve({code,summary}));
    });
    const handshake=setTimeout(()=>req.destroy(Error('Planner runtime unavailable')),1000);
    req.on('error',error=>{clearTimeout(handshake);reject(error);});
    req.end(JSON.stringify(body));
  });
}
export async function tryPressRuntime(args){
  const qt=qtRequest(args);
  // Explicit profiling, environment tuning and other Node runtimes retain their original behaviour.
  if(!qt||process.execArgv.length||process.env.NODE_OPTIONS||Object.keys(process.env).some(k=>k.startsWith('UNFOLD_')&&k!=='UNFOLD_NETWORKX_PATH')||!existsSync(runtimeDir))return null;
  let temp;
  try{
    const socket=await socketPath();
    if(!existsSync(socket))return null;
    temp=await mkdtemp(join(dirname(qt.output),'.runtime-'));
    const result=await submit(socket,{args,temp});
    if(!result)return null;
    const file=join(temp,'result.json');
    if([0,2,3].includes(result.code))await rename(file,qt.output);
    if(result.summary)process.stdout.write(result.summary.replaceAll(JSON.stringify(file),JSON.stringify(qt.output)));
    return result.code;
  }catch{return null;}
  finally{if(temp)await rm(temp,{recursive:true,force:true}).catch(()=>{});}
}

