/** File-based adapter for desktop applications; geometry only, no machine commands. */
const args=process.argv.slice(2);
let status=null;
if(process.platform==='linux' && process.arch==='arm64'){
  const {tryPressRuntime}=await import('./press-runtime-client.mjs');
  status=await tryPressRuntime(args);
}
if(status===null){
  const {runPlanFile}=await import('./plan-file.mjs');
  status=await runPlanFile(args);
}
process.exitCode=status;
