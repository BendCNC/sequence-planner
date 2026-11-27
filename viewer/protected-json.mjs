/** Protected catalog files (.bct): tooling/finger geometry shipped to CAD PCs is not plain JSON.
 *  Format: "BCTE1\n" | nonce(16) | HMAC-SHA256 tag(32) | ciphertext. Plaintext is zlib(JSON).
 *  Keystream block i = SHA-256(key | nonce | uint32be(i)). Same scheme as tools/protect_catalog.py.
 *  This deters casual copying; the key ships with the planner, so it is not strong secrecy. */
import {readFile,access} from 'node:fs/promises';
import {createHash,createHmac,timingSafeEqual} from 'node:crypto';
import {inflateSync} from 'node:zlib';

export const PROTECTED_SUFFIX='.bct';
const MAGIC=Buffer.from('BCTE1\n','ascii');
const KEY=createHash('sha256').update('BendCNC tooling catalog v1 \u00b7 7f3c91e2a4d84b0c').digest();

export function isProtected(bytes){return bytes.length>=MAGIC.length+48&&bytes.subarray(0,MAGIC.length).equals(MAGIC);}

export function unprotect(bytes){
  if(!isProtected(bytes))throw new Error('Not a protected catalog');
  const nonce=bytes.subarray(6,22),tag=bytes.subarray(22,54),body=bytes.subarray(54);
  const expected=createHmac('sha256',KEY).update(nonce).update(body).digest();
  if(!timingSafeEqual(tag,expected))throw new Error('Protected catalog is corrupt or was modified');
  const out=Buffer.allocUnsafe(body.length),counter=Buffer.alloc(4);
  for(let i=0,block=0;i<body.length;i+=32,block++){
    counter.writeUInt32BE(block);
    const ks=createHash('sha256').update(KEY).update(nonce).update(counter).digest();
    for(let j=0;j<32&&i+j<body.length;j++)out[i+j]=body[i+j]^ks[j];
  }
  return JSON.parse(inflateSync(out).toString('utf8'));
}

/** Read plain JSON or a protected catalog; falls back to `<path>.bct` when `path` is missing. */
export async function readCatalogJson(path){
  let target=path;
  try{await access(target);}catch{target=path+PROTECTED_SUFFIX;}
  const bytes=await readFile(target);
  return isProtected(bytes)?unprotect(bytes):JSON.parse(bytes.toString('utf8'));
}
