// npm install ws
// node vam_ws_udp_bridge.cjs
// Localhost-only WebSocket -> UDP binary pass-through (19 x 7 float32 LE).
const { WebSocketServer } = require('ws');
const dgram = require('node:dgram');
const WS_HOST = process.env.WS_HOST || '127.0.0.1';
const WS_PORT = Number(process.env.WS_PORT || 8765);
const UDP_HOST = process.env.UDP_HOST || '127.0.0.1';
const UDP_PORT = Number(process.env.UDP_PORT || 9998);
if(!Number.isInteger(WS_PORT)||WS_PORT<1||WS_PORT>65535||
   !Number.isInteger(UDP_PORT)||UDP_PORT<1||UDP_PORT>65535) throw Error('Invalid port');
const udp=dgram.createSocket('udp4');
const server=new WebSocketServer({host:WS_HOST,port:WS_PORT,perMessageDeflate:false,maxPayload:532});
let total=0,invalid=0;
server.on('connection',(ws,req)=>{
  console.log('[bridge] Connected:',req.socket.remoteAddress);
  ws.on('message',(bytes,isBinary)=>{
    if(!isBinary||bytes.length!==532){invalid++;return;}
    // Do not reinterpret or reorder floats. The existing VaM receiver expects <133f.
    udp.send(bytes,UDP_PORT,UDP_HOST,e=>{if(e)console.error('[bridge] UDP:',e.message);});
    total++;
  });
  ws.on('error',e=>console.error('[bridge] WS:',e.message));
});
server.on('error',e=>console.error('[bridge] Listen:',e.message));
const log=setInterval(()=>{console.log(`[bridge] sent=${total} invalid=${invalid} -> udp://${UDP_HOST}:${UDP_PORT}`);},10000);
console.log(`[bridge] ws://${WS_HOST}:${WS_PORT} -> udp://${UDP_HOST}:${UDP_PORT}`);
function shutdown(){clearInterval(log);server.close();udp.close();}
process.on('SIGINT',shutdown);process.on('SIGTERM',shutdown);
