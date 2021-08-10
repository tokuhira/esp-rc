#define I1 18
#define I2 19
#define I3 17
#define I4 16

#define PWM_FREQ 1045.0
#define PWM_BIT 8
#define PWM_MAX 256
//PWM_MAX=2^PWM_BIT
//2^16=65536
#define WS_ZERO_PAD 3
//shuuld be grater than length of PWM_MAX


const char* ssid="esp_rc_proto";
const char* pass="sazanka_";
// QR
// WIFI:S:<SSID>;T:<WEP|WPA|無記入>;P:<パスワード>;H:<true|false|無記入>;
const char html[] PROGMEM=R"rawliteral(
	<!DOCTYPE html>
	<html lang="en" dir="ltr">
		<head>
			<meta charset="utf-8">
			<title></title>
		</head>
		<body>
			<style>
				:root{transition:.5s;}@media(prefers-color-scheme:dark){:root{background-color:#222;color:#fff;}}body{margin:0;}
				#stick{position:relative;width:80vmin;height:80vmin;box-shadow:0 0 0 4vmin #8882;border-radius:50%;background:#8884;margin:10vmin auto;touch-action:pinch-zoom;user-select:none;-webkit-user-select:none;}
				#stick>*{position:absolute;width:10%;height:10%;border-radius:50%;background-color:#888;pointer-events:none;transform:translate(-50%,-50%);top:50%;left:50%;transition:.05s;will-change:transform;}
			</style>
			<div id="stick"><div></div></div>
			<pre id="log">Connecting…</pre>
			<script>
				'use strict';
				let ws,ws_send=()=>console.log('uninitialized'),recieved={},timer=0,stick_stat=false;
				const ws_init=()=>{
					console.log('ws_init');
					ws=new WebSocket(`ws://${window.location.hostname}/ws`);
					ws.onopen=e=>{
						log.textContent='Opened :)';console.log('opened');
					};
					ws.onclose=e=>{
						log.textContent='Closed :(';console.log('closed');
						ws_send=()=>{};
						setTimeout(ws_init,2000);
					};
					ws.onmessage=e=>{
						(async()=>{
							recieved={...recieved,...JSON.parse(e.data)};
							log.textContent=JSON.stringify(recieved,null,'\t');
							if(recieved.purpose=="init")
								ws_send=(lr)=>ws.send(`V_CTR${lr.map(x=>String(Math.round((x+1)*recieved.max)).padStart(recieved.zpad,'0')).join('')}`);
						})().catch(()=>{
							log.textContent+=`\n${e.data}`;
						});
					};
					ws.onerror=e=>{
						console.log(e);
					};
				};

				stick.onpointerdown=e=>{
					stick_stat=true;
					window.onpointermove(e);
				};
				window.onpointerup=window.onpointerleave=window.onpointercancel=()=>{
					stick_stat=false;
					clearTimeout(timer);timer=0;ws_send([0,0]);
					stick.children[0].style.transform='';
				};
				window.onpointermove=e=>{
					if(!stick_stat)return;
					const stick_style=stick.getBoundingClientRect();
					let pos=[
						(e.clientX-stick_style.left-stick_style.width*.5)||0,
						(e.clientY-stick_style.top-stick_style.height*.5)||0
					],l=Math.sqrt(pos[0]*pos[0]+pos[1]*pos[1]);
					pos=pos.map(x=>x*Math.min(stick_style.width*.5,l)/l);
					stick.children[0].style.transform=`translate(-50%,-50%)translate(${pos.join('px,')}px)`;
					if(timer)return;
					//https://openrtm.org/openrtm/sites/default/files/6357/171108-04.pdf
					pos=[Math.atan2(...pos)-Math.PI*.75,Math.min(1,l/stick_style.width*2)];
					ws_send([Math.cos(pos[0])*pos[1],Math.sin(pos[0])*pos[1]]);
					timer=setTimeout(()=>timer=0,100);
				};

				window.onload=ws_init;
			</script>
		</body>
	</html>
)rawliteral";
