// 0: init o
// 1: op io
// 2: vel io
// 3: txt io

#include <WiFi.h>
#include <Wire.h>

#include <AsyncTCP.h>// https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h>// https://github.com/me-no-dev/ESPAsyncWebServer
// requires bug fix: https://github.com/me-no-dev/ESPAsyncWebServer/issues/1101
#include <ArduinoOTA.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SSD1306_NO_SPLASH

#include <MadgwickAHRS.h>
#include <Ticker.h>

#define SSID "esp_rc_omni_ex"
#define PASS "mcbeeringi"
#define PWM_FREQ 20000
#define PWM_BIT 10
#define M0PPIN 32
#define M0NPIN 33
#define M1PPIN 25
#define M1NPIN 26
#define M2PPIN 27
#define M2NPIN 14
#define STBY 13
#define I2CG 21
#define I2CV 19
#define I2CC 5
#define I2CD 16

#define M0PPWM 0
#define M0NPWM 1
#define M1PPWM 2
#define M1NPWM 3
#define M2PPWM 4
#define M2NPWM 5
#define PWM_MAX (1<<PWM_BIT)

const uint8_t PROGMEM icon[]={
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xde, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x30, 0x00, 0x38, 0x00, 0x00, 
	0x00, 0x00, 0x38, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x06, 0x00, 0x00, 
	0x00, 0x01, 0xc0, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x03, 0xb0, 0x00, 0x00, 0x01, 0xe0, 0x00, 
	0x00, 0x07, 0x60, 0x00, 0x00, 0x0c, 0x20, 0x00, 0x00, 0x0e, 0xc0, 0x00, 0x01, 0xf8, 0xb0, 0x00, 
	0x00, 0x1f, 0x80, 0x00, 0x00, 0x07, 0x38, 0x00, 0x00, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 
	0x00, 0x3c, 0x00, 0x00, 0x20, 0x00, 0x1e, 0x00, 0x00, 0x54, 0x00, 0x00, 0x40, 0x00, 0x0a, 0x00, 
	0x00, 0x70, 0x00, 0x31, 0x80, 0x00, 0x07, 0x00, 0x00, 0xa0, 0x01, 0xf3, 0x0e, 0x00, 0x0f, 0x00, 
	0x00, 0xa0, 0x06, 0x7c, 0x03, 0xfe, 0x07, 0x00, 0x01, 0xe0, 0x18, 0x00, 0x00, 0x00, 0x07, 0x80, 
	0x01, 0x40, 0x30, 0x00, 0x00, 0x03, 0x05, 0x80, 0x01, 0x40, 0x60, 0x00, 0x00, 0x01, 0x87, 0x80, 
	0x01, 0x40, 0xc0, 0x00, 0x00, 0x00, 0x43, 0x80, 0x01, 0x40, 0x80, 0x08, 0x00, 0x00, 0x61, 0x80, 
	0x01, 0x41, 0x00, 0x78, 0x07, 0xf0, 0x21, 0x80, 0x01, 0x41, 0x01, 0x80, 0x00, 0x18, 0x21, 0x80, 
	0x01, 0x41, 0x02, 0x00, 0x00, 0x00, 0x31, 0x80, 0x00, 0x41, 0x04, 0x10, 0x01, 0x00, 0x12, 0x80, 
	0x00, 0x41, 0x00, 0x10, 0x01, 0x00, 0x0b, 0x00, 0x00, 0x20, 0x00, 0x10, 0x01, 0x00, 0x03, 0x00, 
	0x00, 0x20, 0x00, 0x10, 0x01, 0x00, 0x06, 0x00, 0x00, 0x30, 0x00, 0x10, 0x01, 0x00, 0x0c, 0x00, 
	0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 
	0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x01, 0xc0, 0x00, 
	0x00, 0x00, 0x00, 0x30, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x01, 0xc0, 0x00, 
	0x00, 0x00, 0x00, 0x18, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const char html[] PROGMEM=R"rawliteral(
<!DOCTYPE html>
<html lang="en" dir="ltr">
	<head>
		<meta charset="utf-8">
		<meta name="viewport" content="width=device-width,initial-scale=1">
		<title>esp_rc_omni</title>
	</head>
	<body>
		<style>
			:root{--box:min(100vmin,480px);--cir:0.7;--dot:0.2;}
			@media(prefers-color-scheme:dark){:root,.ui{background-color:#222;color:#fff;}}
			.ui{z-index:1;position:relative;}
			#c{width:100%;height:100%;position:fixed;top:0;left:0;touch-action:none;user-select:none;-webkit-user-select:none;}
			#log{opacity:.5;pointer-events:none;white-space:pre-wrap;overflow-wrap:anywhere;}
		</style>
		<canvas id="c"></canvas>
		<select class="ui" onchange="this.value&&send(this.value.split(','),console.log(1))"><option value="" disabled selected>select command to run...</option><optgroup id="cmd"></optgroup></select>
		<input class="ui" placeholder="chat..." onchange="send([3,...te.encode(`<${w.id}> ${this.value}`)],this.value='')">
		<pre id="log"></pre>
		<script>
			'use strict';
			let ws={},timer;
			const
				w={},ptr={},
				ctx=c.getContext('2d'),
				te=new TextEncoder(),td=new TextDecoder(),
				strfy=(w,n=0)=>['Array','Object'].includes(w.constructor.name)?'\n'+Object.entries(w).map(([x,y])=>'\t'.repeat(n)+x+': '+strfy(y,n+1)).join(''):w+'\n',
				main=_=>(
					log.textContent+='Connecting…\n',
					ws=Object.assign(new WebSocket(`ws://${location.hostname}/ws`),{
						binaryType:'arraybuffer',
						onopen:_=>log.textContent+='Opened\n',
						onclose:_=>setTimeout(main,2000,log.textContent+='Closed\n'),
						onmessage:e=>(
							e.data.constructor.name=='String'?w.msg=e.data:
							(
								e=new Uint8Array(e.data),
								([
									_=>(w.id=e[1],w.max=2**e[2]),
									_=>flush(w.op=e[1],w.clis=[...e].slice(2)),
									_=>w.vel=[(e[1]<<8|e[2])-w.max,(e[3]<<8|e[4])-w.max,(e[5]<<8|e[6])-w.max],
									_=>w.chat=[...(w.chat||[]),td.decode(e.slice(1))].slice(-8),
								][e[0]]||(_=>w.msg=e))()
							),
							log.textContent=strfy(w)
						)
					})
				),
				send=w=>(console.log(...w),ws.readyState==1)&&ws.send(new Uint8Array(w).buffer),
				flush=_=>(cmd.innerHTML=w.clis.reduce((a,x)=>(x!=w.id&&w.id==w.op&&(a+=`<option value="1,${x}">/op @a[id=${x}]</option>`),a),''),cmd.previousElementSibling.selected=1),
				calc=bcr=>(core=>
					Object.values(ptr,ctx.clearRect(0,0,bcr.width,bcr.height)).map(x=>(
						ctx.strokeStyle=['#44f','#f44'][x.i],ctx.lineWidth=8,ctx.stroke(new Path2D(`M${x.o}L${[x.o[0]-x.op[1],x.o[1]+x.op[0], x.o[0]+x.op[0],x.o[1]+x.op[1], x.o[0]+x.op[1],x.o[1]-x.op[0], x.o[0]-x.op[0],x.o[1]-x.op[1]]}`)),
						ctx.strokeStyle=['#88f','#f88'][x.i],ctx.lineWidth=2,ctx.stroke(new Path2D(`M${[x.o[0]-x.max,x.o[1]]}L${[x.o[0]+x.max,x.o[1]]}M${[x.o[0],x.o[1]-x.max]}L${[x.o[0],x.o[1]+x.max]}`))
					)).length?
					(core(),clearTimeout(timer),timer=0):
					(timer||(core(),timer=setTimeout(_=>timer=0,50)))
				)(_=>send([2,...Object.values(ptr).reduce((a,x)=>(
					[
						{p:[0,1],n:[-1,0]},
						{p:[.866,-.5],n:[.5,.866]},
						{p:[-.866,-.5],n:[.5,-.866]}
					].map((y,i)=>[
						_=>(y.n[0]*x.opn[0]+y.n[1]*-x.opn[1])*x.opnl,
						_=>Math.hypot(...y.p)*-x.opn[0]*x.opnl
					][x.i]()+a[i])
				),[0,0,0]).flatMap(x=>(x=(x+1)*w.max,[x>>8,x&0xff]))]));

			c.onpointerdown=(
				e,bcr=c.getBoundingClientRect(),p=[(e.clientX-bcr.left)||0,(e.clientY-bcr.top)||0],i=+(p[0]>bcr.width*.5)
			)=>Object.values(ptr).every(x=>x.i!=i)&&(c.setPointerCapture(e.pointerId),ptr[e.pointerId]={o:p,i,max:128},c.onpointermove(e,bcr,p));
			c.onpointerup=c.onpointerleave=c.onpointercancel=(e,bcr=c.getBoundingClientRect())=>calc(bcr,delete ptr[e.pointerId]);
			c.onpointermove=(
				e,bcr=c.getBoundingClientRect(),p=[(e.clientX-bcr.left)||0,(e.clientY-bcr.top)||0],x=ptr[e.pointerId],
				op=x&&[p[0]-x.o[0],p[1]-x.o[1]],opl=op&&Math.hypot(...op),opn=opl?op.map(x=>x/opl):op
			)=>x&&calc(bcr,op=opn.map(x=>x*opl,opl=Math.min(opl,x.max)),ptr[e.pointerId]={...x,p,op,opn,opnl:opl/x.max});

			onload=main;
			(onresize=(_,bcr=c.getBoundingClientRect())=>(c.width=bcr.width*devicePixelRatio,c.height=bcr.height*devicePixelRatio,ctx.scale(devicePixelRatio,devicePixelRatio),ctx.lineCap=ctx.lineJoin='round'))();
		</script>
	</body>
</html>

)rawliteral";