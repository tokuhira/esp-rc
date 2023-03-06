//#define ARDUINO_OTA
//#define CAPTIVE_PORTAL

// ESP8266 on RC buggy
  // I2C SDA PIN 4    // DRV8830 via I2C
  // I2C SCL PIN 5    // DRV8830 via I2C
#define SRV_GPIO 12   // PWM for stearing servo
#define STAT_LED 13   // LED of status on board
#define SGMT_DIO 14   // TM1637 for digits display
#define SGMT_CLK 15   // TM1637 for digits display
#define N_DIGITS 4    // TOF-3407HS-B: 4 digits anode common

#define PWM_BIT 8

const char* ssid="esp_rc_proto";
const char* pass="sazanka_";
// QR
// WIFI:S:<SSID>;T:<WEP|WPA|無記入>;P:<パスワード>;H:<true|false|無記入>;
const char html[] PROGMEM=R"rawliteral(
<!DOCTYPE html>
<html lang="en" dir="ltr">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>esp_rc</title>
  </head>
  <body>
    <style>
      :root{transition:.5s;--box:min(100vmin,480px);--cir:0.8;--dot:0.1;image-rendering:pixelated;}@media(prefers-color-scheme:dark){:root{background-color:#222;color:#fff;}}body{margin:0;}
      #stick{position:relative;width:calc(var(--box)*var(--cir));height:calc(var(--box)*var(--cir));box-shadow:0 0 0 calc(var(--box)*calc(calc(var(--cir)*var(--dot))/2)) #8882;border-radius:50%;background:#8884 0 0/100% url(data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAIAAAACCAYAAABytg0kAAAAGklEQVQI12NkYGDg/f//PwPT////GRgZGRkAObwGDol2alsAAAAASUVORK5CYII=);margin:calc(var(--box) * calc(calc(1 - var(--cir)) / 2)) auto;user-select:none;-webkit-user-select:none;}
      #stick>*{position:absolute;width:calc(100%*var(--dot));height:calc(100%*var(--dot));border-radius:50%;background-color:#888;pointer-events:none;top:50%;left:50%;transition:.05s;will-change:transform;}
    </style>
    <div id="stick"><div></div></div>
    <pre id="log">Connecting…</pre>
    <script>
      'use strict';
      let ws,ws_send=()=>console.log('uninitialized'),recieved={},timer,stick_stat;
      const ws_init=()=>{
        console.log('ws_init');
        ws=new WebSocket(`ws://${window.location.hostname}/ws`);
        ws.onopen=e=>{
          console.log(log.textContent='Opened :)');
        };
        ws.onclose=e=>{
          console.log(log.textContent='Closed :(');
          ws_send=()=>{};
          setTimeout(ws_init,2000);
        };
        ws.onmessage=e=>{
          if(ws.timer)return;ws.timer=!0;
          Object.assign(recieved,JSON.parse(e.data));
          log.textContent=JSON.stringify(recieved,null,'\t');
          if(recieved.purpose=='init')
            ws_send=lr=>ws.send(`V_CTR${lr.map(x=>String(Math.round((Math.sqrt(Math.abs(x))*Math.sign(x)+1)*recieved.max)).padStart(recieved.zpad,'0')).join('')}`);
          delete ws.timer;
        };
        ws.onerror=console.log;
      };

      stick.ontouchstart=e=>e.preventDefault();
      stick.onpointerdown=e=>{
        stick_stat=true;
        window.onpointermove(e);
      };
      (window.onpointerup=window.onpointerleave=window.onpointercancel=()=>{
        stick_stat=false;
        clearTimeout(timer);timer=0;ws_send([0,0]);
        stick.children[0].style.transform='translate(-50%,-50%)';
      })();
      window.onpointermove=e=>{
        if(!stick_stat||timer)return;
        const stick_style=stick.getBoundingClientRect();
        let pos=[
          (e.clientX-stick_style.left-stick_style.width*.5)||0,
          (e.clientY-stick_style.top-stick_style.height*.5)||0
        ],l=Math.sqrt(pos[0]*pos[0]+pos[1]*pos[1]);
        pos=pos.map(x=>x*Math.min(stick_style.width*.5,l)/l);
        stick.children[0].style.transform=`translate(calc(${pos[0]}px - 50%),calc(${pos[1]}px - 50%))`;
        //https://openrtm.org/openrtm/sites/default/files/6357/171108-04.pdf
        pos=[Math.atan2(...pos),Math.min(1,l/stick_style.width*2)];
        ws_send([Math.sin(pos[0])*pos[1],-Math.cos(pos[0])*pos[1]]);
      };

      window.onload=ws_init;
    </script>
  </body>
</html>
)rawliteral";
