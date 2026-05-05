const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; text-align: center; padding: 20px; }
    h2 { margin-bottom: 20px; }
    .grid {
      display: inline-grid;
      grid-template-columns: repeat(3, 80px);
      grid-template-rows: repeat(3, 80px);
      gap: 10px;
    }
    button {
      font-size: 28px;
      border: none;
      border-radius: 12px;
      background: #4CAF50;
      color: white;
      cursor: pointer;
      width: 80px;
      height: 80px;
    }
    button:active { background: #2e7d32; }
    .empty { visibility: hidden; }
    #strikeBtn {
      margin-top: 20px;
      width: 180px;
      height: 60px;
      font-size: 20px;
      background: #e53935;
      border-radius: 12px;
      border: none;
      color: white;
      cursor: pointer;
    }
    #strikeBtn:active { background: #b71c1c; }
    .wf-row {
      margin-top: 14px;
      display: flex;
      justify-content: center;
      gap: 14px;
    }
    .wfBtn {
      width: 160px;
      height: 60px;
      font-size: 16px;
      background: #1565C0;
      border-radius: 12px;
      border: none;
      color: white;
      cursor: pointer;
    }
    .wfBtn:active { background: #0d47a1; }
    #autoBtn {
      margin-top: 14px;
      width: 260px;
      height: 60px;
      font-size: 16px;
      background: #6a1b9a;
      border-radius: 12px;
      border: none;
      color: white;
      cursor: pointer;
    }
    #autoBtn:active { background: #4a148c; }
  </style>
</head>
<body>
  <h2>Robot Control</h2>
  <div class="grid">
    <div class="empty"></div>
    <button onclick="sendCmd('forward')">&#9650;</button>
    <div class="empty"></div>
    <button onclick="sendCmd('left')">&#9664;</button>
    <button onclick="sendCmd('stop')">&#9632;</button>
    <button onclick="sendCmd('right')">&#9654;</button>
    <div class="empty"></div>
    <button onclick="sendCmd('backward')">&#9660;</button>
    <div class="empty"></div>
  </div>
  <br>
  <button id="strikeBtn" onclick="sendCmd('strike')">Strike</button>
  <div class="wf-row">
    <button class="wfBtn" onclick="sendCmd('wallfollowRight')">Wall Right</button>
    <button class="wfBtn" onclick="sendCmd('wallfollowLeft')">Wall Left</button>
  </div>
  <div class="wf-row">
    <button id="autoBtn" onclick="sendCmd('autocircuit')">Auto Circuit</button>
  </div>
    <div class="wf-row">
    <input id="targetX" type="number" placeholder="Target X" style="width:120px; height:40px; font-size:16px;">
    <input id="targetY" type="number" placeholder="Target Y" style="width:120px; height:40px; font-size:16px;">
    <button id="gotoBtn" onclick="goToTarget()" 
      style="width:260px; height:60px; font-size:16px; background:#00897B; color:white; border:none; border-radius:12px;">
      Go to Vive Location
    </button>
  </div>
</body>

<div id="telemetry" style="margin-top: 20px; text-align: left; display: inline-block; font-family: monospace; background: #eee; padding: 10px; border-radius: 8px;">
  <strong>Status:</strong> <span id="status">Ready</span><br>
  <strong>Health:</strong> <span id="disp_health">0</span>%<br>
  <strong>Position:</strong> X:<span id="disp_x">0</span>, Y:<span id="disp_y">0</span>, &theta;:<span id="disp_t">0</span>&deg;<br>
  <strong>TOF (cm):</strong> L:<span id="disp_tof2">0</span>, C:<span id="disp_tof3">0</span>, R:<span id="disp_tof1">0</span><br>
  <strong>PWM:</strong> L:<span id="disp_pwmL">0</span>, R:<span id="disp_pwmR">0</span><br>
  <strong>Encoder:</strong> L:<span id="disp_encL">0</span>, R:<span id="disp_encR">0</span>
</div>

<script>
  function sendCmd(dir) {
    document.getElementById("status").innerText = dir.charAt(0).toUpperCase() + dir.slice(1);
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "/" + dir, true);
    xhttp.send();
  }

  function goToTarget() {
    let x = document.getElementById("targetX").value;
    let y = document.getElementById("targetY").value;

    document.getElementById("status").innerText = "Going to (" + x + ", " + y + ")";

    fetch(`/goto?x=${x}&y=${y}`);
  }

  function updateState() {
    fetch('/state').then(response => response.json()).then(data => {
      document.getElementById("disp_health").innerText = data.health;
      document.getElementById("disp_x").innerText      = data.x.toFixed(1);
      document.getElementById("disp_y").innerText      = data.y.toFixed(1);
      document.getElementById("disp_t").innerText      = data.theta.toFixed(1);
      document.getElementById("disp_tof1").innerText   = data.tof1;
      document.getElementById("disp_tof2").innerText   = data.tof2;
      document.getElementById("disp_tof3").innerText   = data.tof3;
      document.getElementById("disp_pwmL").innerText   = data.pwmL;
      document.getElementById("disp_pwmR").innerText   = data.pwmR;
      document.getElementById("disp_encL").innerText   = data.encL;
      document.getElementById("disp_encR").innerText   = data.encR;

      if(data.health <= 0) document.getElementById("status").innerText = "DEAD";
    });
  }

  // Poll every 200ms
  setInterval(updateState, 200);

</script>
</html>
)===";