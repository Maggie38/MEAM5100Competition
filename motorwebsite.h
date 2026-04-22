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
  <button id="strikeBtn" onclick="sendCmd('strike')"> Strike</button>
  <p id="status">Ready</p>
</body>
<script>
  function sendCmd(dir) {
    document.getElementById("status").innerText = dir.charAt(0).toUpperCase() + dir.slice(1);
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "/" + dir, true);
    xhttp.send();
  }
</script>
</html>
)===";