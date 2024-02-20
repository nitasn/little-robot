///////////////////////////////////////////////////////////////
///                A R E N A   D R A W I N G                ///
///////////////////////////////////////////////////////////////

const canvas = document.querySelector("canvas");
const ctx = canvas.getContext("2d");


(function setCanvasResolution() {
  const dpr = window.devicePixelRatio;
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * dpr;
  canvas.height = rect.width * dpr; // aspect-ratio should be 1
  ctx.scale(canvas.width / 100, canvas.width / 100);
})();

function drawRobot() {
  ctx.lineWidth = 0.1;

  ctx.beginPath();
  ctx.fillStyle = '#777';
  ctx.arc(...robot.position, robot.radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  ctx.beginPath();
  ctx.fillStyle = '#000';
  const eyeRadius = robot.radius * 0.3;
  const eyePoint = pointPlusPolarVector(robot.position, robot.degrees, robot.radius - eyeRadius);
  ctx.arc(...eyePoint, eyeRadius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
}

function drawObstacles() {
  obstacles.forEach(({ x, y, w, h }) => {
    ctx.fillStyle = 'maroon';
    ctx.beginPath();
    ctx.fillRect(x, y, w, h)
  });
}

function drawTarget() {
  ctx.lineWidth = 0.1;
  ctx.fillStyle = '#aaa';
  ctx.beginPath();
  ctx.arc(...target.position, target.radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
}

function drawOuchSegments() {
  for (const [x1, y1, x2, y2] of ouchSegments) {
    ctx.beginPath();
    ctx.lineWidth = 0.25;
    ctx.lineCap = 'round'
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  }
}

function drawFrontDistance() {
  const [robotEdge, visionEdge] = _vectorFrontDistanceSensor();

  ctx.beginPath();
  ctx.lineWidth = 0.15;
  ctx.lineCap = 'round'
  ctx.moveTo(...robotEdge);
  ctx.lineTo(...visionEdge);
  ctx.stroke();
}

function drawEverything() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawObstacles();
  drawOuchSegments();
  drawTarget();
  drawRobot();

  drawFrontDistance();
}

drawEverything();
window.addEventListener("resize", drawEverything);