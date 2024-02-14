const canvas = document.querySelector("canvas");
const ctx = canvas.getContext("2d");


///////////////////////////////////////////////////////////////
///                  A R E A   C O N F I G                  ///
///////////////////////////////////////////////////////////////

const robot = {
  position: [10, 10],
  radius: 1,
  degrees: 0,
};

const obstacles = [
  { x: 25, y: 15, w: 8, h: 35 },
  { x: 65, y: 35, w: 8, h: 55 },
];

const target = {
  position: [90, 80],
  radius: 1,
};

/** degrees per clocktick */
const MAX_ANGULAR_SPEED = 2.718281828;

/** units per clocktick */
const MAX_FORWARD_SPEED = 0.2;

/** if an object is further than this, the sensor sees infinite distance */
const DISTANCE_SENSOR_MAX = 6;


///////////////////////////////////////////////////////////////
///                A R E N A   D R A W I N G                ///
///////////////////////////////////////////////////////////////

const arenaFrame = { x: 0, y: 0, w: 100, h: 100 };
const ouchSegments = [...obstacles, arenaFrame]
  .flatMap(({ x, y, w, h }) => [
    [x, y, x + w, y],
    [x, y, x, y + h],
    [x, y + h, x + w, y + h],
    [x + w, y, x + w, y + h],
  ]);


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
  const [robotEdge, visionEdge] = _frontDistanceVector();

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


///////////////////////////////////////////////////////////////
///                   M A T H   U T I L S                   ///
///////////////////////////////////////////////////////////////

function pointPlusPolarVector([x, y], degrees, radius) {
  const toRadians = Math.PI / 180;
  return [
    x + radius * Math.cos(degrees * toRadians),
    y + radius * Math.sin(degrees * toRadians),
  ];
}

function segmentsIntersection(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y) {
  // Calculate parts of the equations
  const dX1 = p2x - p1x, dY1 = p2y - p1y;
  const dX2 = p4x - p3x, dY2 = p4y - p3y;
  const determinant = dX1 * dY2 - dY1 * dX2;

  // If determinant is zero, lines are parallel and have no intersection
  if (determinant === 0) return null;

  const t = ((p3x - p1x) * dY2 + (p1y - p3y) * dX2) / determinant;
  const u = ((p1x - p3x) * dY1 + (p3y - p1y) * dX1) / -determinant;

  // If intersection is out of the bounds of any of the segments
  if (t < 0 || t > 1 || u < 0 || u > 1) {
    return null; // No intersection
  }

  // Calculate the intersection point
  return [p1x + t * dX1, p1y + t * dY1];
}

function sqrDistance([x1, y1], [x2, y2]) {
  const dx = x1 - x2;
  const dy = y1 - y2;
  return dx * dx + dy * dy;
}

function mod360(degrees) {
  return (degrees % 360 + 360) % 360;
}

function clamp(low, val, high) {
  return Math.max(low, Math.min(val, high));
}


///////////////////////////////////////////////////////////////
///         R O B O T   C O R E   F U N C T I O N S         ///
///////////////////////////////////////////////////////////////

function drive(forwardPower, angularPower) {
  robot.degrees += angularPower / 100 * MAX_ANGULAR_SPEED;
  const unitsForward = forwardPower / 100 * MAX_FORWARD_SPEED;
  robot.position = pointPlusPolarVector(robot.position, robot.degrees, unitsForward);
}

function _frontDistanceVector() {
  const robotEdge = pointPlusPolarVector(robot.position, robot.degrees, robot.radius);
  const visionEdge = pointPlusPolarVector(robotEdge, robot.degrees, DISTANCE_SENSOR_MAX);
  return [robotEdge, visionEdge];
}

function frontDistance() {
  const [robotEdge, visionEdge] = _frontDistanceVector();
  const visionTrajectory = [...robotEdge, ...visionEdge];

  let minSqrDist = Number.POSITIVE_INFINITY;

  for (const ouchSegment of ouchSegments) {
    const intersection = segmentsIntersection(...visionTrajectory, ...ouchSegment);
    if (!intersection) continue;
    const sqrDist = sqrDistance(robotEdge, intersection);
    minSqrDist = Math.min(sqrDist, minSqrDist);
  }

  return Math.sqrt(minSqrDist);
}


///////////////////////////////////////////////////////////////
///       A L G O R I T H M   H E L P E R   F U N C S       ///
///////////////////////////////////////////////////////////////

/** result_deg is in the range (-180, 180] */
function signed_delta_deg(alpha, beta) {
  const delta_deg = mod360(alpha - beta);
  return delta_deg > 180 ? delta_deg - 360 : delta_deg;
}

function turn_to_degrees(desired_deg) {
  const delta_deg = signed_delta_deg(desired_deg, robot.degrees);

  let rotation_power = Math.abs(delta_deg) / MAX_ANGULAR_SPEED * 100;
  rotation_power = clamp(1, rotation_power, 100);

  const right = (delta_deg > 0 || delta_deg < -180) ? +1 : -1;
  return drive(0, right * rotation_power);
}

function is_close_to_deg(desired_deg) {
  const THRESHOLD_DEG = 0.2;

  const delta_deg = mod360(desired_deg - robot.degrees);
  return delta_deg < THRESHOLD_DEG || delta_deg > (360 - THRESHOLD_DEG);
}

function degressToTarget() {
  const [rx, ry] = robot.position;
  const [tx, ty] = target.position;
  const radians = Math.atan2(ty - ry, tx - rx);
  return mod360(radians / Math.PI * 180);
}

function distanceToTarget() {
  const sqrDist = sqrDistance(robot.position, target.position);
  return Math.sqrt(sqrDist);
}

function currentDirectionIsGettingAwayFromTarget() {
  const delta_deg = signed_delta_deg(degressToTarget(), robot.degrees);
  return Math.abs(delta_deg) <= 90;
}


///////////////////////////////////////////////////////////////
///               M A I N   A L G O R I T H M               ///
///////////////////////////////////////////////////////////////

///

const iter = mainAlgorithm();

function loop() {
  iter.next();
}

///

function* mainAlgorithm() {
  while (true) {
    const whyStopped = yield* walkTowardsTarget();

    if (whyStopped == "REACHED TARGET") return; // algorithm terminates

    // if we're here, whyStopped == "SEES OBSTACLE"

    do {
      yield* parallelizeObstacleAndWalkUntilBoom();
    }
    while (!currentDirectionIsGettingAwayFromTarget());
  }
}

function* parallelizeObstacleAndWalkUntilBoom() {
  const degreesAlongObstacle = yield* measureDegreesParallelToObject();

  yield* rotateUntilInAngle(degreesAlongObstacle);

  yield* continueInThisDirectionUntilBoom();

  const delta_deg = signed_delta_deg(degressToTarget(), robot.degrees);
  const shouldDriveDirectlyToTarget = Math.abs(delta_deg) > 90;

  if (shouldDriveDirectlyToTarget) {
    return yield* rotateUntilInAngle(degressToTarget());;
  }

  yield* parallelizeObstacleAndWalkUntilBoom();
}

function* continueInThisDirectionUntilBoom() {
  while (frontDistance() > 5) {
    yield drive(100, 0);
  }
}

function* walkTowardsTarget() {
  while (true) {
    if (distanceToTarget() < 0.1) {
      return "REACHED TARGET";
    }
    if (frontDistance() < 5) {
      return "SEES OBSTACLE";
    }
    yield* rotateUntilInAngle(degressToTarget());
    yield drive(100, 0);
  }
}

function* rotateUntilInAngle(degrees) {
  while (!is_close_to_deg(degrees)) {
    yield turn_to_degrees(degrees);
  }
}

function* measureDegreesParallelToObject() {
  const normalDirection = yield* measureDegreesToClosestObject();

  const optionA = mod360(normalDirection + 90); // circumvent obstacle from left
  const optionB = mod360(normalDirection - 90); // circumvent obstacle from right

  const angleTowardsTarget = degressToTarget();
  const costOptionA = Math.abs(signed_delta_deg(optionA, angleTowardsTarget));
  const costOptionB = Math.abs(signed_delta_deg(optionB, angleTowardsTarget));

  return (costOptionA < costOptionB) ? optionA : optionB;
}

function* measureDegreesToClosestObject() {
  const DEGREES_TO_MOVE_EACH_TURN = 2;

  let minDistance = Number.POSITIVE_INFINITY;
  let argminDegrees;

  for (degreesCovered = 0; degreesCovered < 360; degreesCovered += DEGREES_TO_MOVE_EACH_TURN) {
    const distance = frontDistance();
    if (distance < minDistance) {
      argminDegrees = robot.degrees;
      minDistance = distance;
    }
    const rotation_power = Math.round(DEGREES_TO_MOVE_EACH_TURN * 100 / MAX_ANGULAR_SPEED);
    yield drive(0, rotation_power);
  }

  return argminDegrees;
}


///////////////////////////////////////////////////////////////
///     C A L L   " L O O P "   P E R I O D I C A L L Y     ///
///////////////////////////////////////////////////////////////

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function callPeriodically({ msInterval }, func) {
  while (true) {
    const start = Date.now();
    func();
    const timePassed = Date.now() - start;
    const remainingTime = msInterval - timePassed;
    if (remainingTime > 0) {
      await sleep(remainingTime);
    }
    else if (remainingTime < 0) {
      console.warn("warning: frame drop");
    }
  }
}

document.querySelector('button').addEventListener('click', () => {
  callPeriodically({ msInterval: 5 }, () => {
    loop();
    drawEverything();
  });
});