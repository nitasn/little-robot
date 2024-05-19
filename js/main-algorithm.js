///////////////////////////////////////////////////////////////
///       A L G O R I T H M   H E L P E R   F U N C S       ///
///////////////////////////////////////////////////////////////

/** result is in the range (-180, 180] */
function signedDeltaDeg(alpha, beta) {
  const deltaDeg = mod360(alpha - beta);
  return deltaDeg > 180 ? deltaDeg - 360 : deltaDeg;
}

function rotateToward(desiredDeg) {
  const deltaDeg = signedDeltaDeg(desiredDeg, robot.degrees);
  const absPower = Math.abs(deltaDeg) / MAX_ANGULAR_SPEED * 100;
  const power = Math.sign(deltaDeg) * clamp(1, absPower, 100);
  return drive(0, power);
}

function isCloseToAngle(desiredDeg) {
  const THRESHOLD_DEG = 0.2;

  const deltaDeg = mod360(desiredDeg - robot.degrees);
  return deltaDeg < THRESHOLD_DEG || deltaDeg > (360 - THRESHOLD_DEG);
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

function isLookingAwayFromTarget() {
  const deltaDeg = signedDeltaDeg(degressToTarget(), robot.degrees);
  return Math.abs(deltaDeg) > 90;
}


///////////////////////////////////////////////////////////////
///               M A I N   A L G O R I T H M               ///
///////////////////////////////////////////////////////////////

///

const iter = mainAlgorithm();

function loop() {
  // read from sensors, then:
  iter.next();
}

///

function* mainAlgorithm() {
  while (true) {
    const whyStopped = yield* walkTowardsTarget();

    if (whyStopped == "REACHED TARGET") return; // algorithm terminates

    // we stopeed because there's an obstacle in front of us

    do {
      const degreesAlongObstacle = yield* measureDegreesParallelToObject();
      yield* rotateUntilInAngle(degreesAlongObstacle);
      yield* continueInThisDirectionUntilBoom();
    }
    while (!isLookingAwayFromTarget());

    // we need to rotate away from the wall before starting to walk towards target,
    // otherwise `walkTowardsTarget` would halt immediately as it sees a wall
    yield* rotateUntilInAngle(degressToTarget());
  }
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

    // fix direction if deviated a little bit
    yield* rotateUntilInAngle(degressToTarget());

    yield drive(100, 0);
  }
}

function* rotateUntilInAngle(degrees) {
  while (!isCloseToAngle(degrees)) {
    yield rotateToward(degrees);
  }
}

function* measureDegreesParallelToObject() {
  const normalDirection = yield* measureDegreesToClosestObject();

  const optionA = mod360(normalDirection + 90); // circumvent obstacle from left
  const optionB = mod360(normalDirection - 90); // circumvent obstacle from right

  const angleTowardsTarget = degressToTarget();
  const costOptionA = Math.abs(signedDeltaDeg(optionA, angleTowardsTarget));
  const costOptionB = Math.abs(signedDeltaDeg(optionB, angleTowardsTarget));

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