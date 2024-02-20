///////////////////////////////////////////////////////////////
///         R O B O T   C O R E   F U N C T I O N S         ///
///////////////////////////////////////////////////////////////

const arenaFrame = { x: 0, y: 0, w: 100, h: 100 };
const ouchSegments = [...obstacles, arenaFrame]
  .flatMap(({ x, y, w, h }) => [
    [x, y, x + w, y],
    [x, y, x, y + h],
    [x, y + h, x + w, y + h],
    [x + w, y, x + w, y + h],
  ]);

function drive(forwardPower, angularPower) {
  robot.degrees += clamp(-100, angularPower, +100) / 100 * MAX_ANGULAR_SPEED;
  const unitsForward = clamp(-100, forwardPower, +100) / 100 * MAX_FORWARD_SPEED;
  robot.position = pointPlusPolarVector(robot.position, robot.degrees, unitsForward);
}

function _vectorFrontDistanceSensor() {
  const robotEdge = pointPlusPolarVector(robot.position, robot.degrees, robot.radius);
  const visionEdge = pointPlusPolarVector(robotEdge, robot.degrees, DISTANCE_SENSOR_MAX);
  return [robotEdge, visionEdge];
}

function frontDistance() {
  const [robotEdge, visionEdge] = _vectorFrontDistanceSensor();
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