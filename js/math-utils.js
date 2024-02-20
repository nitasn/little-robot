///////////////////////////////////////////////////////////////
///                   M A T H   U T I L S                   ///
///////////////////////////////////////////////////////////////

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

function pointPlusPolarVector([x, y], degrees, radius) {
  const toRadians = Math.PI / 180;
  return [
    x + radius * Math.cos(degrees * toRadians),
    y + radius * Math.sin(degrees * toRadians),
  ];
}