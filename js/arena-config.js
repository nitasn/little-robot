///////////////////////////////////////////////////////////////
///                  A R E A   C O N F I G                  ///
///////////////////////////////////////////////////////////////

const robot = {
  position: [10, 10],
  radius: 1.25,
  degrees: 0,
};

const obstacles = [
  { x: 25, y: 15, w: 8, h: 35 },
  { x: 65, y: 35, w: 8, h: 55 },
];

const target = {
  position: [90, 80],
  radius: 1.25,
};

/** degrees per clocktick */
const MAX_ANGULAR_SPEED = 2.718281828;

/** units per clocktick */
const MAX_FORWARD_SPEED = 0.2;

/** if an object is further than this, the sensor sees infinite distance */
const DISTANCE_SENSOR_MAX = 6;

/** frames per second */
const FPS = 150;
