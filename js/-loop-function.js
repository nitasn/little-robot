///////////////////////////////////////////////////////////////
///     C A L L   " L O O P "   P E R I O D I C A L L Y     ///
///////////////////////////////////////////////////////////////

let msPerFrame = 1000 / FPS;

const btnGo = document.querySelector('.btn-go');
const btnPause = document.querySelector('.btn-pause');

btnGo.addEventListener('click', () => {
  _timeout_id = setTimeout(call_Loop_and_DrawEverything);
});

btnPause.addEventListener('click', () => {
  clearTimeout(_timeout_id);
});

let _timeout_id;

function call_Loop_and_DrawEverything() {
  const start = Date.now();

  loop();
  drawEverything();

  const timePassed = Date.now() - start;
  const remainingTime = msPerFrame - timePassed;
  
  if (remainingTime < 0) {
    console.warn(`loop() took too long (msPerFrame is ${msPerFrame}ms)`);
  }
  
  _timeout_id = setTimeout(call_Loop_and_DrawEverything, remainingTime);
}