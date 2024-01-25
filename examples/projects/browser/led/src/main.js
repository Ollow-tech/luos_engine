const program = require('../.pio/build/browser/program');
program
  .then(({ Luos_Init, Led_Init, Luos_Loop, Led_Loop, Ws_Init, Ws_Loop }) => {
    Luos_Init();
    Ws_Init();
    Led_Init();

    const mainLoop = () => {
      Luos_Loop();
      Ws_Loop();
      Led_Loop();
    };

    return new Promise(() => setInterval(mainLoop, 0));
  })
  .catch((err) => {
    console.error('Error', err);
    process.exit(-1);
  });
