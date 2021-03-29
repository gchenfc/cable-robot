bool painton[400];
uint8_t colorinds[400];
uint8_t colorpalette[2][3] = {{0,0,0}, {255,255,255}};
float traj[400][2];

#define TRAJ_SETUP
#define TRAJ_LOOP

void setup_traj() {
  for (int i = 0; i < 100; i++) {
    painton[i] = true;
    colorinds[i] = 0;
    traj[i][0] = 1 + (i/100.0);
    traj[i][1] = 1.15;
  }
  for (int i = 100; i < 200; i++) {
    painton[i] = false;
    colorinds[i] = 0;
    traj[i][0] = 2;
    traj[i][1] = 1.15;
  }
  for (int i = 200; i < 300; i++) {
    painton[i] = true;
    colorinds[i] = 1;
    traj[i][0] = 2 - ((i-200)/100.0);
    traj[i][1] = 1.15;
  }
  for (int i = 300; i < 400; i++) {
    painton[i] = false;
    colorinds[i] = 1;
    traj[i][0] = 1;
    traj[i][1] = 1.15;
  }
}
