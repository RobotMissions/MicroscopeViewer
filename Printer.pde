void goHome() {
  if(found_port) serialport.write("G1 X" + X_HOME + " Y" + Y_HOME + " Z" + Z_HOME + "\n");
  current_x_ind = 0;
  current_y_ind = 0;
  x_current = X_HOME;
  y_current = Y_HOME;
  z_current = Z_HOME;
}

void moveZ(float the_z) {
  if(found_port) serialport.write("G1 Z" + the_z + "\n");
  z_current = the_z;
}

void moveX(float the_x) {
  if(found_port) serialport.write("G1 X" + the_x + "\n");
  x_current = the_x;
}

void moveY(float the_y) {
  if(found_port) serialport.write("G1 Y" + the_y + "\n");
  y_current = the_y;
}

void moveXY(float the_x, float the_y) {
  if(found_port) serialport.write("G1 X" + the_x + " Y" + the_y + "\n");
  x_current = the_x;
  y_current = the_y;
}