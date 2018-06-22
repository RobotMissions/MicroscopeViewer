import processing.video.*;
import processing.serial.*;

// convolution based on example
// https://processing.org/examples/edgedetection.html

// other references:
// http://setosa.io/ev/image-kernels/
// https://forums.ni.com/t5/Example-Programs/Detect-Quality-of-Focus-on-a-Camera/ta-p/3501181


// -- CAM & CONVOLUTION -- //
int EDGE_THRESH = 200;

Capture cam;
PImage img;
PImage edgeImg;
int edge_count = 0;

float[][] kernel = {{ -1, -1, -1}, 
                    { -1,  8, -1}, 
                    { -1, -1, -1}};

//float[][] kernel = {{ 0, -1, 0}, 
//                    { -1,  0, 1}, 
//                    { 0, 1, -1}};


// -- PRINTRBOT -- //
Serial serialport;
int port = 0;
String theserial = "/dev/tty.usbmodem1411";
int BAUD = 115200;
boolean found_port = false;

// -- NAVIGATION -- //
float X_HOME = 0.0;
float Y_HOME = 0.0;
float Z_HOME = 9.0;

float x_current = X_HOME;
float y_current = Y_HOME;

float Z_STEP = 0.1;
float Z_MIN = 0.0;
float Z_MAX = 50.0;
float Z_NUMS = 500; // manually input (Z_MAX - Z_MIN) / Z_STEP
float z_distances[] = new float[500]; // Z_NUMS
float z_vals[] = new float[500]; // Z_NUMS

// -- MEASUREMENT -- //
float z_current = Z_HOME;
int z_ind = (int)( (z_current + Z_MIN) / Z_STEP );
boolean z_dir = true;
int z_range = 10;
int num_steps = 0;
int STEP_DELAY = 500;
int measure_count = 0;
boolean measure_active = false;
int step_resolution = 1;
long measurement_start = 0;
long measurement_end = 0;
int measurement_z_min = 0;
int measurement_z_max = 0;
boolean alreadysent = false;
boolean paused_measurement = false;

// -- STEPS -- //
// these steps are when the camera is at 40x magnification
float step_distance_x = 7.6;
float step_distance_y = 5.6;
int num_x_steps = 18;
int num_y_steps = 14;
float x_min = 4.8;
float x_max = 141.6;
float y_min = 146.3;
float y_max = 56.7;
float corner_a[] = { x_min, y_min }; // corner A = back left
float corner_b[] = { x_max, y_min }; // corner B = back right
float corner_c[] = { x_max, y_max }; // corner C = front right
float corner_d[] = { x_min, y_max }; // corner D = front left

// -- GRID -- //
int GRID_PIXEL = 20;
int GRID_WIDTH = 18;
int GRID_HEIGHT = 14;
int grid_complete[][] = new int[GRID_WIDTH][GRID_HEIGHT];

int total_width = (GRID_WIDTH-1)*(GRID_PIXEL);
int total_height = (GRID_HEIGHT-1)*(GRID_PIXEL);

int current_x_ind = 0;
int current_y_ind = 0;

long last_advance = 0;
boolean first_advance = true;
boolean analysing_grid = false;
boolean x_dir = true;
int tiles_completed = 0;
int total_tiles = GRID_WIDTH * GRID_HEIGHT;
boolean auto_mode = false;

// -- UI -- //
boolean shift_pressed = false;
String directory_name = "";


void setup() {
  size(1280, 800);
  img = loadImage("test.jpg"); 
  X_HOME = corner_a[0];
  Y_HOME = corner_a[1];
  x_current = X_HOME;
  y_current = Y_HOME;
  z_current = Z_HOME;

  // find our 3d printer
  for (int i=0; i<Serial.list().length; i++) {
    println(Serial.list()[i]);
    if (Serial.list()[i].equals(theserial)) {
      println("ding!");
      found_port = true;
      port = i;
      break;
    }
  }

  // open our 3d printer
  if(found_port) {
    println("found the printer and connecting");
    serialport = new Serial(this, Serial.list()[port], BAUD);
  } else {
    println("couldn't connect to 3d printer"); 
  }

  // find our camera
  String[] cameras = Capture.list();
  if (cameras.length == 0) {
    println("There are no cameras available for capture.");
    exit();
  } else {
    println("Available cameras:");
    for (int i = 0; i < cameras.length; i++) {
      print(i + " ");
      println(cameras[i]);
    }
    
    cam = new Capture(this, cameras[15]);
    //cam = new Capture(this, cameras[0]); // just for testing
    cam.start();     
  }
  
  // setup
  noStroke();
  for(int i=0; i<Z_NUMS; i++) {
    z_distances[i] = Z_MIN + (Z_STEP*i); 
    z_vals[i] = 0.0;
  }
  for(int i=0; i<GRID_WIDTH; i++) {
    for(int j=0; j<GRID_HEIGHT; j++) {
      grid_complete[i][j] = 0;
    }
  }
  
  // initialisation sequence of printer
  println("homing");
  serialport.write("G28\n");
  delay(2000);
  serialport.write("G90\n");
  // todo: at this point have it go to a loading position to load the tray
  println(millis() + "moving z");
  serialport.write("G1 F1500\n");
  serialport.write("G1 Z3.0\n"); // todo: this should be z_home
  delay(100);
  goHome();
  
}

void draw() {
  
  if(!cam.available()) return; // no point if we can't connect to the microscope

  // if we're measuring
  if(measure_active == true) {
    if(measure_count == 0 && num_steps == 0) {
      
      for(int i=0; i<Z_NUMS; i++) { // make sure to start w/ a clean slate
        z_vals[i] = 0;
      }
      
      measurement_start = millis();
      step_resolution = 10; // 0, 1
      
      // set the initial vars
      measurement_z_max = floor(10.0/Z_STEP);
      measurement_z_min = floor(0.0/Z_STEP);
      z_ind = measurement_z_min; // start in indexes
      z_range = measurement_z_max; // stop in indexes
      z_dir = true;
      
      println("going to start measurement at " + z_distances[z_ind] + "mm [" + z_ind + "]");
      println("going to measure to " + z_distances[z_range] + "mm [" + z_range + "]");
      println("with steps = " + step_resolution*Z_STEP + "mm");
      moveZ(z_distances[z_ind]);
      delay(STEP_DELAY*3);
      
    }
    if(measure_count == 2 && num_steps == 0) {
      step_resolution = 5;
      
      // set the initial vars
      float the_max_val = 0;
      int the_max_ind = 0;
      for(int i=measurement_z_min; i<measurement_z_max; i++) {
        z_vals[i] = z_vals[i] / 2;
        if(z_vals[i] >= the_max_val) {
          the_max_val = z_vals[i];
          the_max_ind = i;
        }
      }
      z_ind = the_max_ind-10;
      if(z_ind < measurement_z_min) z_ind = measurement_z_min;
      measurement_z_min = z_ind;
      z_range = the_max_ind+10;
      if(z_range > measurement_z_max) z_range = measurement_z_max;
      measurement_z_max = z_range;
      z_dir = true;
      
      for(int i=0; i<Z_NUMS; i++) { // make sure to start w/ a clean slate
        z_vals[i] = 0;
      }
      
      println("going to start measurement at " + z_distances[z_ind] + "mm [" + z_ind + "]");
      println("going to measure to " + z_distances[z_range] + "mm [" + z_range + "]");
      println("with steps = " + step_resolution*Z_STEP + "mm");
      moveZ(z_distances[z_ind]);
      delay(STEP_DELAY*3);
      
    }
    if(measure_count == 4 && num_steps == 0) {
      step_resolution = 1;
      
      // set the initial vars
      float the_max_val = 0;
      int the_max_ind = 0;
      for(int i=measurement_z_min; i<measurement_z_max; i++) {
        z_vals[i] = z_vals[i] / 2;
        if(z_vals[i] >= the_max_val) {
          the_max_val = z_vals[i];
          the_max_ind = i;
        }
      }
      z_ind = the_max_ind-3;
      if(z_ind < measurement_z_min) z_ind = measurement_z_min;
      measurement_z_min = z_ind;
      z_range = the_max_ind+3;
      if(z_range > measurement_z_max) z_range = measurement_z_max;
      measurement_z_max = z_range;
      z_dir = true;
      
      for(int i=0; i<Z_NUMS; i++) { // make sure to start w/ a clean slate
        z_vals[i] = 0;
      }
      
      println("going to start measurement at " + z_distances[z_ind] + "mm [" + z_ind + "]");
      println("going to measure to " + z_distances[z_range] + "mm [" + z_range + "]");
      println("with steps = " + step_resolution*Z_STEP + "mm");
      moveZ(z_distances[z_ind]);
      delay(STEP_DELAY*3);
      
    }
    measurementMode();
  }
  
  // if we're measuring still, then now go to the location in focus
  if(measure_count > 5 && measure_active == true) { // *2 because it has to go up & down
    
    int the_max_ind = 0;
    for(int i=0; i<Z_NUMS; i++) {
      z_vals[i] = z_vals[i] / 2;
      if(z_vals[i] > z_vals[the_max_ind]) {
       the_max_ind = i;
     }
    }
    // go to the location
    println(the_max_ind + " in focus at " + z_distances[the_max_ind] + "mm");
    moveZ(z_distances[the_max_ind]);
    delay(STEP_DELAY*3);
    
    measurement_end = millis();
    println("took " + (measurement_end-measurement_start) + "ms  to measure");
    
    for(int i=0; i<Z_NUMS; i++) {
      z_vals[i] = 0;
    }
    
    measure_count = 0;
    num_steps = 0;
    measure_active = false;
    tiles_completed++;
    
    if(auto_mode) {
      grid_complete[current_x_ind][current_y_ind] = 1;
      advance();
      cam.save(directory_name + "/tile_cam" + (tiles_completed-1) + ".jpg");
      edgeImg.save(directory_name + "/tile_conv" + (tiles_completed-1) + ".jpg");
    }
    
  }
  
  // TODO:
  // if there's a max on one of the extremes, then re-do the
  // measurement with z_ind starting on that extreme
  
  background(0);
  refreshConvolution();
  
  // draw the entire grid
  stroke(3);
  for(int i=0; i<GRID_WIDTH-1; i++) {
    for(int j=0; j<GRID_HEIGHT-1; j++) {
      if(grid_complete[i][j] == 0) {
        fill(20, 0, 220);
      } else if(grid_complete[i][j] == 1) {
        fill(0, 255, 0);
      } else if(grid_complete[i][j] == 99) {
        fill(255, 255, 255);
      }
      rect(i*(GRID_PIXEL)+300-(total_width/2), j*(GRID_PIXEL)+640-(total_height/2), GRID_PIXEL, GRID_PIXEL);
    }
  }
  noStroke();
  // and our cursour indicator
  //float cursour_x = map(x_current, x_min, x_max, 1*(GRID_PIXEL)+300-(total_width/2), (GRID_WIDTH-1)*(GRID_PIXEL)+300-(total_width/2));
  //float cursour_y = map(y_current, y_min, y_max, 1*(GRID_PIXEL)+640-(total_height/2), (GRID_HEIGHT-1)*(GRID_PIXEL)+640-(total_height/2));
  //fill(200, 0, 100);
  //ellipse(cursour_x-8, cursour_y-7, 20, 20);
  
  // analysing grid without measuring
  if(millis()-last_advance >= 1000) {
    if(analysing_grid == true && first_advance == true) {
      // process what's in frame now, before advancing to the next tile
      grid_complete[current_x_ind][current_y_ind] = 99;
      first_advance = false;
    }
    if(analysing_grid) {
      advance();
      last_advance = millis();
    }
  }
  
  // text
  String s;
  textSize(32);
  fill(255, 255, 255);
  int y_start_pos = 520;
  s = "(" + current_x_ind + ", " + current_y_ind + ")";
  text(s, 650, y_start_pos);
  s = "Edge count: " + edge_count;
  text(s, 650, y_start_pos+(1*40));
  s = "Tiles completed: " + tiles_completed + " / " + total_tiles;
  text(s, 650, y_start_pos+(2*40));
  s = "X: " + x_current + "   Y: " + y_current + "   Z: " + z_current;
  text(s, 650, y_start_pos+(3*40));
  
  textSize(18);
  fill(255, 255, 255);
  int y_start_pos2 = 680;
  int sp = 20;
  s = "enter: start / pause";
  text(s, 650, y_start_pos2+(0*sp));
  s = "r: restart";
  text(s, 650, y_start_pos2+(1*sp));
  s = "s: save img";
  text(s, 650, y_start_pos2+(2*sp));
  s = "a: analyse grid (start / stop)";
  text(s, 650, y_start_pos2+(3*sp));
  s = "m: measure (start / stop)";
  text(s, 650, y_start_pos2+(4*sp));
  
  y_start_pos2 = 680;
  s = "n: new folder dir";
  text(s, 920, y_start_pos2+(0*sp));
  s = "g: generate cmds";
  text(s, 920, y_start_pos2+(1*sp));
  s = "h: go home";
  text(s, 920, y_start_pos2+(2*sp));
  s = "shift + arrows, z: 0.1mm";
  text(s, 920, y_start_pos2+(3*sp));
  s = "arrows, z: tile movement";
  text(s, 920, y_start_pos2+(4*sp));
  
}

void advance() {
  if(x_dir) {
    current_x_ind++;
  } else {
    current_x_ind--;
  }

  if(current_x_ind >= GRID_WIDTH-1 || current_x_ind < 0) {
    if(current_x_ind < 0) current_x_ind = 0;
    if(current_x_ind >= GRID_WIDTH-1) current_x_ind = GRID_WIDTH-2; //hmm
     x_dir = !x_dir;
     current_y_ind++;
     if(current_y_ind >= GRID_HEIGHT-1) {
       // done!
       analysing_grid = false;
       // go back to home
       goHome();
       // stop measuring (just in case)
       measure_active = false;
       auto_mode = false;
       paused_measurement = false;
     }
  }
  moveXY( (current_x_ind * step_distance_x)+x_min, y_min-(current_y_ind * step_distance_y) );
  grid_complete[current_x_ind][current_y_ind] = 99;
  
  // now measure
  if(auto_mode) measure_active = true;
}

void measurementMode() {
 
  // the convolution refreshes right before this is called, so that's why it
  // moves at the end, (then refreshes), then will add the value to the array.
  z_vals[z_ind] += edge_count;
  println(z_ind + " edge count at " + z_distances[z_ind] + "mm = " + edge_count + " z_range = " + z_range + " z_inds: " + z_ind);

  num_steps++;
  if(num_steps > 1) {
    if(z_ind >= measurement_z_max || z_ind <= measurement_z_min) {
      z_dir = !z_dir;
      num_steps = 0;
      measure_count++;
      print("\n----- flip ----- (" + measure_count + ")\n");
      
      // counteracting the skipped middle step after a flip
      if(z_dir == true) {
        z_ind-= step_resolution;
      } else {
        z_ind+= step_resolution;
      }
      
    }
  }
  
  if(z_dir == true) {
    z_ind+= step_resolution;
  } else {
    z_ind-= step_resolution;
  }
  
  if(z_ind < 0) z_ind = 0;
  if(z_ind > Z_NUMS) z_ind = (int)Z_NUMS;
  
  moveZ(z_distances[z_ind]);
  delay(STEP_DELAY);
  
}

void keyPressed() {
 
  if(key == 'a') {
    if(!analysing_grid) {
      println("starting analysing mode");
      analysing_grid = true;
    } else {
      println("stopping analysing mode");
      analysing_grid = false;
    }
    auto_mode = false;
  }
  
  if(key == 's') {
    println("saving image");
    cam.save("outputimage.jpg");
    edgeImg.save("outputimage-conv.jpg");
  }
  
  if(key == 'r') {
    println("resetting");
    // TODO: reset the array, go to home location, reset the bools
  }
  
  if(key == 'm') {
    if(!measure_active) {
      println("starting measuring");
      grid_complete[current_x_ind][current_y_ind] = 99;
      measure_active = true;
    } else {
      println("pausing measuring");
      grid_complete[current_x_ind][current_y_ind] = 0;
      measure_active = false;
    }
    auto_mode = false;
  }
  
  if(key == 'h') {
    goHome();
    paused_measurement = false;
  }
  
  if(keyCode == ENTER || keyCode == RETURN) {
    if(!measure_active) {
      println("going to enter measurement mode!");
      if(!paused_measurement) newFolderDir();
      grid_complete[current_x_ind][current_y_ind] = 99;
      measure_active = true;
      auto_mode = true;
      paused_measurement = false;
    } else {
      println("pausing measurement mode!");
      paused_measurement = true;
      measure_active = false;
      auto_mode = false;
    }
  }
  
  if(keyCode == SHIFT) shift_pressed = true; 
  
  // TODO: protect against going off the sides
  if(shift_pressed) {
    if(keyCode == UP) {
      println("y current: " + y_current);
      moveY(y_current-0.1);
    } else if(keyCode == DOWN) {
      println("y current: " + y_current);
      moveY(y_current+0.1);
    } else if(keyCode == LEFT) {
      println("x current: " + x_current);
      moveX(x_current-0.1);
    } else if(keyCode == RIGHT) {
      println("x current: " + x_current);
      moveX(x_current+0.1);
    }
    if(key == 'z') {
      moveZ(z_current+0.1);
    }
  } else {
    if(keyCode == UP) {
      println("y current: " + y_current);
      moveY(y_current-step_distance_y);
    } else if(keyCode == DOWN) {
      println("y current: " + y_current);
      moveY(y_current+step_distance_y);
    } else if(keyCode == LEFT) {
      println("x current: " + x_current);
      moveX(x_current-step_distance_x);
    } else if(keyCode == RIGHT) {
      println("x current: " + x_current);
      moveX(x_current+step_distance_x);
    }
    if(key == 'z') {
      moveZ(z_current-0.1);
    }
  }
  
  if(key == 'n') {
    // new folder dir
    newFolderDir();
  }
  
  if(key == 'g') {
    // generate the mosaic commands for us
    println("--- Copy the commands below this line! ---\n");
    println("cd " + directory_name);
    
    print("montage ");
    //for(int i=0; i<total_tiles; i++) {
    //  print("tile_cam" + i + ".jpg");
    //  print(" ");
    //}
    // the tiles have to swap direction, since they are taken sequentially
    // but the montage would require it to "snake" around per-say
    // eg: 1 2 3
    //     6 5 4
    //     7 8 9
    int the_tile_num = 0;
    int last_swap = 0;
    boolean snake_around = true;
    for(int i=0; i<(GRID_HEIGHT-1); i++) {
      if(last_swap >= (GRID_WIDTH-1)-1) {
        snake_around = !snake_around; 
        last_swap = 0;
        //println("swap");
        
        if(snake_around == false) {
          the_tile_num += (GRID_WIDTH-1)-1;
        } else {
          the_tile_num += 2;
          the_tile_num += (GRID_WIDTH-1)-1;
        }
        
      }
      if(snake_around) {
        for(int j=0; j<(GRID_WIDTH-1); j++) {
          print("tile_cam" + the_tile_num + ".jpg");
          print(" ");
          the_tile_num++; 
          last_swap++;
        }
      } else {
        for(int j=(GRID_WIDTH-1); j>0; j--) {
          print("tile_cam" + the_tile_num + ".jpg");
          print(" ");
          the_tile_num--;
          last_swap++;
        }
      }
    }
    print("-tile " + (GRID_WIDTH-1) + "x" + (GRID_HEIGHT-1) + " ");
    print("-geometry 640x480+0+0 montage_cam" + directory_name + ".jpg");
    print("\n\n\n");
    
    print("montage ");
    the_tile_num = 0;
    last_swap = 0;
    snake_around = true;
    for(int i=0; i<(GRID_HEIGHT-1); i++) {
      if(last_swap >= (GRID_WIDTH-1)-1) {
        snake_around = !snake_around; 
        last_swap = 0;
        //println("swap");
        
        if(snake_around == false) {
          the_tile_num += (GRID_WIDTH-1)-1;
        } else {
          the_tile_num += 2;
          the_tile_num += (GRID_WIDTH-1)-1;
        }
        
      }
      if(snake_around) {
        for(int j=0; j<(GRID_WIDTH-1); j++) {
          print("tile_conv" + the_tile_num + ".jpg");
          print(" ");
          the_tile_num++; 
          last_swap++;
        }
      } else {
        for(int j=(GRID_WIDTH-1); j>0; j--) {
          print("tile_conv" + the_tile_num + ".jpg");
          print(" ");
          the_tile_num--;
          last_swap++;
        }
      }
    }
    //for(int i=0; i<total_tiles; i++) {
    //  print("tile_conv" + i + ".jpg");
    //  print(" ");
    //}
    print("-tile " + (GRID_WIDTH-1) + "x" + (GRID_HEIGHT-1) + " ");
    print("-geometry 640x480+0+0 montage_conv_" + directory_name + ".jpg");
  }
  
}

void keyReleased() {
  if(key == CODED) {
    if(keyCode == SHIFT) shift_pressed = false;
  }
}

void refreshConvolution() {
  
  cam.read();
  image(cam, 0, 0);
  img = cam;
  
  image(img, 0, 0); // Displays the image from point (0,0) 
  img.loadPixels();
  // Create an opaque image of the same size as the original
  edgeImg = createImage(img.width, img.height, RGB);
  // Loop through every pixel in the image.
  float sum = 0; // Kernel sum for this pixel
  edge_count = 0;
  for (int y = 1; y < img.height-1; y++) { // Skip top and bottom edges
    for (int x = 1; x < img.width-1; x++) { // Skip left and right edges
      sum = 0; // Kernel sum for this pixel
      for (int ky = -1; ky <= 1; ky++) {
        for (int kx = -1; kx <= 1; kx++) {
          // Calculate the adjacent pixel for this kernel point
          int pos = (y + ky)*img.width + (x + kx);
          // Image is grayscale, red/green/blue are identical
          float val = red(img.pixels[pos]);
          // Multiply adjacent pixels based on the kernel values
          sum += kernel[ky+1][kx+1] * val;
        }
      }
      // For this pixel in the new image, set the gray value
      // based on the sum from the kernel
      edgeImg.pixels[y*img.width + x] = color(sum, sum, sum);
      if(sum > EDGE_THRESH) {
        edge_count++;
        //println("Edge count: " + edge_count);
      }
    }
  }
  // State that there are changes to edgeImg.pixels[]
  edgeImg.updatePixels();
  image(edgeImg, width/2, 0); // Draw the new image
  
}

void newFolderDir() {
  String mdy = "";
  if(month() < 10) mdy += "0";
  mdy += month();
  mdy += "-";
  if(day() < 10) mdy += "0";
  mdy += day();
  mdy += "-";
  mdy += year();
  mdy += "_";
  if(hour() < 10) mdy += "0";
  mdy += hour();
  mdy += "-";
  if(minute() < 10) mdy += "0";
  mdy += minute();
  directory_name = mdy; 
}