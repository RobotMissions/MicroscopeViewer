void previousCode() {

  if(measure_active == true) {
  
    int z_range = 10;
    z_ind = (int)( (z_current + Z_MIN) / Z_STEP );
    z_ind += z_range; // center it between the min & max
    moveZ(z_distances[z_ind]);
    delay(STEP_DELAY*3);
    
    for(int i=0; i<z_range; i++) {
      z_ind++;
      moveZ(z_distances[z_ind]);
      delay(STEP_DELAY);
      refreshConvolution();
      redraw();
      delay(20);
      z_vals[z_ind] += edge_count;
      println(z_ind + " edge count at " + z_distances[z_ind] + "mm = " + edge_count);
    }
    
    for(int i=0; i<z_range; i++) {
      z_ind--;
      moveZ(z_distances[z_ind]);
      delay(STEP_DELAY);
      refreshConvolution();
      redraw();
      delay(20);
      z_vals[z_ind] += edge_count;
      println(z_ind + " edge count at " + z_distances[z_ind] + "mm = " + edge_count);
    }
    
    println("\n\n------------\n\n");
    measure_count++;
  
  }
  
  if(measure_count > 2) {
    measure_active = false;
  }
  
  
  
}