class Robot{
  int lastCamber = 0;
  
  int last_osc_amp = 0;
  float last_osc_w = 0;
  float last_osc_phi = 0;
  
  int last_oscX = 0;
  int last_oscY = 0;
  int last_oscZ = 0;
  
  int last_oscXphi = 0;
  int last_oscYphi = 0;
 
  void walk_on(){
    sendCommand("#WL:1;");
  }
  void walk_off(){
    sendCommand("#WL:0;");
  }
  void telemetry_on(){
    sendCommand("#TL:1;");
  }
  void telemetry_off(){
    sendCommand("#TL:0;");
  }
  void hold_orient_on(){
    sendCommand("#HZ:1;");
  }
  void hold_orient_off(){
    sendCommand("#HZ:0;");
  }
  void trot_on(){
    sendCommand("#TR:1;");
  }
  void trot_off(){
    sendCommand("#TR:0;");
  }
  void roll_on(){
    sendCommand("#RO:10;");
  }
  void roll_off(){
    sendCommand("#RO:0;");
  }
  void pitch_on(){
    sendCommand("#PI:10;");
  }
  void pitch_off(){
    sendCommand("#PI:0;");
  }
  void reset(){
    sendCommand("#R:1;");
  }
  void sendOscilatorW(int osc_w){
    if (osc_w != last_osc_w){
      last_osc_w = osc_w;
      
      sendCommand("#OW:" + str(osc_w) + ";");
    }
  }
  void sendOscilatorX(int osc_x){
    if (osc_x != last_oscX){
      last_oscX = osc_x;
      
      sendCommand("#OX:" + str(osc_x) + ";");
    }
  }
  void sendOscilatorY(int osc_y){
    if (osc_y != last_oscY){
      last_oscY = osc_y;
      
      sendCommand("#OY:" + str(osc_y) + ";");
    }
  }
  void sendOscilatorZ(int osc_z){
    if (osc_z != last_oscZ){
      last_oscZ = osc_z;
      
      sendCommand("#OZ:" + str(osc_z) + ";");
    }
  }
  
  void sendCommand(String cmd_str){
    if (isConnected){
      println(cmd_str);
      tcpWrite(cmd_str);
    }
  }
}
