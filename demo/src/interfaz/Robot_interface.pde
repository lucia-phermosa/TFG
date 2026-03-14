import net.java.games.input.*;
import processing.net.*;

//ControlIO control;
//Configuration config;
//ControlDevice gpad;

Robot robot;

Client client;
String socket_str;
boolean isConnected = false;
boolean newData = false;

PrintWriter telemetryFile;
int testNumber = 1;
boolean saveTelem = false;

int robotX, robotY, robotZ;
boolean newPose = false;

int velX, lastVelX = 0;
int velY, lastVelY = 0;

int height_amplitude = 20;
int height_increment = 0;

int pitch, lastPitch = 0;
int pitch_amplitude = 20;

int roll, lastRoll = 0;
int roll_amplitude = 20;

int yaw, lastYaw = 0;
int yaw_amplitude = 20;

MainScreen mainScreen;
CalibrationScreen calib_screen;

boolean servosON = false;
boolean walkON = false;
boolean trotON = false;
boolean telemON = false;

int footX, footY, footZ;
int last_footX=0, last_footY=0, last_footZ=0;
boolean newFootPos = false;

class Leg_Data{
  int[] servoAngle;
  Leg_Data(){
    servoAngle = new int [3];
  }
}
Leg_Data[] leg_data;
boolean newServosData = false;

//Graph data
float angX, angY, angZ;
float worldX, worldY, worldZ;
float floorX, floorY;
float current;

float cgX, cgY;
int camber = 0;

// --- NUEVO: define el tamaño inicial de la ventana en settings()
void settings() {
  size(int(displayWidth*0.65), int(displayHeight*0.75));
}

void setup(){
  surface.setResizable(true);
  surface.setTitle("Control 4LBOT");
  
  socket_str = new String();
  // NUEVO: arrancar servidor TCP
  try {
    client = new Client(this, "127.0.0.1", 9000);
    client.clear();
  } catch(Exception e) {
    println("No se pudo iniciar TCP client: " + e.getMessage());
  }
  
  if (client.active()){
    println("Connected to node");
    isConnected = true;
  } else {
    println("Can't connect to node process");
    isConnected = false;
  }

  mainScreen = new MainScreen();
  calib_screen = new CalibrationScreen();
  mainScreen.isActive = true;

  robot = new Robot();
  leg_data = new Leg_Data[4];
  for(int i = 0; i < 4; i++){
    leg_data[i] = new Leg_Data();
  }
}

void draw(){
 
  if (mainScreen.isActive){

    if(newData){
      newData = false;
      mainScreen.roll_graph.addPoint(angX);
      mainScreen.pitch_graph.addPoint(angY);
      mainScreen.yaw_graph.addPoint(angZ);
      //mainScreen.floor_graph.addPoint(0,floorX);
      //mainScreen.floor_graph.addPoint(1,floorY);
      //mainScreen.current_graph.addPoint(current);
    }

    mainScreen.draw();
  }

  if (calib_screen.isActive){
    calib_screen.draw();
  }
}

void mousePressed(){
  if (mainScreen.isActive){
    mainScreen.mousePressed();
    if (isConnected) {
        //mainScreen.com_btn.setColor(50,255,50);
        println("Robot: cliente TCP ya conectado");
     }
  }
}

void clientEvent(Client c){
  if (c == client && isConnected){
    socket_str = c.readString();
    //println(socket_str);
    if (socket_str != null){ 
      if (socket_str.charAt(0) == '@' && telemON){ 
        int parseOK = parseTelem(socket_str); 
        newData = (parseOK != 1) ? true : false; 
        if(saveTelem){ 
          int index = socket_str.indexOf(';'); 
          socket_str = socket_str.substring(1,index); 
          telemetryFile.println(socket_str); 
        } 
      } else if (socket_str.charAt(0) == '&' && telemON){ 
        parseRobotPose(socket_str); 
      } else if (socket_str.charAt(0) == '%' && telemON){  
        parseServosAngle(socket_str); 
      } else{ 
        print(socket_str); 
      } 
    }
  }
}

//void mouseReleased(){
  //mainScreen.leg.footPressed = false;
//}



void handleRobotLine(String inString) {
  if (inString == null || inString.length() == 0) return;

  if (inString.charAt(0) == '@' && telemON){
    int parseOK = parseTelem(inString);
    newData = (parseOK != 1) ? true : false;
    if(saveTelem){
      int index = inString.indexOf(';');
      if (index > 0) {
        String s = inString.substring(1,index);
        telemetryFile.println(s);
      }
    }
  }
  else if (inString.charAt(0) == '&' && telemON){
    parseRobotPose(inString);
  }
  else if (inString.charAt(0) == '%' && telemON){
    parseServosAngle(inString);
  }
  else{
    println(inString);
  }
}

void keyPressed() {
  if (key == 't'){
    mainScreen.telem_btn.toggleState();
    if (mainScreen.telem_btn.on_state == true){
      robot.telemetry_on();
    }
    else{
      robot.telemetry_off();
    }
  }
  else if (key == 'h'){
    mainScreen.holdOrient_btn.toggleState();
    if (mainScreen.holdOrient_btn.on_state == true){
      robot.hold_orient_on();
    }
    else{
      robot.hold_orient_off();
    }
  }
}

void keyReleased() {
  //Nothing
}

// === TCP SEND helpers (reemplaza serial.write) ===
void tcpWrite(String s) {
  if (isConnected) {
    client.write(s);
  }
}

void sendFootPosMsg(){
  if (footX != last_footX || footY != last_footY || footZ != last_footZ){
    newFootPos = true;
    last_footX = footX;
    last_footY = footY;
    last_footZ = footZ;

    tcpWrite("#XY:" + footX + "," + footY + "," + footZ + ";\n");
  }
}

void sendAttitudeMsg(){
  if (pitch != lastPitch){
    lastPitch = pitch;
    tcpWrite("#PI:" + pitch + ";\n");
  }
  if (roll != lastRoll){
    lastRoll = roll;
    tcpWrite("#RO:" + roll + ";\n");
  }
  if (yaw != lastYaw){
    lastYaw = yaw;
    tcpWrite("#YW:" + yaw + ";\n");
  }
}

void sendVelMsg(){
  if (lastVelX != velX || lastVelY != velY){
    lastVelX = velX;
    lastVelY = velY;

    tcpWrite("#VE:" + velX + "," + velY + ";\n");
  }
}
