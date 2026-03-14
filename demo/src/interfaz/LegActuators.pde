class LegActuators{
  int posX;
  int posY;
  int sizeX;
  int sizeY;
  
  BarIndicator[] servoIndicator;
  
  LegActuators (int posX, int posY, int sizeX, int sizeY){
    this.posX = posX;
    this.posY = posY;
    this.sizeX = sizeX;
    this.sizeY = sizeY;
    
    servoIndicator = new BarIndicator[3];
    for (int i = 0; i < 3; i++){
      servoIndicator[i] = new BarIndicator(posX, posY + i*50, sizeX, sizeY);
      servoIndicator[i].setLimits(-45, 45);
      servoIndicator[i].nDecimals = 0;
    }
  }
  
  void draw(Leg_Data data){
    for (int i = 0; i < 3; i++){
      servoIndicator[i].draw(data.servoAngle[i]);
    }
  }
}

class RobotActuators{
  int posX, posY;
  int sizeX, sizeY;
  
  LegActuators[] legActuators;
  
  RobotActuators (int posX, int posY, int sizeX, int sizeY){
    this.posX = posX;
    this.posY = posY;
    this.sizeX = sizeX;
    this.sizeY = sizeY;
    
    legActuators = new LegActuators[4];
    legActuators[0] = new LegActuators(posX+200, posY, sizeX, sizeY);
    legActuators[1] = new LegActuators(posX+200, posY+200, sizeX, sizeY);
    legActuators[2] = new LegActuators(posX, posY, sizeX, sizeY);
    legActuators[3] = new LegActuators(posX, posY+200, sizeX, sizeY);
  }
  
  void draw(Leg_Data[] data){
    for (int i = 0; i < 4; i++){
      legActuators[i].draw(data[i]);
    }
  }
}
