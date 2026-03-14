import processing.serial.*;

class KnobArray {
  Serial serial;
  String comPort;
  PApplet parent_this;
  boolean serialError = false;
  boolean connected = false;

  int[] values;
  KnobArray(PApplet parent, String serialPort){
    parent_this = parent;
    comPort = serialPort;
    
    try{
      //printArray(Serial.list());
      serial = new Serial (parent_this, comPort, 250000);
      serial.bufferUntil('\n');
      delay(1000);    //Important! Allows MCU to reset
    }
    catch(RuntimeException exc){
      println("Can't connect to KnobArray");
      serialError = true;
    }
    if (!serialError){
      connected = true;
      println("KnobArray: Starting datastream");
      startDataStream();
    }
    values = new int[6];
  }
  
  void serialEvent(){
    String inString = serial.readString();
    if (inString != null){
      if (inString.charAt(0) == '#'){
        int parseOK = parseData(inString);
      }
    }
  }
  
  int parseData(String data_str){
    int index = data_str.indexOf(';');
    if(index < 0){
      return -1;
    }
    data_str = data_str.substring(1, index);
    String[] list = split(data_str, ',');
    for (int i=0; i<6; i++){
      values[i] = int(list[i]);
    }
    return 0;
  }
  
  void startDataStream(){
    String str = "#DS:1;";
    sendCommand(str);
  }
  
  void stopDataStream(){
    String str = "#DS:0;";
    sendCommand(str);
  }
  
  void sendCommand(String str){
    if (!serialError){
      serial.write(str);
    }
  }
}
