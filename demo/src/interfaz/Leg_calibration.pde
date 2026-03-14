class Calib_data{
    int q0_min, q0_max;
    int q1_min, q1_max;
    int q2_min, q2_max;
    int q0, q1, q2;
    
    Calib_data(){
      q0_min = 0; q0_max = 0;
      q1_min = 0; q1_max = 0;
      q2_min = 0; q2_max = 0;
      q0 = 0; q1 = 0; q2 = 0;
    }
  }

class Leg_calibration{
  int pX;
  int pY;
  
  Adjustment_buttons calib_shoulder1;
  Adjustment_buttons calib_shoulder2;
  Adjustment_buttons calib_elbow;
  Button setLimits_btn;
  boolean setLimits;
  
  PFont font;
  String text;
  int textSize;
  
  Calib_data calib_data;
  
  Leg_calibration(int posX, int posY){
    pX = posX;
    pY = posY;
    calib_shoulder1 = new Adjustment_buttons(pX,posY);
    calib_shoulder1.value = 0;
    calib_shoulder2 = new Adjustment_buttons(pX,posY+50);
    calib_shoulder2.value = 0;
    calib_elbow = new Adjustment_buttons(pX,posY+100);
    calib_elbow.value = 0;
    
    setLimits_btn = new Button(pX+50, posY+180, 200, 40);
    setLimits_btn.setColor(200,230,90);
    setLimits_btn.text = "Set limits";
    setLimits_btn.vertex = 20;
    
    text = "";
    textSize = 20;
    font = createFont("Arial",textSize,true);
    
    setLimits = false;
    calib_data = new Calib_data();
  }

  void draw(){
    calib_shoulder1.draw();
    calib_shoulder2.draw();
    calib_elbow.draw();
    
    setLimits_btn.draw();
    
    textAlign(CENTER,CENTER);
    textFont(font,textSize);
    text(text,pX+50, pY-50);
    
  }
  
  void setCalib_data(byte[] data){
    //Offsets
    calib_shoulder1.value = byte2int16(data[0],data[1]);
    calib_shoulder2.value = byte2int16(data[2],data[3]);
    calib_elbow.value = byte2int16(data[4],data[5]);
    //Limits
    calib_data.q0_min = byte2int16(data[6],data[7]);
    calib_data.q1_min = byte2int16(data[8],data[9]);
    calib_data.q2_min = byte2int16(data[10],data[11]);
    calib_data.q0_max = byte2int16(data[12],data[13]);
    calib_data.q1_max = byte2int16(data[14],data[15]);
    calib_data.q2_max = byte2int16(data[16],data[17]);
    calib_data.q0 = byte2int16(data[18],data[19]);
    calib_data.q1 = byte2int16(data[20],data[21]);
    calib_data.q2 = byte2int16(data[22],data[23]);
  }
  
  int byte2int16(byte lsb, byte msb){
    int imsb = msb & 0xFF;
    int ilsb = lsb & 0xFF;
    int val = (imsb<<8) | ilsb;
    if (val > 32767){
      val = val + 0xFFFF0000;
    }
    return val;
  }
  
}
