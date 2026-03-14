class LimitsScreen{
  SaturationSlider slider_shoulder1;
  SaturationSlider slider_shoulder2;
  SaturationSlider slider_elbow;
  Button saveLimits_btn;
  
  int origenX;
  int origenY;
  int screenWidth;
  int screenHeight;
  
  boolean isActive;
  int leg_id;
  
  LimitsScreen(){
    screenWidth = 300;
    screenHeight = 400;
    origenX = (width - screenWidth)/2;
    origenY = (height - screenHeight)/2;
    isActive = false;
    
    slider_shoulder1 = new SaturationSlider(origenX+50,origenY+50,200);
    slider_shoulder1.setLimits(0,180);
    
    slider_shoulder2 = new SaturationSlider(origenX+50,origenY+150,200);
    slider_shoulder2.setLimits(0,180);
    
    slider_elbow = new SaturationSlider(origenX+50,origenY+250,200);
    slider_elbow.setLimits(0,180);
    
    saveLimits_btn = new Button(origenX+150, origenY+350, 200, 40);
    saveLimits_btn.setColor(200,230,90);
    saveLimits_btn.text = "Save limits";
    saveLimits_btn.vertex = 20;
  }
  
  void mousePressed(){
    if (isActive){
      print("Limits_screen: ");
      println(isActive);
      slider_shoulder1.mousePressed();
      slider_shoulder2.mousePressed();
      slider_elbow.mousePressed();
      
    }
  }
  
  void draw(){
    //Background 
    fill(220);
    rectMode(CORNER);
    rect(origenX, origenY, screenWidth, screenHeight, 30);
    
    slider_shoulder1.draw();
    slider_shoulder2.draw();
    slider_elbow.draw();
    
    saveLimits_btn.draw();
  
  }
  
  void setLimits(Calib_data cd){
    slider_shoulder1.lowerLimit = cd.q0_min;
    slider_shoulder1.upperLimit = cd.q0_max;
    slider_shoulder2.lowerLimit = cd.q1_min;
    slider_shoulder2.upperLimit = cd.q1_max;
    slider_elbow.lowerLimit = cd.q2_min;
    slider_elbow.upperLimit = cd.q2_max;
    slider_shoulder1.value = cd.q0;
    slider_shoulder2.value = cd.q1;
    slider_elbow.value = cd.q2;
  }
}
