
class CalibrationScreen extends Screen{
  
  Leg_calibration calib_FRleg;
  Leg_calibration calib_RRleg;
  Leg_calibration calib_FLleg;
  Leg_calibration calib_RLleg;
  LimitsScreen limits_screen;
  
  PFont font;
  String text;
  int textSize;
  
  CalibrationScreen(){
    calib_FRleg = new Leg_calibration(550,150);
    calib_RRleg = new Leg_calibration(550,550);
    calib_FLleg = new Leg_calibration(150,150);
    calib_RLleg = new Leg_calibration(150,550);
    calib_FRleg.text = "Front Right";
    calib_RRleg.text = "Rear Right";
    calib_FLleg.text = "Front Left";
    calib_RLleg.text = "Rear Left";
    
    text = "";
    textSize = 20;
    font = createFont("Arial",textSize,true);
    
    limits_screen = new LimitsScreen();
  }
  
  void draw(){
    draw_basic();
    
    calib_FRleg.draw();
    calib_RRleg.draw();
    calib_FLleg.draw();
    calib_RLleg.draw();
    
    textAlign(CENTER,CENTER);
    textFont(font,textSize);
    
    text = "Shoulder 1 [q0]";
    text(text, 350, 150);
    text(text, 350, 550);
    text = "Shoulder 2 [q1]";
    text(text, 350, 200);
    text(text, 350, 600);
    text = "Elbow [q2]";
    text(text, 350, 250);
    text(text, 350, 650);
    
    if (calib_FRleg.setLimits){
      limits_screen.draw();
    }
    else if (calib_RRleg.setLimits){
      limits_screen.draw();
    }
    else if (calib_FLleg.setLimits){
      limits_screen.draw();
    }
    else if (calib_RLleg.setLimits){
      limits_screen.draw();
    }
  }
}
