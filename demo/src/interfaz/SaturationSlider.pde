class SaturationSlider extends Slider{
  float upperLimit;
  float lowerLimit;
  Button set_uppLim_btn;
  Button set_lowLim_btn;
  
  SaturationSlider(int pX, int pY, int size){
    super(pX, pY, size);
    set_lowLim_btn = new Button (posX, posY+30, 80, 30);
    set_lowLim_btn.text = "set Min";
    set_lowLim_btn.setColor(200,200,100);
    
    set_uppLim_btn = new Button (posX+size, posY+30, 80, 30);
    set_uppLim_btn.text = "set Max";
    set_uppLim_btn.setColor(200,200,100);
    
  }
  
  void draw(){
    super.draw();
    
    text = str(int(lowerLimit));
    text(text, posX, posY-30);
    
    text = str(int(upperLimit));
    text(text, posX+size, posY-30);
    
    set_lowLim_btn.draw();
    set_uppLim_btn.draw();
  }
  
  boolean mousePressed(){
    if (set_uppLim_btn.isMouseOver()){
      print("Set upper limit");
      upperLimit = value;
    }
    else if (set_lowLim_btn.isMouseOver()){
      print("Set lower limit");
      lowerLimit = value;
    }
    return false;
  }
}
