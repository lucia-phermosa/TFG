class Adjustment_buttons{
  float posX, posY;
  Button up_btn;
  Button down_btn;
  
  //Display
  PFont font;
  String text;
  int textSize;
  int value;
  
  Adjustment_buttons(float posX, float posY){
    this.posX = posX;
    this.posY = posY;
    down_btn = new Button (posX, posY, 30, 30);
    down_btn.text = "-";
    down_btn.setColor(200,100,100);
    
    up_btn = new Button (posX + 100, posY, 30, 30);
    up_btn.text = "+";
    up_btn.setColor(100,200,100);
    
    this.textSize = 20;
    font = createFont("Arial",textSize,true);
  }
  
  void draw(){
    down_btn.draw();
    up_btn.draw();
    
    this.text = str(value);
    textAlign(CENTER,CENTER);
    textFont(font,textSize);
    text(text,posX+50, posY);
  }
  
  boolean isMouseOver(){
    if (down_btn.isMouseOver()){
      value--;
      return true;
    }
    else if(up_btn.isMouseOver()){
      value++;
      return true;
    }
    return false;
  }
}
