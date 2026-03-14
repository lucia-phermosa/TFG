class Screen{
  boolean isActive;
  Button back_btn;
  Button close_btn;
  boolean back_btn_hidden;
  
  Screen(){
    isActive = false;
    close_btn = new Button (width-30, 10, 50, 30);
    close_btn.text = "Exit";
    close_btn.setColor(255,0,0);
    close_btn.setVertex(10);
    
    back_btn = new Button(width-100, 10, 50, 30);
    back_btn.text = "←";
    back_btn.setColor(255,165,0);
    back_btn.setVertex(10); 
    back_btn_hidden = false;
  }
  
  boolean isPressed(){
    if (isActive){
      if (!back_btn_hidden && back_btn.isMouseOver()){
        isActive = false;
        return true;
      }
      else if(close_btn.isMouseOver()){
        exit();
      }
    }
    return false;
  }
  
  void draw_basic(){
    background(200);
    close_btn.draw();
    if (!back_btn_hidden){
      back_btn.draw();  
    }    
  }
}
