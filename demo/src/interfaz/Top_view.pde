class Top_view{
  int posX, posY;
  int sizeX, sizeY;
  
  PVector bodyPos;
  int body_length;
  int body_width;
  float scale;
  PImage cg;  //Center of gravity
  PVector cg_pos;
  
  PVector[] footPos;
  PVector[] footPos_circle;
  color[] footState_color;
  boolean[] mouseOver;
  boolean[] isPressed;
  
  Top_view(int pX, int pY, int map_width, int map_height){
    this.posX = pX;
    this.posY = pY;
    this.sizeX = map_width;
    this.sizeY = map_height;
    
    //Defaults
    this.scale = 1;
    cg = loadImage("240px-Secchi_disk_pattern.png");
    
    bodyPos = new PVector(0,0,0);
    footPos = new PVector[4];
    footPos_circle = new PVector[4];
    footState_color = new color[4];
    mouseOver = new boolean[4];
    isPressed = new boolean[4];
    for(int i = 0; i < 4; i++){
      footPos[i] = new PVector(footX, footY, footZ);
      footPos_circle[i] = new PVector(0, 0, 0);
      footState_color[i] = color(255, 0, 0);
      mouseOver[i] = false;
      isPressed[i] = false;
    }
    
  }
  
  void set_robotSize(int bodyLength, int bodyWidth){
    body_length = bodyLength;
    body_width = bodyWidth;
  }
  
  void draw(){
    rectMode(CENTER);
    fill(250);
    rect(posX, posY, sizeX, sizeY, 10);
  
    // Si footPos[] ya está centrado en el cuerpo, el "centro" es (0,0)
    float centerX = 0;
    float centerY = 0;
  
    // Actualiza la posición del cuerpo (world -> pantalla)
    if (newPose){
      bodyPos.x = worldX;   // sin restar centerX
      bodyPos.y = worldY;   // sin restar centerY
      bodyPos.z = worldZ;
      newPose = false;
    }
  
    // Robot (misma convención que usabas: pantallaX = +y, pantallaY = -x)
    fill(100,100,250);
    rect(posX + bodyPos.y * scale,
         posY - bodyPos.x * scale,
         body_width * scale,
         body_length * scale, 5);
  
    // CG (cgX/cgY también asumidos en coordenadas del cuerpo)
    image(cg,
          posX - 10 + (bodyPos.y + cgY) * scale,
          posY - 10 - (bodyPos.x + cgX) * scale,
          20, 20);
    
    println("bodyPos=" + bodyPos.x + "," + bodyPos.y +
        " foot0=" + footPos[0].x + "," + footPos[0].y);

    for (int i = 0; i < 4; i++){
      // footPos[i] está en coords del cuerpo -> lo llevamos a world sumando bodyPos
      float wx = bodyPos.x + footPos[i].x;
      float wy = bodyPos.y + footPos[i].y;
  
      // world -> pantalla (Xpantalla = +wy, Ypantalla = -wx)
      footPos_circle[i].set(
        posX - wy * scale,
        posY - wx * scale
      );
    }
  
    for(int i = 0; i < 4; i++){
      fill(footState_color[i]);
      circle(footPos_circle[i].x, footPos_circle[i].y, 12);
    }
  }


  
  void isMouseOver(){
    PVector mousePos = new PVector(mouseX,mouseY);
    for(int i = 0; i < 4; i++){
      float dist2foot = footPos_circle[i].dist(mousePos);
      if (dist2foot <= 12){
        mouseOver[i] = true;
      }
      else{
        mouseOver[i] = false;
      }
    }
  }
  void isPressed(){
    if (mousePressed){
      for (int i=0; i < 4; i++){
        if (mouseOver[i]){
          footState_color[i] = color(0,255,0);
          isPressed[i] = true;
          break;
        }
      }
    }
  }
  
  void isReleased(){
    if(!mousePressed){
      for (int i=0; i < 4; i++){
        if(isPressed[i]){
          footState_color[i] = color(255,0,0);
          isPressed[i] = false;
          footPos[i].z = 0;
        }
      }
    }
  }
  
  void update(){
    if (isPressed[0]){        
      footPos[0].x = (posY - mouseY)/scale - 90;
      footPos[0].y = (mouseX - posX)/scale - 90;
      footPos[0].z = 30;
    }
    else if (isPressed[1]){        
      footPos[1].x = (posY - mouseY)/scale + 90;
      footPos[1].y = (mouseX - posX)/scale - 90;
      footPos[1].z = 30;
    }
    else if (isPressed[2]){        
      footPos[2].x = (posY - mouseY)/scale - 90;
      footPos[2].y = (mouseX - posX)/scale + 90;
      footPos[2].z = 30;
    }
    else if (isPressed[3]){        
      footPos[3].x = (posY - mouseY)/scale + 90;
      footPos[3].y = (mouseX - posX)/scale + 90;
      footPos[3].z = 30;
    }
  }
}
