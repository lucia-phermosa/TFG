class Telemetria{
  PFont font;
  float posX;
  float posY;
  float nameColumnWidth;
  float dataColumnWidth;
  float unitsColumnWidth;
  float rowHeight;
  int margenIzquierdo;
  int distText2upperLimmit;
  StringList name;
  StringList data;
  StringList units;
  int textSize;
  
  Telemetria(float posX, float posY){
    this.posX = posX;
    this.posY = posY;
    this.nameColumnWidth = 175;
    this.dataColumnWidth = 85;
    this.unitsColumnWidth = 60;
    rowHeight = 25;
    margenIzquierdo = 10;
    distText2upperLimmit = 19;
    
    name = new StringList();
    data = new StringList();
    units = new StringList();
    textSize = 16;
    font = createFont("Arial",textSize,true);
    
    name.append("Orientation");
    name.append(" - roll");
    name.append(" - pitch");
    name.append(" - yaw");
    
    name.append("Robot");
    name.append(" - world X");
    name.append(" - world Y");
    name.append(" - world Z");
    
    name.append("");
    name.append("CG X");
    name.append("CG Y");
    
    name.append("");
    name.append("FR footZ");
    name.append("RR footZ");
    name.append("FL footZ");
    name.append("RL footZ");
    
    units.append("Units");
    units.append("deg");
    units.append("deg");
    units.append("deg");
    
    units.append("");
    units.append("m");
    units.append("m");
    units.append("m");
    
    units.append("");
    units.append("mm");
    units.append("mm");
    
    units.append("");
    units.append("mm");
    units.append("mm");
    units.append("mm");
    units.append("mm");
  }
  
  void update(){
    data.clear();
    data.append("");
    data.append(nf(angX,0,6));
    data.append(nf(angY,0,6));
    data.append(nf(angZ,0,6));
    
    data.append("");
    data.append(nf(worldX,0,6));
    data.append(nf(worldX,0,6));
    data.append(nf(worldX,0,6));
    
    data.append("");
    data.append(nf(cgX,0,6));
    data.append(nf(cgY,0,6));
    
    data.append("");
    data.append(nf(mainScreen.topView.footPos[0].z,0,3));
    data.append(nf(mainScreen.topView.footPos[1].z,0,3));
    data.append(nf(mainScreen.topView.footPos[2].z,0,3));
    data.append(nf(mainScreen.topView.footPos[3].z,0,3));

  }
  
  void draw(){
    update();
    textAlign(LEFT);
    fill(0);
    float dY = rowHeight;
    textFont(font,textSize);
    
    //Creacion de la tabla
    fill(255);
    rectMode(CORNER);
    rect(posX, posY, nameColumnWidth + dataColumnWidth + unitsColumnWidth, rowHeight * data.size());
    line(posX, posY, posX + nameColumnWidth + dataColumnWidth + unitsColumnWidth, posY);
    for (int i = 0; i<data.size(); i++){
      strokeWeight(1);
      line(posX, posY + dY * i, posX + nameColumnWidth + dataColumnWidth + unitsColumnWidth, posY + dY * i);
      if (i + 1 == data.size()){
        line(posX, posY + dY + dY * i, posX + nameColumnWidth + dataColumnWidth + unitsColumnWidth, posY + dY + dY * i);
      }
    }
    line(posX, posY, posX, posY + rowHeight * data.size());
    line(posX + nameColumnWidth, posY, posX + nameColumnWidth, posY + rowHeight * data.size());
    line(posX + nameColumnWidth + dataColumnWidth, posY, posX + nameColumnWidth + dataColumnWidth, posY + rowHeight * data.size());
    
    //Se rellena con los datos
    fill(0);
    for (int i = 0; i<data.size(); i++){
      String str = name.get(i);
      text(str, posX + margenIzquierdo, posY + distText2upperLimmit + dY * i);
      str = data.get(i);
      text(str, posX + nameColumnWidth + margenIzquierdo, posY + distText2upperLimmit + dY * i);
      str = units.get(i);
      text(str, posX + nameColumnWidth + dataColumnWidth + margenIzquierdo, posY + distText2upperLimmit + dY * i);
    }
  }
}
