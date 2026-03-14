

class Leg {
  int px, py;
  float l1, l2;
  float q1, q2;
  
  int targetX, targetY;
  float scale;
  float pieX;
  float pieY;
  boolean footPressed;
  
  Leg (int length_upper_part, int length_lower_part, float dim_scale){
    l1 = length_upper_part;
    l2 = length_lower_part;
    q1 = PI/2;
    q2 = 0;
    scale = dim_scale;
    footPressed = false;
  }
  
  void setShoulderPos(int x, int y){
    px = x;
    py = y;
  }
  
  void getJoint_byPos(float x, float y){
    //inverse kinematics
    
    float beta = atan2(y, x);
    float cos_q2 = (x*x + y*y - l1*l1 - l2*l2) / (2 * l1 * l2);
    float sin_q2 = -sqrt(1.0 - cos_q2 * cos_q2);
    float a2 = atan2(sin_q2, cos_q2);
    float alpha = atan2(l2 * sin(a2), l1 + l2 * cos_q2);
    float a1 = beta - alpha;
    q1 = a1;
    q2 = a2;
  }
  
  void update(){
    if (footPressed){
      float dist = sqrt((mouseX-px) * (mouseX-px) + (mouseY-py) * (mouseY-py)) / scale;
      if (dist < l1 + l2){
        targetX = int((mouseX-px) / scale);
        targetY = int((mouseY-py) / scale);
      }
      getJoint_byPos(targetX, targetY);
    }
  }
  
  void draw(){
    strokeWeight(10);
    float codoX = px + l1*cos(q1) * scale;
    float codoY = py + l1*sin(q1) * scale;
    pieX = codoX + l2*cos(q1 + q2) * scale;
    pieY = codoY + l2*sin(q1 + q2) * scale;
    line(px, py, codoX, codoY);
    line(codoX, codoY, pieX, pieY);
    strokeWeight(1);
    fill(255);
    circle(px,py,20);
    circle(codoX,codoY,20);
    fill(255,0,0);
    circle(pieX,pieY,20);
    fill(255);
  }
}
