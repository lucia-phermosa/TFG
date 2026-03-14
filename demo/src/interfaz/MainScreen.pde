
class MainScreen extends Screen{
  
  //Leg leg;
  
  //Buttons
  Graph roll_graph;
  Graph pitch_graph;
  Graph yaw_graph;
  Slider oscil_w_slider;
  Slider oscil_Xphi_slider;
  Slider oscil_Yphi_slider;
  Slider oscil_Zphi_slider;
  
  StateButton telem_btn;
  StateButton walk_btn;
  StateButton holdOrient_btn;
  StateButton trot_btn;
  StateButton roll_btn;
  StateButton pitch_btn;
  StateButton reset_btn;

  Top_view topView;
  Telemetria telemetry;
  RobotActuators robotActuators;
  
  MainScreen(){
    back_btn_hidden = true;
    
    telem_btn = new StateButton (100,50,130,40);
    telem_btn.text = "Telem ON";
    telem_btn.setColorOFF(255,165,0);
    telem_btn.setColorON(160,255,110);
    
    walk_btn = new StateButton (100,100,130,40);
    walk_btn.text = "Walk ON";
    walk_btn.setColorOFF(255,165,0);
    walk_btn.setColorON(160,255,110);
    
    holdOrient_btn = new StateButton (100,150,130,40);
    holdOrient_btn.text = "Horiz ON";
    holdOrient_btn.setColorOFF(255,165,0);
    holdOrient_btn.setColorON(160,255,110);
    
    trot_btn = new StateButton (100,200,130,40);
    trot_btn.text = "Trot ON";
    trot_btn.setColorOFF(255,165,0);
    trot_btn.setColorON(160,255,110);
    
    roll_btn = new StateButton (100,250,130,40);
    roll_btn.text = "Roll";
    roll_btn.setColorOFF(255,165,0);
    roll_btn.setColorON(160,255,110);
    
    pitch_btn = new StateButton (100,300,130,40);
    pitch_btn.text = "Pitch";
    pitch_btn.setColorOFF(255,165,0);
    pitch_btn.setColorON(160,255,110);
    
    reset_btn = new StateButton (100,350,130,40);
    reset_btn.text = "Reset";
    reset_btn.setColorOFF(255,165,0);
    reset_btn.setColorON(160,255,110);
    
    oscil_w_slider = new Slider(250,290,200);
    oscil_w_slider.setLimits(0,50);
    
    oscil_Zphi_slider = new Slider(250,350,200);
    oscil_Zphi_slider.setLimits(0,10);
    oscil_Xphi_slider = new Slider(600,290,200);
    oscil_Xphi_slider.setLimits(-10,10);
    oscil_Yphi_slider = new Slider(600,350,200);
    oscil_Yphi_slider.setLimits(-10,10);
    
    roll_graph = new Graph(200,30,300,200);
    roll_graph.nPoints = 200;
    roll_graph.title = "Roll";
    roll_graph.set_Ylimits(-15,15);
    
    pitch_graph = new Graph(550,30,300,200);
    pitch_graph.nPoints = 200;
    pitch_graph.title = "Pitch";
    pitch_graph.set_Ylimits(-15,15);
    
    yaw_graph = new Graph(900,30,300,200);
    yaw_graph.nPoints = 200;
    yaw_graph.title = "Yaw";
    yaw_graph.set_Ylimits(-15,15);
    
    //floor_graph = new Graph(1200,300,300,200);
    //floor_graph.nPoints = 200;
    //floor_graph.title = "Floor inclination";
    //floor_graph.yAxis = "degrees";
    //floor_graph.set_Ylimits(-30,30);    
    //floor_graph.series.add(new GraphSeries());
    //floor_graph.series.get(0).data_color = color(255,0,0);
    //floor_graph.series.get(1).data_color = color(0,150,0);
    
    //current_graph = new Graph(1200,550,300,200);
    //current_graph.nPoints = 200;
    //current_graph.title = "Current";
    //current_graph.set_Ylimits(-1,5);
    
    topView = new Top_view(130, 600, 200, 350);
    topView.set_robotSize(250, 100);
    topView.scale = 0.5;
    
    telemetry = new Telemetria(850,300);
    robotActuators = new RobotActuators(350,450,150,15);
  }
  
  void draw(){
    draw_basic();
    
    //leg.update();
    //leg.draw();
    walk_btn.draw();
    telem_btn.draw();
    holdOrient_btn.draw();
    trot_btn.draw();
    roll_btn.draw();
    pitch_btn.draw();
    reset_btn.draw();
    
    roll_graph.draw();
    pitch_graph.draw();
    yaw_graph.draw();

    topView.draw();
    telemetry.draw();
    robotActuators.draw(leg_data);
    

    
    //Osciladores
    oscil_w_slider.draw();
    int osc_w = int(oscil_w_slider.value);
    //int osc_w = int(oscil_w_slider.value * 10);   // rad/s * 10
    
    oscil_Xphi_slider.draw();
    int osc_x = int(oscil_Xphi_slider.value);
    //int osc_x = int(oscil_Xphi_slider.value * PI/18); // PI/180 * 10  // rad*10
    oscil_Yphi_slider.draw();
    int osc_y = int(oscil_Yphi_slider.value);
    //int osc_y = int(oscil_Yphi_slider.value * PI/18); // PI/180 * 10  // rad*10
    oscil_Zphi_slider.draw();
    int osc_z = int(oscil_Zphi_slider.value);    //mm
    
    robot.sendOscilatorW(osc_w);
    robot.sendOscilatorX(osc_x);
    robot.sendOscilatorY(osc_y);
    robot.sendOscilatorZ(osc_z);
  }
  
  void mousePressed(){
    super.isPressed();
    
    //Messages
    if (isConnected){
      
     if(walk_btn.isMouseOver()){
       walk_btn.toggleState();
        if (walkON){
          robot.walk_off();
          walkON = false;
        }
        else{
          robot.walk_on();
          walkON = true;
        }
      }
      
      else if(telem_btn.isMouseOver()){
        telem_btn.toggleState();
        if (telem_btn.on_state == true){
          telemON = true;
          robot.telemetry_on();
        }
        else{
          telemON = false;
          robot.telemetry_off();
        }
      }
      
      else if(holdOrient_btn.isMouseOver()){
        holdOrient_btn.toggleState();
        if (holdOrient_btn.on_state == true){
          robot.hold_orient_on();
        }
        else{
          robot.hold_orient_off();
        }
      }
      
      else if(trot_btn.isMouseOver()){
        trot_btn.toggleState();
        if (trot_btn.on_state == true){
          trotON = true;
          robot.trot_on();
        }
        else{
          trotON = false;
          robot.trot_off();
        }
      }
      else if(roll_btn.isMouseOver()){
        roll_btn.toggleState();
        if (roll_btn.on_state == true){
          robot.roll_on();
        }
        else{
          robot.roll_off();
        }
      }
      
      else if(pitch_btn.isMouseOver()){
        pitch_btn.toggleState();
        if (pitch_btn.on_state == true){
          robot.pitch_on();
        }
        else{
          robot.pitch_off();
        }
      }
      else if(reset_btn.isMouseOver()){
        robot.reset();
      }
    }
  }
}
