
int parseTelem(String telem_str){
  int end = telem_str.indexOf(';');
  if (end < 0) return -1;

  telem_str = trim(telem_str);

  // Quitar '@' si está y recortar hasta ';'
  int start = (telem_str.length() > 0 && telem_str.charAt(0) == '@') ? 1 : 0;
  String payload = telem_str.substring(start, end);

  String[] list = split(payload, ',');
  if (list.length < 8) return -1;   // esperamos 8 campos

  angX = float(list[0]) * 57.2958;  // roll  (rad->deg si aplica)
  angY = float(list[1]) * 57.2958;  // pitch
  angZ = float(list[2]) * 57.2958;  // yaw

  worldX = float(list[3]);
  worldY = float(list[4]);
  worldZ = float(list[5]);

  cgX = float(list[6]);
  cgY = float(list[7]);

  return 0;
}

int parseRobotPose(String pose_str){
  int end = pose_str.indexOf(';');
  if (end < 0) return -1;

  pose_str = trim(pose_str);

  // Quitar '&' si está y recortar hasta ';'
  int start = (pose_str.length() > 0 && pose_str.charAt(0) == '&') ? 1 : 0;
  String payload = pose_str.substring(start, end);

  String[] list = split(payload, ',');
  if (list.length < 15) return -1;   // 3 (body) + 12 (4 patas * 3)

  // Body pose (índices 0..2)
  robotX = int(float(list[0]))*1000;
  robotY = int(float(list[1]))*1000;
  robotZ = int(float(list[2]))*1000;

  // Pies: orden del mensaje = FR, RR, FL, RL
  // Los guardamos como footPos[0]=FR, footPos[1]=RR, footPos[2]=FL, footPos[3]=RL
  for (int i = 0; i < 4; i++){
    int base = 3 + i * 3;  // empieza en list[3]
    mainScreen.topView.footPos[i].x = float(list[base + 0])*1000.0;
    mainScreen.topView.footPos[i].y = float(list[base + 1])*1000.0;
    mainScreen.topView.footPos[i].z = float(list[base + 2])*1000.0;
  }
  println();

  newPose = true;
  return 0;
}

int parseServosAngle(String servos_str){
  int end = servos_str.indexOf(';');
  if (end < 0) return -1;

  servos_str = trim(servos_str);

  // Quitar '%' si está y recortar hasta ';'
  int start = (servos_str.length() > 0 && servos_str.charAt(0) == '%') ? 1 : 0;
  String payload = servos_str.substring(start, end);

  String[] list = split(payload, ',');
  if (list.length < 12) return -1;   // 4 patas * 3 ángulos

  final float RAD2DEG = 180.0 / PI;  // 57.29578...

  for (int i = 0; i < 4; i++){
    leg_data[i].servoAngle[0] = int(float(list[i*3 + 0]) * RAD2DEG);
    leg_data[i].servoAngle[1] = int(float(list[i*3 + 1]) * RAD2DEG);
    leg_data[i].servoAngle[2] = int(float(list[i*3 + 2]) * RAD2DEG);
  }

  newServosData = true;
  return 0;
}
