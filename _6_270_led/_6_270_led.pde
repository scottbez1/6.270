import processing.serial.*;


Serial rfmon;
Serial ledCtrl;

PFont myFont;

int owner[] = new int[6];
long capture_time[] = new long[6];

int balls_remaining[] = new int[6];
long mine_time[] = new long[6];

static final int MODE_TERRITORIES = 0;
static final int MODE_RAINBOW = 1;
static final int MODE_PULSE = 2;
static final int LAST_MODE = MODE_PULSE;
boolean round_running = false;
int non_round_mode = MODE_RAINBOW;

int red_team = -1;
int blue_team = -1;

int r_id[] = new int[2];
int r_x[] = new int[2];
int r_y[] = new int[2];
int r_theta[] = new int[2];
int r_score[] = new int[2];


void setup() {
  size(800,600);
  background(0);
  
  for (int i = 0; i < 6; i++) {
    owner[i] = 0;
    capture_time[i] = 0;
  }
  
  r_id[0] = 170;
  r_id[1] = 170;
  
  rfmon = new Serial(this, "/dev/tty.usbserial-A800cBal", 19200);
  rfmon.bufferUntil(13);
  
  ledCtrl = new Serial(this, "/dev/tty.usbserial-A6007x5c", 38400, 'E', 8, 1);
  
  myFont = createFont("FFScala", 32);
  textFont(myFont);
  textAlign(CENTER);
}

void dispose() {
  println("Stopping!");
  for (int i = 0; i < 3; i++) {
    println("Stopping!");
    ledCtrl.write(170);
    ledCtrl.write(255);
    
    ledCtrl.write(0);
    ledCtrl.write(0);
    ledCtrl.write(0);
    
    ledCtrl.write(0);
    ledCtrl.write(0);
    ledCtrl.write(0);
    
    ledCtrl.write(0);
    ledCtrl.write(0);
    ledCtrl.write(0);
    
    ledCtrl.write(0);
    ledCtrl.write(0);
    ledCtrl.write(0);
    
    ledCtrl.write(0);
    ledCtrl.write(0);
    ledCtrl.write(0);
    delay(10);
  }
}

void serialEvent(Serial p) {
  String inString = (p.readString());
  if (inString.startsWith("\nRound start")) {
    round_running = true;
    if (r_x[0] < 0 && r_x[1] > 0) {
      red_team = r_id[0];
      blue_team = r_id[1];
    } else {
      red_team = r_id[1];
      blue_team = r_id[0];
    }
    
    for (int i = 0; i < 6; i++) {
      owner[i] = 0;
      capture_time[i] = 0;
      balls_remaining[i] = 0;
    }
    
    println("Red team: " + red_team + "\tBlue team: " + blue_team);
  } else if (inString.startsWith("\nRound end")) { 
    non_round_mode = MODE_TERRITORIES;
    round_running = false;
    
    for (int i = 0; i < 6; i++) {
      owner[i] = 0;
      capture_time[i] = 0;
      balls_remaining[i] = 5;
      mine_time[i] = 0;
    }
    
  } else if (inString.startsWith("\nROBOT")) {
    inString = inString.replace("\nROBOT:", "");
    String parts[] = inString.split(";");
    
    int i = int(parts[0]);
    
    r_id[i] = int(parts[1]);
    r_x[i] = int(parts[2]);
    r_y[i] = int(parts[3]);
    r_theta[i] = int(parts[4]);
    r_score[i] = int(parts[5]);
  } else if (inString.startsWith("\nTERR")) {
    inString = inString.replace("\nTERR:", "");
    String parts[] = inString.split(";");
    int i = int(parts[0]);
    if (owner[i] != int(parts[1])) {
      capture_time[i] = millis();
    }
    owner[i] = int(parts[1]);
    
    if (balls_remaining[i] > int(parts[2])) {
      mine_time[i] = millis();
    }
    balls_remaining[i] = int(parts[2]);
  }
}



static final int HEX_RADIUS = 300;
static final int HEX_CENTER_X = 400;
static final int HEX_CENTER_Y = 300;

static final int INNER_HEX_RADIUS = HEX_RADIUS / 4;

static final float DIVIDER_LEN = HEX_RADIUS * cos(PI/6);


void drawBorderHex() {
  // Draw outer hexagon
  stroke(100);
  strokeWeight(4);
  fill(0,100,0);
  beginShape();
  for (int ang = 0; ang < 360; ang += 60) {
    float rad = radians(ang);
    vertex(HEX_CENTER_X + HEX_RADIUS * cos(rad), HEX_CENTER_Y + HEX_RADIUS * sin(rad));
  }
  endShape(CLOSE);
}


void drawInnerHexAndTerritories() {
  // draw inner hexagon and territory lines 
  beginShape();
  for (int ang = 30; ang < 360; ang += 60) {
    float rad = radians(ang);
    float x = HEX_CENTER_X + INNER_HEX_RADIUS * cos(rad);
    float y = HEX_CENTER_Y + INNER_HEX_RADIUS * sin(rad);
    // inner hex vertex
    vertex(x,y);
    
    // divider line begins from inner hex vertex
    strokeWeight(2);
    stroke(0);
    line(x,y, HEX_CENTER_X + DIVIDER_LEN * cos(rad), HEX_CENTER_Y + DIVIDER_LEN * sin(rad));
  }
  stroke(0);
  fill(50,50,50);
  strokeWeight(4);
  endShape(CLOSE); 
}


static final float LIGHTBOX_OUTWARD = 0.33;
static final float LIGHTBOX_RADIUS = (1-LIGHTBOX_OUTWARD)*INNER_HEX_RADIUS + LIGHTBOX_OUTWARD*HEX_RADIUS;
static final float LIGHTBOX_SCALE = 0.25;
static final float LIGHTBOX_LONG = LIGHTBOX_SCALE * DIVIDER_LEN;
static final float LIGHTBOX_SHORT = LIGHTBOX_SCALE * (HEX_RADIUS * sin(PI/6));

color lightbox[] = new color[6];

void drawLightboxes() {
  for (int ang = 0; ang < 360; ang += 60) {
    float rad = -radians(ang);
    
    float a_x = HEX_CENTER_X + LIGHTBOX_RADIUS * cos(rad);
    float a_y = HEX_CENTER_Y + LIGHTBOX_RADIUS * sin(rad);

    float b_x = a_x + LIGHTBOX_LONG * cos(rad + PI/6);
    float b_y = a_y + LIGHTBOX_LONG * sin(rad + PI/6);

    float c_x = b_x + LIGHTBOX_SHORT * cos(rad - PI/3);
    float c_y = b_y + LIGHTBOX_SHORT * sin(rad - PI/3);

    float d_x = c_x + LIGHTBOX_SHORT * cos(rad - 2*PI/3);
    float d_y = c_y + LIGHTBOX_SHORT * sin(rad - 2*PI/3);
    
    beginShape();
    vertex(a_x, a_y);
    vertex(b_x, b_y);
    vertex(c_x, c_y);
    vertex(d_x, d_y);
    noStroke();
    fill(lightbox[ang/60]);//TODO
    endShape(CLOSE);
  }
}


int boardToScreenX(int boardX) {
  return boardX * HEX_RADIUS / 2047 + HEX_CENTER_X;
}
int boardToScreenY(int boardY) {
  return HEX_CENTER_Y - boardY * HEX_RADIUS / 2047;
}
float boardToScreenTheta(int boardTheta) {
  return boardTheta / 2047. * PI; 
}


void drawRobots() {
  stroke(255);
  strokeWeight(2);
  
  
  for (int i = 0; i < 2; i++) {
    if (r_id[i] == 170) {
      continue;
    }
    if (r_id[i] == red_team) {
      fill(255,0,0);
    } else if (r_id[i] == blue_team) {
      fill(0,0,255);
    } else {
      fill(255);
    }
    
    
    int ax = boardToScreenX(r_x[i]);
    int ay = boardToScreenY(r_y[i]);
    float a_rad = boardToScreenTheta(r_theta[i]);
    triangle(ax + 30*cos(a_rad),
             ay - 30*sin(a_rad),
             ax + 20*cos(a_rad + 0.75*PI),
             ay - 20*sin(a_rad + 0.75*PI),
             ax + 20*cos(a_rad - 0.75*PI),
             ay - 20*sin(a_rad - 0.75*PI));
             
    fill(255);
    textAlign(CENTER);
    text("Team " + r_id[i], ax, ay+60);
  }
}

void drawScoreboard() {
  if (!round_running) {
    //return;
  }
  for (int i = 0; i < 2; i++) {
    int x;
    if (r_id[i] == red_team) {
      textAlign(LEFT);
      x = 10;
      fill(200,50,50);
    } else if (r_id[i] == blue_team) {
      textAlign(RIGHT);
      x = width - 10;
      fill(50,50,200);
    } else {
      return;
    }
    
    text("Team " + r_id[i], x, 50);
    fill(255);
    text(str(r_score[i]), x, 90);
  }
}

void draw() {
  background(0);
  drawBorderHex();
  drawInnerHexAndTerritories();
  drawLightboxes();
  
  drawRobots();
  drawScoreboard();
  
  animate();
  
  updateLeds();
  
  delay(5);
}


int scaleColor(int c) {
  int ret = int(c*0.5);
  return ret == 170 ? 171 : ret;
}


void updateLeds() {
  /*
  ledCtrl.write(170);
  ledCtrl.write(1);
  
  ledCtrl.write(scaleColor(int(red(lightbox[0]))));
  ledCtrl.write(scaleColor(int(green(lightbox[0]))));
  ledCtrl.write(scaleColor(int(blue(lightbox[0]))));
  
  ledCtrl.write(scaleColor(int(red(lightbox[1]))));
  ledCtrl.write(scaleColor(int(green(lightbox[1]))));
  ledCtrl.write(scaleColor(int(blue(lightbox[1]))));
  
  ledCtrl.write(scaleColor(int(red(lightbox[2]))));
  ledCtrl.write(scaleColor(int(green(lightbox[2]))));
  ledCtrl.write(scaleColor(int(blue(lightbox[2]))));
  
  ledCtrl.write(0);
  ledCtrl.write(0);
  ledCtrl.write(0);
  
  ledCtrl.write(0);
  ledCtrl.write(0);
  ledCtrl.write(0);
  
  
  
  ledCtrl.write(170);
  ledCtrl.write(0);
  
  ledCtrl.write(scaleColor(int(red(lightbox[3]))));
  ledCtrl.write(scaleColor(int(green(lightbox[3]))));
  ledCtrl.write(scaleColor(int(blue(lightbox[3]))));
  
  ledCtrl.write(0);
  ledCtrl.write(0);
  ledCtrl.write(0);
  
  ledCtrl.write(scaleColor(int(red(lightbox[5]))));
  ledCtrl.write(scaleColor(int(green(lightbox[5]))));
  ledCtrl.write(scaleColor(int(blue(lightbox[5]))));
  
  ledCtrl.write(0);
  ledCtrl.write(0);
  ledCtrl.write(0);
  
  ledCtrl.write(0);
  ledCtrl.write(0);
  ledCtrl.write(0);
  */
}




public color anim_rainbow(int i, int frame) {
  colorMode(HSB, 360, 1, 1);
  color ret = color((millis()/30 + i*60) % 360, 1, 1);
  colorMode(RGB, 255,255,255);
  return ret;
}

int frame = 0;
void animate() {
  for (int i = 0; i < 6; i++) {
    if (round_running || non_round_mode == MODE_TERRITORIES) {
      if (owner[i] == red_team) {
        lightbox[i] = color(255,0,0);
      } else if (owner[i] == blue_team) {
        lightbox[i] = color(0,0,255);
      } else {
        lightbox[i] = color(0,0,0);
      }
      
      // blink yellow if just captured
      if (millis() - capture_time[i] < 500) {
        int yellow = int( sin((millis() - capture_time[i])*2*PI/100)*255);
        lightbox[i] = color(yellow,yellow,0);
      }
      
      if (millis() - mine_time[i] < 400) {
        int x = int(400 - (millis() - mine_time[i]));
        int white = color(x,x,x);
        lightbox[i] = blendColor(lightbox[i], white, ADD); 
      }
    } else {
      switch (non_round_mode) {
        case MODE_RAINBOW:
          lightbox[i] = anim_rainbow(i, frame);
          break;
        case MODE_PULSE:
          int v = int( sin(millis()*2*PI/3000)*100) + 155;
          lightbox[0] = color(0,0,v);
          lightbox[1] = color(0);
          lightbox[2] = color(0);
          lightbox[3] = color(v,0,0);
          lightbox[4] = color(0);
          lightbox[5] = color(0);
          break;
      }
    }
  }
  frame++;
}


void mousePressed() {
  if(!round_running) {
    non_round_mode = (non_round_mode + 1) % (LAST_MODE+1);
  }
}
