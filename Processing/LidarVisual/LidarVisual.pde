import processing.serial.*;
Serial lidarSerial;

int sectors = 12;
int sectorWight = 520;
int sectorHeght = 520;

int lf = 10;    // Linefeed in ASCII

void setup() {
  size(700, 700, P2D);
  lidarSerial = new Serial(this, Serial.list()[2], 115200);
  lidarSerial.clear();
}

void draw() {
  while (lidarSerial.available() > 0) {
    background(0);
    String inBuffer = lidarSerial.readStringUntil(lf);

    if (inBuffer != null) {
      //println(inBuffer);
      String[] valuesLidar = split(inBuffer, ' ');
      if (valuesLidar.length<sectors) {
        continue;
      }
      // printArray(valuesLidar);
      for (int i=0; i < sectors; i++) {
        switch (valuesLidar[i]) {
        case "0":
          fill(#35E800);
          break;
        case "1":
          fill(#FCE50D);
          break;
        case "2":
          fill(#C62A0E);
          break;
        }


        arc(width/2, height/2, sectorWight, sectorHeght, PI/2+(TWO_PI/12*i), PI/2+ (TWO_PI/12*(i+1)), PIE);
      }
    }
  }
}
