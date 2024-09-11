void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available()>5){
    byte b1 = Serial.read();
    byte b2 = Serial.read();
    if(b1 == 255 && b2 == 2){
      for (int i = 0 ; i<3; i++){
        value(i) = Serial.read();
      }
      eleased_time = Serial.read();
    }else{
      while(Serial.available())Serial.read();
    }
  }

}
#define d 9.75 // distance bewteen Anchor1 and Anchor2
#define p3_i 0.7 // x cordinate of Anchor3 
#define p3_j 22.31 // y cordinate of Anchor3

void getPosition() {
  //https://en.wikipedia.org/wiki/Trilateration

  float r1 = distToAnchorSmoothed1.get(); // distancesToAnchors[0];
  float r2 = distToAnchorSmoothed2.get(); // distancesToAnchors[1];
  float r3 = distToAnchorSmoothed3.get(); // distancesToAnchors[2];

  if ((r1 < max_distance[0] && r1 > min_distance[0]) && (r2 < max_distance[1] && r2 > min_distance[1]) && (r3 < max_distance[2] && r3 > min_distance[2])) {
    x = ((r1 * r1) - (r2 * r2) + (d * d)) / (2 * d);
    y =  ((r1 * r1) - (r3 * r3) + (p3_i * p3_i) + (p3_j * p3_j)) / (2 * p3_j) - (p3_i * x / p3_j);
    z = sqrt(r1 * r1 - x * x - y * y );
    displayData05();
  } else {
    // display error for debug
    displayData06();
  }



}
