// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MY_RAD_TO_DEGREE 57.30

#define R 0.03

#define Q 0.000004

//equally the choice is to use rotation sequence Rxyz as that is a pseudo-standard

//X, 0 = Roll, Y, 0 = Elevation, Z, 0 = Azimuth
//static in_t x[3][2];       //state
//static in_t P[3][2][2];   //covariance
//static in_t K[3][2];      //Kalman gain
float x[2][2] = { { 0, 0 }, { 0, 0 } };
float P[2][4] = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };; //realistically a 2x2 matrix
//float K[2][2] = { { 0, 0 }, { 0, 0 } };;
float measuredAngle[2];
sensors_event_t a, g, temp;

Adafruit_MPU6050 mpu;


float zOffset = 0;

void kalman(int i, float dt){
  if(i = 0){
    x[i][0] += dt*(x[i][1] + g.gyro.x * MY_RAD_TO_DEGREE);
  }else{
    x[i][0] += dt*(x[i][1] + g.gyro.y * MY_RAD_TO_DEGREE);
  }
  
//  x[i][1] = x[i][1];

  P[i][0] = P[i][0] + dt*P[i][2] + dt*(P[i][1] + dt*P[i][3]) + Q;
  P[i][1] = P[i][1] + dt*P[i][3];
  P[i][2] = P[i][2] + dt*P[i][3];
//  P[i][3] = P[i][3];

  double recSR = 1/(R + P[i][0] + P[i][1]);
  double K0 = P[i][0] * recSR;
  double K1 = P[i][2] * recSR;
//    K[i][0] = P[i][0] * recSR;
//    K[i][1] = P[i][2] * recSR;

  
  x[i][1] -= K1*(x[i][0] - measuredAngle[i]);
  x[i][0] -= K0*(x[i][0] - measuredAngle[i]);


  //have to e in this order as 2,3 need to use the old values
  P[i][2] -= K1*P[i][0];
  P[i][3] -= K1*P[i][1];

  P[i][0] *= 1 - K0;
  P[i][1] *= 1 - K0;
}


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens


  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.getEvent(&a, &g, &temp);

  zOffset = g.gyro.z;
  delay(100);
}
unsigned long prev = 0;
double azimuth = 0;
void loop() {

//  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long microsDt = (micros()-prev);
  float dt = microsDt*0.000001;

  measuredAngle[0] = atan2(a.acceleration.y, a.acceleration.z) * MY_RAD_TO_DEGREE;
  measuredAngle[1] = atan(-a.acceleration.x / sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * MY_RAD_TO_DEGREE;

  kalman(0, dt);
  kalman(1, dt);
  azimuth += dt*(g.gyro.z-zOffset)*MY_RAD_TO_DEGREE;

  
  Serial.print("dt ");
  Serial.print(dt*1000);
  Serial.print("     ");
  Serial.print("Angle ");
  Serial.print(x[0][0]);
  Serial.print("   ");
  Serial.print(x[1][0]);
  Serial.print("   ");
  Serial.println(azimuth);
  prev = micros();

}
