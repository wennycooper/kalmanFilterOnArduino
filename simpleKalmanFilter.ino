
#include <math.h>

class KalmanFilter
{
   public:

      KalmanFilter(double q,double r);
      double Update(double);
      double GetK(){return k;}

   private:

      double k; //kalman gain
      double p; //estimation error covariance
      double q; //process noise covariance
      double r; //measurement noise covariance
      double x; //value
};

KalmanFilter::KalmanFilter(double q,double r):q(q),r(r),x(0.0)
{
  p = sqrt(q * q + r * r);
}

double KalmanFilter::Update(double value)
{
   p += q;
   k = p / (p + r);
   x += k * (value - x);
   p *= (1 - k);

   return x;
}

void printDouble( double val, byte precision){
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: lcdPrintDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  if(val < 0.0){
    Serial.print('-');
    val = -val;
  }

  Serial.print (int(val));  //prints the int part
  if( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
     mult *=10;

    if(val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;
    while( frac1 /= 10 )
      padding--;
    while(  padding--)
      Serial.print("0");
    Serial.print(frac,DEC) ;
 }
}

float measurements[300];


void setup() {
  // generate raw data

  Serial.begin(57600);
  randomSeed(0);

  unsigned i;

  for (i=0; i<300; i++) {    
    measurements[i] = 100*sin(2*3.1416*0.004*i);
    measurements[i] += random(-10, 10);

    //Serial.print(i);
    //Serial.print(": ");
    //printDouble(measurements[i], 4);
    //Serial.println();
  }

}

void loop() {
  // put your main code here, to run repeatedly:

  KalmanFilter kalman(1.0, 10.0);

  unsigned i;
  float m;
  float km;
  
  for (i=0; i<300; i++)  {
    m = measurements[i];
    km = kalman.Update(m);
    
    //Serial.print(i);
    //Serial.print(": ");
    printDouble(measurements[i], 4);
    Serial.print("    ");
    printDouble(km, 4);
    Serial.println();
    
  }

  while(1);
}
