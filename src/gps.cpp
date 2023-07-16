#include "gps.h"
#include "TinyGPSPlus.h"

TinyGPSPlus gps;
ICM20948_WE myIMU(0x55);
static void smartDelay(unsigned long ms);


void gps_setup(){
    Wire.begin();
    SerialUSB.begin(115200);
    Serial1.begin(9600);
    while(!SerialUSB) {}
    while(!myIMU.init()){
        SerialUSB.println("ICM20948 does not respond");
        delay(100);
    }

    SerialUSB.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  SerialUSB.println("Done!"); 
  
  /* enables or disables the acceleration sensor, default: enabled */
  // myIMU.enableAcc(true);

  /*  ICM20948_ACC_RANGE_2G      2 g   (default)
   *  ICM20948_ACC_RANGE_4G      4 g
   *  ICM20948_ACC_RANGE_8G      8 g   
   *  ICM20948_ACC_RANGE_16G    16 g
   */
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  
  /*  Choose a level for the Digital Low Pass Filter or switch it off.  
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *  
   *  IMPORTANT: This needs to be ICM20948_DLPF_7 if DLPF is used in cycle mode!
   *  
   *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
   *    0              246.0               1125/(1+ASRD) 
   *    1              246.0               1125/(1+ASRD)
   *    2              111.4               1125/(1+ASRD)
   *    3               50.4               1125/(1+ASRD)
   *    4               23.9               1125/(1+ASRD)
   *    5               11.5               1125/(1+ASRD)
   *    6                5.7               1125/(1+ASRD) 
   *    7              473.0               1125/(1+ASRD)
   *    OFF           1209.0               4500
   *    
   *    ASRD = Accelerometer Sample Rate Divider (0...4095)
   *    You achieve lowest noise using level 6  
   */
  myIMU.setAccDLPF(ICM20948_DLPF_6);    
  
  /*  Acceleration sample rate divider divides the output rate of the accelerometer.
   *  Sample rate = Basic sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is not off!
   *  Divider is a number 0...4095 (different range compared to gyroscope)
   *  If sample rates are set for the accelerometer and the gyroscope, the gyroscope
   *  sample rate has priority.
   */
  myIMU.setAccSampleRateDivider(10);
}

void gps_loop(){
    myIMU.readSensor();

//   xyzFloat gValue = myIMU.getGValues();
//   xyzFloat angle = myIMU.getAngles();
  
// /* For g-values the corrected raws are used */
//   SerialUSB.print("g-x      = ");
//   SerialUSB.print(gValue.x);
//   SerialUSB.print("  |  g-y      = ");
//   SerialUSB.print(gValue.y);
//   SerialUSB.print("  |  g-z      = ");
//   SerialUSB.println(gValue.z);

// /* Angles are also based on the corrected raws. Angles are simply calculated by
//    angle = arcsin(g Value) */
//   SerialUSB.print("Angle x  = ");
//   SerialUSB.print(angle.x);
//   SerialUSB.print("  |  Angle y  = ");
//   SerialUSB.print(angle.y);
//   SerialUSB.print("  |  Angle z  = ");
//   SerialUSB.println(angle.z);

//   SerialUSB.print("Orientation of the module: ");
//   SerialUSB.println(myIMU.getOrientationAsString());
SerialUSB.print("Latitude: ");
SerialUSB.print(gps.location.lat(),10);
SerialUSB.print(" Longitude: ");
SerialUSB.print(gps.location.lng(),10);
SerialUSB.print(" GPS satellites: ");
SerialUSB.print(gps.satellites.value());
SerialUSB.print(" Valid: ");
SerialUSB.println(gps.satellites.isValid());

smartDelay(1000);
//   SerialUSB.print(Serial1.read());

//   delay(1000);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}