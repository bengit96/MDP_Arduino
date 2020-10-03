
#ifdef Arduino
  #include "Arduino.h"
#elif defined(SPARK)
  #include "Particle.h"
  #include "math.h"
#endif
#include "SharpIR.h"

// Initialisation function
//  + irPin : is obviously the pin where the IR sensor is attached
//  + sensorModel is a int to differentiate the two sensor models this library currently supports:
//    > 1080 is the int for the GP2Y0A21Y and 
//    > 20150 is the int for GP2Y0A02YK and 
//    > 100500 is the long for GP2Y0A710K0F
//    The numbers reflect the distance range they are designed for (in cm)
SharpIR::SharpIR(int irPin, long sensorModel) {
  
    _irPin=irPin;
    _model=sensorModel;
    
    // Define pin as Input
    pinMode (_irPin, INPUT);
    
    #ifdef ARDUINO
      analogReference(DEFAULT);
    #endif
}

void SharpIR::sort(int a[], int size){
    int temp;
    for (int i=0; i<size; i++){
      for (int j=i; j>0; j--){
        if (a[j] < a[j-1]){
          temp = a[j];
          a[j] = a[j-1];
          a[j-1] = temp;
        }
        else
          break;
      }
    }
}
/*
// Sort an array
void SharpIR::sort(int a[], int size) {
    for(int i=0; i<(size-1); i++) {
        bool flag = true;
        for(int o=0; o<(size-(i+1)); o++) {
            if(a[o] > a[o+1]) {
                int t = a[o];
                a[o] = a[o+1];
                a[o+1] = t;
                flag = false;
            }
        }
        if (flag) break;
    }
}
*/
// Read distance and compute it
float SharpIR::distance() {

    int ir_val[NB_SAMPLE];
    float distanceCM;
    float current;


    for (int i=0; i<NB_SAMPLE; i++){
        // Read analog value
        ir_val[i] = analogRead(_irPin);
    }
    
    // Sort it 
    sort(ir_val,NB_SAMPLE);

    
    if (_model==1080) {
        
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
          //distanceCM = 29.988 * pow(map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000)/1000.0, -1.173);
          distanceCM = (float)4800/(ir_val[NB_SAMPLE / 2] - 20);

    } else if (_model==20150){

        // Previous formula used by  Dr. Marcal Casas-Cartagena
        // puntualDistance=61.573*pow(voltFromRaw/1000, -1.1068);
        
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
          distanceCM = 60.374 * pow(map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000)/1000.0, -1.16);
          //distanceCM = ir_val[NB_SAMPLE / 2];
          //distanceCM = (float)9462/(ir_val[NB_SAMPLE / 2] - 16.92);
          //distanceCM = 61.573 * pow(map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000)/1000.0, -1.1068);


    }

    return distanceCM;
}

float SharpIR::median_Voltage_Sampling() {
  
  int ir_val[NB_SAMPLE];
  float voltage;

  for (int i=0; i<NB_SAMPLE; i++){
    // Read analog value
    ir_val[i] = analogRead(_irPin);
    //Serial.println(ir_val[i]);
  }

  sort(ir_val,NB_SAMPLE);

  return voltage = map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000)/1000.0;
  
}
