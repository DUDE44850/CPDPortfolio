#include "mbed.h"
#include "C12832.h"
#include "QEI.h"

#include <string>

typedef enum {flat, flat2, uphill, downhill, off, off2} ProgramState;
ProgramState state = off;

/*
slight oscillatory behaviour at the beginning
could be due to wait statement in initiaialiseer could not be

*/
Serial hm10(PA_11, PA_12);








class Sensor
{
    private:
        AnalogIn sensorIn;
        DigitalOut LEDPin;
        float VDD;
        float output;
        float waitTime; //turn on/off time is 0.25 us for darlington array. Includ
        float samplingPeriod;
        
        Ticker sensorSampler;   //ticker for sampling the sensors

    public:

        Sensor(PinName pinS, PinName pinLED) : sensorIn(pinS), LEDPin(pinLED){
            off(); 
            waitTime = 0.25;
            samplingPeriod = 0.1f;
        }

        void on(void)
        {
           LEDPin = 1; 
        }

        void off(void)
        {
            LEDPin = 0;
        }
        bool test(){
            if(sample() > 0.4){
                return 1;
            }else return 0;

        }
        void sampleOn(){
            sensorSampler.attach(callback(this, &Sensor::sampleUpdate), samplingPeriod);
        }
        void sampleOff(){
            sensorSampler.detach();
        }
        float sample(){
            if(sensorIn < 0.33){
                return 0;
            }else if(sensorIn > 0.8){
                return 0.8f;
            }else{
                return sensorIn;
       }

        }
        void sampleUpdate(){
            output = sensorIn.read();
        }
}; 
Sensor S1(PB_1,PC_13), S2(PC_5, PC_10), S3(PC_2, PC_12), S4(PC_3, PA_13);
Sensor SB(PC_0,PB_15);

string sensorCalc(){
    string SS4 = S4.test() ? "1" : "0";
    string SS3 = S3.test() ? "1" : "0";
    string SSB = SB.test() ? "1" : "0";
    string SS2 = S2.test() ? "1" : "0";
    string SS1 = S1.test() ? "1" : "0";
    return SS4 + SS3 + SSB + SS2 + SS1;
}




class Wheel
{
private:
    static DigitalOut enablePin;
    int distance;
    int prevPulses;
    int nowPulses, pulseDiff;
    float range;
    float reqSpeed;
    float dutyCycle; // Default duty cycle
    float PWMPeriod;
    float variance;
    float dt;

    
    
    Ticker tSpeed;
    QEI Encoder;
    PwmOut motor;
    DigitalOut bipolarPin;

public:
    // Constructor updated to include motor, enable, and bipolar pins
    Wheel(PinName motorPin, PinName pinA, PinName pinB, PinName bipolar) :
    Encoder(pinA, pinB, NC, 1024, QEI::X4_ENCODING), // Assuming the Encoder does not use an index pin
    motor(motorPin),
    bipolarPin(bipolar)
        {
            bipolarPin = 1;
            this->setDutyCycle(0.5f);
            distance = 0;
            PWMPeriod = 25.0f;
            motor.period_us(PWMPeriod);
            variance = 0;
            dt = 1.0f;
        }
    void setTime(float t){
        dt = t;
    }
    void setRange(float r)
    {
        range = r;
    }

    void setSpeed(float s)
    {
        reqSpeed = s;
    }

    void setVariance(float v)
    {
        variance = v;
    }

    int getDistance() 
    {
        distance = Encoder.getPulses();
        return distance;
    }

    float getSpeed() 
    {
        return pulseDiff/dt;
        // Code to calculate speed based on pulses can go here
    }

    // Function to activate the speed control ticker
    void speedOn()
    {
        tSpeed.attach(callback(this, &Wheel::speedUpdate), dt);
    }

    // Function to deactivate the speed control ticker
    void speedOff()
    {
        tSpeed.detach();
    }

    // Speed control function
    void resetEncoder(){
        Encoder.reset();
    }
    int encoderPulse(){
        return Encoder.getPulses();
    }
    void speedUpdate()
    {
        nowPulses = Encoder.getPulses();
        pulseDiff = nowPulses - prevPulses;
        prevPulses = nowPulses;
    }

    // Functions to get and set the duty cycle of the motor
    void setDutyCycle(float duty)
    {
        dutyCycle = duty;
        motor.write(dutyCycle); // Adjust the motor PWM duty cycle
    }

    float getDutyCycle() const
    {
        return dutyCycle;
    }

    // Enable or disable the motor
    static void enableMotor(bool enable)
    {
        enablePin = enable ? 1 : 0;
    }

    // Set the motor direction
    void setBipolar(bool direction)
    {
        bipolarPin = direction ? 1 : 0;
    }
};
Wheel L(PC_8, PA_15, PB_7, PD_2), R(PC_6, PB_13, PB_14, PB_12);
DigitalOut Wheel::enablePin(PB_2);
//DigitalOut enable(PB_2);


class PIDcontroller
{
    private:
        float Kp, Ki, Kd;
        float error, prevError, integral, derivative;
        float output;
        float dt;
        float maxOutput, minOutput;
        float maxIntegral, minIntegral;
        float processVariable;
        Ticker computeTick;
    public:
    
        PIDcontroller(float Kp, float Ki, float Kd, float dt, float maxOutput, float minOutput, float maxIntegral, float minIntegral)
        {
            this->Kp = Kp;
            this->Ki = Ki;
            this->Kd = Kd;
            this->dt = dt;
            this->maxOutput = maxOutput;
            this->minOutput = minOutput;
            this->maxIntegral = maxIntegral;
            this->minIntegral = minIntegral;
        }
        void setKp(float Kp)
        {
            this->Kp = Kp;
        }
        void setKi(float Ki)
        {
            this->Ki = Ki;
        }
        void setKd(float Kd)
        {
            this->Kd = Kd;
        }
        void setDt(float dt)
        {
            this->dt = dt;
        }
        void setMaxOutput(float maxOutput)
        {
            this->maxOutput = maxOutput;
        }
        void setMinOutput(float minOutput)
        {
            this->minOutput = minOutput;
        }
        void setMaxIntegral(float maxIntegral)
        {
            this->maxIntegral = maxIntegral;
        }
        void setMinIntegral(float minIntegral)
        {
            this->minIntegral = minIntegral;
        }
        
        void setProcessVariable(float processVariable)
        {
            this->processVariable = processVariable;
        }
        void computeToggle(bool tog){
            if(tog){
                computeTick.attach(callback(this, &PIDcontroller::compute),dt);
            }else{
                computeTick.detach();
            }
        }
        void compute()
        {
            string ptr = sensorCalc();
            if(ptr == "00100" || ptr == "01110" || ptr == "01010"){
                error = 0;
            }else if(ptr == "01100"){
                error = -0.5;
            }else if (ptr == "00110"){
                error = 0.5;
            }else if(ptr == "01000"){
                error = -1.;
            }else if(ptr == "00010"){
                error = 1.;
            }else if(ptr == "11000"){
                error = -2.5;
            }else if(ptr == "10000"){
                error = -3;
            }else if(ptr == "00011"){
                error = 2.5;
            }else if(ptr == "00001"){
                error = 3;
            }else if(ptr == "00000"){
                error = (error > 0) ? 5 : -5;
            }
            integral += error * dt;
            derivative = (error - prevError) / dt;
            output = Kp * error + Ki * integral + Kd * derivative;
            if(output > maxOutput)
            {
                output = maxOutput;
            }else if(output < minOutput)
            {
                output = minOutput;
            }
            if(integral > maxIntegral)
            {
                integral = maxIntegral;
            }else if(integral < minIntegral)
            {
                integral = minIntegral;
            }
            prevError = error;
            
        }
        float getOutput()
        {
            return output;
        }
        float getError()
        {
            return error;
        }
        float getIntegral()
        {
            return integral;
        }
        float getDerivative()
        {
            return derivative;
        }
};

PIDcontroller controlLoop(0.5, 0.02, 0.05, 0.1, 1000, -1000, 10, 1); //flat Kp=0.5 up_slope Kp=0.2 down_slope Kp=0.5
float linearSpeed = 0.7f;// flat 0.7f up_slope 0.85f down_slope 0.6f
Ticker turnUpdater, speedUpdater;

int targetSpeed = 1700;
void linearSpeedUpdate(){
    float avgSpeed = (L.getSpeed() + R.getSpeed()) * 0.5f;
    if(avgSpeed < targetSpeed ){
        linearSpeed += 0.01f;
    }else{
        linearSpeed -= 0.01f;
    }   
    if (linearSpeed >= 0.9f){
        linearSpeed = 0.9f;
    }
}
void turnUpdate(){   
        L.setDutyCycle(linearSpeed + controlLoop.getOutput()* 0.05f);
        R.setDutyCycle(linearSpeed - controlLoop.getOutput()* 0.05f);

}


void firePressedISR()   //Interupt state routine for pressing button to switch between motor states
{
    switch(state)
    {
        case off:
            state = off2;
            
            break;
        default:
            break;
        
    }
};
int main()
{



    C12832 testLCD(D11, D13, D12, D7, D10);
    InterruptIn joyFire(D4);
   /* , joyDown(A3), joyUp(A2);
    InterruptIn encoderPulse(PC_4);
    joyUp.rise(&upPressedISR);
    joyDown.rise(&downPressedISR);*/
    joyFire.rise(&firePressedISR);
    DigitalOut redLED(D5), blueLED(D8);
    blueLED = 0;
    redLED = 1;
    float setRMotor, setLMotor;

    //ProgramState lastState = state;
    
    Wheel::enableMotor(1);
    
    
    L.setRange(3);
    L.setSpeed(30);
    R.setRange(3);
    R.setSpeed(30);
    R.setTime(0.02f);
    L.setTime(0.02f);
    R.speedOn();
    L.speedOn();
    float testPWM = 0.5f;// flat 0.7f up_slope 0.85f down_slope 0.6f
    L.setDutyCycle(testPWM);
    R.setDutyCycle(testPWM);


    

    controlLoop.setDt(0.001f);

    controlLoop.computeToggle(1);
    S1.on();
    S2.on();
    S3.on();
    S4.on();
    SB.on();
    char c;
    
    
    while(1)
    {   
        if(sensorCalc() == "00000"){
                controlLoop.computeToggle(0);
                Wheel::enableMotor(0);
            }else{
                controlLoop.computeToggle(1);
                Wheel::enableMotor(1);
            }
        switch(state)
    {
        case off:
        testLCD.locate(0,20);
        testLCD.printf("State: Off");
            break;
        case off2:
        testLCD.locate(0,20);
        testLCD.printf("State: Off");
        L.setDutyCycle(0.7f);
        R.setDutyCycle(0.7f);
        turnUpdater.attach(&turnUpdate ,0.001);
        speedUpdater.attach(&linearSpeedUpdate, 0.05); 
        state = flat; 
            break;  
        default:
            testLCD.locate(0,20);
            testLCD.printf("State: On");
           break; 
            }
        if(hm10.readable()){
                c = hm10.getc();
                if(c =='B'){
                    controlLoop.computeToggle(0);
                    Wheel::enableMotor(0);

                }else if(c == 'A'){
                    turnUpdater.detach();
                    Wheel::enableMotor(1);
                    speedUpdater.detach();
                    L.resetEncoder();
                    R.resetEncoder();

                    L.setDutyCycle(0.3f);
                    R.setDutyCycle(0.7f);

                    int encoderPulseCountR = 0;
                    int encoderPulseCountL = 0;

                    while(encoderPulseCountL < 1400)
                        {
                            encoderPulseCountL = L.encoderPulse() * -1 ;
                        }

                    L.setDutyCycle(0.5f);
                    R.setDutyCycle(0.5f);
                    turnUpdater.attach(&turnUpdate ,0.001);
                    speedUpdater.attach(&linearSpeedUpdate, 0.05); 

                    controlLoop.computeToggle(1);
                }

            
            
        
    }
        
        testLCD.locate(0,0);
        testLCD.printf("Lspd: %6.2f  Rspd: %6.2f",L.getSpeed(),R.getSpeed());
        testLCD.locate(0,10);
        testLCD.printf("PWM: %3.2f  Error: %3.2f",linearSpeed, controlLoop.getOutput());
        
        

        
        
        
       };
};