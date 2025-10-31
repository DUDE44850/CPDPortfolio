#include "mbed.h"
#include "C12832.h" //http://os.mbed.com/users/askksa12543/code/C12832/

typedef enum {homeState, setTime, currentTime, worldTime, stopwatchState, stopwatchChange, timerChange, timerState, timerDone} ProgramState;
ProgramState state = homeState;

class LED{
protected:                                          //Protected (Private) data member declaration
    DigitalOut outputSignal;                        //Declaration of DigitalOut object
    bool status; 
    Ticker LEDTicker;                                   //Variable to recall the state of the LED
public:                                             //Public declarations
    LED(PinName pin) : outputSignal(pin){off();}
    void on(void)                                   //Public member function for turning the LED on
    {
        outputSignal = 0;                           //Set output to 0 (LED is active low)
        status = true;                              //Set the status variable to show the LED is on
    }
    void off(void)                                  //Public member function for turning the LED off
    {
        outputSignal = 1;                           //Set output to 1 (LED is active low)
        status = false;  
        LEDTicker.detach();                           //Set the status variable to show the LED is off
    }
    void toggle(void)                               //Public member function for toggling the LED
    {
        if (status){                                 //Check if the LED is currently on
            off();                                  //Turn off if so
        }else{                                    //Otherwise...
            on(); 
        }                                  //Turn the LED on
    }
    bool getStatus(void)                            //Public member function for returning the status of the LED
    {
        return status;                              //Returns whether the LED is currently on or off
    }
};
class Speaker {
private:
    DigitalOut outputSignal;
    bool state; // Can be set to either 1 or 0 to record output value
    Ticker oscillateTicker;
public:
    Speaker(PinName pin)
        :outputSignal(pin), state('0'){}
    void on(void){
        outputSignal = 1;
        state = 1;
    };
    void off(void){
        outputSignal = 0;
        state = 0;
    };
    void toggle(void){
        if (state){
            off();
        }else{
            on();
        }
    };
    bool getStatus(void)                            //Public member function for returning the status of the LED
    {
        return state;                              //Returns whether the LED is currently on or off
    };
    void oscillateOn(float frequency){
        float period = 1/frequency;
        oscillateTicker.attach(callback(this, &Speaker::toggle), period);
    }
    void oscillateOff(){
        oscillateTicker.detach();
    }
}; 
class Clock{
    private:
        int totalTime;
        Ticker clockTick;
    public:    
        Clock(){
            clockTick.attach(callback(this, &Clock::tick), 1);
            reset();
        };
        void reset(){
            totalTime = 0;
        };
        void tick(){
            totalTime++;
        };
        void setClock(int setHours, int setMinutes){
            totalTime = (setHours * 60 * 60) + (setMinutes * 60);
        };
        int getHours(){
            int hours = ((totalTime / 60) / 60) % 24;
            return hours;
        };
        int getShiftHours(float shift){
            int hours = (((totalTime / 60) / 60) + static_cast<int>(shift)) % 24;
            return hours;
        }
        int getMins(){
            int minutes = (totalTime / 60) % 60;
            return minutes;
        };
        int getShiftMins(float shift){
            int minutes = ((totalTime / 60) + static_cast<int>(shift * 60)) % 60;
            return minutes;
        }
        int getSecs(){
            int seconds = totalTime % 60;
            return seconds;
        };  
};
void timerDoneISR(){
    state = timerDone;
}
class Countdown{
    private:
        Timeout countdownTimer;
        int totalTime;
        Ticker clockTick;
        bool state;
    public:
        Countdown(){
            reset();
        };
        void start(float countdownTime){
            clockTick.attach(callback(this, &Countdown::tick), 1);
            countdownTimer.attach(callback(this, &Countdown::timerFinished), countdownTime);
            state = 1;
        };
        bool getState(){
            return state;
        }
        void tick(){
                totalTime++;
            };
        int read(int countdownTime){
            return countdownTime - totalTime;
        };
        void reset(){
            clockTick.detach();
            countdownTimer.detach();
            totalTime = 0;
            state = 0;
        };
        void timerFinished(){
            reset();
            state = 1;
            timerDoneISR();
        }
};
class Potentiometer                                 //Begin Potentiometer class definition
{
private:                                            //Private data member declaration
    AnalogIn inputSignal;                           //Declaration of AnalogIn object
    float VDD, currentSampleNorm, currentSampleVolts;
public:                                             // Public declarations
    Potentiometer(PinName pin, float v) : inputSignal(pin), VDD(v) {}                                                                       //VDD is also provided to determine maximum measurable voltage
    float amplitudeVolts(void)                      //Public member function to measure the amplitude in volts
    {
        return (inputSignal.read()*VDD);            //Scales the 0.0-1.0 value by VDD to read the input in volts
    }
    float amplitudeNorm(void)                       //Public member function to measure the normalised amplitude
    {
        return inputSignal.read();                  //Returns the ADC value normalised to range 0.0 - 1.0
    }
    void sample(void)                               //Public member function to sample an analogue voltage
    {
        currentSampleNorm = inputSignal.read();       //Stores the current ADC value to the class's data member for normalised values (0.0 - 1.0)
        currentSampleVolts = currentSampleNorm * VDD; //Converts the normalised value to the equivalent voltage (0.0 - 3.3 V) and stores this information
    }
    float getCurrentSampleVolts(void)               //Public member function to return the most recent sample from the potentiometer (in volts)
    {
        return currentSampleVolts;                  //Return the contents of the data member currentSampleVolts
    }
    float getCurrentSampleNorm(void)                //Public member function to return the most recent sample from the potentiometer (normalised)
    {
        return currentSampleNorm;                   //Return the contents of the data member currentSampleNorm  
    }
};
class SamplingPotentiometer : public Potentiometer {
private:
 float samplingFrequency, samplingPeriod;
 Ticker sampler;
public:
 SamplingPotentiometer(PinName p, float v, float fs): Potentiometer(p,v){
     
     samplingFrequency = fs;
     samplingPeriod = 1/fs;
     sampler.attach(callback(this, &SamplingPotentiometer::sample),fs);
 };
};
void firePressedISR(){
    switch(state){
        case homeState:
            state = setTime;
            break;
        case setTime:
            state = homeState;
            break;
        case currentTime:// nothing remove case statement
            break;
        case worldTime:// only p1 affects the condition of worldTime
            break;
        case stopwatchState:
                state = stopwatchChange;
            break;
        case timerState:
            state = timerChange;
            break;
        case timerDone:
            state = timerChange;
            break;
    }
};
void upPressedISR(){
    switch(state){
        case homeState:
            state = timerState;
            break;
        case setTime:
            break;
        case currentTime:
            state = homeState;
            break;
        case worldTime:
            state = currentTime;
            break;
        case stopwatchState:
            state = worldTime;
            break;
        case timerState:
            state = stopwatchState;
            break;
        case timerDone:
            break;
        
    }
};

void downPressedISR(){
    switch(state){
        case homeState:
            state = currentTime;
            break;
        case setTime:
            break;
        case currentTime:
            state = worldTime;
            break;
        case worldTime:
            state = stopwatchState;
            break;
        case stopwatchState:
            state = timerState;
            break;
        case timerState:
            state = homeState;
            break;
        case timerDone:
            break;
    }
};


int main() {
    C12832 testLCD(D11, D13, D12, D7, D10);

    InterruptIn joyFire(D4), joyDown(A3), joyUp(A2);

    LED timerLED(D9);
    LED stopwatchLED(D8);

    joyUp.rise(&upPressedISR);
    joyDown.rise(&downPressedISR);
    joyFire.rise(&firePressedISR);
     
    Speaker testSpeaker(D6);

    SamplingPotentiometer P1(A0, 3.3, 100), P2(A1, 3.3, 100);

    Clock worldClock;
    Countdown countdownTimer;
    Timer stopwatch;
    Ticker blinkLED, oscillateSpeaker;
    char cities[32][25] = {"Alofi (GMT-11)", "Honolulu (GMT-10)", "Taiohae (GMT-9.5)","Badger (GMT-9)","Tijuana (GMT-8)","Idaho Falls (GMT-7)", "Chihuahua (GMT-6)","Grand Rapids (GMT-5)","Piggotts (GMT-4)","Mount Pearl (GMT-3.5)","Nuuk (GMT-3)","Grytviken (GMT-2)","Mindelo (GMT-1)","Freetown (GMT+0)","Brazzaville (GMT+1)","Lilongwe (GMT+2)","Damascus (GMT+3)","Karaj (GMT+3.5)","Ganja (GMT+4)","Kabul (GMT+4.5)","Oral (GMT+5)","Bengaluru (GMT+ 5.5)","Kathmandu (GMT+5.75)","Bishek (GMT+6)","Mandalay (GMT+6.5)","Bangkok (GMT+7)","Kuala Lumpur (GMT+8)","Pyongyang (GMT+9)","Broken Hill (GMT+9.5)","Yigo Village (GMT+10)","Buka (GMT+11)","Funafuti (GMT+12)"};
    float timeShift[32] = {13,14,14.5,15,16,17,18,19,20,20.5,21,22,23,0,1,2,3,3.5,4,4.5,5,5.5,5.75,6,6.5,7,8,9,9.5,10,11,12};

    bool stopwatchOn = 0;

    float frequency = 500;

    float lastTime = 0.00;
    int setMins, setHours, countMins, countSecs, countTime, i;
    ProgramState lastState = state;
    while(1) {
        if(state != lastState){
            testLCD.fillrect(1, 1, 127, 31, 0);
        }
        switch(state){
            case homeState:
                lastState = state;
                testLCD.locate(0,0);
                testLCD.printf("Press Fire to set time");
                testLCD.locate(0,10);
                testLCD.printf("%02d:%02d:%02d", worldClock.getHours(), worldClock.getMins(), worldClock.getSecs());
                
                break;
            case setTime:
                lastState = state;
                setMins = P2.amplitudeNorm() * 59.9;
                setHours = P1.amplitudeNorm() * 23.9;
                worldClock.setClock(setHours, setMins);
                testLCD.locate(0,0);
                testLCD.printf("Set new time (HH MM)");
                testLCD.locate(0,10);
                testLCD.printf("%02d:%02d", setHours, setMins);
                
                break;
            case currentTime:
                lastState = state;
                testLCD.locate(0,0);
                testLCD.printf("Current Time:");
                testLCD.locate(0,10);
                testLCD.printf("%02d:%02d:%02d", worldClock.getHours(), worldClock.getMins(), worldClock.getSecs());
                break;
            case worldTime:
                lastState = state;
                
                i = static_cast<int>(P1.amplitudeNorm() * 31.1);
                
                testLCD.locate(0,0);
                testLCD.printf("%s             ",cities[i]);
                testLCD.locate(0,10);
                testLCD.printf("%02d:%02d:%02d", worldClock.getShiftHours(timeShift[i]), worldClock.getShiftMins(timeShift[i]), worldClock.getSecs());
                testLCD.locate(0,20);
                testLCD.printf("%02d:%02d:%02d  (Manchester)", worldClock.getHours(), worldClock.getMins(), worldClock.getSecs());
                
                break;
            case stopwatchState:
                lastState = state;
                if(stopwatch.read() == 0){
                    testLCD.locate(0,0);
                    testLCD.printf("Stopwatch: Inactive");
                    testLCD.locate(0,10);
                    testLCD.printf("Last Time. %.2f s", lastTime);
                }else{
                    testLCD.locate(0,0);
                    testLCD.printf("Stopwatch: Running");
                    testLCD.locate(0,10);
                    testLCD.printf("Time %.2f s", stopwatch.read());
                }
                break;
            case stopwatchChange:
                if(stopwatch.read() == 0){
                    stopwatch.start();
                    stopwatchLED.on();
                    stopwatchOn = 1;
                }else{
                    stopwatch.stop();
                    stopwatchLED.off();
                    lastTime = stopwatch.read();
                    stopwatch.reset();
                    stopwatchOn = 0;
                }
                state = stopwatchState;
                break;
            
            case timerState:
                lastState = state;
                if (countdownTimer.getState()){
                    testLCD.locate(0,0);
                    testLCD.printf("Countdown Timer: Running");
                    testLCD.locate(0,10);
                    testLCD.printf("%03d  /  %03d s", countdownTimer.read(countTime) , countTime);
                }else{
                    countMins = P1.amplitudeNorm() * 59.9;
                    countSecs = P2.amplitudeNorm() * 59.9;
                    countTime = ((countMins*60)+countSecs);
                    testLCD.locate(0,0);
                    testLCD.printf("Set Countdown Period.");
                    testLCD.locate(0,10);
                    testLCD.printf("%02d:%02d", countMins, countSecs);
                }
                break;
            case timerChange:
                testSpeaker.off();
                if (countdownTimer.getState()){
                    
                    timerLED.off();
                    oscillateSpeaker.detach();
                    testSpeaker.off();
                    countdownTimer.reset();
                    
                }else{
                    countdownTimer.reset();
                    blinkLED.attach(callback(&timerLED, &LED::toggle), 1);
                    countdownTimer.start(((countMins*60)+countSecs));
                    
                }
                state = timerState;
                break;
            case timerDone:
                lastState = state;
                testLCD.locate(0,0);
                testLCD.printf("Time Period Elapsed");
                blinkLED.detach();
                timerLED.on();
                oscillateSpeaker.attach(callback(&testSpeaker, &Speaker::toggle), 0.5/frequency);
                break;
            default:
                state = homeState;
        }
        
    }
}


