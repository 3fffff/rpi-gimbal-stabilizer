use std::{time::{self, SystemTime, UNIX_EPOCH}, error::Error};

pub struct PID {
    kp: f64, // * (P)roportional Tuning Parameter
    ki: f64, // * (I)ntegral Tuning Parameter
    kd: f64, // * (D)erivative Tuning Parameter
    dir: bool,
    POn: bool,
    lastTime: SystemTime,
    outputSum: f64,
    lastOutput: f64,
    lastInput: f64,
    sampleTime: u128, //default Controller Sample Time is 0.1 seconds
    outMin: f64,
    outMax: f64,
    controlVal: f64,
}

impl PID {
    pub fn new(
        Kp: f64,
        Ki: f64,
        Kd: f64,
        Min: f64,
        Max: f64,
        sampleTime: u128,
        direction: bool,
        POn: bool,
        controlVal: f64,
    ) -> Self{
        /*if (Kp < 0.0 || Ki < 0.0 || Kd < 0.0 || Min >= Max) {
            Err("Params error");
        }*/
        let sample_time_in_sec: f64 = sampleTime as f64 / 1000.0;
        let mut kp = Kp;
        let mut ki = Ki * sample_time_in_sec;
        let mut kd = Kd / sample_time_in_sec;
        if direction {
            kp = -kp;
            ki = -ki;
            kd = -kd;
        }
        Self {
            kp: kp,
            ki: ki,
            kd: kd,
            dir: direction,
            sampleTime: sampleTime,
            POn: POn,
            lastTime: SystemTime::now(),
            outputSum: 0.0,
            lastInput: 0.0,
            outMin: Min,
            outMax: Max,
            controlVal: controlVal,
            lastOutput: 0.0,
        }
    }
    /**
     * The PID will either be connected to a DIRECT acting process (+Output leads
     * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
     * know which one, because otherwise we may increase the output when we should
     * be decreasing.
     *     This, as they say, is where the magic happens.  this function should be called
     *   every time "void loop()" executes.  the function will decide for itself whether a new
     *   pid Output needs to be computed.  returns true when the output is computed,
     *   false when nothing has been done.
     */
    pub fn compute(&mut self,input:&f64,mut out: &mut f64) -> bool {
        let now = SystemTime::now();
        let timeChange = now.duration_since(self.lastTime).unwrap();
        if (timeChange.as_millis() >= self.sampleTime) {
            let mut output: f64=0.0;
            /*Compute all the working error variables*/
            let error = self.controlVal - *input;
            let dInput = *input - self.lastInput;
            self.outputSum += self.ki * error;

            /*Add Proportional on Measurement*/
            if (!self.POn) {
                self.outputSum -= self.kp * dInput;
            } else {
                /*Add Proportional on Error*/
                output = self.kp * error;
            }

            /*saturation */
            if (self.outputSum > self.outMax) {
                self.outputSum = self.outMax;
            } else if (self.outputSum < self.outMin) {
                self.outputSum = self.outMin;
            }

            /*Compute Rest of PID Output*/
            output += self.outputSum - self.kd * dInput;

            if (output > self.outMax) {
                output = self.outMax;
            } else if (output < self.outMin) {
                output = self.outMin;
            }

            /*Remember some variables for next time*/
            self.lastInput = *input;
            self.lastOutput = output;
            self.lastTime = now;
            *out = self.lastOutput;
            return true;
        } else {
            return false;
        }
    }
}
