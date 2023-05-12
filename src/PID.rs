use std::time::SystemTime;

pub struct PID {
    kp: f64, // * (P)roportional Tuning Parameter
    ki: f64, // * (I)ntegral Tuning Parameter
    kd: f64, // * (D)erivative Tuning Parameter
    POn: bool,
    prev_time: SystemTime,
    output_integ: f64,
    prev_output: f64,
    prev_input: f64,
    sample_time: u128, //default Controller Sample Time is 0.1 seconds
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
        sample_time: u128,
        POn: bool,
        controlVal: f64,
    ) -> Self {
        /*if (Kp < 0.0 || Ki < 0.0 || Kd < 0.0 || Min >= Max) {
            Err("Params error");
        }*/
        let sample_time_in_sec: f64 = sample_time as f64 / 1000.0;
        let mut kp = Kp;
        let mut ki = Ki * sample_time_in_sec;
        let mut kd = Kd / sample_time_in_sec;
        Self {
            kp: kp,
            ki: ki,
            kd: kd,
            sample_time: sample_time,
            POn: POn,
            prev_time: SystemTime::now(),
            output_integ: 0.0,
            prev_input: 0.0,
            outMin: Min,
            outMax: Max,
            controlVal: controlVal,
            prev_output: 0.0,
        }
    }

    pub fn reset_integral(&mut self){
        self.output_integ = 0.0;
    }

    pub fn updateGains(&mut self, Kp: f64, Ki: f64, Kd: f64) {
        let sample_time_in_sec: f64 = self.sample_time as f64 / 1000.0;
        self.kp = Kp;
        self.ki = Ki * sample_time_in_sec;
        self.kd = Kd / sample_time_in_sec;
    }
    /*
     *     This, as they say, is where the magic happens.  this function should be called
     *   every time "void loop()" executes.  the function will decide for itself whether a new
     *   pid Output needs to be computed.  returns true when the output is computed,
     *   false when nothing has been done.
     */
    pub fn compute(&mut self, input: &f64, mut out: &mut f64) -> bool {
        let now = SystemTime::now();
        let timeChange = now.duration_since(self.prev_time).unwrap();
        if (timeChange.as_millis() >= self.sample_time) {
            let mut output: f64 = 0.0;
            /*Compute all the working error variables*/
            let error = self.controlVal - *input;
            let dInput = *input - self.prev_input;
            self.output_integ += self.ki * error;

            /*Add Proportional on Measurement*/
            if (!self.POn) {
                self.output_integ -= self.kp * dInput;
            } else {
                /*Add Proportional on Error*/
                output = self.kp * error;
            }

            /*saturation */
            if (self.output_integ > self.outMax) {
                self.output_integ = self.outMax;
            } else if (self.output_integ < self.outMin) {
                self.output_integ = self.outMin;
            }

            /*Compute Rest of PID Output*/
            output += self.output_integ - self.kd * dInput;

            if (output > self.outMax) {
                output = self.outMax;
            } else if (output < self.outMin) {
                output = self.outMin;
            }

            /*Remember some variables for next time*/
            self.prev_input = *input;
            self.prev_output = output;
            self.prev_time = now;
            *out = self.prev_output;
            return true;
        } else {
            return false;
        }
    }
}

enum PIDError {
    ParamsError,
}
