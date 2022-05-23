use std::time::Duration;

pub struct PID {
    kp: f32,
    ki: f32,
    kd: f32,
    setpoint: f32,
    last_input: f32,
    output_sum: f32,
    error: f32,
    // TODO enum
    p_on_e: bool,
    min: f32,
    max: f32,
}

impl PID {
    pub fn new(kp: f32, ki: f32, kd: f32, p_on_e: bool, setpoint: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            p_on_e,
            setpoint,
            last_input: 0.,
            output_sum: 0.,
            error: 0.,
            min: f32::MIN,
            max: f32::MAX,
        }
    }

    /// Toggle the PID to either a direct acting process (`+output` leads to `+input`)
    /// or a reverse acting process(`+output` leads to `-input`.)
    pub fn reverse(&mut self) {
        self.kp = -self.kp;
        self.ki = -self.ki;
        self.kd = -self.kd;
    }

    /// Tune `kp`, `ki`, and `kd` to adjust the PID's performance after initialization.
    pub fn tune(&mut self, kp: f32, ki: f32, kd: f32, p_on_e: bool, sample_rate: Duration) {
        if kp < 0. || ki < 0. || kd < 0. {
            return;
        }

        self.p_on_e = p_on_e;

        let sample_secs = sample_rate.as_secs() as f32;
        self.kp = kp;
        self.ki = ki * sample_secs;
        self.kd = kd / sample_secs;
    }

    pub fn output(&mut self, input: f32) -> f32 {
        // Compute all the working error variables
        let error = self.setpoint - input;
        let d_input = input - self.last_input;
        self.output_sum += self.ki * self.error;

        self.output_sum = self.output_sum.min(self.max).max(self.min);

        /*Add Proportional on Measurement, if P_ON_M is specified*/
        if !self.p_on_e {
            self.output_sum -= self.kp * d_input;
        }

        /*Add Proportional on Error, if P_ON_E is specified*/
        let mut output = if self.p_on_e { self.kp * error } else { 0. };
        output += self.output_sum - self.kd * d_input;
        output = output.min(self.max).max(self.min);

        output
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
