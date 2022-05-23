pub struct PID {
    kp: f32,
    ki: f32,
    kd: f32,
    last_input: f32,
    output_sum: f32,
    error: f32,
    min: f32,
    max: f32,
}

impl Default for PID {
    fn default() -> Self {
        Self::new(0., 0., 0.)
    }
}

impl PID {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            last_input: 0.,
            output_sum: 0.,
            error: 0.,
            min: f32::MIN,
            max: f32::MAX,
        }
    }

    pub fn new_with_sample_rate(kp: f32, ki: f32, kd: f32, sample_rate: f32) -> Self {
        let mut pid = Self::default();
        pid.tune(kp, ki, kd, sample_rate);
        pid
    }

    /// Returns the proportional constant of the PID
    pub fn kp(&self) -> f32 {
        self.kp
    }

    /// Returns the integral constant of the PID
    pub fn ki(&self) -> f32 {
        self.ki
    }

    /// Returns the derivative constant of the PID
    pub fn kd(&self) -> f32 {
        self.kd
    }

    /// Returns the minimum output value of the PID
    pub fn min(&self) -> f32 {
        self.min
    }

    /// Returns the maximum output value of the PID
    pub fn max(&self) -> f32 {
        self.max
    }

    /// Tune `kp`, `ki`, and `kd` to adjust the PID's performance after initialization.
    pub fn tune(&mut self, kp: f32, ki: f32, kd: f32, sample_rate: f32) {
        if kp >= 0. && ki >= 0. && kd >= 0. {
            self.kp = kp;
            self.ki = ki * sample_rate;
            self.kd = kd / sample_rate;
        }
    }

    /// Set the minimum output value of this PID
    pub fn set_min(&mut self, min: f32) {
        self.min = min;
        self.output_sum = self.output_sum.max(self.min);
    }

    /// Set the maximum output value of this PID
    pub fn set_max(&mut self, max: f32) {
        self.max = max;
        self.output_sum = self.output_sum.min(self.max);
    }

    /// Toggle the PID to either a direct acting process (`+output` leads to `+input`)
    /// or a reverse acting process(`+output` leads to `-input`.)
    pub fn reverse(&mut self) {
        self.kp = -self.kp;
        self.ki = -self.ki;
        self.kd = -self.kd;
    }

    /// Calculate the PID output with the proportional added on measurement
    pub fn output(&mut self, input: f32, setpoint: f32) -> f32 {
        self.output_inner(input, setpoint, false)
    }

    /// Calculate the PID output with the proportional added on error
    pub fn output_on_error(&mut self, input: f32, setpoint: f32) -> f32 {
        self.output_inner(input, setpoint, true)
    }

    fn output_inner(&mut self, input: f32, setpoint: f32, add_on_error: bool) -> f32 {
        // Compute all the working error variables
        let error = setpoint - input;
        let d_input = input - self.last_input;
        self.output_sum += self.ki * self.error;

        self.output_sum = self.output_sum.min(self.max).max(self.min);

        /*Add Proportional on Measurement, if P_ON_M is specified*/
        if !add_on_error {
            self.output_sum -= self.kp * d_input;
        }

        /*Add Proportional on Error, if P_ON_E is specified*/
        let mut output = if add_on_error { self.kp * error } else { 0. };
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
