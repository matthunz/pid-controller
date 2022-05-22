use std::time::{Duration, SystemTime};

pub struct PID {
    kp: f32,
    ki: f32,
    kd: f32,
    setpoint: f32,
    last_input: f32,
    last_time: SystemTime,
    sample_rate: Duration,
    output_sum: f32,
    error: f32,
    // TODO enum
    p_on_e: bool,
    min: f32,
    max: f32,
}

impl PID {
    pub fn new(kp: f32, ki: f32, kd: f32, p_on_e: bool) -> Self {
        Self {
            kp,
            ki,
            kd,
            p_on_e,
            setpoint: 0.,
            last_input: 0.,
            last_time: SystemTime::now(),
            output_sum: 0.,
            error: 0.,
            sample_rate: Duration::ZERO,
            min: f32::MIN,
            max: f32::MAX,
        }
    }

    pub fn tune(&mut self, kp: f32, ki: f32, kd: f32, p_on_e: bool) {
        if kp < 0. || ki < 0. || kd < 0. {
            return;
        }

        self.p_on_e = p_on_e;

        let sample_secs = self.sample_rate.as_secs() as f32;
        self.kp = kp;
        self.ki = ki * sample_secs;
        self.kd = kd / sample_secs;
    }

    pub fn output(&mut self, input: f32) -> Option<f32> {
        let now = SystemTime::now();
        let time_change = now.duration_since(self.last_time).unwrap();
        self.last_time = now;

        if time_change > self.sample_rate {
            // Compute all the working error variables
            let error = self.setpoint - input;
            let d_input = input - self.last_input;
            self.output_sum += self.ki * self.error;

            self.output_sum = self.output_sum.min(self.max).max(self.min);

            /*Add Proportional on Measurement, if P_ON_M is specified*/
            if !self.p_on_e {
                self.output_sum -= self.kp * d_input;
            }

            let mut output = if self.p_on_e { self.kp * error } else { 0. };
            output += self.output_sum - self.kd * d_input;
            output = output.min(self.max).max(self.min);

            Some(output)
        } else {
            None
        }
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
