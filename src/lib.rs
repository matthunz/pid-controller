#[derive(Clone, Copy, Debug, Default)]
pub struct P {
    pub kp: f32,
}

impl P {
    // Between 0.7 and 1
    pub fn new(damping_ratio: f32, time_constant: f32) -> Self {
        let kp = (1. / time_constant.powi(2)) * (1. + 2. * damping_ratio);
        Self { kp }
    }

    pub fn control(self, target: f32, actual: f32) -> f32 {
        let error = error(target, actual);
        self.control_with_error(error)
    }

    pub fn control_with_error(&self, error: f32) -> f32 {
        error * self.kp
    }
}

pub struct PD {
    pub p: P,
    pub kd: f32,
}

impl PD {
    pub fn new(damping_ratio: f32, time_constant: f32) -> Self {
        let kd = (1. / time_constant) * (1. + 2. * time_constant.powi(2));
        Self {
            p: P::new(damping_ratio, time_constant),
            kd,
        }
    }

    pub fn control(&self, target: f32, target_dot: f32, actual: f32, actual_dot: f32) -> f32 {
        let p = self.p.control(target, actual);
        self.control_with_p(p, target_dot, actual_dot)
    }

    pub fn control_with_p(&self, p: f32, target_dot: f32, actual_dot: f32) -> f32 {
        self.kd * (target_dot - actual_dot) + p
    }
}

pub fn error(target: f32, actual: f32) -> f32 {
    target - actual
}

pub struct PID {
    pub pd: PD,
    pub ki: f32,
    pub integrated_error: f32,
}

impl PID {
    pub fn new(damping_ratio: f32, time_constant: f32) -> Self {
        let ki = 1. / time_constant.powi(3);
        Self {
            pd: PD::new(damping_ratio, time_constant),
            ki,
            integrated_error: 0.,
        }
    }

    pub fn control(
        &mut self,
        target: f32,
        target_dot: f32,
        actual: f32,
        actual_dot: f32,
        dt: f32,
    ) -> f32 {
        let error = error(target, actual);
        self.integrated_error += error * dt;

        let p = self.pd.p.control_with_error(error);
        let i = self.integrated_error * self.ki;
        let d = self.pd.control_with_p(p, target_dot, actual_dot);

        p + i + d
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
