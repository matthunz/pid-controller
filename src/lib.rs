use std::ops::{Add, Div, Mul, Sub};

use num_traits::One;

#[derive(Clone, Copy, Debug)]

/// Proportional controller
pub struct P<T> {
    pub kp: T,
}

impl<T: One> Default for P<T> {
    fn default() -> Self {
        Self { kp: T::one() }
    }
}

impl<T> P<T> {
    // Between 0.7 and 1
    pub fn new(damping_ratio: T, time_constant: T) -> Self
    where
        T: One + Add<Output = T> + Div<Output = T> + Copy,
    {
        let kp = (T::one() / (time_constant * time_constant))
            * (T::one() + (T::one() + T::one()) * damping_ratio);
        Self { kp }
    }

    pub fn control<U>(self, target: U, actual: U) -> T::Output
    where
        T: Mul<U> + Clone,
        U: Sub<Output = U>,
    {
        let error = error(target, actual);
        self.control_with_error(error)
    }

    pub fn control_with_error<U>(&self, error: U) -> T::Output
    where
        T: Mul<U> + Clone,
    {
        self.kp.clone() * error
    }
}

#[derive(Clone, Debug)]
pub struct PD<T> {
    pub p: P<T>,
    pub kd: T,
}

impl<T> PD<T> {
    pub fn new(damping_ratio: T, time_constant: T) -> Self
    where
        T: One + Add<Output = T> + Div<Output = T> + Copy,
    {
        let kd = (T::one() / time_constant)
            * (T::one() + (T::one() + T::one()) * time_constant * time_constant);
        Self {
            p: P::new(damping_ratio, time_constant),
            kd,
        }
    }

    pub fn control<U>(&self, target: U, target_dot: U, actual: U, actual_dot: U) -> U
    where
        T: Mul<U, Output = U> + Sub<Output = T> + Clone,
        U: Add<Output = U> + Sub<Output = U>,
    {
        let p = self.p.clone().control(target, actual);
        self.control_with_p(p, target_dot, actual_dot)
    }

    pub fn control_with_p<U>(&self, p: U, target_dot: U, actual_dot: U) -> U
    where
        T: Mul<U, Output = U> + Clone,
        U: Add<Output = U> + Sub<Output = U>,
    {
        self.kd.clone() * (target_dot - actual_dot) + p
    }
}

pub fn error<T>(target: T, actual: T) -> T
where
    T: Sub<Output = T>,
{
    target - actual
}

#[derive(Clone, Debug)]
pub struct PID {
    pub pd: PD<f32>,
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
