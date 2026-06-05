//! Simple linear algebra types for MPU6050 sensor readings
//!
//! This module provides minimal 2D and 3D vector types without external dependencies.

/// 2D vector for roll/pitch angles
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct _Vector2 {
    pub x: f32,
    pub y: f32,
}

impl _Vector2 {
    pub fn _new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}

/// 3D vector for accelerometer/gyroscope readings
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

impl core::ops::Div<f32> for Vector3 {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl core::ops::DivAssign<f32> for Vector3 {
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

impl core::ops::Mul<f32> for Vector3 {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl core::ops::MulAssign<f32> for Vector3 {
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}
