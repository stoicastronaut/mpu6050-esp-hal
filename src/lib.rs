#![no_std]

pub mod config;
mod bits;
mod helpers;
mod linear;

use core::f32;
use core::f32::consts::PI;

use embedded_hal::i2c::I2c;
use crate::bits::*;
use crate::config::*;
use crate::helpers::delay_ms;
use crate::linear::Vector3;

pub const PI_180: f32 = PI / 180.0;

#[derive(Debug)]
pub enum Mpu6050Error<E> {
    I2c(E),

    InvalidChip(u8),
}

#[derive(Debug)]
pub struct Mpu6050Builder<I2C> {
    i2c: I2C,
    slave_addr: MpuRegister,
    accel_range: AccelRange,
    gyro_range: GyroRange,
}

impl<I2C> Mpu6050Builder<I2C>
where
    I2C: I2c,

{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            accel_range: AccelRange::G2,
            gyro_range: GyroRange::D250,
        }
    }

    pub fn with_addr(mut self, addr: MpuRegister) -> Self {
        self.slave_addr = addr;
        self
    }

    pub fn with_accel_range(mut self, range: AccelRange) -> Self {
        self.accel_range = range;
        self
    }

    pub fn with_gyro_range(mut self, range: GyroRange) -> Self {
        self.gyro_range = range;
        self
    }

    pub fn build(self) -> Mpu6050<I2C> {
        Mpu6050 { 
            i2c: self.i2c,
            slave_addr: self.slave_addr,
            acc_sensivity: self.accel_range,
            gyro_sensivity: self.gyro_range
        }
    }
}


#[derive(Debug)]
pub struct Mpu6050<I2C> {
    i2c: I2C,
    slave_addr: MpuRegister,
    acc_sensivity: AccelRange,
    gyro_sensivity: GyroRange,
}

impl<I2C> Mpu6050<I2C>
where
    I2C: I2c,
{
    pub fn init(&mut self) -> Result<(), Mpu6050Error<I2C::Error>> {
        self.wake()?;
        self.verify()?;
        self.set_accel_range(AccelRange::G2)?;
        self.set_gyro_range(GyroRange::D250)?;
        self.set_accel_hpf(ACCEL_HPF::_RESET)?;
        Ok(())
    }

    fn wake(&mut self) -> Result<(), Mpu6050Error<I2C::Error>> {
        self.write_byte(PWR_MGMT_1::ADDR, 0x01)?;
        delay_ms(100);
        Ok(())
    }

    fn verify(&mut self) -> Result<(), Mpu6050Error<I2C::Error>> {
        let chip_id = self.read_byte(WHO_AM_I)?;
        if !VALID_CHIP_IDS.contains(&chip_id) {
            return Err(Mpu6050Error::InvalidChip(chip_id));
        }
        Ok(())
    }

    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Mpu6050Error<I2C::Error>> {
        let mut byte = self.read_byte(ACCEL_CONFIG::ADDR)?;
        set_bits(&mut byte, ACCEL_CONFIG::FS_SEL.bit, ACCEL_CONFIG::FS_SEL.length, range as u8);
        self.write_byte(ACCEL_CONFIG::ADDR, byte)?;
        self.acc_sensivity = range;
        Ok(())
    }

    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Mpu6050Error<I2C::Error>> {
        let mut byte = self.read_byte(GYRO_CONFIG::ADDR)?;
        set_bits(&mut byte, GYRO_CONFIG::FS_SEL.bit, GYRO_CONFIG::FS_SEL.length, range as u8);
        self.write_byte(GYRO_CONFIG::ADDR, byte)?;
        self.gyro_sensivity = range;
        Ok(())
    }
    
    pub fn set_accel_hpf(&mut self, mode: ACCEL_HPF) -> Result<(), Mpu6050Error<I2C::Error>> {
        let mut byte = self.read_byte(ACCEL_CONFIG::ADDR)?;
        set_bits(&mut byte, ACCEL_CONFIG::ACCEL_HPF.bit, ACCEL_CONFIG::ACCEL_HPF.length, mode as u8);
        self.write_byte(ACCEL_CONFIG::ADDR, byte)?;
        Ok(())
    }

    fn read_rot(&mut self, reg: MpuRegister) -> Result<Vector3, Mpu6050Error<I2C::Error>> {
        let mut buf: [u8; 6] = [0;6];
        self.read_bytes(reg, &mut buf)?;
        
        Ok(
            Vector3::new(
                i16::from_be_bytes(buf[0..2].try_into().unwrap()) as f32,
                i16::from_be_bytes(buf[2..4].try_into().unwrap()) as f32,
                i16::from_be_bytes(buf[4..6].try_into().unwrap()) as f32,
                )
        )
    }

    pub fn get_acc(&mut self) -> Result<Vector3, Mpu6050Error<I2C::Error>> {
        let mut acc = self.read_rot(ACC_REGX_H)?;

        acc /= self.acc_sensivity.sensitivity();

        Ok(acc)
    }

    pub fn get_gyro(&mut self) -> Result<Vector3, Mpu6050Error<I2C::Error>> {
        let mut gyro = self.read_rot(GYRO_REGX_H)?;
        
        gyro *= PI_180 / self.gyro_sensivity.sensitivity();

        Ok(gyro)
    }

    pub fn get_raw_accel(&mut self) -> Result<Vector3, Mpu6050Error<I2C::Error>> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(ACC_REGX_H, &mut buf)?;

        Ok(Vector3::new(
            i16::from_be_bytes(buf[0..2].try_into().unwrap()) as f32,
            i16::from_be_bytes(buf[2..4].try_into().unwrap()) as f32,
            i16::from_be_bytes(buf[4..6].try_into().unwrap()) as f32,
        ))
    }

    pub fn get_raw_gyro(&mut self) -> Result<Vector3, Mpu6050Error<I2C::Error>> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(GYRO_REGX_H, &mut buf)?;

        Ok(Vector3::new(
            i16::from_be_bytes(buf[0..2].try_into().unwrap()) as f32,
            i16::from_be_bytes(buf[2..4].try_into().unwrap()) as f32,
            i16::from_be_bytes(buf[4..6].try_into().unwrap()) as f32,
        ))
    }

    pub fn get_all_raw(&mut self) -> Result<(Vector3, Vector3, i16), Mpu6050Error<I2C::Error>> {
        let mut buf: [u8; 14] = [0; 14];
        self.read_bytes(ACC_REGX_H, &mut buf)?;

        let accel = Vector3::new(
            i16::from_be_bytes(buf[0..2].try_into().unwrap()) as f32,
            i16::from_be_bytes(buf[2..4].try_into().unwrap()) as f32,
            i16::from_be_bytes(buf[4..6].try_into().unwrap()) as f32,
        );

        let temp = i16::from_be_bytes(buf[6..8].try_into().unwrap());

        let gyro = Vector3::new(
            i16::from_be_bytes(buf[8..10].try_into().unwrap()) as f32,
            i16::from_be_bytes(buf[10..12].try_into().unwrap()) as f32,
            i16::from_be_bytes(buf[12..14].try_into().unwrap()) as f32,
        );

        Ok((accel, gyro, temp))
    }

    pub fn get_temperature(&mut self) -> Result<f32, Mpu6050Error<I2C::Error>> {
        let mut buff: [u8; 2] = [0; 2];
        self.read_bytes(TEMP_OUT_H, &mut buff)?;
        let raw_temp = i16::from_be_bytes(buff[0..2].try_into().unwrap()) as f32;

        Ok((raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET )

    }

    pub fn write_byte(&mut self, reg: MpuRegister, byte: u8) -> Result<(), Mpu6050Error<I2C::Error>> {
        self.i2c.write(self.slave_addr, &[reg, byte])
            .map_err(Mpu6050Error::I2c)
    }

    pub fn write_bit(&mut self, reg: MpuRegister, bit_n: u8, enable: bool) -> Result<(), Mpu6050Error<I2C::Error>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bit(&mut byte[0], bit_n, enable);
        Ok(self.write_byte(reg, byte[0])?)
    }

    pub fn write_bits(&mut self, reg: MpuRegister, start_bit: u8, length: u8, data: u8) -> Result<(), Mpu6050Error<I2C::Error>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bits(&mut byte[0], start_bit, length, data);
        Ok(self.write_byte(reg, byte[0])?)
    }

    pub fn read_bit(&mut self, reg: MpuRegister, bit_n: u8) -> Result<u8, Mpu6050Error<I2C::Error>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        Ok(bits::get_bit(byte[0], bit_n))
    }

    pub fn read_bits(&mut self, reg: MpuRegister, start_bit: u8, length: u8) -> Result<u8, Mpu6050Error<I2C::Error>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        Ok(bits::get_bits(byte[0], start_bit, length))
    }

    pub fn read_byte(&mut self, reg: MpuRegister) -> Result<MpuRegister, Mpu6050Error<I2C::Error>> {
        let mut byte: [MpuRegister; 1] = [0; 1];
        self.i2c.write_read(self.slave_addr, &[reg], &mut byte)
            .map_err(Mpu6050Error::I2c)?;
        Ok(byte[0])
    }

    pub fn read_bytes(&mut self, reg: MpuRegister, buffer: &mut[u8]) -> Result<(), Mpu6050Error<I2C::Error>> {
        self.i2c.write_read(self.slave_addr, &[reg], buffer)
            .map_err(Mpu6050Error::I2c)?;
        Ok(())
    }
}


