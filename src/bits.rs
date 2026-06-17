//! Bit operations on registers
//! Mostly taken from https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/I2Cdev/I2Cdev.cpp

/// get bit n of byte
pub fn get_bit(byte: u8, n: u8) -> u8 {
    (byte >> n) & 1
}

/// get bits start - start+length from byte
pub fn get_bits(mut byte: u8, bit_start: u8, length: u8) -> u8 {
    let mask_shift: u8 = if bit_start < length { 0 } else { bit_start - length + 1 };
    let mask: u8 = ((1 << length) - 1) << mask_shift;
    byte &= mask as u8;
    byte >>= mask_shift;
    byte
}

/// set bit n in byte
pub fn set_bit(byte: &mut u8, n: u8, enable: bool) {
    if enable {
        *byte |= 1_u8 << n;
    } else {
        *byte &= !(1_u8 << n);
    }
}

/// Fill bits bitstart-bitstart+length in byte with data
pub fn set_bits(byte: &mut u8, bit_start: u8, length: u8, mut data: u8) {
    let mask_shift: u8 = if bit_start < length { 0 } else { bit_start - length + 1 };
    let mask: u8 = ((1 << length) - 1) << mask_shift;
    data <<= mask_shift;
    data &= mask;
    *byte &= !(mask);
    *byte |= data;
}
