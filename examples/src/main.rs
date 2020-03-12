/// This is a simple example that prints out the atlitude as fast as it can while timing it
/// You can also output the temperature or pressure
/// It uses the Raspberry Pi as the processor, but any HAL crate should work.

use std::time::{Duration};
use std::thread;

use rppal::i2c::I2c;

use mma84::*;

fn main() -> ! {
    let mut i2c = I2c::new().unwrap(); 

    let mut mma8452q = MMA8452q::new(i2c,SlaveAddr::Alternative(true)).unwrap();

    mma8452q.standby().unwrap();
 
    mma8452q.set_mode(Mode::HighResolution).unwrap();
    mma8452q.set_odr(Odr::Hz400).unwrap();
    mma8452q.set_fs(FullScale::G2).unwrap();
    mma8452q.active().unwrap();

    loop {

        let all = mma8452q.accel_norm().unwrap();
        let all_2 = mma8452q.accel_raw().unwrap();
        //(x,y,z)
        println!("Norm: {} {} {}",all.x,all.y,all.z);
        println!("Raw:  {} {} {}",all_2.x,all_2.y,all_2.z);
        
        thread::sleep(Duration::from_secs(1));
    }

}
