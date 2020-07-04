#![deny(unsafe_code)]
#![no_main]
#![no_std]
/* 

Test of different communications interfaces

SPI


I2C
PB6 - scl
PB7 - sda

USART
PA9
PA10

*/


// Halt on panic
#[allow(unused_extern_crates)] // NOTE(allow) bug rust-lang/rust#53964
extern crate panic_halt; // panic handler

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use crate::hal::{
    prelude::*, 
    stm32,
    spi::Spi,
    i2c::I2c,
    pwm,
    serial::Serial,
};

use hal::spi::{Mode, Phase, Polarity};
use hal::serial::config;

const I2C_ADDR: u8 = 0xF0u8;

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        stm32::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        // Set up the LED. On the Nucleo-446RE it's connected to pin PA5.
        let gpioa = dp.GPIOA.split();
        // let mut led = gpioa.pa5.into_push_pull_output();

        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        // Create a delay abstraction based on SysTick
        let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

        let channels = gpioa.pa8.into_alternate_af1();

        // Set up PWM
        let pwm = pwm::tim1(dp.TIM1, channels, clocks, 50.hz());
        let (mut pwm_pa8) = pwm;
        let max_duty = pwm_pa8.get_max_duty();
        pwm_pa8.set_duty(max_duty / 2);
        pwm_pa8.enable();

        // Set up SPI
        let gpiob = dp.GPIOB.split();
        let sck = gpioa.pa5.into_alternate_af5();
        let miso = gpioa.pa6.into_alternate_af5();
        let mosi = gpioa.pa7.into_alternate_af5();
        let mut cs = gpiob.pb5.into_push_pull_output();

        cs.set_high().unwrap();

        let mut spi = Spi::spi1(
            dp.SPI1,
            (sck, miso, mosi),
            Mode {polarity: Polarity::IdleLow, phase: Phase::CaptureOnFirstTransition},
            stm32f4xx_hal::time::KiloHertz(2000).into(),
            clocks,
        );

        // set up I2C
        let scl = gpiob.pb6.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb7.into_alternate_af4().set_open_drain();

        let mut i2c1 = I2c::i2c1(
            dp.I2C1,
            (scl, sda),
            400.khz(),
            clocks,
        );

        let tx = gpioa.pa9.into_alternate_af7();
        let rx = gpioa.pa10.into_alternate_af7();
        let usart1_config = config::Config::default().baudrate(112_500.bps());
        let mut usart1 = Serial::usart1(
            dp.USART1,
            (tx, rx),
            usart1_config,
            clocks,
        ).unwrap();

        loop {
            let mut spi_test: [u8; 4] = [0x55, 0x55, 0xFF, 0x00];
            let mut i2c_test: [u8; 4] = [0x55, 0x55, 0xFF, 0x00];

            let mut res_spi;
            let mut res_i2c;

            delay.delay_ms(100_u32);

            cs.set_low().unwrap();
            res_spi = spi.transfer(&mut spi_test);
            cs.set_high().unwrap();

            if Result::is_ok(&res_spi) {
                for i in 0..spi_test.len() {
                    spi_test[i] = 0xFFu8;
                }
            }

            cs.set_low().unwrap();
            res_spi = spi.transfer(&mut spi_test);
            cs.set_high().unwrap();

            // res_i2c = i2c1.write(I2C_ADDR, &mut i2c_test).ok();
            // res_i2c = i2c1.write(I2C_ADDR, &mut i2c_test).ok();
        
            usart1.flush();
            let word = 0x55u8;
            let res_usart1 = usart1.write(word);
            if res_usart1.is_err() {
                panic!();
            }
        }
    }

    loop {}
}