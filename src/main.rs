#![no_std]
#![no_main]

/**
 * Rust program for Longan Nano microcontroller board and DHT77
 * temperature and humidity sensor. Reads the data from the sensor
 * prints the values to the Longan Nano's LCD screen.
 * 
 * Authors: Teemu Miettunen, teemu.miettunen@tuni.fi
 *          Elias Hagelberg, elias.hagelberg@tuni.fi
 */

use heapless::String;
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Rectangle, PrimitiveStyle},
    text::Text,
    mono_font::{MonoTextStyleBuilder, iso_8859_1::FONT_10X20}
};
use embedded_hal::digital::v2::{OutputPin, InputPin};
use longan_nano::hal::{
    {pac, rcu::RcuExt, prelude::*},
    delay::{McycleDelay},
    {eclic::{EclicExt, Level, LevelPriorityBits, Priority, TriggerType}},
    timer::{Event, Timer},
    gpio::{Floating, Input},
    gpio::gpioa::PA0
};
use longan_nano::{lcd, lcd_pins};
use riscv_rt::entry;
use panic_halt as _;

//Global variables for data and timer 
static mut DATA:(f32, f32) = (0.0, 0.0);
static mut TIMER: Option<Timer<longan_nano::hal::pac::TIMER1>> = None;

static mut DELAY: Option<McycleDelay> = None;
static mut SIGNAL_PIN: Option<PA0<Input<Floating>>> = None;

//Function for reading data from the sensor
fn read_data(signal_pin: PA0<Input<Floating>>, mut delay: McycleDelay) -> Result<(f32, f32), &'static str> {

    // same as count_ in c library
    let count_ = 22;

    // same as MAXTIMINGS in c library
    let maxtimings_ = 85;

    let mut laststate: bool = true;
    let mut counter: i32;
    let mut  i: u8 = 0;
    let mut j: u8 = 0;

    let mut data: [u8; 5] = [0, 0, 0, 0, 0];

    let mut out_pin = signal_pin.into_push_pull_output();

    out_pin.set_high().unwrap();
    delay.delay_ms(250);

    out_pin.set_low().unwrap();
    delay.delay_ms(20);

    out_pin.set_high().unwrap();
    delay.delay_us(40);

    let in_pin = out_pin.into_pull_up_input();

    // read in timings
    while i < maxtimings_{
        counter = 0;
        while in_pin.is_high().unwrap() == laststate {
            counter += 1;
            delay.delay_us(1);
            if counter == 255 {
                break;
            }
        }
        laststate = in_pin.is_high().unwrap();

        if counter == 255 {
            break;
        }

        // ignore first 3 transitions
        if (i >= 4) && (i % 2 == 0) {
            // shove each bit into the storage bytes
            let index = (j / 8) as usize;
            data[index] <<= 1;
            if counter > count_ {
                data[index] |= 1;
            }
            j += 1;
        }
        i += 1;
    }
    // check we read 40 bits and that the checksum matches
    if (j >= 40) && (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        
        // temperature
        let mut t = data[2] as f32;

        let value = data[3]%128;
        match value {
            0..=9 => t += (data[3]%128/10) as f32,

            10..=100 => t += (data[3]%128/100) as f32,

            _ => t += ((data[3]%128) as i32 /1000) as f32,
        }

        // The left-most digit indicate the negative sign. 
        if data[3]>=128 { 
            t = -t;
        }

        // Humidity
        let h = data[0] as f32;

        //Return temp and humidity values
        return Ok((t, h));

    }
    // return this when something failed
    return Err("Could not read values!"); 
}

//Interrupt handler function
fn TIMER1(){

    unsafe{
        riscv::interrupt::free(|_|{

            //tämä ei vielä toimi  
            let signal_pin = SIGNAL_PIN.unwrap();
            let delay = DELAY.unwrap();
            
            let data= read_data(signal_pin, delay);
            match data {
                Ok(v) => DATA = v,
                Err(_e) => DATA = (1111.0, 1111.0),
            }

            TIMER.as_mut().unwrap().clear_update_interrupt_flag();
        });
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure clocks
    let mut rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(80.mhz())
        .freeze();
    let mut afio = dp.AFIO.constrain(&mut rcu);

    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpiob = dp.GPIOB.split(&mut rcu);

    let signal_pin  = gpioa.pa0;
    let delay = McycleDelay::new(&rcu.clocks);

    unsafe{
        SIGNAL_PIN = Some(signal_pin);
        DELAY = Some(delay);
    }

    let lcd_pins = lcd_pins!(gpioa, gpiob);
    let mut lcd = lcd::configure(dp.SPI0, lcd_pins, &mut afio, &mut rcu);
    let (width, height) = (lcd.size().width as i32, lcd.size().height as i32);

    //Set timer
    unsafe{
        let mut timer = Timer::timer1(dp.TIMER1, 1.hz(), &mut rcu);
        timer.listen(Event::Update);
        TIMER = Some(timer);
    }

    //ECLIC setup
    pac::ECLIC::reset();
    pac::ECLIC::set_level_priority_bits(LevelPriorityBits::L0P4);
    pac::ECLIC::set_threshold_level(Level::L1);
    pac::ECLIC::setup(pac::Interrupt::TIMER1, TriggerType::Level, Level::L1, Priority::P1);
    unsafe{
        pac::ECLIC::unmask(pac::Interrupt::TIMER1)
    };

    //Enable interrupts
    unsafe{riscv::interrupt::enable()};

    // Clear screen
    Rectangle::new(Point::new(0, 0), Size::new(width as u32, height as u32))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(&mut lcd)
        .unwrap();

    let style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb565::new(50, 50, 50))
        .background_color(Rgb565::BLACK)
        .build();

    // (temperature, humidity) pair
    //let data = read_data(signal_pin, delay);

    loop {
        unsafe{
            //set text from counter
            let mut t_as_text: String<10> = String::from(DATA.0 as i32);

            t_as_text.push('°').unwrap();
            t_as_text.push('C').unwrap();

            // Draw temperature
            Text::new(t_as_text.as_str(), Point::new(40, 35), style)
                .draw(&mut lcd)
                .unwrap();
            
            let mut h_as_text: String<10> = String::from(DATA.1 as i32);

            h_as_text.push('%').unwrap();
            
            // Draw humidity
            Text::new(h_as_text.as_str(), Point::new(40, 60), style)
                .draw(&mut lcd)
                .unwrap();
            }
    
        //set chip to sleep
        unsafe{riscv::asm::wfi();}
    }
}



