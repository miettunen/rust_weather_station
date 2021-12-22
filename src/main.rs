#![no_std]
#![no_main]

/**
 * Rust program for Longan Nano microcontroller board and DHT77
 * temperature and humidity sensor. Reads the data from the sensor
 * prints the values to the Longan Nano's LCD screen.
 *
 * read_data-function inspired by Seedstudio's C++ library:
 * https://github.com/Seeed-Studio/Grove_Temperature_And_Humidity_Sensor
 *
 *
 * Authors: Teemu Miettunen, teemu.miettunen@tuni.fi
 *          Elias Hagelberg, elias.hagelberg@tuni.fi
 */

use core::cell::RefCell;
use core::ops::DerefMut;
use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use heapless::String;
use longan_nano::hal::{
    delay::McycleDelay,
    eclic::{EclicExt, Level, LevelPriorityBits, Priority, TriggerType},
    gpio::gpioa::PA0,
    gpio::{Output, PushPull},
    timer::{Event, Timer},
    {pac, prelude::*, rcu::RcuExt},
};
use longan_nano::{lcd, lcd_pins};
use panic_halt as _;
use riscv::interrupt::{free, Mutex};
use riscv_rt::entry;

// Interrupt timer
static TIMER: Mutex<RefCell<Option<Timer<longan_nano::hal::pac::TIMER1>>>> =
    Mutex::new(RefCell::new(None));

// Measured (temperature, humidity) pair
static DATA: Mutex<RefCell<Option<(f32, f32)>>> = Mutex::new(RefCell::new(Some((0.0, 0.0))));

// Used for creating delays in read_data-function
static DELAY: Mutex<RefCell<Option<McycleDelay>>> = Mutex::new(RefCell::new(None));

// Pin used for reading data from sensor
static SIGNAL_PIN: Mutex<RefCell<Option<PA0<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

// Counter to only read data on specific interrupts to decrease update inverval from 1 Hz
static mut TIMER_COUNTER: u32 = 0;

// Update interval in seconds
static UPDATE_INTERVAL: u32 = 3;

//Function for reading data from the sensor
fn read_data() -> Result<(f32, f32), &'static str> {
    // Used for saving and replacing signal pin after taking it from SIGNAL_PIN
    let mut signal_pin_saved = None;

    // Keeps track of signal pin needing to be replaced in SIGNAL_PIN
    let mut replaced = false;

    // Initialize variables for temperature and humidity
    let mut t = 1001.0;
    let mut h = 1001.0;

    free(|cs| {
        if let Some(mut signal_pin) = SIGNAL_PIN.borrow(*cs).take() {
            if let Some(ref mut delay) = DELAY.borrow(*cs).borrow_mut().deref_mut() {
                replaced = true;

                // same as count_ in c++ library, based on cpu clock speed which in this project is 80 MHz
                let count_ = 22;

                // how many timing transitions are needed to keep track of. 2 * number bits + extra
                let maxtimings_ = 85;

                let mut laststate: bool = true;
                let mut counter: i32;
                let mut i: u8 = 0;
                let mut j: u8 = 0;

                // Storing read data, first byte for humidity, 3rd and 4th for temperature
                let mut data: [u8; 5] = [0, 0, 0, 0, 0];

                signal_pin.set_high().unwrap();
                delay.delay_ms(250);

                signal_pin.set_low().unwrap();
                delay.delay_ms(20);

                signal_pin.set_high().unwrap();
                delay.delay_us(40);

                let signal_pin_input = signal_pin.into_pull_up_input();

                // read in timings
                while i < maxtimings_ {
                    counter = 0;
                    while signal_pin_input.is_high().unwrap() == laststate {
                        counter += 1;
                        delay.delay_us(1);
                        if counter == 255 {
                            break;
                        }
                    }
                    laststate = signal_pin_input.is_high().unwrap();

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
                // Saving signal pin to replace value in SIGNAL_PIN
                signal_pin_saved = Some(signal_pin_input);

                // check we read 40 bits and that the checksum matches
                if (j >= 40) && (data[4] == (data[0] + data[1] + data[2] + data[3])) {
                    // converting read temperature to float
                    t = data[2] as f32;

                    let value = data[3] % 128;
                    match value {
                        0..=9 => t += (data[3] % 128 / 10) as f32,

                        10..=100 => t += (data[3] % 128 / 100) as f32,

                        _ => t += ((data[3] % 128) as i32 / 1000) as f32,
                    }

                    // The left-most digit indicate the negative sign.
                    if data[3] >= 128 {
                        t = -t;
                    }

                    // Humidity
                    h = data[0] as f32;
                }
            }
        }
    });

    // Put signal pin back to SIGNAL_PIN for next call
    if replaced {
        free(|cs| {
            SIGNAL_PIN
                .borrow(*cs)
                .replace(Some(signal_pin_saved.unwrap().into_push_pull_output()));
        });
    }

    //Return temp and humidity values
    if t < 1000.0 && h < 1000.0 {
        return Ok((t, h));
    }

    // return this when something failed
    Err("Could not read values!")
}

//Interrupt handler function
#[allow(non_snake_case)]
#[no_mangle]
fn TIMER1() {
    // Only update on specific intervals, didn't find way to setup interrupt timer freq below 1 Hz
    let mut do_update = false;
    unsafe {
        if TIMER_COUNTER % UPDATE_INTERVAL == 0 {
            do_update = true;
        }
        TIMER_COUNTER += 1;
    }

    if do_update {
        let data = read_data();
        match data {
            Ok(v) => {
                free(|cs| {
                    if let Some(ref mut data_stored) = DATA.borrow(*cs).borrow_mut().deref_mut() {
                        *data_stored = v;
                    }
                });
            }
            // Value t = 112 h = 112 used to show error in reading
            Err(_e) => {
                free(|cs| {
                    if let Some(ref mut data_stored) = DATA.borrow(*cs).borrow_mut().deref_mut() {
                        *data_stored = (112.0, 112.0);
                    }
                });
            }
        }
    }

    free(|cs| {
        if let Some(ref mut timer) = TIMER.borrow(*cs).borrow_mut().deref_mut() {
            timer.clear_update_interrupt_flag();
        }
    });
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure clocks, sysclk set to 80 MHz to work properly with count_ in read_data
    let mut rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(80.mhz())
        .freeze();
    let mut afio = dp.AFIO.constrain(&mut rcu);

    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpiob = dp.GPIOB.split(&mut rcu);

    let out_pin = gpioa.pa0.into_push_pull_output();

    let delay = McycleDelay::new(&rcu.clocks);

    free(|cs| {
        SIGNAL_PIN.borrow(*cs).replace(Some(out_pin));
        DELAY.borrow(*cs).replace(Some(delay));
    });

    let lcd_pins = lcd_pins!(gpioa, gpiob);
    let mut lcd = lcd::configure(dp.SPI0, lcd_pins, &mut afio, &mut rcu);
    let (width, height) = (lcd.size().width as i32, lcd.size().height as i32);

    //Set timer
    let mut timer = Timer::timer1(dp.TIMER1, 1.hz(), &mut rcu);
    timer.listen(Event::Update);
    free(|cs| {
        TIMER.borrow(*cs).replace(Some(timer));
    });

    //ECLIC setup
    pac::ECLIC::reset();
    pac::ECLIC::set_level_priority_bits(LevelPriorityBits::L0P4);
    pac::ECLIC::set_threshold_level(Level::L1);
    pac::ECLIC::setup(
        pac::Interrupt::TIMER1,
        TriggerType::Level,
        Level::L1,
        Priority::P1,
    );
    unsafe { pac::ECLIC::unmask(pac::Interrupt::TIMER1) };

    //Enable interrupts
    unsafe { riscv::interrupt::enable() };

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

    loop {
        // Write temperature and humidity values on screen
        free(|cs| {
            if let Some(ref mut data) = DATA.borrow(*cs).borrow_mut().deref_mut() {
                let mut t_as_text: String<10> = String::from(data.0 as i32);
                t_as_text.push('°').unwrap();
                t_as_text.push('C').unwrap();
                t_as_text.push(' ').unwrap(); // Push extra spaces to overwrite last print if string gets shorter (e.g. 12°C -> 9°C )
                t_as_text.push(' ').unwrap();

                Text::new(t_as_text.as_str(), Point::new(40, 35), style)
                    .draw(&mut lcd)
                    .unwrap();

                let mut h_as_text: String<10> = String::from(data.1 as i32);
                h_as_text.push('%').unwrap();
                h_as_text.push(' ').unwrap(); // Push extra spaces to overwrite last print if string gets shorter (e.g. 15% -> 9%)
                t_as_text.push(' ').unwrap();
                Text::new(h_as_text.as_str(), Point::new(40, 60), style)
                    .draw(&mut lcd)
                    .unwrap();
            }
        });

        //set chip to sleep
        unsafe {
            riscv::asm::wfi();
        }
    }
}
