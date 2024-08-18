#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, 
    delay::Delay, 
    embassy, 
    gpio::{self, Event, Input, PullDown, PullUp, IO}, 
    i2c::I2C, 
    peripherals::{Peripherals, UART0}, 
    prelude::*, 
    timer::{TimerGroup}, 
    uart::{config::AtCmdConfig, Uart, UartRx, UartTx}, Async,
};
use portable_atomic::AtomicU32;
use core::{ cell::RefCell, sync::atomic::Ordering};

use critical_section::Mutex;
use lcd1602_driver::{
    command::{DataWidth, State},
    lcd::{self, Basic, Ext, Lcd},
    sender::I2cSender,
};
use heapless::String;


static UPBUTTON: Mutex<RefCell<Option<gpio::Gpio0<Input<PullUp>>>>> =
    Mutex::new(RefCell::new(None));
static DOWNBUTTON: Mutex<RefCell<Option<gpio::Gpio1<Input<PullUp>>>>> =
    Mutex::new(RefCell::new(None));
    static RESETBUTTON: Mutex<RefCell<Option<gpio::Gpio6<Input<PullUp>>>>> =
    Mutex::new(RefCell::new(None));
static BLINK_DELAY: AtomicU32 = AtomicU32::new(1_u32);

const AT_CMD: u8 = 0x04;
const READ_BUF_SIZE: usize = 64;

#[embassy_executor::task]
async fn writer(
    mut tx: UartTx<'static, UART0, Async>,
) {
    use core::fmt::Write;
    embedded_io_async::Write::write(
        &mut tx,
        b"Serial is open.\r\n",
    )
    .await
    .unwrap();
    embedded_io_async::Write::flush(&mut tx).await.unwrap();
    loop {
        write!(&mut tx, "\r\n-- Delay Time {}--\r\n",BLINK_DELAY.load(Ordering::Relaxed)).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
        Timer::after(Duration::from_millis(BLINK_DELAY.load(Ordering::Relaxed) as u64)).await;
    }
}
#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let mut io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut uart0 = Uart::new_async(peripherals.UART0, &clocks);
    io.set_interrupt_handler(delayup);
    io.set_interrupt_handler(delaydown);
    io.set_interrupt_handler(reset);
    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timg0);
    esp_println::logger::init_logger_from_env();
    uart0.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));
    uart0
        .set_rx_fifo_full_threshold(READ_BUF_SIZE as u16)
        .unwrap();
    let (tx, rx) = uart0.split();
    spawner.spawn(writer(tx)).ok();
    let mut led = io.pins.gpio4.into_push_pull_output();
    let mut led1 = io.pins.gpio5.into_push_pull_output();
    let mut upbutton = io.pins.gpio0.into_pull_up_input();
    let mut downbutton = io.pins.gpio1.into_pull_up_input();
    let mut resetbutton = io.pins.gpio6.into_pull_up_input();
    let mut i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio3,
        io.pins.gpio2,
        100.kHz(),
        &clocks,
        None,
    );
    critical_section::with(|cs| {
        upbutton.listen(Event::FallingEdge);
        UPBUTTON.borrow_ref_mut(cs).replace(upbutton)
    });
    critical_section::with(|cs| {
        downbutton.listen(Event::FallingEdge);
        DOWNBUTTON.borrow_ref_mut(cs).replace(downbutton)
    });
    critical_section::with(|cs| {
        resetbutton.listen(Event::FallingEdge);
        RESETBUTTON.borrow_ref_mut(cs).replace(resetbutton)
    });
    led.set_low();
    led1.set_low();
    let mut sender = I2cSender::new(&mut i2c, 0x27);
    let lcd_config = lcd::Config::default().set_data_width(DataWidth::Bit4);
    let mut delay1=delay.clone();
    let mut lcd= Lcd::new(&mut sender, &mut delay1, lcd_config, 10);
    lcd.set_cursor_blink_state(State::Off);
    lcd.set_cursor_state(State::Off);
    lcd.set_cursor_pos((0, 0));
    lcd.write_str_to_cur("DelayTime:");
    lcd.set_cursor_pos((14, 0));
    lcd.write_str_to_cur("ms");
    loop {
        critical_section::with(|cs| {
            if UPBUTTON.borrow_ref_mut(cs).as_mut().unwrap().is_low(){
                led.set_high();
                let num=BLINK_DELAY.load(Ordering::Relaxed)as u32;
                BLINK_DELAY.store(num+1, Ordering::Relaxed);
                let nums: String<10> = String::try_from(BLINK_DELAY.load(Ordering::Relaxed)).unwrap();
                lcd.set_cursor_pos((10, 0));
                lcd.write_str_to_cur("    ");
                lcd.set_cursor_pos((14-nums.len()as u8, 0));
                lcd.write_str_to_cur(nums.as_str());
            }else{
                led.set_low();
            }
        });
        critical_section::with(|cs| {
            if DOWNBUTTON.borrow_ref_mut(cs).as_mut().unwrap().is_low(){
                led1.set_high();
                let num=BLINK_DELAY.load(Ordering::Relaxed)as u32;
                if num >0{
                    BLINK_DELAY.store(num-1, Ordering::Relaxed);
                };
                let nums: String<10> = String::try_from(BLINK_DELAY.load(Ordering::Relaxed)).unwrap();
                lcd.set_cursor_pos((10, 0));
                lcd.write_str_to_cur("    ");
                lcd.set_cursor_pos((14-nums.len()as u8, 0));
                lcd.write_str_to_cur(nums.as_str());
            }else{
                led1.set_low();
            }
        });
        critical_section::with(|cs| {
            if RESETBUTTON.borrow_ref_mut(cs).as_mut().unwrap().is_low(){
                led.set_high();
                led1.set_high();
                lcd.set_cursor_pos((10, 0));
                lcd.write_str_to_cur("    ");
            }else{
                led.set_high();
                led1.set_low();
            }
        });
        Timer::after(Duration::from_millis(10)).await;
    }
}



#[handler]
#[ram]
fn delayup() {
    if critical_section::with(|cs| {
        UPBUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        // 한번씩만 동작하고 싶을때
        // let num=BLINK_DELAY.load(Ordering::Relaxed)as u32;
        // BLINK_DELAY.store(num+1, Ordering::Relaxed);
        esp_println::println!("Clicked Delay Up Botton");
    } else {
        esp_println::println!("Button was not the source of the interrupt");
    }

    critical_section::with(|cs| {
        UPBUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

#[handler]
#[ram]
fn delaydown() {
    if critical_section::with(|cs| {
        DOWNBUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        // 한번씩만 동작하고 싶을때
        // let num=BLINK_DELAY.load(Ordering::Relaxed)as u32;
        // BLINK_DELAY.store(num+1, Ordering::Relaxed);
        esp_println::println!("Clicked Delay Down Botton");
    } else {
        esp_println::println!("Button was not the source of the interrupt");
    }

    critical_section::with(|cs| {
        DOWNBUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}
#[handler]
#[ram]
fn reset() {
    if critical_section::with(|cs| {
        RESETBUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        BLINK_DELAY.store(1, Ordering::Relaxed);
        esp_println::println!("Clicked Delay Reset");
    } else {
        esp_println::println!("Button was not the source of the interrupt");
    }

    critical_section::with(|cs| {
        RESETBUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}