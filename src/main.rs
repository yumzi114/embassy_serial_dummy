#![no_std]
#![no_main]
// #![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, 
    delay::Delay, 
    embassy, 
    // cpu_control::{CpuControl, Stack},
    gpio::{self, Event, Input, PullDown, PullUp, IO}, 
    i2c::I2C, 
    peripherals::{Peripherals, UART0}, 
    prelude::*, 
    rtc_cntl::Rtc, 
    system::SystemClockControl, 
    systimer::SystemTimer, 
    timer::{TimerGroup, TimerInterrupts}, 
    uart::{config::AtCmdConfig, Uart, UartRx, UartTx}, Async,
};
use portable_atomic::AtomicU32;
use core::{borrow::{Borrow, BorrowMut}, cell::RefCell, fmt::Debug, sync::atomic::Ordering};

use critical_section::Mutex;
// use embedded_hal::i2c::{I2c, Error};
use lcd1602_driver::{
    command::{DataWidth, MoveDirection, State},
    lcd::{self, Anim, Basic, Ext, FlipStyle, Lcd, MoveStyle},
    sender::I2cSender,
    utils::BitOps,
};
use core::fmt::Write;
use heapless::String;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use static_cell::make_static;


static UPBUTTON: Mutex<RefCell<Option<gpio::Gpio0<Input<PullUp>>>>> =
    Mutex::new(RefCell::new(None));
static DOWNBUTTON: Mutex<RefCell<Option<gpio::Gpio1<Input<PullUp>>>>> =
    Mutex::new(RefCell::new(None));

static BLINK_DELAY: AtomicU32 = AtomicU32::new(0_u32);

const AT_CMD: u8 = 0x04;
const READ_BUF_SIZE: usize = 64;
// #[embassy_executor::task]
// async fn run() {
//     loop {
//         log::info!("Hello from an embassy thread");
//         Timer::after(Duration::from_millis(1_000)).await;
//     }
// }

#[embassy_executor::task]
async fn writer(
    mut tx: UartTx<'static, UART0, Async>,
    // signal: &'static Signal<NoopRawMutex, usize>,
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
        // let bytes_read = signal.wait().await;
        // signal.reset();
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
    // let mut rtc = Rtc::new(peripherals.LPWR,None);
    let mut delay = Delay::new(&clocks);
    let mut io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut uart0 = Uart::new_async(peripherals.UART0, &clocks);
    io.set_interrupt_handler(delayup);
    io.set_interrupt_handler(delaydown);
    // let systimer = SystemTimer::new_async(peripherals.SYSTIMER);
    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timg0);
    esp_println::logger::init_logger_from_env();
    uart0.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));
    uart0
        .set_rx_fifo_full_threshold(READ_BUF_SIZE as u16)
        .unwrap();
    let (tx, rx) = uart0.split();

    // let signal = &*make_static!(Signal::new());
    // spawner.spawn(run()).ok();
    spawner.spawn(writer(tx)).ok();
    let mut led = io.pins.gpio4.into_push_pull_output();
    let mut led1 = io.pins.gpio5.into_push_pull_output();
    let mut upbutton = io.pins.gpio0.into_pull_up_input();
    let mut downbutton = io.pins.gpio1.into_pull_up_input();
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
    
    led.set_low();
    led1.set_low();
    let mut sender = I2cSender::new(&mut i2c, 0x27);
    let lcd_config = lcd::Config::default().set_data_width(DataWidth::Bit4);
    let mut delay1=delay.clone();
    let mut lcd= Lcd::new(&mut sender, &mut delay1, lcd_config, 10);
    // lcd.write_graph_to_cgram(1, &HEART);
    let mut graph_data = lcd.read_graph_from_cgram(1);
    graph_data[1].set_bit(2);
    graph_data[2].set_bit(2);
    lcd.write_graph_to_cgram(2, &graph_data);
    lcd.set_cursor_blink_state(State::On);
    lcd.set_cursor_pos((0, 0));
    // lcd.offset_cursor_pos((1, 0));
    lcd.write_str_to_cur("DelayTime");
    lcd.set_cursor_blink_state(State::Off);
    lcd.set_cursor_state(State::Off);
    let s1: String<6> = String::try_from(": ").unwrap();
    lcd.write_str_to_cur(s1.as_str());
    let s2: String<4> = String::try_from("ms").unwrap();

    loop {
        critical_section::with(|cs| {
            if UPBUTTON.borrow_ref_mut(cs).as_mut().unwrap().is_low(){
                led.set_high();
                let num=BLINK_DELAY.load(Ordering::Relaxed)as u32;
                BLINK_DELAY.store(num+1, Ordering::Relaxed);
                let nums: String<10> = String::try_from(BLINK_DELAY.load(Ordering::Relaxed)).unwrap();
                lcd.set_cursor_pos((10, 0));
                lcd.write_str_to_cur(nums.as_str());
                lcd.write_str_to_cur(s2.as_str());
                // delay.delay_millis(1);
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
                lcd.write_str_to_cur(nums.as_str());
                lcd.write_str_to_cur(s2.as_str());
                // delay.delay_millis(1);
            }else{
                led1.set_low();
            }
        });
        // log::info!("{:?}",BLINK_DELAY.load(Ordering::Relaxed));
        // delay.delay_millis(100);
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