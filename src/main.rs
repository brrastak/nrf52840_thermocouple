#![deny(unsafe_code)]
#![no_main]
#![no_std]


use display_interface_i2c::I2CInterface;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
};
use embedded_hal::digital::StatefulOutputPin;
use max31855::{Max31855, Unit, Error};
use micromath::F32Ext;
use nrf52840_hal as hal;
use hal::{
    gpio::*,
    clocks::Clocks,
    twim::{self, Twim},
    spi::{self, Spi},
    spim::MODE_0,
};
use panic_rtt_target as _;
use rtt_target::rtt_init_print;
// use rtt_target::rprintln;
use rtic_monotonics::nrf::timer::prelude::*;
// use smart_leds::{SmartLedsWrite, RGB8, colors};
use ssd1306::{prelude::*, Ssd1306};
use u8g2_fonts::{fonts, FontRenderer, types::*};
// use ws2812_spi as ws2812;
// use ws2812::Ws2812;


nrf_timer0_monotonic!(Mono, 125_000);



#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [UARTE0_UART0])]
mod app {

    use super::*;

    type DispType = Ssd1306<I2CInterface<Twim<you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::TWIM0>>, DisplaySize128x64, ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>>;


    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        led: Pin<Output<PushPull>>,
        spi: Spi<hal::pac::SPI1>,
        disp: DispType,
        thermo_cs: Pin<Output<PushPull>>
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {

        rtt_init_print!();

        let p = cx.device;
        let _clocks = Clocks::new(p.CLOCK).enable_ext_hfosc();
        Mono::start(p.TIMER0);

        let port0 = p0::Parts::new(p.P0);
        let led = port0.p0_24.into_push_pull_output(Level::Low).degrade();
        let disp_scl = port0.p0_05.into_floating_input().degrade();
        let disp_sda = port0.p0_04.into_floating_input().degrade();

        let _thermo_power = port0.p0_02.into_push_pull_output(Level::High);
        let _thermo_gnd = port0.p0_28.into_push_pull_output(Level::Low);
        let thermo_cs = port0.p0_29.into_push_pull_output(Level::High).degrade();
        let thermo_scl = port0.p0_30.into_push_pull_output(Level::Low).degrade();
        let thermo_sda = port0.p0_31.into_floating_input().degrade();

        // let scl = port1.p1_13.into_push_pull_output(Level::Low).degrade();
        // let rgb_led_pin = port1.p1_12.into_push_pull_output(Level::Low).degrade();

        // let rgb_spi_pins = spi::Pins {
        //     sck: Some(scl),
        //     miso: None,
        //     mosi: Some(rgb_led_pin),
        // };
        // let rgb_spi = Spi::new(
        //     p.SPI1,
        //     rgb_spi_pins,
        //     spi::Frequency::M2,
        //     MODE_0
        // );

        // let mut rgb = Ws2812::new(rgb_spi);
        // let color = [colors::GREEN];
        // rgb.write(color.iter().cloned()).unwrap();


        let i2c_pins = twim::Pins {
            scl: disp_scl, 
            sda: disp_sda
        };
        let i2c = Twim::new(p.TWIM0, i2c_pins, twim::Frequency::K100);

        let i2c_interface = I2CInterface::new(i2c, 0x3C, 0x40);

        let mut disp = Ssd1306::new(
            i2c_interface, 
            DisplaySize128x64, 
            DisplayRotation::Rotate180
        ).into_buffered_graphics_mode();
        disp.init().expect("Display initialization failure");
        disp.flush().unwrap();
        

        let pins = spi::Pins {
            sck: Some(thermo_scl),
            miso: Some(thermo_sda),
            mosi: None,
        };
        let spi = Spi::new(
            p.SPI1,
            pins,
            spi::Frequency::M2,
            MODE_0
        );


        heartbeat::spawn().ok();
        data::spawn().ok();

        (
            Shared {
            },
            Local {
               led,
               spi,
               disp,
               thermo_cs
            },
        )
    }


    // Get thermocouple data and display it
    #[task(local = [spi, disp, thermo_cs], priority = 1)]
    async fn data(cx: data::Context) {

        let data::LocalResources
            {spi, disp, thermo_cs, ..} = cx.local;

        let font = FontRenderer::new::<fonts::u8g2_font_logisoso16_tf>();

        loop {
            
            let mut buf = [0u8; 100];
            let message = match spi.read_all(thermo_cs, Unit::Celsius) {

                Ok(result) => {
                    
                    let room_temp = round_one_digit(result.internal);
                    let thermocouple = round_one_digit(result.internal + result.thermocouple);

                    let str: &str = format_no_std::show(
                        &mut buf, 
                        format_args!("{}°C\n{}°C", 
                        room_temp,
                        thermocouple)).unwrap();
                    str
                },
                Err(error) => match error {
                    
                    Error::MissingThermocoupleFault => "Connect\nthermocouple!",
                    _ => "Something\nis wrong!"
                }
            };

            font.render_aligned(
                message,
                disp.bounding_box().center(),
                VerticalPosition::Center,
                HorizontalAlignment::Center,
                FontColor::Transparent(BinaryColor::On),
                disp,
            )
            .unwrap();

            disp.flush().ok();

            Mono::delay(100.millis()).await;

            disp.clear(BinaryColor::Off).ok();
        }
    }

    // Blink on-board LED
    #[task(local = [led], priority = 1)]
    async fn heartbeat(cx: heartbeat::Context) {

        let heartbeat::LocalResources
            {led, ..} = cx.local;

        loop {
            led.toggle().ok();
            Mono::delay(1000.millis()).await;
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {

        loop {
            continue;
        }
    }
}


/// Save only one digit after the decimal point
fn round_one_digit(value: f32) -> f32 {

    (value * 10.0).round() / 10.0
}
