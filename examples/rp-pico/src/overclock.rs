use bsp::hal::{
    clocks::{Clock, ClocksManager, ClockSource, InitError},
    pac,
    pac::vreg_and_chip_reset::vreg::VSEL_A,
    pll::{setup_pll_blocking, PLLConfig},
    vreg::set_voltage,
    xosc::setup_xosc_blocking,
    Watchdog,
};
use fugit::{HertzU32, RateExtU32};
use rp_pico as bsp;

const PLL_SYS_192MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1536), // 12MHz * 128
    refdiv: 1,
    post_div1: 4,
    post_div2: 2,
};

const PLL_SYS_240MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1440), // 12MHz * 120
    refdiv: 1,
    post_div1: 6,
    post_div2: 1,
};

const PLL_SYS_256MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1536),
    refdiv: 1,
    post_div1: 6,
    post_div2: 1,
};

pub fn init_clocks(
    xosc_dev: pac::XOSC,
    clocks_dev: pac::CLOCKS,
    pll_sys_dev: pac::PLL_SYS,
    pll_usb_dev: pac::PLL_USB,
    vreg_dev: &mut pac::VREG_AND_CHIP_RESET,
    resets: &mut pac::RESETS,
    watchdog: &mut Watchdog,
) -> Result<ClocksManager, InitError> {
    set_voltage(vreg_dev, VSEL_A::VOLTAGE1_20);
    cortex_m::asm::delay(1000);

    let xosc = setup_xosc_blocking(xosc_dev, 12_000_000u32.Hz())
        .map_err(InitError::XoscErr)?;
    watchdog.enable_tick_generation(12u8);

    let mut clocks = ClocksManager::new(clocks_dev);

    let pll_sys = setup_pll_blocking(
        pll_sys_dev, xosc.operating_frequency(), PLL_SYS_240MHZ, &mut clocks, resets,
    ).map_err(InitError::PllError)?;

    let pll_usb = setup_pll_blocking(
        pll_usb_dev, xosc.operating_frequency(),
        bsp::hal::pll::common_configs::PLL_USB_48MHZ, &mut clocks, resets,
    ).map_err(InitError::PllError)?;

    clocks.reference_clock.configure_clock(&xosc, xosc.get_freq()).unwrap();
    clocks.system_clock.configure_clock(&pll_sys, pll_sys.get_freq()).unwrap();
    clocks.usb_clock.configure_clock(&pll_usb, pll_usb.get_freq()).unwrap();
    clocks.adc_clock.configure_clock(&pll_usb, pll_usb.get_freq()).unwrap();
    clocks.rtc_clock.configure_clock(&pll_usb, 46875u32.Hz()).unwrap();
    clocks.peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .unwrap();

    Ok(clocks)
}