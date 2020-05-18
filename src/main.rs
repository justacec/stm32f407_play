#![no_std]
#![no_main]

extern crate cortex_m_rt as rt;
extern crate stm32f4;
extern crate panic_itm;
extern crate byte;
//extern crate panic_semihosting;

use cortex_m::asm;
use cortex_m::iprintln;
use core::ops::{Deref, DerefMut};

use rtfm::app;
use rtfm::cyccnt::{U32Ext};
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    gpio::gpioa::{PA6},
    gpio::gpiob::{PB11},
    gpio::gpioe::{PE4},
    gpio::{ExtiPin, Edge, Output, PushPull, Input, PullUp},
    timer::{Timer, Event},
    spi::{Phase, Polarity},
    interrupt
};
use num_derive::FromPrimitive;
use heapless::Vec; // fixed capacity `std::Vec`
use heapless::consts::U64; // type level integer used to specify capacity
use bbqueue::{BBBuffer, ConstBBBuffer,
              framed::{FrameConsumer, FrameProducer, FrameGrantW, FrameGrantR}, 
              consts::*};
use byte::{BytesExt, LE};

#[allow(unused_macros)]
macro_rules! myprintln {
//    ($cx:ident, $fmt:expr) => ();
//    ($cx:ident, $fmt:expr, $($arg:tt)*) => ();
    ($cx:ident, $fmt:expr) => (iprintln!(&mut $cx.resources.itm.stim[0], $fmt));
    ($cx:ident, $fmt:expr, $($arg:tt)*) => (iprintln!(&mut $cx.resources.itm.stim[0], $fmt, $($arg)*));
}

// Extension code for the heapless vec to provide the command and the data seperatly
trait SplitCommandData {
    fn get_command(&self) -> Command;
    fn get_data(&self) -> Self;
}

impl SplitCommandData for heapless::Vec<u8,U64> {
    fn get_command(& self) -> Command {
        if self.len() == 0 {
            return Command::None
        }

        let bytes: u16 = ((self[0] as u16) << 8) | (self[1] as u16);
        match num::FromPrimitive::from_u16(bytes) {
            Some(cmd) => cmd,
            None => Command::None
        }
    }

    fn get_data(&self) -> Self {
        let mut ret = Self::new();

        for i in 2..self.len() {
            ret.push(self[i]).unwrap();
        }

        ret
    }
}

// Define the possible SPI states
#[derive(PartialEq)]
pub enum SPIState {
    WaitForCommand,
    SendingResponse
}

#[derive(FromPrimitive, Debug)]
pub enum Command {
    None = 0x0000,
    Echo = 0x0001,
}

// Define the BBQueue queue's
static rx_queue: BBBuffer<U1024> = BBBuffer( ConstBBBuffer::new() );
static tx_queue: BBBuffer<U1024> = BBBuffer( ConstBBBuffer::new() );

#[app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        spi: stm32::SPI2,
        #[init(SPIState::WaitForCommand)]
        spi_state: SPIState,
        spi_busy: PB11<Output<PushPull>>,
        dma: stm32::DMA1,
        led: PA6<Output<PushPull>>,
        #[init(false)]
        led_state: bool,
        button: PE4<Input<PullUp>>,
        timer: Timer<stm32::TIM2>,
        itm: stm32f4::stm32f407::ITM,
        rx_queue_producer: FrameProducer<'static, U1024>,
        rx_queue_consumer: FrameConsumer<'static, U1024>,
        tx_queue_producer: FrameProducer<'static, U1024>,
        tx_queue_consumer: FrameConsumer<'static, U1024>,
        rx_queue_read_grant: Option<FrameGrantR<'static, U1024>>,
        rx_queue_write_grant: Option<FrameGrantW<'static, U1024>>,
        tx_queue_read_grant: Option<FrameGrantR<'static, U1024>>,
        tx_queue_write_grant: Option<FrameGrantW<'static, U1024>>,
        #[init(false)]
        process_requests_state: bool,
        #[init(false)]
        transmit_command_results_state: bool
    }

    #[init()]
    fn init(cx: init::Context) -> init::LateResources {

        // Cortex-M peripherals
        let mut _core: cortex_m::Peripherals = cx.core;

        // Device specific peripherals
        let mut _device: stm32::Peripherals = cx.device;

        _device.RCC.apb2enr.write(|w| w.syscfgen().enabled()); 

        // Specify and fix the clock speeds
        let rcc = _device.RCC.constrain();

        let _clocks = rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(168.mhz())
//            .pclk1(32.mhz())
            .freeze();


        // Initialize (enable) the monotonic timer (CYCCNT)
        _core.DCB.enable_trace();
        _core.DWT.enable_cycle_counter();

        let stim0 = &mut _core.ITM.stim[0];
        iprintln!(stim0, "Starting Motorcontroller...");
        
        let gpioa = _device.GPIOA.split();
        let gpiob = _device.GPIOB.split();

        let mut led = gpioa.pa6.into_push_pull_output();
        led.set_high().unwrap();

        let gpioe = _device.GPIOE.split();
        let mut button = gpioe.pe4.into_pull_up_input();
        button.make_interrupt_source(&mut _device.SYSCFG);
        button.enable_interrupt(&mut _device.EXTI);
        button.trigger_on_edge(&mut _device.EXTI, Edge::FALLING);

        // Estblish a debounce timer
        let timer = Timer::tim2(_device.TIM2, ((1.0/(50.0 * 1.0e-3)) as u32).hz(), _clocks);
        cortex_m::peripheral::NVIC::mask(stm32::Interrupt::TIM2);

        unsafe{
            cortex_m::peripheral::NVIC::unmask(stm32::Interrupt::EXTI4);
            _core.NVIC.set_priority(stm32::Interrupt::EXTI4, 3);
        };
        cortex_m::peripheral::NVIC::unpend(stm32::Interrupt::TIM2);
        cortex_m::peripheral::NVIC::unpend(stm32::Interrupt::EXTI4);

        // Manually create a slave SPI device
        // Manually configure the SPI2 deivce to be a slave
        let mut _ack = gpiob.pb11.into_push_pull_output();
        let mut _ss = gpiob.pb12.into_pull_up_input();
        let _sck = gpiob.pb13.into_alternate_af5();
        let _miso = gpiob.pb14.into_alternate_af5();
        let _mosi = gpiob.pb15.into_alternate_af5();

        _ack.set_low().unwrap();

        // Turn on the SPI device clock
        let rcc_bus_pointer: *const stm32f4::stm32f407::rcc::RegisterBlock = stm32f4xx_hal::stm32::RCC::ptr();
        unsafe{ 
            (*rcc_bus_pointer).apb1enr.modify(|_, w| w.spi2en().set_bit());
            (*rcc_bus_pointer).apb1rstr.modify(|_, w| w.spi2rst().set_bit());
            (*rcc_bus_pointer).apb1rstr.modify(|_, w| w.spi2rst().clear_bit());
        }

        // Configure the SPI device registers
        _device.SPI2.cr1.write(|w| w
            // 8-bit frames
            .dff().clear_bit()
            .cpol().bit(Polarity::IdleLow == Polarity::IdleHigh)
            .cpha().bit(Phase::CaptureOnFirstTransition == Phase::CaptureOnSecondTransition)
            // MSB transmitted first
            .lsbfirst().clear_bit()
            // Use hardware SS line
            .ssm().clear_bit()
            // Set as slave mode
            .mstr().clear_bit()
            // Enable the peripheral
            .spe().set_bit()
        );

        // Disable the TI mode
        _device.SPI2.cr2.write(|w| w
            .frf().clear_bit()
        );

        // Enable the receive interrupt
        _device.SPI2.cr2.write(|w| w
            .rxneie().set_bit()
        );

        // Enable the SS line to trigger SPI data transmission stops
        _ss.make_interrupt_source(&mut _device.SYSCFG);
        _ss.enable_interrupt(&mut _device.EXTI);
        _ss.trigger_on_edge(&mut _device.EXTI, Edge::RISING);

        // Enable the DMA (Proceedure outlined on page 322 of the RM0090 from ST)
        // Turn on the DMA clock
        let rcc_bus_pointer: *const stm32f4::stm32f407::rcc::RegisterBlock = stm32f4xx_hal::stm32::RCC::ptr();
        unsafe{ 
            (*rcc_bus_pointer).ahb1enr.modify(|_, w| w.dma1en().set_bit());
            (*rcc_bus_pointer).ahb1rstr.modify(|_, w| w.dma1rst().set_bit());
            (*rcc_bus_pointer).ahb1rstr.modify(|_, w| w.dma1rst().clear_bit());
        }
        
        // Receive Stream
        if _device.DMA1.st[3].cr.read().en() == true {
            _device.DMA1.st[3].cr.modify(|_,w| w.en().clear_bit());
            // Should wait here, and double check that the bit went to zero
            // after a read, but this is the startup process and I am pretty
            // sure there are no stale operations going on here to watch out
            // for.
        } 
        unsafe {
            _device.DMA1.st[3].par.modify(|_,w| w.bits(0x4000_3800 + 0x0c));               // Set the peripheral address (SPI2 DR)
            _device.DMA1.st[3].ndtr.modify(|_,w| w.bits(32));                              // Set number of bytes to read to 32
            _device.DMA1.st[3].cr.modify(|_,w| w.chsel().bits(0));                         // Use Channel 0
            _device.DMA1.st[3].cr.modify(|_,w| w.pfctrl().clear_bit());                    // Explicitly disable peripheral flow control
            _device.DMA1.st[3].cr.modify(|_,w| w.pl().bits(0x10));                         // Set the priority to 2 (TX will have 3)
            _device.DMA1.st[3].fcr.modify(|_,w| w.dmdis().clear_bit());                    // Ensure the FIFO is disabled
            _device.DMA1.st[3].cr.modify(|_,w| w.msize().bits(8));                         // Set memory chunk size
            _device.DMA1.st[3].cr.modify(|_,w| w.psize().bits(8));                         // Set peripheral chunk size
            _device.DMA1.st[3].cr.modify(|_,w| w.minc().set_bit());                        // The memory location should shift after each byte
            _device.DMA1.st[3].cr.modify(|_,w| w.pinc().clear_bit());                      // The peripheral location should shift after each byte
            _device.DMA1.st[3].cr.modify(|_,w| w.dir().bits(0x00));                        // Set the direction to "Peripheral to Memory"

        }

        // Transmit Stream
        if _device.DMA1.st[4].cr.read().en() == true {
            _device.DMA1.st[4].cr.modify(|_,w| w.en().clear_bit());
            // Should wait here, and double check that the bit went to zero
            // after a read, but this is the startup process and I am pretty
            // sure there are no stale operations going on here to watch out
            // for.
        } 
        unsafe {
            _device.DMA1.st[4].par.modify(|_,w| w.bits(0x4000_3800 + 0x0c));               // Set the peripheral address (SPI2 DR)
            _device.DMA1.st[4].ndtr.modify(|_,w| w.bits(32));                              // Set number of bytes to read to 32
            _device.DMA1.st[4].cr.modify(|_,w| w.chsel().bits(0));                         // Use Channel 0
            _device.DMA1.st[4].cr.modify(|_,w| w.pfctrl().clear_bit());                    // Explicitly disable peripheral flow control
            _device.DMA1.st[4].cr.modify(|_,w| w.pl().bits(0x11));                         // Set the priority to 3
            _device.DMA1.st[4].fcr.modify(|_,w| w.dmdis().clear_bit());                    // Ensure the FIFO is disabled
            _device.DMA1.st[4].cr.modify(|_,w| w.msize().bits(8));                         // Set memory chunk size
            _device.DMA1.st[4].cr.modify(|_,w| w.psize().bits(8));                         // Set peripheral chunk size
            _device.DMA1.st[4].cr.modify(|_,w| w.minc().set_bit());                        // The memory location should shift after each byte
            _device.DMA1.st[4].cr.modify(|_,w| w.pinc().clear_bit());                      // The peripheral location should shift after each byte
            _device.DMA1.st[4].cr.modify(|_,w| w.dir().bits(0x01));                        // Set the direction to "Memory to Peripheral"                    

            _device.DMA1.st[4].cr.modify(|_,w| w.tcie().set_bit());                         // Enable inturrupt generation for DMA
        }
        
        _device.SPI2.cr2.modify(|_,w| w.rxdmaen().set_bit());
        _device.SPI2.cr2.modify(|_,w| w.txdmaen().set_bit());

        unsafe {
            _device.EXTI.imr.modify(|_,w| w.mr12().set_bit());
            _device.EXTI.rtsr.modify(|_,w| w.tr12().set_bit());
            _core.NVIC.set_priority(interrupt::EXTI15_10, 1);
            cortex_m::peripheral::NVIC::unmask(interrupt::EXTI15_10);
        }

        // Split the rx and tx queue's
        let (rx_prod, rx_cons) = rx_queue.try_split_framed().unwrap();
        let (tx_prod, tx_cons) = tx_queue.try_split_framed().unwrap();

        // Call pend the spi_complete function in order to setup the rx_queue memory locations in DMA
        cortex_m::peripheral::NVIC::pend(interrupt::EXTI15_10);

        init::LateResources {
            spi: _device.SPI2,
            spi_busy: _ack,
            dma: _device.DMA1,
            button: button,
            led: led,
            timer: timer,
            itm: _core.ITM,
            rx_queue_producer: rx_prod,
            rx_queue_consumer: rx_cons,
            tx_queue_producer: tx_prod,
            tx_queue_consumer: tx_cons,
            rx_queue_read_grant: None,            
            rx_queue_write_grant: None,            
            tx_queue_read_grant: None,            
            tx_queue_write_grant: None            
        }
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    #[task(binds=EXTI4, resources=[led_state, led, button, timer])]
    fn exti4_ISR(cx: exti4_ISR::Context) {
        cortex_m::peripheral::NVIC::mask(stm32::Interrupt::EXTI4);

        cx.resources.timer.start(((1.0/(50.0 * 1.0e-3)) as u32).hz());
        cx.resources.timer.listen(Event::TimeOut);

        cx.resources.button.clear_interrupt_pending_bit();
        unsafe { stm32::NVIC::unmask(stm32::Interrupt::TIM2) };

        match cx.resources.led_state {
            true => {
                cx.resources.led.set_low().unwrap();
                *cx.resources.led_state = false;
            },
            false => {
                cx.resources.led.set_high().unwrap();
                *cx.resources.led_state = true;
            }
        }
    }

    // This function is used for the switch debounceing proceedure
    #[task(binds=TIM2, resources=[timer])]
    fn tim2_ISR(cx: tim2_ISR::Context) {
        cortex_m::peripheral::NVIC::mask(stm32::Interrupt::TIM2);
        unsafe { cortex_m::peripheral::NVIC::unmask(stm32::Interrupt::EXTI4) };
        cx.resources.timer.clear_interrupt(Event::TimeOut);
    }

    // -------------------------------------------
    // Tx of Command Response Section
    // -------------------------------------------
    
    // This function will be executed when the return DMA counter goes to zero
    // The point is ot set the spi_state back to WaitForCommand and to 
    // release the grant for the tx.
    #[task(binds=DMA1_STREAM4, spawn = [transmit_command_results], resources = [dma, spi, spi_state, itm, tx_queue_read_grant, spi_busy])]
    fn tx_dma_complete(cx: tx_dma_complete::Context) {

        // Clear the inturrupt pend
        cx.resources.dma.hifcr.write(|w| w.ctcif4().set_bit());
        
        // Stop both of the DMA channels
        cx.resources.dma.st[3].cr.modify(|_,w| w.en().clear_bit());
        cx.resources.dma.st[4].cr.modify(|_,w| w.en().clear_bit());
 
        // Free up the memory in the tx queue
        cx.resources.tx_queue_read_grant.take().unwrap().release();

        // Tell the master that the data is completed
        cx.resources.spi_busy.set_low().unwrap();

        // Set the SPI DR to 0 to clear out the tx buffer
        unsafe {
            cx.resources.spi.dr.write(|w| w.bits(0));
        }

        // Spawn the transmit_results again
        // If there are no more results to send, it will set the flags accordingly
        cx.spawn.transmit_command_results().unwrap();
    }

    // This is the primary task  which sets up the SPI/DMA for return data mode
    // It is called once for each commited write grant into the tx_queue
    // In the event that there are no more data chunks in the tx_queue, 
    // it will reset the state back to "Wait for Command" mode
    #[task(resources = [dma, spi, spi_busy, spi_state, tx_queue_consumer, itm, tx_queue_read_grant, transmit_command_results_state])]
    fn transmit_command_results(cx: transmit_command_results::Context) {
        // Only operate on a single return
        // If there are multiple returns in the queue, this will be respawned from the 
        // SPI completion IRQ
        match cx.resources.tx_queue_consumer.read() {
            None => {
                *cx.resources.spi_state = SPIState::WaitForCommand;
                return
            },
            Some(request) => {
                // Check to make sure that we are not in the middle of a command reception
                let rx_ndtr = cx.resources.dma.st[3].ndtr.read().bits() as u32;
                loop {
                    if rx_ndtr == 32 {
                        break;
                    }
                }

                // Set the memory locaiton here of the buffer for the DMA
                let grant_address: u32 = request.deref().as_ptr() as u32;
                unsafe {
                    cx.resources.dma.st[4].m0ar.write(|w| w.bits(grant_address + 1));
                    cx.resources.spi.dr.write(|w| w.bits(request.deref()[0] as u32));
                }
                        
                // Set the size
                unsafe {
                    // Reset the counter to the max message size
                    cx.resources.dma.st[4].ndtr.write(|w| w.bits(request.deref().len() as u32));
        
                    // Cler the previous event flag so the DMA can be re-enabled
                    // [Yea, lost a lot of time here... :/]
                    cx.resources.dma.hifcr.write(|w| w.ctcif4().set_bit());
                }
        
                // Set the resources object so tha the tx complete can release the memory
                *cx.resources.tx_queue_read_grant = Some(request);

                // Disable RX DMA
                cx.resources.dma.st[3].cr.modify(|_,w| w.en().clear_bit());

                // Enable TX DMA
                cx.resources.dma.st[4].cr.modify(|_,w| w.en().set_bit());
        
                *cx.resources.spi_state = SPIState::SendingResponse;

                // Tell the master stuff is ready
                cx.resources.spi_busy.set_high().unwrap();
            }
        }
    }

    // -------------------------------------------
    // Rx of Command and Processing Section
    // -------------------------------------------

    #[task(spawn = [transmit_command_results], capacity = 4, resources = [tx_queue_producer, rx_queue_consumer, itm, tx_queue_write_grant, process_requests_state])]
    fn process_command(cx: process_command::Context) {
        // Check to see if we are already running...
        if *cx.resources.process_requests_state == true {
            return;
        } else {
            *cx.resources.process_requests_state = true
        }

        // Try to get a read from the rx_queue
        loop {
            match cx.resources.rx_queue_consumer.read() {
                // There is nothing else to process and we should exit this routine
                None => {
                    *cx.resources.process_requests_state = false;
                    return
                },
                // There is a request avaialbe to process
                Some(request) => {
                    // Extract the data from the rx_queue
                    let bytes = request.deref();

                    if bytes.len() == 0 {
                        // Release the memory in the rx_queue
                        request.release();

                        continue;
                    }

                    // Extract the message data
                    let message_id: u16 = bytes.read_with::<u16>(&mut 0, LE).unwrap();
                    let command_raw: u16 = bytes.read_with::<u16>(&mut 2, LE).unwrap();
                    let command: Command = num::FromPrimitive::from_u16(command_raw).unwrap_or(Command::None);
                    let data = Vec::<u8, U32>::from_slice(&bytes[4..]).unwrap();

                    // Release the memory in the rx_queue
                    request.release();

                    // A temporary place to store the packaed data
                    // Each command type is respoinsible for packing it correctly
                    let mut out_buffer: Vec<u8, U32> = Vec::new();

                    // Process the command
                    match command {
                        Command::None => {},
                        Command::Echo => {
                            // Copy the data into the buffers
                            out_buffer.extend_from_slice(&data).unwrap();
                        }                        
                    };

                    // Fill the tx_queue

                    // Open a tx_queue write grant
                    let write_grant = cx.resources.tx_queue_producer.grant(32);
                    match write_grant {
                        Ok(mut grant) => {
                            // Pack the data
                            let n_bytes = out_buffer.len();
                            
                            // Put the data in
                            grant.deref_mut().write_with(&mut 0, message_id as u16, LE).unwrap();
                            grant.deref_mut().write_with(&mut 2, n_bytes as u16, LE).unwrap();
                            grant.deref_mut()[4..(out_buffer.len()+4)].copy_from_slice(&out_buffer);

                            // Commit the tx_queue write grant
                            grant.commit(n_bytes + 4);

                            // Spawn the transmission process
                            cx.spawn.transmit_command_results().unwrap();
                        },
                        Err(_) => {
                            iprintln!(&mut cx.resources.itm.stim[0], "Unable to get write grant for TX Queue");
                        }
                    }
                }
            }
        }
    }

    // This function is called when the SPI master stops talking to the slave and sets the NSS line high
    // This function will then call the process command function
    #[task(binds=EXTI15_10, spawn=[process_command], resources = [dma, spi, spi_state, itm, rx_queue_producer, rx_queue_write_grant, process_requests_state])]
    fn spi_complete(cx: spi_complete::Context) {

        // I have lost track of what this actaully does
        let exti_pointer: *const stm32f4::stm32f407::exti::RegisterBlock = stm32f4xx_hal::stm32::EXTI::ptr();
        unsafe {
            (*exti_pointer).pr.modify(|_,w| w.pr12().set_bit());
        }

        // If the process is to return some data, then just return here
        if *cx.resources.spi_state == SPIState::SendingResponse {
            return;
        }

        // Stop both of the DMA channels
        cx.resources.dma.st[3].cr.modify(|_,w| w.en().clear_bit());
        cx.resources.dma.st[4].cr.modify(|_,w| w.en().clear_bit());

        if cx.resources.rx_queue_write_grant.is_some() {
            // Get the size of the data that was read in
            let n_bytes = 32 - cx.resources.dma.st[3].ndtr.read().bits() as u32;

            // Commit the grant
            cx.resources.rx_queue_write_grant.take().unwrap().commit(n_bytes as usize);

            // Set the rx_prod_grant to None
            // Maybe this line is not needed because of the `take()` in the above line?
            *cx.resources.rx_queue_write_grant = None;

            // Fire off the process command task
            if *cx.resources.process_requests_state == false {
                cx.spawn.process_command().unwrap();
            }
        }

        // Get a grant
        *cx.resources.rx_queue_write_grant = Some(cx.resources.rx_queue_producer.grant(32).unwrap());

        // Set the memory address of the DMA
        let grant_address: u32 = cx.resources.rx_queue_write_grant.as_ref().unwrap().deref().as_ptr() as u32;
        unsafe {
            cx.resources.dma.st[3].m0ar.write(|w| w.bits(grant_address));
        }

        // Ensure the rx dma is enabled here
        unsafe {
            // Cler the previous event flag so the DMA can be re-enabled
            // [Yea, lost a lot of time here... :/]
            cx.resources.dma.lifcr.write(|w| w.ctcif3().set_bit());

            // Reset the counter to the max message size
            cx.resources.dma.st[3].ndtr.write(|w| w.bits(32));
        }

        // Read the DR on the SPI to ensure that the OVR flag is cleared
        cx.resources.spi.dr.read();

        cx.resources.dma.st[3].cr.modify(|_,w| w.en().set_bit());
    }

    extern "C" {
        fn USART1();
        fn USART2();
    }

};
