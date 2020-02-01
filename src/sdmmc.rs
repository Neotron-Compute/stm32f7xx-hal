//! Interface to the SDMMC peripheral. Currently only supports 4-bit mode (CLK, CMD, DAT[0..3]).
//!
//! See chapter 35 in the STM32F746 Reference Manual.

use as_slice::{AsMutSlice, AsSlice};
use core::marker::PhantomData;
use core::ops::DerefMut;
use core::pin::Pin;
pub use embedded_sdmmc::blockdevice::BlockDevice;

use crate::{
    device::SDMMC1,
    dma,
    gpio::{
        gpioc::{PC10, PC11, PC12, PC8, PC9},
        gpiod::PD2,
        Alternate, AF12,
    },
    rcc::Rcc,
    state,
};

// ---------------------------------------------------------------------------
//
// Public Types
//
// ---------------------------------------------------------------------------

/// This is our SDMMC controller object. It holds a low-level peripheral
/// instance (of type `I: Instance`), and some GPIO pins (of type `P:
/// Pins<I>`). You get a compile time error unless you pass in the right pins,
/// in the right mode. Look at the docs for `Pins` to see which pins are
/// available for which SDMMC peripheral.
pub struct Sdmmc<I, P, STATE>
where
    I: Instance,
    P: Pins<I>,
{
    sdmmc: I,
    pins: P,
    _state: STATE,
}

pub enum ClockEdge {
    Rising,
    Falling,
}

pub enum BusWidth {
    OneBit,
    FourBits,
    EightBits,
}

pub enum ClockSpeed {
    Slow,
    Fast,
}

pub struct InitConfig {
    pub clockEdge: ClockEdge,
    pub clockBypassEnabled: bool,
    pub clockPowerSaveEnabled: bool,
    pub busWidth: BusWidth,
    pub hardwareFlowControlEnabled: bool,
    pub clockSpeed: ClockSpeed,
}

pub enum Power {
    On,
    Up,
    Off,
}

pub enum ResponseRegister {
    CommandResponse,
    Response1,
    Response2,
    Response3,
    Response4,
}

/// Implemented for all instances of the SDMMC peripheral. Currently we only
/// support SDMMC1 (which is all the STM32F746 has), but you could imagine
/// other STM32 SoCs having multiple SDMMC interfaces.
///
/// Users of this crate should not implement this trait.
pub trait Instance {
    /// Initialise the SDMMC peripheral
    fn init(&mut self, config: &InitConfig);
    /// Control the SDMMC clock
    fn clock_enabled(&mut self, enabled: bool);
    /// Control the SD card power state
    fn set_power_state(&mut self, power: Power);
    /// Get the SD card power state
    fn get_power_state(&self) -> Result<Power, Error>;
    /// Read an arbitrary number of data bytes from the card. Always fills the
    /// given buffer, so make sure it's the right size. Buffer should be a
    /// multiple of four bytes long.
    fn read_data(&mut self, buffer: &mut [u8]) -> Result<(), Error>;
    /// Write an arbitrary number of data bytes to the card. Buffer should be
    /// a multiple of four bytes long.
    fn write_data(&mut self, buffer: &[u8]) -> Result<(), Error>;
    /// Perform a command.
    fn card_command(
        &mut self,
        command: u8,
        cpsm_enabled: bool,
        wait_for_interrupt: bool,
        wait_for_response: WaitForResponse,
        arg: u32,
    ) -> Result<(), Error>;
    /// Get a response from the card
    fn get_response(&mut self, reg: ResponseRegister) -> u32;
    /// Get the address of the data FIFO so we can DMA to/from it
    fn fifo_address(&self) -> u32;
}

#[derive(Debug)]
pub enum Error {
    FrameFormat,
    Overrun,
    ModeFault,
    UnknownValue,
}

/// RX token used for DMA transfers
pub struct Rx<I>(PhantomData<I>);

/// TX token used for DMA transfers
pub struct Tx<I>(PhantomData<I>);

/// A DMA transfer to the SDMMC peripheral / SDMMC card.
pub struct TransferOut<I: Instance, P: Pins<I>, Buffer, Tx: dma::Target, STATE> {
    buffer: Pin<Buffer>,
    target: Sdmmc<I, P, state::Enabled>,
    tx: dma::Transfer<Tx, dma::PtrBuffer<u8>, STATE>,
    _state: STATE,
}

/// A DMA transfer from the SDMMC peripheral / SDMMC card.
pub struct TransferIn<I: Instance, P: Pins<I>, Buffer, Rx: dma::Target, STATE> {
    buffer: Pin<Buffer>,
    target: Sdmmc<I, P, state::Enabled>,
    rx: dma::Transfer<Rx, dma::PtrBuffer<u8>, STATE>,
    _state: STATE,
}

/// Implemented for all tuples that contain a full set of valid SDMMC pins
pub trait Pins<I> {}

/// Implemented for all pins that can function as the SCK pin
///
/// Users of this crate should not implement this trait.
pub trait Sck<I> {}

/// Implemented for all pins that can function as the Cmd pin
///
/// Users of this crate should not implement this trait.
pub trait Cmd<I> {}

/// Implemented for all pins that can function as the Dat0 pin
///
/// Users of this crate should not implement this trait.
pub trait Dat0<I> {}

/// Implemented for all pins that can function as the Dat1 pin
///
/// Users of this crate should not implement this trait.
pub trait Dat1<I> {}

/// Implemented for all pins that can function as the Dat2 pin
///
/// Users of this crate should not implement this trait.
pub trait Dat2<I> {}

/// Implemented for all pins that can function as the Dat3 pin
///
/// Users of this crate should not implement this trait.
pub trait Dat3<I> {}

/// The resources that an ongoing transfer needs exclusive access to
pub struct TransferResources<I: Instance, P: Pins<I>, TRG: dma::Target, BUF> {
    pub stream: TRG::Stream,
    pub target: Sdmmc<I, P, state::Enabled>,
    pub buffer: Pin<BUF>,
}

/// Whether want to wait for a response from the card, and if so, what sort of response do we want.
pub enum WaitForResponse {
    No,
    Short,
    Long,
}

// ---------------------------------------------------------------------------
//
// Public Constants and Statics
//
// ---------------------------------------------------------------------------

// None

// ---------------------------------------------------------------------------
//
// Private Types
//
// ---------------------------------------------------------------------------

// None

// ---------------------------------------------------------------------------
//
// Private Constants and Statics
//
// ---------------------------------------------------------------------------

/// SDMMC Initialization Frequency (400KHz max) for a Peripheral CLK @ 200MHz
const SDMMC_INIT_CLK_DIV: u8 = 0xFA;

// Write to the pwrctrl field to power the card off
const POWER_OFF: u8 = 0x00;

// Write to the pwrctrl field to power the card up
const POWER_UP: u8 = 0x02;

// Write to the pwrctrl field to power the card on
const POWER_ON: u8 = 0x03;

// ---------------------------------------------------------------------------
//
// Impl on Public Types
//
// ---------------------------------------------------------------------------

impl<I, P> Sdmmc<I, P, state::Disabled>
where
    I: Instance,
    P: Pins<I>,
{
    /// Create a new instance of the SDMMC API
    pub fn new(instance: I, pins: P) -> Self {
        Self {
            sdmmc: instance,
            pins,
            _state: state::Disabled,
        }
    }

    /// Initialize the SDMMC peripheral
    pub fn enable(self, rcc: &mut Rcc) -> Sdmmc<I, P, state::Enabled> {
        // Reset the peripheral
        rcc.apb2.rstr().write(|w| w.sdmmc1rst().set_bit());
        // Enable the peripheral
        rcc.apb2.enr().modify(|_, w| w.sdmmc1en().set_bit());
        // Move our fields into an `Sdmmc` object with the Enabled type state
        Sdmmc {
            sdmmc: self.sdmmc,
            pins: self.pins,
            _state: state::Enabled,
        }
    }
}

pub struct SdCardInfo {
    size_blocks: u32,
}

/// TODO: Fix Me
const SD_CMD_GO_IDLE_STATE: u8 = 0;

impl<I, P> Sdmmc<I, P, state::Enabled>
where
    I: Instance,
    P: Pins<I>,
{
    pub fn init(&mut self) -> Result<SdCardInfo, Error> {
        // SDMMC Init, low speed
        self.sdmmc.init(&InitConfig {
            clockEdge: ClockEdge::Rising,
            clockBypassEnabled: false,
            clockPowerSaveEnabled: false,
            busWidth: BusWidth::OneBit,
            hardwareFlowControlEnabled: false,
            clockSpeed: ClockSpeed::Slow,
        });
        // Power ON
        self.power_on()?;
        // Initialise Cards
        // Get Card Info
        // Select Card
        // SDMMC Init, high speed
        Ok(SdCardInfo { size_blocks: 0 })
    }

    /// Go through the SD card's power-on sequence
    pub fn power_on(&mut self) -> Result<(), Error> {
        // Disable SDMMC clock
        self.sdmmc.clock_enabled(false);
        // Set power state to ON
        self.sdmmc.set_power_state(Power::On);
        // Sleep for at least 1ms
        let x = 0;
        for _ in 0..216_000 {
            unsafe {
                core::ptr::read_volatile(&x);
            }
        }
        // Enable SDMMC clock
        self.sdmmc.clock_enabled(true);
        // Send Command 0 - Go To Idle State
        self.sdmmc
            .card_command(SD_CMD_GO_IDLE_STATE, true, false, WaitForResponse::No, 0)?;
        // Check what the card says
        self.get_error()?;

        // Command  SD_SDMMC_SEND_IF_COND

        // Get SD_CmdResp7Error

        // Check what sort of SD card we have

        // Send CMD55

        // Get SD_CmdResp1Error

        // If OK, then it's an SD card. If it timesout, then it's an MMC card.
        // If SD card, loop until we find a voltage we're all happy with

        Ok(())
    }

    /// Get any error code from the card
    pub fn get_error(&mut self) -> Result<(), Error> {
        unimplemented!();
    }

    /// Start an SDMMC transfer using DMA
    ///
    /// Sends the data in `buffer` and writes the received data into buffer
    /// right after. Returns a [`Transfer`], to represent the ongoing SDMMC
    /// transfer.
    ///
    /// Please note that the word "transfer" is used with two different meanings
    /// here:
    /// - An SDMMC transfer, as in an SDMMC transaction that involves either sending
    ///   or receiving data. The method name refers to this kind of transfer.
    /// - A DMA transfer, as in an ongoing DMA operation. The name of the return
    ///   type refers to this kind of transfer.
    ///
    /// This method, as well as all other DMA-related methods in this module,
    /// requires references to two DMA handles, one each for the RX and TX
    /// streams. This will actually always be the same handle, as each SDMMC
    /// instance uses the same DMA instance for both sending and receiving. It
    /// would be nice to simplify that, but I believe that requires an equality
    /// constraint in the where clause, which is not supported yet by the
    /// compiler.
    pub fn transfer_out<B>(
        self,
        buffer: Pin<B>,
        dma_tx: &dma::Handle<<Tx<I> as dma::Target>::Instance, state::Enabled>,
        tx: <Tx<I> as dma::Target>::Stream,
    ) -> TransferOut<I, P, B, Tx<I>, dma::Ready>
    where
        Tx<I>: dma::Target,
        B: DerefMut + 'static,
        B::Target: AsMutSlice<Element = u8>,
    {
        // Create the RX/TX tokens for the transfer. Those must only exist once,
        // otherwise it would be possible to create multiple transfers trying to
        // use the same hardware resources.
        //
        // We guarantee that they only exist once by only creating them where we
        // have access to `self`, moving `self` into the `Transfer` while they
        // are in use, and dropping them when returning `self` from the
        // transfer.
        let tx_token = Tx(PhantomData);

        // We need to move a buffer into the `dma::Transfer` instance,
        // while keeping the original buffer around to return to the caller
        // later, when the transfer is finished.
        let tx_buffer = dma::PtrBuffer {
            ptr: buffer.as_slice().as_ptr(),
            len: buffer.as_slice().len(),
        };

        // Create the DMA transfer. This is safe, for the following
        // reasons:
        // 1. The trait bounds on this method guarantee that `buffer`, which we
        //    created the two buffer instances from, can be safely read from and
        //    written to.
        // 2. The semantics of the SDMMC peripheral guarantee that the buffer
        //    reads/writes are synchronized, preventing race conditions.
        let tx_transfer = unsafe {
            dma::Transfer::new(
                dma_tx,
                tx,
                Pin::new(tx_buffer),
                tx_token,
                self.sdmmc.fifo_address(),
                dma::Direction::MemoryToPeripheral,
            )
        };

        TransferOut {
            buffer,
            target: self,
            tx: tx_transfer,
            _state: dma::Ready,
        }
    }

    /// Start an SDMMC transfer using DMA
    ///
    /// Reads data into the `buffer`. Returns a [`Transfer`], to represent the ongoing SDMMC
    /// transfer.
    ///
    /// Please note that the word "transfer" is used with two different meanings
    /// here:
    /// - An SDMMC transfer, as in an SDMMC transaction that involves either sending
    ///   or receiving data. The method name refers to this kind of transfer.
    /// - A DMA transfer, as in an ongoing DMA operation. The name of the return
    ///   type refers to this kind of transfer.
    ///
    /// This method, as well as all other DMA-related methods in this module,
    /// requires references to two DMA handles, one each for the RX and TX
    /// streams. This will actually always be the same handle, as each SDMMC
    /// instance uses the same DMA instance for both sending and receiving. It
    /// would be nice to simplify that, but I believe that requires an equality
    /// constraint in the where clause, which is not supported yet by the
    /// compiler.
    pub fn transfer_in<B>(
        self,
        buffer: Pin<B>,
        dma_rx: &dma::Handle<<Rx<I> as dma::Target>::Instance, state::Enabled>,
        rx: <Rx<I> as dma::Target>::Stream,
    ) -> TransferIn<I, P, B, Rx<I>, dma::Ready>
    where
        Rx<I>: dma::Target,
        B: DerefMut + 'static,
        B::Target: AsMutSlice<Element = u8>,
    {
        // Create the RX/TX tokens for the transfer. Those must only exist once,
        // otherwise it would be possible to create multiple transfers trying to
        // use the same hardware resources.
        //
        // We guarantee that they only exist once by only creating them where we
        // have access to `self`, moving `self` into the `Transfer` while they
        // are in use, and dropping them when returning `self` from the
        // transfer.
        let rx_token = Rx(PhantomData);

        // We need to move a buffer into the `dma::Transfer` instance,
        // while keeping the original buffer around to return to the caller
        // later, when the transfer is finished.
        //
        // Here we create two `Buffer` from raw pointers acquired from `buffer`.
        let rx_buffer = dma::PtrBuffer {
            ptr: buffer.as_slice().as_ptr(),
            len: buffer.as_slice().len(),
        };

        // Create the DMA transfer. This is safe, for the following
        // reasons:
        // 1. The trait bounds on this method guarantee that `buffer`, which we
        //    created the two buffer instances from, can be safely read from and
        //    written to.
        // 2. The semantics of the SDMMC peripheral guarantee that the buffer
        //    reads/writes are synchronized, preventing race conditions.
        let rx_transfer = unsafe {
            dma::Transfer::new(
                dma_rx,
                rx,
                Pin::new(rx_buffer),
                rx_token,
                self.sdmmc.fifo_address(),
                dma::Direction::PeripheralToMemory,
            )
        };

        TransferIn {
            buffer,
            target: self,
            rx: rx_transfer,
            _state: dma::Ready,
        }
    }
}

impl<I, P, STATE> Sdmmc<I, P, STATE>
where
    I: Instance,
    P: Pins<I>,
{
    /// Destroy the peripheral API and return a raw SDMMC peripheral instance
    pub fn free(self) -> (I, P) {
        (self.sdmmc, self.pins)
    }
}

impl Instance for SDMMC1 {
    /// Initialise the SDMMC peripheral
    fn init(&mut self, config: &InitConfig) {
        self.clkcr.write(|w| {
            // HW Flow Control disable
            if config.clockPowerSaveEnabled {
                w.hwfc_en().set_bit();
            } else {
                w.hwfc_en().clear_bit();
            }
            // SDIO_CK dephasing selection bit
            match config.clockEdge {
                ClockEdge::Rising => {
                    w.negedge().clear_bit();
                }
                ClockEdge::Falling => {
                    w.negedge().set_bit();
                }
            }
            // Bus width
            unsafe {
                w.widbus().bits(match config.busWidth {
                    BusWidth::OneBit => 0b00,
                    BusWidth::FourBits => 0b01,
                    BusWidth::EightBits => 0b10,
                });
            }
            // Clock divider bypass enable bit
            if config.clockBypassEnabled {
                w.bypass().set_bit();
            } else {
                w.bypass().clear_bit();
            }
            // Power saving configuration bit
            if config.clockPowerSaveEnabled {
                w.pwrsav().set_bit();
            } else {
                w.pwrsav().clear_bit();
            }
            // Clock enable bit
            w.clken().set_bit();
            // Clock divide factor
            unsafe {
                w.clkdiv().bits(match config.clockSpeed {
                    ClockSpeed::Slow => SDMMC_INIT_CLK_DIV,
                    ClockSpeed::Fast => 1,
                });
            }
            w
        });
    }

    /// Control the SDMMC clock
    fn clock_enabled(&mut self, enabled: bool) {
        self.clkcr.modify(|_, w| w.clken().bit(enabled));
    }

    /// Control the SD card power state
    fn set_power_state(&mut self, power: Power) {
        self.power.write(|w| unsafe {
            w.pwrctrl().bits(match power {
                Power::On => POWER_ON,
                Power::Off => POWER_OFF,
                Power::Up => POWER_UP,
            })
        });
    }

    /// Get the SD card power state
    fn get_power_state(&self) -> Result<Power, Error> {
        match self.power.read().pwrctrl().bits() {
            POWER_ON => Ok(Power::On),
            POWER_UP => Ok(Power::Up),
            POWER_OFF => Ok(Power::Off),
            _ => Err(Error::UnknownValue),
        }
    }

    /// Read an arbitrary number of data bytes from the card. Always fills the
    /// given buffer, so make sure it's the right size.
    ///
    /// The length must be a multiple of four, because the SDMMC peripheral
    /// does 32-bit reads.
    fn read_data(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        for b in buffer.chunks_mut(4) {
            while self.sta.read().rxfifoe().bit_is_set() {
                // Spin waiting for FIFO data as FIFO is empty
            }
            let bytes = self.fifo.read().fifodata().bits().to_le_bytes();
            // Unpack the 32-bit word from the FIFO into four bytes
            b[0] = bytes[0];
            b[1] = bytes[1];
            b[2] = bytes[2];
            b[3] = bytes[3];
        }
        Ok(())
    }

    /// Write an arbitrary number of data bytes to the card.
    ///
    /// The length must be a multiple of four, because the SDMMC peripheral
    /// does 32-bit reads.
    fn write_data(&mut self, buffer: &[u8]) -> Result<(), Error> {
        for b in buffer.chunks(4) {
            // Bundle four bytes into a 32-bit word for the FIFO
            let bytes = [b[0], b[1], b[2], b[3]];
            let word = u32::from_le_bytes(bytes);
            while self.sta.read().txfifof().bit_is_set() {
                // Spin waiting for FIFO data as FIFO is full
            }
            self.fifo.write(|w| unsafe { w.fifodata().bits(word) });
        }
        Ok(())
    }

    /// Perform a command.
    fn card_command(
        &mut self,
        command: u8,
        cpsm_enabled: bool,
        wait_for_interrupt: bool,
        wait_for_response: WaitForResponse,
        arg: u32,
    ) -> Result<(), Error> {
        if command >= 0x40 {
            return Err(Error::UnknownValue);
        }
        self.arg.write(|w| unsafe { w.cmdarg().bits(arg) });
        self.cmd.modify(|_, w| unsafe {
            // Bit 10 - Command path state machine (CPSM) Enable bit
            w.cpsmen().bit(cpsm_enabled);
            // Bit 8 - CPSM waits for interrupt request
            w.waitint().bit(wait_for_interrupt);
            // Bits 6:7 - Wait for response bits
            match wait_for_response {
                WaitForResponse::No => {
                    w.waitresp().bits(0b00);
                }
                WaitForResponse::Short => {
                    w.waitresp().bits(0b01);
                }
                WaitForResponse::Long => {
                    w.waitresp().bits(0b11);
                }
            }
            // Bits 0:5 - Command index
            w.cmdindex().bits(command);
            w
        });
        Ok(())
    }

    /// Get a response from the card
    fn get_response(&mut self, reg: ResponseRegister) -> u32 {
        match reg {
            ResponseRegister::CommandResponse => self.respcmd.read().bits(),
            ResponseRegister::Response1 => self.resp1.read().bits(),
            ResponseRegister::Response2 => self.resp2.read().bits(),
            ResponseRegister::Response3 => self.resp3.read().bits(),
            ResponseRegister::Response4 => self.resp4.read().bits(),
        }
    }

    /// Get the address of the data FIFO so we can DMA to/from it
    fn fifo_address(&self) -> u32 {
        &self.fifo as *const _ as _
    }
}

impl<I, SCK, CMD, D0, D1, D2, D3> Pins<I> for (SCK, CMD, D0, D1, D2, D3)
where
    SCK: Sck<I>,
    CMD: Cmd<I>,
    D0: Dat0<I>,
    D1: Dat1<I>,
    D2: Dat2<I>,
    D3: Dat3<I>,
{
}

impl Sck<SDMMC1> for PC12<Alternate<AF12>> {}
impl Cmd<SDMMC1> for PD2<Alternate<AF12>> {}
impl Dat0<SDMMC1> for PC8<Alternate<AF12>> {}
impl Dat1<SDMMC1> for PC9<Alternate<AF12>> {}
impl Dat2<SDMMC1> for PC10<Alternate<AF12>> {}
impl Dat3<SDMMC1> for PC11<Alternate<AF12>> {}

impl<I, P, Buffer, Tx> TransferOut<I, P, Buffer, Tx, dma::Ready>
where
    Tx: dma::Target,
    I: Instance,
    P: Pins<I>,
{
    /// Enables the given interrupts for this DMA transfer
    ///
    /// These interrupts are only enabled for this transfer. The settings
    /// doesn't affect other transfers, nor subsequent transfers using the same
    /// DMA streams.
    pub fn enable_interrupts(
        &mut self,
        tx_handle: &dma::Handle<Tx::Instance, state::Enabled>,
        interrupts: dma::Interrupts,
    ) {
        self.tx.enable_interrupts(tx_handle, interrupts);
    }

    /// Start the DMA transfer
    ///
    /// Consumes this instance of `Transfer` and returns another instance with
    /// its type state set to indicate the transfer has been started.
    pub fn start(
        self,
        tx_handle: &dma::Handle<Tx::Instance, state::Enabled>,
    ) -> TransferOut<I, P, Buffer, Tx, dma::Started> {
        TransferOut {
            buffer: self.buffer,
            target: self.target,
            tx: self.tx.start(tx_handle),
            _state: dma::Started,
        }
    }
}

impl<I, P, Buffer, Rx> TransferIn<I, P, Buffer, Rx, dma::Ready>
where
    Rx: dma::Target,
    I: Instance,
    P: Pins<I>,
{
    /// Enables the given interrupts for this DMA transfer
    ///
    /// These interrupts are only enabled for this transfer. The settings
    /// doesn't affect other transfers, nor subsequent transfers using the same
    /// DMA streams.
    pub fn enable_interrupts(
        &mut self,
        rx_handle: &dma::Handle<Rx::Instance, state::Enabled>,
        interrupts: dma::Interrupts,
    ) {
        self.rx.enable_interrupts(rx_handle, interrupts);
    }

    /// Start the DMA transfer
    ///
    /// Consumes this instance of `Transfer` and returns another instance with
    /// its type state set to indicate the transfer has been started.
    pub fn start(
        self,
        rx_handle: &dma::Handle<Rx::Instance, state::Enabled>,
    ) -> TransferIn<I, P, Buffer, Rx, dma::Started> {
        TransferIn {
            buffer: self.buffer,
            target: self.target,
            rx: self.rx.start(rx_handle),
            _state: dma::Started,
        }
    }
}

// As `TransferResources` is used in the error variant of `Result`, it needs a
// `Debug` implementation to enable stuff like `unwrap` and `expect`. This can't
// be derived without putting requirements on the type arguments.
impl<I, P, TRG, BUF> core::fmt::Debug for TransferResources<I, P, TRG, BUF>
where
    TRG: dma::Target,
    I: Instance,
    P: Pins<I>,
{
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "TransferResources {{ .. }}")
    }
}

// ---------------------------------------------------------------------------
//
// Impl on Private Types
//
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
//
// End of File
//
// ---------------------------------------------------------------------------
