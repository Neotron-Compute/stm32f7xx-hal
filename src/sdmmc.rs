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

/// Entry point to the SDMMC API
pub struct Sdmmc<I, P, State> {
    sdmmc: I,
    pins: P,
    _state: State,
}

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
    pub fn enable(self, _rcc: &mut Rcc) -> Sdmmc<I, P, Enabled> {
        // TODO enable the peripheral here
        Sdmmc {
            sdmmc: self.sdmmc,
            pins: self.pins,
            _state: Enabled,
        }
    }
}

impl<I, P> Sdmmc<I, P, Enabled>
where
    I: Instance,
    P: Pins<I>,
{
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

impl<I, P, State> Sdmmc<I, P, State>
where
    I: Instance,
    P: Pins<I>,
{
    /// Destroy the peripheral API and return a raw SDMMC peripheral instance
    pub fn free(self) -> (I, P) {
        (self.sdmmc, self.pins)
    }
}

/// Implemented for all instances of the SDMMC peripheral
///
/// Users of this crate should not implement this trait.
pub trait Instance {
    fn enable_clock(&self, rcc: &mut Rcc);
    fn configure(&self, br: u8, cpol: bool, cpha: bool);
    fn read(&self) -> nb::Result<u8, Error>;
    fn send(&self, word: u8) -> nb::Result<(), Error>;
    fn fifo_address(&self) -> u32;
}

/// Implemented for all tuples that contain a full set of valid SDMMC pins
pub trait Pins<I> {}

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

impl Sck<SDMMC1> for PC12<Alternate<AF12>> {}
impl Cmd<SDMMC1> for PD2<Alternate<AF12>> {}
impl Dat0<SDMMC1> for PC8<Alternate<AF12>> {}
impl Dat1<SDMMC1> for PC9<Alternate<AF12>> {}
impl Dat2<SDMMC1> for PC10<Alternate<AF12>> {}
impl Dat3<SDMMC1> for PC11<Alternate<AF12>> {}

impl Instance for SDMMC1 {
    fn enable_clock(&self, _rcc: &mut Rcc) {
        unimplemented!();
    }

    // I don't like putting this much code into the macro, but I
    // have to: There are two different SDMMC variants in the PAC, and
    // while I haven't found any actual differences between them,
    // they're still using different sets of types, and I need to
    // generate different methods to interface with them, even
    // though these methods end up looking identical.
    //
    // Maybe this is a problem in the SVD file that can be fixed
    // there.

    fn configure(&self, _br: u8, _cpol: bool, _cpha: bool) {
        unimplemented!();
    }

    fn read(&self) -> nb::Result<u8, Error> {
        unimplemented!();
    }

    fn send(&self, _word: u8) -> nb::Result<(), Error> {
        unimplemented!();
    }

    fn fifo_address(&self) -> u32 {
        &self.fifo as *const _ as _
    }
}

#[derive(Debug)]
pub enum Error {
    FrameFormat,
    Overrun,
    ModeFault,
}

/// RX token used for DMA transfers
pub struct Rx<I>(PhantomData<I>);

/// TX token used for DMA transfers
pub struct Tx<I>(PhantomData<I>);

/// A DMA transfer to the SDMMC peripheral / SDMMC card.
pub struct TransferOut<I, P, Buffer, Tx: dma::Target, State> {
    buffer: Pin<Buffer>,
    target: Sdmmc<I, P, Enabled>,
    tx: dma::Transfer<Tx, dma::PtrBuffer<u8>, State>,
    _state: State,
}

/// A DMA transfer from the SDMMC peripheral / SDMMC card.
pub struct TransferIn<I, P, Buffer, Rx: dma::Target, State> {
    buffer: Pin<Buffer>,
    target: Sdmmc<I, P, Enabled>,
    rx: dma::Transfer<Rx, dma::PtrBuffer<u8>, State>,
    _state: State,
}

impl<I, P, Buffer, Tx> TransferOut<I, P, Buffer, Tx, dma::Ready>
where
    Tx: dma::Target,
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

/// The resources that an ongoing transfer needs exclusive access to
pub struct TransferResources<I, P, TRG: dma::Target, BUF> {
    pub stream: TRG::Stream,
    pub target: Sdmmc<I, P, Enabled>,
    pub buffer: Pin<BUF>,
}

// As `TransferResources` is used in the error variant of `Result`, it needs a
// `Debug` implementation to enable stuff like `unwrap` and `expect`. This can't
// be derived without putting requirements on the type arguments.
impl<I, P, TRG, BUF> core::fmt::Debug for TransferResources<I, P, TRG, BUF>
where
    TRG: dma::Target,
{
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "TransferResources {{ .. }}")
    }
}

/// Indicates that the SDMMC peripheral is enabled
pub struct Enabled;
