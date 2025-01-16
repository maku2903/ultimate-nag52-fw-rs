#![feature(prelude_import)]
#![no_std]
#![no_main]
#[prelude_import]
use core::prelude::rust_2021::*;
#[macro_use]
extern crate core;
extern crate compiler_builtins as _;
use panic_rtt_target as _;
use atsamd_hal::pac;
mod tle8242 {
    use atsamd_hal::{
        bind_multiple_interrupts, clock::{Tc2Tc3Clock, Tcc0Tcc1Clock},
        dmac::{DmaController, PriorityLevel},
        ehal_async::spi::{SpiBus, SpiDevice},
        gpio::{AlternateF, PA16},
        nb::block, pac::{Dmac, Mclk, Pm, Tc2, Tcc1},
        prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin,
        pwm::{Channel, Pwm0, Pwm2, TC2Pinout, TCC1Pinout, Tcc1Pwm},
        sercom::{
            spi::{self, Duplex, Flags, Spi, SpiFuture, SpiFutureDuplexDma},
            Sercom2,
        },
        time::Hertz, typelevel::NoneT,
    };
    use bsp::{pin_alias, TleClk, TleReset, TleSpi, TleSpiPads};
    use cortex_m::prelude::{
        _embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write,
    };
    use embedded_hal::pwm::SetDutyCycle;
    struct SpiIrqs;
    #[automatically_derived]
    impl ::core::marker::Copy for SpiIrqs {}
    #[automatically_derived]
    impl ::core::clone::Clone for SpiIrqs {
        #[inline]
        fn clone(&self) -> SpiIrqs {
            *self
        }
    }
    #[allow(non_snake_case)]
    #[no_mangle]
    unsafe extern "C" fn SERCOM2_0() {
        <atsamd_hal::sercom::spi::InterruptHandler<
            Sercom2,
        > as ::atsamd_hal::async_hal::interrupts::Handler<
            ::atsamd_hal::async_hal::interrupts::SERCOM2,
        >>::on_interrupt();
    }
    #[allow(non_snake_case)]
    #[no_mangle]
    unsafe extern "C" fn SERCOM2_1() {
        <atsamd_hal::sercom::spi::InterruptHandler<
            Sercom2,
        > as ::atsamd_hal::async_hal::interrupts::Handler<
            ::atsamd_hal::async_hal::interrupts::SERCOM2,
        >>::on_interrupt();
    }
    #[allow(non_snake_case)]
    #[no_mangle]
    unsafe extern "C" fn SERCOM2_2() {
        <atsamd_hal::sercom::spi::InterruptHandler<
            Sercom2,
        > as ::atsamd_hal::async_hal::interrupts::Handler<
            ::atsamd_hal::async_hal::interrupts::SERCOM2,
        >>::on_interrupt();
    }
    #[allow(non_snake_case)]
    #[no_mangle]
    unsafe extern "C" fn SERCOM2_3() {
        <atsamd_hal::sercom::spi::InterruptHandler<
            Sercom2,
        > as ::atsamd_hal::async_hal::interrupts::Handler<
            ::atsamd_hal::async_hal::interrupts::SERCOM2,
        >>::on_interrupt();
    }
    #[allow(non_snake_case)]
    #[no_mangle]
    unsafe extern "C" fn SERCOM2_OTHER() {
        <atsamd_hal::sercom::spi::InterruptHandler<
            Sercom2,
        > as ::atsamd_hal::async_hal::interrupts::Handler<
            ::atsamd_hal::async_hal::interrupts::SERCOM2,
        >>::on_interrupt();
    }
    unsafe impl ::atsamd_hal::async_hal::interrupts::Binding<
        ::atsamd_hal::async_hal::interrupts::SERCOM2,
        atsamd_hal::sercom::spi::InterruptHandler<Sercom2>,
    > for SpiIrqs
    where
        ::atsamd_hal::async_hal::interrupts::SERCOM2: ::atsamd_hal::async_hal::interrupts::MultipleInterruptSources,
    {}
    struct DmacIrqs;
    #[automatically_derived]
    impl ::core::marker::Copy for DmacIrqs {}
    #[automatically_derived]
    impl ::core::clone::Clone for DmacIrqs {
        #[inline]
        fn clone(&self) -> DmacIrqs {
            *self
        }
    }
    #[allow(non_snake_case)]
    #[no_mangle]
    unsafe extern "C" fn DMAC_0() {
        <atsamd_hal::dmac::InterruptHandler as ::atsamd_hal::async_hal::interrupts::Handler<
            ::atsamd_hal::async_hal::interrupts::DMAC,
        >>::on_interrupt();
    }
    #[allow(non_snake_case)]
    #[no_mangle]
    unsafe extern "C" fn DMAC_1() {
        <atsamd_hal::dmac::InterruptHandler as ::atsamd_hal::async_hal::interrupts::Handler<
            ::atsamd_hal::async_hal::interrupts::DMAC,
        >>::on_interrupt();
    }
    #[allow(non_snake_case)]
    #[no_mangle]
    unsafe extern "C" fn DMAC_2() {
        <atsamd_hal::dmac::InterruptHandler as ::atsamd_hal::async_hal::interrupts::Handler<
            ::atsamd_hal::async_hal::interrupts::DMAC,
        >>::on_interrupt();
    }
    #[allow(non_snake_case)]
    #[no_mangle]
    unsafe extern "C" fn DMAC_OTHER() {
        <atsamd_hal::dmac::InterruptHandler as ::atsamd_hal::async_hal::interrupts::Handler<
            ::atsamd_hal::async_hal::interrupts::DMAC,
        >>::on_interrupt();
    }
    unsafe impl ::atsamd_hal::async_hal::interrupts::Binding<
        ::atsamd_hal::async_hal::interrupts::DMAC,
        atsamd_hal::dmac::InterruptHandler,
    > for DmacIrqs
    where
        ::atsamd_hal::async_hal::interrupts::DMAC: ::atsamd_hal::async_hal::interrupts::MultipleInterruptSources,
    {}
    use atsamd_hal::prelude::_embedded_hal_Pwm;
    use embedded_hal_nb::spi::FullDuplex;
    use rtt_target::rprintln;
    mod commands {
        use mcan::generic_array::typenum::B1;
        use modular_bitfield::{
            bitfield, prelude::{B2, B23, B24, B7},
            BitfieldSpecifier,
        };
        #[allow(clippy::identity_op)]
        struct PayloadFmt {
            bytes: [::core::primitive::u8; { (((32usize - 1) / 8) + 1) * 8 } / 8usize],
        }
        #[allow(clippy::identity_op)]
        const _: () = {
            impl ::modular_bitfield::private::checks::CheckFillsUnalignedBits
            for PayloadFmt {
                type CheckType = [(); (32usize
                    == {
                        0usize + <bool as ::modular_bitfield::Specifier>::BITS
                            + <B7 as ::modular_bitfield::Specifier>::BITS
                            + <B24 as ::modular_bitfield::Specifier>::BITS
                    }) as usize];
            }
        };
        impl PayloadFmt {
            /// Returns an instance with zero initialized data.
            #[allow(clippy::identity_op)]
            pub const fn new() -> Self {
                Self {
                    bytes: [0u8; { (((32usize - 1) / 8) + 1) * 8 } / 8usize],
                }
            }
        }
        impl PayloadFmt {
            /// Returns the underlying bits.
            ///
            /// # Layout
            ///
            /// The returned byte array is layed out in the same way as described
            /// [here](https://docs.rs/modular-bitfield/#generated-structure).
            #[inline]
            #[allow(clippy::identity_op)]
            pub const fn into_bytes(
                self,
            ) -> [::core::primitive::u8; { (((32usize - 1) / 8) + 1) * 8 } / 8usize] {
                self.bytes
            }
            /// Converts the given bytes directly into the bitfield struct.
            #[inline]
            #[allow(clippy::identity_op)]
            pub const fn from_bytes(
                bytes: [::core::primitive::u8; { (((32usize - 1) / 8) + 1) * 8 }
                    / 8usize],
            ) -> Self {
                Self { bytes }
            }
        }
        const _: () = {
            const _: () = {};
            const _: () = {};
            const _: () = {
                let _: ::modular_bitfield::private::checks::BitsCheck<[(); 24usize]> = ::modular_bitfield::private::checks::BitsCheck::<
                    [(); 24usize],
                > {
                    arr: [(); <B24 as ::modular_bitfield::Specifier>::BITS],
                };
            };
        };
        impl PayloadFmt {
            ///Returns the value of write.
            #[inline]
            pub fn write(&self) -> <bool as ::modular_bitfield::Specifier>::InOut {
                self.write_or_err()
                    .expect(
                        "value contains invalid bit pattern for field PayloadFmt.write",
                    )
            }
            /**Returns the value of write.

#Errors

If the returned value contains an invalid bit pattern for write.*/
            #[inline]
            #[allow(dead_code)]
            pub fn write_or_err(
                &self,
            ) -> ::core::result::Result<
                <bool as ::modular_bitfield::Specifier>::InOut,
                ::modular_bitfield::error::InvalidBitPattern<
                    <bool as ::modular_bitfield::Specifier>::Bytes,
                >,
            > {
                let __bf_read: <bool as ::modular_bitfield::Specifier>::Bytes = {
                    ::modular_bitfield::private::read_specifier::<
                        bool,
                    >(&self.bytes[..], 0usize)
                };
                <bool as ::modular_bitfield::Specifier>::from_bytes(__bf_read)
            }
            /**Returns a copy of the bitfield with the value of write set to the given value.

#Panics

If the given value is out of bounds for write.*/
            #[inline]
            #[allow(dead_code)]
            pub fn with_write(
                mut self,
                new_val: <bool as ::modular_bitfield::Specifier>::InOut,
            ) -> Self {
                self.set_write(new_val);
                self
            }
            /**Returns a copy of the bitfield with the value of write set to the given value.

#Errors

If the given value is out of bounds for write.*/
            #[inline]
            #[allow(dead_code)]
            pub fn with_write_checked(
                mut self,
                new_val: <bool as ::modular_bitfield::Specifier>::InOut,
            ) -> ::core::result::Result<Self, ::modular_bitfield::error::OutOfBounds> {
                self.set_write_checked(new_val)?;
                ::core::result::Result::Ok(self)
            }
            /**Sets the value of write to the given value.

#Panics

If the given value is out of bounds for write.*/
            #[inline]
            #[allow(dead_code)]
            pub fn set_write(
                &mut self,
                new_val: <bool as ::modular_bitfield::Specifier>::InOut,
            ) {
                self.set_write_checked(new_val)
                    .expect("value out of bounds for field PayloadFmt.write")
            }
            /**Sets the value of write to the given value.

#Errors

If the given value is out of bounds for write.*/
            #[inline]
            pub fn set_write_checked(
                &mut self,
                new_val: <bool as ::modular_bitfield::Specifier>::InOut,
            ) -> ::core::result::Result<(), ::modular_bitfield::error::OutOfBounds> {
                let __bf_base_bits: ::core::primitive::usize = 8usize
                    * ::core::mem::size_of::<
                        <bool as ::modular_bitfield::Specifier>::Bytes,
                    >();
                let __bf_max_value: <bool as ::modular_bitfield::Specifier>::Bytes = {
                    !0
                        >> (__bf_base_bits
                            - <bool as ::modular_bitfield::Specifier>::BITS)
                };
                let __bf_spec_bits: ::core::primitive::usize = <bool as ::modular_bitfield::Specifier>::BITS;
                let __bf_raw_val: <bool as ::modular_bitfield::Specifier>::Bytes = {
                    <bool as ::modular_bitfield::Specifier>::into_bytes(new_val)
                }?;
                if !(__bf_base_bits == __bf_spec_bits || __bf_raw_val <= __bf_max_value)
                {
                    return ::core::result::Result::Err(
                        ::modular_bitfield::error::OutOfBounds,
                    );
                }
                ::modular_bitfield::private::write_specifier::<
                    bool,
                >(&mut self.bytes[..], 0usize, __bf_raw_val);
                ::core::result::Result::Ok(())
            }
            ///Returns the value of msg_id.
            #[inline]
            pub fn msg_id(&self) -> <B7 as ::modular_bitfield::Specifier>::InOut {
                self.msg_id_or_err()
                    .expect(
                        "value contains invalid bit pattern for field PayloadFmt.msg_id",
                    )
            }
            /**Returns the value of msg_id.

#Errors

If the returned value contains an invalid bit pattern for msg_id.*/
            #[inline]
            #[allow(dead_code)]
            pub fn msg_id_or_err(
                &self,
            ) -> ::core::result::Result<
                <B7 as ::modular_bitfield::Specifier>::InOut,
                ::modular_bitfield::error::InvalidBitPattern<
                    <B7 as ::modular_bitfield::Specifier>::Bytes,
                >,
            > {
                let __bf_read: <B7 as ::modular_bitfield::Specifier>::Bytes = {
                    ::modular_bitfield::private::read_specifier::<
                        B7,
                    >(
                        &self.bytes[..],
                        0usize + <bool as ::modular_bitfield::Specifier>::BITS,
                    )
                };
                <B7 as ::modular_bitfield::Specifier>::from_bytes(__bf_read)
            }
            /**Returns a copy of the bitfield with the value of msg_id set to the given value.

#Panics

If the given value is out of bounds for msg_id.*/
            #[inline]
            #[allow(dead_code)]
            pub fn with_msg_id(
                mut self,
                new_val: <B7 as ::modular_bitfield::Specifier>::InOut,
            ) -> Self {
                self.set_msg_id(new_val);
                self
            }
            /**Returns a copy of the bitfield with the value of msg_id set to the given value.

#Errors

If the given value is out of bounds for msg_id.*/
            #[inline]
            #[allow(dead_code)]
            pub fn with_msg_id_checked(
                mut self,
                new_val: <B7 as ::modular_bitfield::Specifier>::InOut,
            ) -> ::core::result::Result<Self, ::modular_bitfield::error::OutOfBounds> {
                self.set_msg_id_checked(new_val)?;
                ::core::result::Result::Ok(self)
            }
            /**Sets the value of msg_id to the given value.

#Panics

If the given value is out of bounds for msg_id.*/
            #[inline]
            #[allow(dead_code)]
            pub fn set_msg_id(
                &mut self,
                new_val: <B7 as ::modular_bitfield::Specifier>::InOut,
            ) {
                self.set_msg_id_checked(new_val)
                    .expect("value out of bounds for field PayloadFmt.msg_id")
            }
            /**Sets the value of msg_id to the given value.

#Errors

If the given value is out of bounds for msg_id.*/
            #[inline]
            pub fn set_msg_id_checked(
                &mut self,
                new_val: <B7 as ::modular_bitfield::Specifier>::InOut,
            ) -> ::core::result::Result<(), ::modular_bitfield::error::OutOfBounds> {
                let __bf_base_bits: ::core::primitive::usize = 8usize
                    * ::core::mem::size_of::<
                        <B7 as ::modular_bitfield::Specifier>::Bytes,
                    >();
                let __bf_max_value: <B7 as ::modular_bitfield::Specifier>::Bytes = {
                    !0 >> (__bf_base_bits - <B7 as ::modular_bitfield::Specifier>::BITS)
                };
                let __bf_spec_bits: ::core::primitive::usize = <B7 as ::modular_bitfield::Specifier>::BITS;
                let __bf_raw_val: <B7 as ::modular_bitfield::Specifier>::Bytes = {
                    <B7 as ::modular_bitfield::Specifier>::into_bytes(new_val)
                }?;
                if !(__bf_base_bits == __bf_spec_bits || __bf_raw_val <= __bf_max_value)
                {
                    return ::core::result::Result::Err(
                        ::modular_bitfield::error::OutOfBounds,
                    );
                }
                ::modular_bitfield::private::write_specifier::<
                    B7,
                >(
                    &mut self.bytes[..],
                    0usize + <bool as ::modular_bitfield::Specifier>::BITS,
                    __bf_raw_val,
                );
                ::core::result::Result::Ok(())
            }
            ///Returns the value of payload.
            #[inline]
            pub fn payload(&self) -> <B24 as ::modular_bitfield::Specifier>::InOut {
                self.payload_or_err()
                    .expect(
                        "value contains invalid bit pattern for field PayloadFmt.payload",
                    )
            }
            /**Returns the value of payload.

#Errors

If the returned value contains an invalid bit pattern for payload.*/
            #[inline]
            #[allow(dead_code)]
            pub fn payload_or_err(
                &self,
            ) -> ::core::result::Result<
                <B24 as ::modular_bitfield::Specifier>::InOut,
                ::modular_bitfield::error::InvalidBitPattern<
                    <B24 as ::modular_bitfield::Specifier>::Bytes,
                >,
            > {
                let __bf_read: <B24 as ::modular_bitfield::Specifier>::Bytes = {
                    ::modular_bitfield::private::read_specifier::<
                        B24,
                    >(
                        &self.bytes[..],
                        0usize + <bool as ::modular_bitfield::Specifier>::BITS
                            + <B7 as ::modular_bitfield::Specifier>::BITS,
                    )
                };
                <B24 as ::modular_bitfield::Specifier>::from_bytes(__bf_read)
            }
            /**Returns a copy of the bitfield with the value of payload set to the given value.

#Panics

If the given value is out of bounds for payload.*/
            #[inline]
            #[allow(dead_code)]
            pub fn with_payload(
                mut self,
                new_val: <B24 as ::modular_bitfield::Specifier>::InOut,
            ) -> Self {
                self.set_payload(new_val);
                self
            }
            /**Returns a copy of the bitfield with the value of payload set to the given value.

#Errors

If the given value is out of bounds for payload.*/
            #[inline]
            #[allow(dead_code)]
            pub fn with_payload_checked(
                mut self,
                new_val: <B24 as ::modular_bitfield::Specifier>::InOut,
            ) -> ::core::result::Result<Self, ::modular_bitfield::error::OutOfBounds> {
                self.set_payload_checked(new_val)?;
                ::core::result::Result::Ok(self)
            }
            /**Sets the value of payload to the given value.

#Panics

If the given value is out of bounds for payload.*/
            #[inline]
            #[allow(dead_code)]
            pub fn set_payload(
                &mut self,
                new_val: <B24 as ::modular_bitfield::Specifier>::InOut,
            ) {
                self.set_payload_checked(new_val)
                    .expect("value out of bounds for field PayloadFmt.payload")
            }
            /**Sets the value of payload to the given value.

#Errors

If the given value is out of bounds for payload.*/
            #[inline]
            pub fn set_payload_checked(
                &mut self,
                new_val: <B24 as ::modular_bitfield::Specifier>::InOut,
            ) -> ::core::result::Result<(), ::modular_bitfield::error::OutOfBounds> {
                let __bf_base_bits: ::core::primitive::usize = 8usize
                    * ::core::mem::size_of::<
                        <B24 as ::modular_bitfield::Specifier>::Bytes,
                    >();
                let __bf_max_value: <B24 as ::modular_bitfield::Specifier>::Bytes = {
                    !0 >> (__bf_base_bits - <B24 as ::modular_bitfield::Specifier>::BITS)
                };
                let __bf_spec_bits: ::core::primitive::usize = <B24 as ::modular_bitfield::Specifier>::BITS;
                let __bf_raw_val: <B24 as ::modular_bitfield::Specifier>::Bytes = {
                    <B24 as ::modular_bitfield::Specifier>::into_bytes(new_val)
                }?;
                if !(__bf_base_bits == __bf_spec_bits || __bf_raw_val <= __bf_max_value)
                {
                    return ::core::result::Result::Err(
                        ::modular_bitfield::error::OutOfBounds,
                    );
                }
                ::modular_bitfield::private::write_specifier::<
                    B24,
                >(
                    &mut self.bytes[..],
                    0usize + <bool as ::modular_bitfield::Specifier>::BITS
                        + <B7 as ::modular_bitfield::Specifier>::BITS,
                    __bf_raw_val,
                );
                ::core::result::Result::Ok(())
            }
        }
        #[allow(clippy::identity_op)]
        const _: () = {
            impl ::modular_bitfield::private::checks::CheckSpecifierHasAtMost128Bits
            for PayloadFmt {
                type CheckType = [(); (32usize <= 128) as ::core::primitive::usize];
            }
        };
        #[allow(clippy::identity_op)]
        impl ::modular_bitfield::Specifier for PayloadFmt {
            const BITS: usize = 32usize;
            #[allow(unused_braces)]
            type Bytes = <[(); if { 32usize } > 128 {
                128
            } else {
                32usize
            }] as ::modular_bitfield::private::SpecifierBytes>::Bytes;
            type InOut = Self;
            #[inline]
            fn into_bytes(
                value: Self::InOut,
            ) -> ::core::result::Result<
                Self::Bytes,
                ::modular_bitfield::error::OutOfBounds,
            > {
                ::core::result::Result::Ok(
                    <[(); {
                        (((32usize - 1) / 8) + 1) * 8
                    }] as ::modular_bitfield::private::ArrayBytesConversion>::array_into_bytes(
                        value.bytes,
                    ),
                )
            }
            #[inline]
            fn from_bytes(
                bytes: Self::Bytes,
            ) -> ::core::result::Result<
                Self::InOut,
                ::modular_bitfield::error::InvalidBitPattern<Self::Bytes>,
            > {
                let __bf_max_value: Self::Bytes = (0x01 as Self::Bytes)
                    .checked_shl(Self::BITS as ::core::primitive::u32)
                    .unwrap_or(<Self::Bytes>::MAX);
                if bytes > __bf_max_value {
                    return ::core::result::Result::Err(
                        ::modular_bitfield::error::InvalidBitPattern::new(bytes),
                    );
                }
                let __bf_bytes = bytes.to_le_bytes();
                ::core::result::Result::Ok(Self {
                    bytes: <[(); {
                        (((32usize - 1) / 8) + 1) * 8
                    }] as ::modular_bitfield::private::ArrayBytesConversion>::bytes_into_array(
                        bytes,
                    ),
                })
            }
        }
        #[allow(clippy::identity_op)]
        struct ICVersion {
            bytes: [::core::primitive::u8; { (((23usize - 1) / 8) + 1) * 8 } / 8usize],
        }
        #[allow(clippy::identity_op)]
        const _: () = {
            impl ::modular_bitfield::private::checks::CheckFillsUnalignedBits
            for ICVersion {
                type CheckType = [(); (23usize
                    == { 0usize + <B23 as ::modular_bitfield::Specifier>::BITS })
                    as usize];
            }
        };
        impl ICVersion {
            /// Returns an instance with zero initialized data.
            #[allow(clippy::identity_op)]
            pub const fn new() -> Self {
                Self {
                    bytes: [0u8; { (((23usize - 1) / 8) + 1) * 8 } / 8usize],
                }
            }
        }
        impl ICVersion {
            /// Returns the underlying bits.
            ///
            /// # Layout
            ///
            /// The returned byte array is layed out in the same way as described
            /// [here](https://docs.rs/modular-bitfield/#generated-structure).
            #[inline]
            #[allow(clippy::identity_op)]
            pub const fn into_bytes(
                self,
            ) -> [::core::primitive::u8; { (((23usize - 1) / 8) + 1) * 8 } / 8usize] {
                self.bytes
            }
            /// Converts the given bytes directly into the bitfield struct.
            #[inline]
            #[allow(clippy::identity_op)]
            pub const fn from_bytes(
                bytes: [::core::primitive::u8; { (((23usize - 1) / 8) + 1) * 8 }
                    / 8usize],
            ) -> Self {
                Self { bytes }
            }
        }
        const _: () = {
            const _: () = {};
        };
        impl ICVersion {
            ///Returns the value of __.
            #[inline]
            fn __(&self) -> <B23 as ::modular_bitfield::Specifier>::InOut {
                self.___or_err()
                    .expect("value contains invalid bit pattern for field ICVersion.__")
            }
            /**Returns the value of __.

#Errors

If the returned value contains an invalid bit pattern for __.*/
            #[inline]
            #[allow(dead_code)]
            fn ___or_err(
                &self,
            ) -> ::core::result::Result<
                <B23 as ::modular_bitfield::Specifier>::InOut,
                ::modular_bitfield::error::InvalidBitPattern<
                    <B23 as ::modular_bitfield::Specifier>::Bytes,
                >,
            > {
                let __bf_read: <B23 as ::modular_bitfield::Specifier>::Bytes = {
                    ::modular_bitfield::private::read_specifier::<
                        B23,
                    >(&self.bytes[..], 0usize)
                };
                <B23 as ::modular_bitfield::Specifier>::from_bytes(__bf_read)
            }
            /**Returns a copy of the bitfield with the value of __ set to the given value.

#Panics

If the given value is out of bounds for __.*/
            #[inline]
            #[allow(dead_code)]
            fn with___(
                mut self,
                new_val: <B23 as ::modular_bitfield::Specifier>::InOut,
            ) -> Self {
                self.set___(new_val);
                self
            }
            /**Returns a copy of the bitfield with the value of __ set to the given value.

#Errors

If the given value is out of bounds for __.*/
            #[inline]
            #[allow(dead_code)]
            fn with____checked(
                mut self,
                new_val: <B23 as ::modular_bitfield::Specifier>::InOut,
            ) -> ::core::result::Result<Self, ::modular_bitfield::error::OutOfBounds> {
                self.set____checked(new_val)?;
                ::core::result::Result::Ok(self)
            }
            /**Sets the value of __ to the given value.

#Panics

If the given value is out of bounds for __.*/
            #[inline]
            #[allow(dead_code)]
            fn set___(
                &mut self,
                new_val: <B23 as ::modular_bitfield::Specifier>::InOut,
            ) {
                self.set____checked(new_val)
                    .expect("value out of bounds for field ICVersion.__")
            }
            /**Sets the value of __ to the given value.

#Errors

If the given value is out of bounds for __.*/
            #[inline]
            fn set____checked(
                &mut self,
                new_val: <B23 as ::modular_bitfield::Specifier>::InOut,
            ) -> ::core::result::Result<(), ::modular_bitfield::error::OutOfBounds> {
                let __bf_base_bits: ::core::primitive::usize = 8usize
                    * ::core::mem::size_of::<
                        <B23 as ::modular_bitfield::Specifier>::Bytes,
                    >();
                let __bf_max_value: <B23 as ::modular_bitfield::Specifier>::Bytes = {
                    !0 >> (__bf_base_bits - <B23 as ::modular_bitfield::Specifier>::BITS)
                };
                let __bf_spec_bits: ::core::primitive::usize = <B23 as ::modular_bitfield::Specifier>::BITS;
                let __bf_raw_val: <B23 as ::modular_bitfield::Specifier>::Bytes = {
                    <B23 as ::modular_bitfield::Specifier>::into_bytes(new_val)
                }?;
                if !(__bf_base_bits == __bf_spec_bits || __bf_raw_val <= __bf_max_value)
                {
                    return ::core::result::Result::Err(
                        ::modular_bitfield::error::OutOfBounds,
                    );
                }
                ::modular_bitfield::private::write_specifier::<
                    B23,
                >(&mut self.bytes[..], 0usize, __bf_raw_val);
                ::core::result::Result::Ok(())
            }
        }
        #[allow(clippy::identity_op)]
        const _: () = {
            impl ::modular_bitfield::private::checks::CheckSpecifierHasAtMost128Bits
            for ICVersion {
                type CheckType = [(); (23usize <= 128) as ::core::primitive::usize];
            }
        };
        #[allow(clippy::identity_op)]
        impl ::modular_bitfield::Specifier for ICVersion {
            const BITS: usize = 23usize;
            #[allow(unused_braces)]
            type Bytes = <[(); if { 23usize } > 128 {
                128
            } else {
                23usize
            }] as ::modular_bitfield::private::SpecifierBytes>::Bytes;
            type InOut = Self;
            #[inline]
            fn into_bytes(
                value: Self::InOut,
            ) -> ::core::result::Result<
                Self::Bytes,
                ::modular_bitfield::error::OutOfBounds,
            > {
                ::core::result::Result::Ok(
                    <[(); {
                        (((23usize - 1) / 8) + 1) * 8
                    }] as ::modular_bitfield::private::ArrayBytesConversion>::array_into_bytes(
                        value.bytes,
                    ),
                )
            }
            #[inline]
            fn from_bytes(
                bytes: Self::Bytes,
            ) -> ::core::result::Result<
                Self::InOut,
                ::modular_bitfield::error::InvalidBitPattern<Self::Bytes>,
            > {
                let __bf_max_value: Self::Bytes = (0x01 as Self::Bytes)
                    .checked_shl(Self::BITS as ::core::primitive::u32)
                    .unwrap_or(<Self::Bytes>::MAX);
                if bytes > __bf_max_value {
                    return ::core::result::Result::Err(
                        ::modular_bitfield::error::InvalidBitPattern::new(bytes),
                    );
                }
                let __bf_bytes = bytes.to_le_bytes();
                ::core::result::Result::Ok(Self {
                    bytes: <[(); {
                        (((23usize - 1) / 8) + 1) * 8
                    }] as ::modular_bitfield::private::ArrayBytesConversion>::bytes_into_array(
                        bytes,
                    ),
                })
            }
        }
        trait CheckedSize: modular_bitfield::Specifier {
            const CHECK: ();
        }
        impl<T: modular_bitfield::Specifier> CheckedSize for T {
            const CHECK: () = [()][(Self::BITS == 24) as usize];
        }
        impl PayloadFmt {
            pub fn f<R: CheckedSize>(&self) {}
            pub fn x(&self) {
                self.f::<ICVersion>();
            }
        }
    }
    pub struct Tle8242 {
        spi: SpiFutureDuplexDma<
            spi::Config<TleSpiPads>,
            atsamd_hal::dmac::Ch0,
            atsamd_hal::dmac::Ch1,
        >,
    }
    impl Tle8242 {
        pub fn new(
            mut spi: TleSpi,
            dmac: Dmac,
            pm: &mut Pm,
            clk_pin: impl Into<TleClk>,
            reset_pin: impl Into<TleReset>,
            clock: &Tcc0Tcc1Clock,
            tcc: Tcc1,
            mclk: &mut Mclk,
        ) -> Self {
            let mut pin: TleClk = clk_pin.into();
            pin.set_drive_strength(true);
            let mut pin_reset: TleReset = reset_pin.into();
            pin_reset.set_drive_strength(true);
            pin_reset.set_low().unwrap();
            ::rtt_target::print_impl::write_fmt(
                0,
                format_args!("Clock is {0}\n", clock.freq()),
            );
            let mut pwm_tcc: Tcc1Pwm<PA16, AlternateF> = Tcc1Pwm::new(
                clock,
                Hertz::MHz(20),
                tcc,
                TCC1Pinout::Pa16(pin),
                mclk,
            );
            let max = pwm_tcc.get_max_duty();
            pwm_tcc.set_duty(Channel::_0, max / 2);
            ::rtt_target::print_impl::write_fmt(
                0,
                format_args!("Max duty is {0}\n", max),
            );
            let mut dmac = DmaController::init(dmac, pm);
            let channels = dmac.into_future(DmacIrqs).split();
            let chan0 = channels.0.init(PriorityLevel::Lvl0);
            let chan1 = channels.1.init(PriorityLevel::Lvl0);
            let mut spi = spi.into_future(SpiIrqs).with_dma_channels(chan0, chan1);
            pin_reset.set_high().unwrap();
            Self { spi }
        }
        pub async fn send_command(&mut self, cmd: u32) -> Option<u32> {
            let mut buf: [u8; 4] = cmd.to_le_bytes().try_into().unwrap();
            self.spi.transfer_in_place(&mut buf).await.ok()?;
            Some(u32::from_le_bytes(buf.try_into().unwrap()))
        }
        pub fn ic_version(&self) {}
        pub async fn write_and_read(&mut self, msg: u32) -> Option<u32> {
            None
        }
    }
}
/// A `Monotonic` based on SysTick.
pub struct Mono;
impl Mono {
    /// Starts the `Monotonic`.
    ///
    /// The `sysclk` parameter is the speed at which SysTick runs at. This value should come from
    /// the clock generation function of the used HAL.
    ///
    /// Panics if it is impossible to achieve the desired monotonic tick rate based
    /// on the given `sysclk` parameter. If that happens, adjust the desired monotonic tick rate.
    ///
    /// This method must be called only once.
    pub fn start(systick: ::rtic_monotonics::systick::SYST, sysclk: u32) {
        #[no_mangle]
        #[allow(non_snake_case)]
        unsafe extern "C" fn SysTick() {
            use ::rtic_monotonics::TimerQueueBackend;
            ::rtic_monotonics::systick::SystickBackend::timer_queue()
                .on_monotonic_interrupt();
        }
        ::rtic_monotonics::systick::SystickBackend::_start(systick, sysclk, 1000);
    }
}
impl ::rtic_monotonics::TimerQueueBasedMonotonic for Mono {
    type Backend = ::rtic_monotonics::systick::SystickBackend;
    type Instant = ::rtic_monotonics::fugit::Instant<
        <Self::Backend as ::rtic_monotonics::TimerQueueBackend>::Ticks,
        1,
        { 1000 },
    >;
    type Duration = ::rtic_monotonics::fugit::Duration<
        <Self::Backend as ::rtic_monotonics::TimerQueueBackend>::Ticks,
        1,
        { 1000 },
    >;
}
impl ::rtic_time::embedded_hal::delay::DelayNs for Mono {
    fn delay_ns(&mut self, ns: u32) {
        let now = <Self as ::rtic_time::Monotonic>::now();
        let mut done = now
            + <Self as ::rtic_time::Monotonic>::Duration::nanos_at_least(ns.into());
        if now != done {
            done += <Self as ::rtic_time::Monotonic>::Duration::from_ticks(1);
        }
        while <Self as ::rtic_time::Monotonic>::now() < done {}
    }
    fn delay_us(&mut self, us: u32) {
        let now = <Self as ::rtic_time::Monotonic>::now();
        let mut done = now
            + <Self as ::rtic_time::Monotonic>::Duration::micros_at_least(us.into());
        if now != done {
            done += <Self as ::rtic_time::Monotonic>::Duration::from_ticks(1);
        }
        while <Self as ::rtic_time::Monotonic>::now() < done {}
    }
    fn delay_ms(&mut self, ms: u32) {
        let now = <Self as ::rtic_time::Monotonic>::now();
        let mut done = now
            + <Self as ::rtic_time::Monotonic>::Duration::millis_at_least(ms.into());
        if now != done {
            done += <Self as ::rtic_time::Monotonic>::Duration::from_ticks(1);
        }
        while <Self as ::rtic_time::Monotonic>::now() < done {}
    }
}
impl ::rtic_time::embedded_hal_async::delay::DelayNs for Mono {
    #[inline]
    async fn delay_ns(&mut self, ns: u32) {
        <Self as ::rtic_time::Monotonic>::delay(
                <Self as ::rtic_time::Monotonic>::Duration::nanos_at_least(ns.into()),
            )
            .await;
    }
    #[inline]
    async fn delay_us(&mut self, us: u32) {
        <Self as ::rtic_time::Monotonic>::delay(
                <Self as ::rtic_time::Monotonic>::Duration::micros_at_least(us.into()),
            )
            .await;
    }
    #[inline]
    async fn delay_ms(&mut self, ms: u32) {
        <Self as ::rtic_time::Monotonic>::delay(
                <Self as ::rtic_time::Monotonic>::Duration::millis_at_least(ms.into()),
            )
            .await;
    }
}
/// The RTIC application module
pub mod app {
    /// Always include the device crate which contains the vector table
    use atsamd_hal::pac as you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml;
    /// Holds the maximum priority level for use by async HAL drivers.
    #[no_mangle]
    static RTIC_ASYNC_MAX_LOGICAL_PRIO: u8 = 1 << atsamd_hal::pac::NVIC_PRIO_BITS;
    use core::time::Duration;
    use super::*;
    use atsamd_hal::{
        clock::{
            v2::{
                clock_system_at_reset, dpll::Dpll, gclk::{Gclk, GclkDiv16},
                pclk::Pclk,
            },
            Tcc0Tcc1Clock,
        },
        prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin, sercom::spi::Flags,
        time::Hertz,
    };
    use fugit::ExtU32;
    use mcan::messageram::SharedMemory;
    use bsp::{can_deps::Capacities, tle_spi, Pins, SolPwrEn};
    use rtic_monotonics::Monotonic;
    use rtt_target::{rprintln, rtt_init_print};
    use tle8242::Tle8242;
    /// User code end
    impl<'a> __rtic_internal_initLocalResources<'a> {
        #[inline(always)]
        #[allow(missing_docs)]
        pub unsafe fn new() -> Self {
            __rtic_internal_initLocalResources {
                can_memory0: &mut *__rtic_internal_local_init_can_memory0.get_mut(),
                can_memory1: &mut *__rtic_internal_local_init_can_memory1.get_mut(),
                __rtic_internal_marker: ::core::marker::PhantomData,
            }
        }
    }
    ///Shared resources
    struct SharedResources {}
    ///Local resources
    struct LocalResources {
        tle8242: Tle8242,
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    ///Local resources `init` has access to
    pub struct __rtic_internal_initLocalResources<'a> {
        #[allow(missing_docs)]
        pub can_memory0: &'static mut SharedMemory<Capacities>,
        #[allow(missing_docs)]
        pub can_memory1: &'static mut SharedMemory<Capacities>,
        #[doc(hidden)]
        pub __rtic_internal_marker: ::core::marker::PhantomData<&'a ()>,
    }
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_init_Context<'a> {
        #[doc(hidden)]
        __rtic_internal_p: ::core::marker::PhantomData<&'a ()>,
        /// The space used to allocate async executors in bytes.
        pub executors_size: usize,
        /// Core peripherals
        pub core: rtic::export::Peripherals,
        /// Device peripherals (PAC)
        pub device: atsamd_hal::pac::Peripherals,
        /// Critical section token for init
        pub cs: rtic::export::CriticalSection<'a>,
        /// Local Resources this task has access to
        pub local: init::LocalResources<'a>,
    }
    impl<'a> __rtic_internal_init_Context<'a> {
        #[inline(always)]
        #[allow(missing_docs)]
        pub unsafe fn new(
            core: rtic::export::Peripherals,
            executors_size: usize,
        ) -> Self {
            __rtic_internal_init_Context {
                __rtic_internal_p: ::core::marker::PhantomData,
                core: core,
                device: atsamd_hal::pac::Peripherals::steal(),
                cs: rtic::export::CriticalSection::new(),
                executors_size,
                local: init::LocalResources::new(),
            }
        }
    }
    #[allow(non_snake_case)]
    ///Initialization function
    pub mod init {
        #[doc(inline)]
        pub use super::__rtic_internal_initLocalResources as LocalResources;
        #[doc(inline)]
        pub use super::__rtic_internal_init_Context as Context;
    }
    #[inline(always)]
    #[allow(non_snake_case)]
    fn init(mut cx: init::Context) -> (SharedResources, LocalResources) {
        {
            use ::rtt_target::ChannelMode::NoBlockSkip;
            {
                let channels = {
                    use core::mem::MaybeUninit;
                    use core::ptr;
                    use core::cell::Cell;
                    use ::rtt_target::UpChannel;
                    use ::rtt_target::DownChannel;
                    use ::rtt_target::rtt::*;
                    #[repr(C)]
                    pub struct RttControlBlock {
                        header: RttHeader,
                        up_channels: [RttChannel; (1 + 0)],
                        down_channels: [RttChannel; (0)],
                    }
                    #[used]
                    #[no_mangle]
                    #[export_name = "_SEGGER_RTT"]
                    pub static mut CONTROL_BLOCK: MaybeUninit<RttControlBlock> = MaybeUninit::uninit();
                    #[allow(unused)]
                    #[export_name = "rtt_init_must_not_be_called_multiple_times"]
                    fn rtt_init_must_not_be_called_multiple_times() {}
                    use ::rtt_target::export::critical_section;
                    static INITIALIZED: critical_section::Mutex<Cell<bool>> = critical_section::Mutex::new(
                        Cell::new(false),
                    );
                    critical_section::with(|cs| {
                        if INITIALIZED.borrow(cs).get() {
                            ::core::panicking::panic(
                                "rtt_init! must not be called multiple times",
                            );
                        }
                        INITIALIZED.borrow(cs).set(true);
                    });
                    unsafe {
                        ptr::write_bytes(CONTROL_BLOCK.as_mut_ptr(), 0, 1);
                        let cb = &mut *CONTROL_BLOCK.as_mut_ptr();
                        let mut name: *const u8 = core::ptr::null();
                        name = "Terminal\u{0}".as_bytes().as_ptr();
                        let mut mode = ::rtt_target::ChannelMode::NoBlockSkip;
                        mode = NoBlockSkip;
                        cb.up_channels[0]
                            .init(
                                name,
                                mode,
                                {
                                    static mut _RTT_CHANNEL_BUFFER: MaybeUninit<[u8; 1024]> = MaybeUninit::uninit();
                                    _RTT_CHANNEL_BUFFER.as_mut_ptr()
                                },
                            );
                        cb.header.init(cb.up_channels.len(), cb.down_channels.len());
                        pub struct Channels {
                            pub up: (UpChannel,),
                        }
                        Channels {
                            up: (UpChannel::new(&mut cb.up_channels[0] as *mut _),),
                        }
                    }
                };
                ::rtt_target::set_print_channel(channels.up.0);
            };
        };
        let pins = Pins::new(cx.device.port);
        let (_buses, clocks, tokens) = clock_system_at_reset(
            cx.device.oscctrl,
            cx.device.osc32kctrl,
            cx.device.gclk,
            cx.device.mclk,
            &mut cx.device.nvmctrl,
        );
        let (_, _, _, mut mclk) = unsafe { clocks.pac.steal() };
        let (gclk1, dfll) = Gclk::from_source(tokens.gclks.gclk1, clocks.dfll);
        let gclk1 = gclk1.div(GclkDiv16::Div(24)).enable();
        let (pclk_dpll0, gclk1) = Pclk::enable(tokens.pclks.dpll0, gclk1);
        let dpll0 = Dpll::from_pclk(tokens.dpll0, pclk_dpll0).loop_div(50, 0).enable();
        let (gclk0, _dfll, _dpll0) = clocks.gclk0.swap_sources(dfll, dpll0);
        ::rtt_target::print_impl::write_str(0, "Test!\n");
        let freq = gclk0.freq();
        let (pclk_sercom2, gclk0) = Pclk::enable(tokens.pclks.sercom2, gclk0);
        let (tcc0tcc1clock, gclk0) = Pclk::enable(tokens.pclks.tcc0_tcc1, gclk0);
        let mut sol_pwr_en: SolPwrEn = pins.sol_pwr_en.into();
        sol_pwr_en.set_drive_strength(true);
        sol_pwr_en.set_high().unwrap();
        let spi = tle_spi(
            pclk_sercom2,
            Hertz::MHz(1),
            cx.device.sercom2,
            &mut mclk,
            pins.tle_sclk,
            pins.tle_si,
            pins.tle_so,
            pins.tle_cs,
        );
        let tle8242 = Tle8242::new(
            spi,
            cx.device.dmac,
            &mut cx.device.pm,
            pins.tle_clk,
            pins.tle_reset,
            &tcc0tcc1clock.into(),
            cx.device.tcc1,
            &mut mclk,
        );
        Mono::start(cx.core.SYST, freq.raw());
        query_tle::spawn();
        (SharedResources {}, LocalResources { tle8242 })
    }
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_idle_Context<'a> {
        #[doc(hidden)]
        __rtic_internal_p: ::core::marker::PhantomData<&'a ()>,
    }
    impl<'a> __rtic_internal_idle_Context<'a> {
        #[inline(always)]
        #[allow(missing_docs)]
        pub unsafe fn new() -> Self {
            __rtic_internal_idle_Context {
                __rtic_internal_p: ::core::marker::PhantomData,
            }
        }
    }
    #[allow(non_snake_case)]
    ///Idle loop
    pub mod idle {
        #[doc(inline)]
        pub use super::__rtic_internal_idle_Context as Context;
    }
    #[allow(non_snake_case)]
    fn idle(mut cx: idle::Context) -> ! {
        use rtic::Mutex as _;
        use rtic::mutex::prelude::*;
        loop {
            cortex_m::asm::wfi();
        }
    }
    impl<'a> __rtic_internal_query_tleLocalResources<'a> {
        #[inline(always)]
        #[allow(missing_docs)]
        pub unsafe fn new() -> Self {
            __rtic_internal_query_tleLocalResources {
                tle8242: &mut *(&mut *__rtic_internal_local_resource_tle8242.get_mut())
                    .as_mut_ptr(),
                __rtic_internal_marker: ::core::marker::PhantomData,
            }
        }
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    ///Local resources `query_tle` has access to
    pub struct __rtic_internal_query_tleLocalResources<'a> {
        #[allow(missing_docs)]
        pub tle8242: &'a mut Tle8242,
        #[doc(hidden)]
        pub __rtic_internal_marker: ::core::marker::PhantomData<&'a ()>,
    }
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_query_tle_Context<'a> {
        #[doc(hidden)]
        __rtic_internal_p: ::core::marker::PhantomData<&'a ()>,
        /// Local Resources this task has access to
        pub local: query_tle::LocalResources<'a>,
    }
    impl<'a> __rtic_internal_query_tle_Context<'a> {
        #[inline(always)]
        #[allow(missing_docs)]
        pub unsafe fn new() -> Self {
            __rtic_internal_query_tle_Context {
                __rtic_internal_p: ::core::marker::PhantomData,
                local: query_tle::LocalResources::new(),
            }
        }
    }
    /// Spawns the task directly
    #[allow(non_snake_case)]
    #[doc(hidden)]
    pub fn __rtic_internal_query_tle_spawn() -> ::core::result::Result<(), ()> {
        unsafe {
            let exec = rtic::export::executor::AsyncTaskExecutor::from_ptr_1_args(
                query_tle,
                &__rtic_internal_query_tle_EXEC,
            );
            if exec.try_allocate() {
                exec.spawn(query_tle(unsafe { query_tle::Context::new() }));
                rtic::export::pend(atsamd_hal::pac::interrupt::EVSYS_0);
                Ok(())
            } else {
                Err(())
            }
        }
    }
    #[allow(non_snake_case)]
    ///Software task
    pub mod query_tle {
        #[doc(inline)]
        pub use super::__rtic_internal_query_tleLocalResources as LocalResources;
        #[doc(inline)]
        pub use super::__rtic_internal_query_tle_Context as Context;
        #[doc(inline)]
        pub use super::__rtic_internal_query_tle_spawn as spawn;
    }
    #[allow(non_snake_case)]
    async fn query_tle<'a>(ctx: query_tle::Context<'a>) {
        use rtic::Mutex as _;
        use rtic::mutex::prelude::*;
        loop {
            ctx.local.tle8242.send_command(0).await;
            Mono::delay(20u32.millis()).await
        }
    }
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".uninit.rtic0"]
    static __rtic_internal_local_resource_tle8242: rtic::RacyCell<
        core::mem::MaybeUninit<Tle8242>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".can"]
    static __rtic_internal_local_init_can_memory0: rtic::RacyCell<
        SharedMemory<Capacities>,
    > = rtic::RacyCell::new(SharedMemory::new());
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".can"]
    static __rtic_internal_local_init_can_memory1: rtic::RacyCell<
        SharedMemory<Capacities>,
    > = rtic::RacyCell::new(SharedMemory::new());
    #[allow(non_upper_case_globals)]
    static __rtic_internal_query_tle_EXEC: rtic::export::executor::AsyncTaskExecutorPtr = rtic::export::executor::AsyncTaskExecutorPtr::new();
    #[allow(non_snake_case)]
    ///Interrupt handler to dispatch async tasks at priority 1
    #[no_mangle]
    unsafe fn EVSYS_0() {
        /// The priority of this interrupt handler
        const PRIORITY: u8 = 1u8;
        rtic::export::run(
            PRIORITY,
            || {
                let exec = rtic::export::executor::AsyncTaskExecutor::from_ptr_1_args(
                    query_tle,
                    &__rtic_internal_query_tle_EXEC,
                );
                exec.poll(|| {
                    let exec = rtic::export::executor::AsyncTaskExecutor::from_ptr_1_args(
                        query_tle,
                        &__rtic_internal_query_tle_EXEC,
                    );
                    exec.set_pending();
                    rtic::export::pend(atsamd_hal::pac::interrupt::EVSYS_0);
                });
            },
        );
    }
    #[doc(hidden)]
    #[no_mangle]
    unsafe extern "C" fn main() -> ! {
        rtic::export::interrupt::disable();
        let mut core: rtic::export::Peripherals = rtic::export::Peripherals::steal()
            .into();
        let _ = you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::EVSYS_0;
        const _: () = if (1 << atsamd_hal::pac::NVIC_PRIO_BITS) < 1u8 as usize {
            {
                ::core::panicking::panic_fmt(
                    format_args!(
                        "Maximum priority used by interrupt vector \'EVSYS_0\' is more than supported by hardware",
                    ),
                );
            };
        };
        core.NVIC
            .set_priority(
                you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::EVSYS_0,
                rtic::export::cortex_logical2hw(1u8, atsamd_hal::pac::NVIC_PRIO_BITS),
            );
        rtic::export::NVIC::unmask(
            you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::EVSYS_0,
        );
        #[inline(never)]
        fn __rtic_init_resources<F>(f: F)
        where
            F: FnOnce(),
        {
            f();
        }
        let mut executors_size = 0;
        let executor = ::core::mem::ManuallyDrop::new(
            rtic::export::executor::AsyncTaskExecutor::new_1_args(query_tle),
        );
        executors_size += ::core::mem::size_of_val(&executor);
        __rtic_internal_query_tle_EXEC.set_in_main(&executor);
        extern "C" {
            pub static _stack_start: u32;
            pub static __ebss: u32;
        }
        let stack_start = &_stack_start as *const _ as u32;
        let ebss = &__ebss as *const _ as u32;
        if stack_start > ebss {
            if rtic::export::msp::read() <= ebss {
                {
                    ::core::panicking::panic_fmt(
                        format_args!("Stack overflow after allocating executors"),
                    );
                };
            }
        }
        __rtic_init_resources(|| {
            let (shared_resources, local_resources) = init(
                init::Context::new(core.into(), executors_size),
            );
            __rtic_internal_local_resource_tle8242
                .get_mut()
                .write(core::mem::MaybeUninit::new(local_resources.tle8242));
            rtic::export::interrupt::enable();
        });
        idle(idle::Context::new())
    }
}
