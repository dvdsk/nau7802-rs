#![no_std]

mod constants;
mod driver;

pub use driver::Nau7802;

#[derive(Debug)]
#[cfg_attr(feature = "thiserror", derive(thiserror::Error))]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    #[cfg_attr(feature = "thiserror", error("I²C error getting registers: {0}"))]
    GettingRegister(E),
    #[cfg_attr(
        feature = "thiserror",
        error("I²C error requesting to read register: {0}")
    )]
    RequestingRegister(E),
    #[cfg_attr(feature = "thiserror", error("I²C error setting register: {0}"))]
    SettingRegister(E),
    #[cfg_attr(
        feature = "thiserror",
        error("Power up failed, device did not respond in time")
    )]
    PowerupFailed,
    #[cfg_attr(feature = "thiserror", error("I²C while reading measurement"))]
    ReadingData(E),
    #[cfg_attr(feature = "thiserror", error("New data was not ready in time"))]
    ReadTimeout,
    #[cfg_attr(feature = "thiserror", error("Device reported calibration failure"))]
    CalibrationFailure,
}

impl<E> Clone for Error<E>
where
    E: Clone,
{
    fn clone(&self) -> Self {
        match self {
            Error::GettingRegister(e) => Error::GettingRegister(e.clone()),
            Error::RequestingRegister(e) => Error::RequestingRegister(e.clone()),
            Error::SettingRegister(e) => Error::SettingRegister(e.clone()),
            Error::PowerupFailed => Error::PowerupFailed,
            Error::ReadingData(e) => Error::ReadingData(e.clone()),
            Error::ReadTimeout => Error::ReadTimeout,
            Error::CalibrationFailure => Error::CalibrationFailure,
        }
    }
}

#[cfg(feature = "postcard")]
impl<E> postcard::experimental::max_size::MaxSize for Error<E>
where
    E: postcard::experimental::max_size::MaxSize,
{
    const POSTCARD_MAX_SIZE: usize = 1 + E::POSTCARD_MAX_SIZE;
}

impl<E> Eq for Error<E> where E: Eq {}

impl<E> PartialEq for Error<E>
where
    E: PartialEq,
{
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Error::GettingRegister(e1), Error::GettingRegister(e2)) => e1 == e2,
            (Error::RequestingRegister(e1), Error::RequestingRegister(e2)) => e1 == e2,
            (Error::SettingRegister(e1), Error::SettingRegister(e2)) => e1 == e2,
            (Error::PowerupFailed, Error::PowerupFailed) => true,
            (Error::ReadingData(e1), Error::ReadingData(e2)) => e1 == e2,
            (Error::ReadTimeout, Error::ReadTimeout) => true,
            (Error::CalibrationFailure, Error::CalibrationFailure) => true,
            (_, _) => false,
        }
    }
}
