use byteorder::ByteOrder as _;
use core::slice;
use embedded_hal_async::delay;

#[cfg(feature = "blocking_i2c")]
use embedded_hal::i2c;
#[cfg(not(feature = "blocking_i2c"))]
use embedded_hal_async::i2c;

pub use crate::constants::*;
use crate::Error;

pub struct Nau7802<I2C: i2c::I2c, DELAY: delay::DelayNs> {
    i2c_dev: I2C,
    delay: DELAY,
}

impl<I2C: i2c::I2c, DELAY: delay::DelayNs> Nau7802<I2C, DELAY> {
    const DEVICE_ADDRESS: u8 = 0x2A;

    #[inline]
    pub async fn new(i2c_dev: I2C, delay: DELAY) -> Result<Self, Error<I2C::Error>> {
        Self::new_with_settings(
            i2c_dev,
            Ldo::L3v3,
            Gain::G128,
            SamplesPerSecond::SPS10,
            delay,
        )
        .await
    }

    pub async fn new_with_settings(
        i2c_dev: I2C,
        ldo: Ldo,
        gain: Gain,
        sps: SamplesPerSecond,
        delay: DELAY,
    ) -> Result<Self, Error<I2C::Error>> {
        let mut adc = Self { i2c_dev, delay };

        adc.reset().await?;
        adc.power_up().await?;
        adc.set_ldo(ldo).await?;
        adc.set_gain(gain).await?;
        adc.set_sample_rate(sps).await?;
        adc.misc_init().await?;
        adc.calibrate().await?;

        Ok(adc)
    }

    async fn data_available(&mut self) -> Result<bool, Error<I2C::Error>> {
        self.get_bit(Register::PuCtrl, PuCtrlBits::CR).await
    }

    /// Checks for new data, will return after ~400 milliseconds with an error if
    /// no data could be read. (100 milliseconds is the longest the chip will
    /// take for a reading.
    ///
    /// result is 24bit unsigned
    pub async fn read(&mut self) -> Result<u32, Error<I2C::Error>> {
        let mut attempt = 0;
        while !self.data_available().await? {
            if attempt >= 4 {
                // 10 (100ms) should work but it does not
                return Err(Error::ReadTimeout);
            }

            self.delay.delay_ms(100).await;
            attempt += 1;
        }

        self.read_unchecked().await
    }

    /// assumes that data_available has been called and returned true
    async fn read_unchecked(&mut self) -> Result<u32, Error<I2C::Error>> {
        self.request_register(Register::AdcoB2).await?;

        let mut buf = [0u8; 3]; // will hold an i24
        let op = self.i2c_dev.read(Self::DEVICE_ADDRESS, &mut buf);
        #[cfg(not(feature = "blocking_i2c"))]
        let op = op.await;
        op.map_err(Error::ReadingData)?;

        let adc_result = byteorder::BigEndian::read_u24(&buf);
        Ok(adc_result)
    }

    pub async fn calibrate(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_bit(Register::Ctrl2, Ctrl2RegisterBits::Cals)
            .await?;

        while self.calibration_status().await? == CalibrationStatus::InProgress {
            self.delay.delay_ms(1).await;
        }

        Ok(())
    }

    async fn calibration_status(&mut self) -> Result<CalibrationStatus, Error<I2C::Error>> {
        if self
            .get_bit(Register::Ctrl2, Ctrl2RegisterBits::Cals)
            .await?
        {
            return Ok(CalibrationStatus::InProgress);
        }

        if self
            .get_bit(Register::Ctrl2, Ctrl2RegisterBits::CalError)
            .await?
        {
            return Err(Error::CalibrationFailure);
        }

        Ok(CalibrationStatus::Success)
    }

    pub async fn set_sample_rate(
        &mut self,
        sps: SamplesPerSecond,
    ) -> Result<(), Error<I2C::Error>> {
        const SPS_MASK: u8 = 0b1000_1111;
        const SPS_START_BIT_IDX: u8 = 4;

        self.set_function_helper(Register::Ctrl2, SPS_MASK, SPS_START_BIT_IDX, sps as u8)
            .await
    }

    pub async fn set_gain(&mut self, gain: Gain) -> Result<(), Error<I2C::Error>> {
        const GAIN_MASK: u8 = 0b1111_1000;
        const GAIN_START_BIT: u8 = 0;

        self.set_function_helper(Register::Ctrl1, GAIN_MASK, GAIN_START_BIT, gain as u8)
            .await
    }

    pub async fn set_ldo(&mut self, ldo: Ldo) -> Result<(), Error<I2C::Error>> {
        const LDO_MASK: u8 = 0b1100_0111;
        const LDO_START_BIT: u8 = 3;

        self.set_function_helper(Register::Ctrl1, LDO_MASK, LDO_START_BIT, ldo as u8)
            .await?;

        self.set_bit(Register::PuCtrl, PuCtrlBits::AVDDS).await
    }

    async fn power_up(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_bit(Register::PuCtrl, PuCtrlBits::PUD).await?;
        self.set_bit(Register::PuCtrl, PuCtrlBits::PUA).await?;

        // After about 200 microseconds, the PWRUP bit will be Logic=1 indicating the
        // device is ready for the remaining programming setup.
        self.delay.delay_us(200).await;
        for attempt in 0..5 {
            if self.is_powered_up().await? {
                return Ok(());
            }
            self.delay.delay_us(5 * attempt).await;
        }

        Err(Error::PowerupFailed)
    }

    async fn is_powered_up(&mut self) -> Result<bool, Error<I2C::Error>> {
        self.get_bit(Register::PuCtrl, PuCtrlBits::PUR).await
    }

    async fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_bit(Register::PuCtrl, PuCtrlBits::RR).await?;
        self.delay.delay_ms(1).await;
        self.clear_bit(Register::PuCtrl, PuCtrlBits::RR).await
    }

    async fn misc_init(&mut self) -> Result<(), Error<I2C::Error>> {
        const TURN_OFF_CLK_CHPL: u8 = 0x30;

        // Turn off CLK_CHP. From 9.1 power on sequencing
        self.set_register(Register::Adc, TURN_OFF_CLK_CHPL).await?;

        // Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note
        self.set_bit(Register::PgaPwr, PgaPwrRegisterBits::CapEn)
            .await
    }

    async fn set_function_helper(
        &mut self,
        reg: Register,
        mask: u8,
        start_idx: u8,
        new_val: u8,
    ) -> Result<(), Error<I2C::Error>> {
        let mut val = self.get_register(reg).await?;
        val &= mask;
        val |= new_val << start_idx;

        self.set_register(reg, val).await
    }

    async fn set_bit<B: RegisterBits>(
        &mut self,
        addr: Register,
        bit_idx: B,
    ) -> Result<(), Error<I2C::Error>> {
        let mut val = self.get_register(addr).await?;
        val |= 1 << bit_idx.get();
        self.set_register(addr, val).await
    }

    async fn clear_bit<B: RegisterBits>(
        &mut self,
        addr: Register,
        bit_idx: B,
    ) -> Result<(), Error<I2C::Error>> {
        let mut val = self.get_register(addr).await?;
        val &= !(1 << bit_idx.get());
        self.set_register(addr, val).await
    }

    async fn get_bit<B: RegisterBits>(
        &mut self,
        addr: Register,
        bit_idx: B,
    ) -> Result<bool, Error<I2C::Error>> {
        let mut val = self.get_register(addr).await?;
        val &= 1 << bit_idx.get();
        Ok(val != 0)
    }

    async fn set_register(&mut self, reg: Register, val: u8) -> Result<(), Error<I2C::Error>> {
        let transaction = [reg as _, val];

        let op = self.i2c_dev.write(Self::DEVICE_ADDRESS, &transaction);
        #[cfg(not(feature = "blocking_i2c"))]
        let op = op.await;
        op.map_err(Error::SettingRegister)
    }

    async fn get_register(&mut self, reg: Register) -> Result<u8, Error<I2C::Error>> {
        let reg = [reg as u8];

        let mut val = 0;
        let op = self
            .i2c_dev
            .write_read(Self::DEVICE_ADDRESS, &reg, slice::from_mut(&mut val));
        #[cfg(not(feature = "blocking_i2c"))]
        let op = op.await;
        op.map_err(Error::GettingRegister)?;
        Ok(val)
    }

    async fn request_register(&mut self, reg: Register) -> Result<(), Error<I2C::Error>> {
        let reg = reg as u8;

        let op = self
            .i2c_dev
            .write(Self::DEVICE_ADDRESS, slice::from_ref(&reg));
        #[cfg(not(feature = "blocking_i2c"))]
        let op = op.await;
        op.map_err(Error::RequestingRegister)
    }
}
