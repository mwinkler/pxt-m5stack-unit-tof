/**
 * M5Stack ToF Unit (VL53L0X) - Time of Flight distance sensor
 */
//% color=#0079B9 icon="\uf140" block="M5 ToF"
namespace m5tof {
    // I2C Constants
    const DEFAULT_I2C_ADDRESS = 0x29
    const VALID_MODEL_ID = 0xEE

    // Register addresses
    const SYSTEM_RANGE_START = 0x00
    const SYSTEM_SEQUENCE_CONFIG = 0x01
    const SYSTEM_INTERMEASUREMENT_PERIOD = 0x04
    const SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
    const SYSTEM_INTERRUPT_CLEAR = 0x0B
    const RESULT_INTERRUPT_STATUS = 0x13
    const RESULT_RANGE_STATUS = 0x14
    const ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30
    const GLOBAL_CONFIG_VCSEL_WIDTH = 0x32
    const FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
    const MSRC_CONFIG_TIMEOUT_MACROP = 0x46
    const FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47
    const FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48
    const PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50
    const PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
    const PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56
    const PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57
    const MSRC_CONFIG_CONTROL = 0x60
    const FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71
    const GPIO_HV_MUX_ACTIVE_HIGH = 0x84
    const I2C_SLAVE_DEVICE_ADDRESS = 0x8A
    const VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV = 0x89
    const SOFT_RESET = 0xBF
    const MODEL_ID = 0xC0
    const ALGO_PHASECAL_LIM = 0x30

    // Sequence steps
    const RANGE_SEQUENCE_STEP_DSS = 0x28
    const RANGE_SEQUENCE_STEP_PRE_RANGE = 0x40
    const RANGE_SEQUENCE_STEP_FINAL_RANGE = 0x80

    /**
     * Operating mode of the sensor
     */
    export enum Operating {
        //% block="standard (1.6-1.9V)"
        Standard = 0,
        //% block="2V8 mode (2.6-3.5V)"
        Mode2V8 = 1
    }

    /**
     * Measurement mode
     */
    export enum Mode {
        //% block="default (30ms, 1.2m)"
        Default = 0,
        //% block="high accuracy (200ms, 1.2m)"
        HighAccuracy = 1,
        //% block="long range (33ms, 2.0m)"
        LongRange = 2,
        //% block="high speed (20ms, 1.2m)"
        HighSpeed = 3
    }

    /**
     * Range status
     */
    export enum RangeStatus {
        //% block="OK"
        OK = 11,
        //% block="hardware failure"
        HardwareFailure = 1,
        //% block="phase failure"
        PhaseFailure = 5,
        //% block="min range failure"
        MinRangeFailure = 6,
        //% block="signal failure"
        SignalFailure = 4,
        //% block="unknown"
        Unknown = 255
    }

    // Internal state
    let currentAddress = DEFAULT_I2C_ADDRESS
    let stopVariable = 0xFF
    let currentMode = Mode.Default
    let isInitialized = false
    let lastRange = 0
    let lastStatus = RangeStatus.Unknown

    // Default values table
    const defaultValues: number[][] = [
        [0xFF, 0x01], [0x00, 0x00], [0xFF, 0x00], [0x09, 0x00],
        [0x10, 0x00], [0x11, 0x00], [0x24, 0x01], [0x25, 0xFF],
        [0x75, 0x00], [0xFF, 0x01], [0x4E, 0x2C], [0x48, 0x00],
        [0x30, 0x20], [0xFF, 0x00], [0x30, 0x09], [0x54, 0x00],
        [0x31, 0x04], [0x32, 0x03], [0x40, 0x83], [0x46, 0x25],
        [0x60, 0x00], [0x27, 0x00], [0x50, 0x06], [0x51, 0x00],
        [0x52, 0x96], [0x56, 0x08], [0x57, 0x30], [0x61, 0x00],
        [0x62, 0x00], [0x64, 0x00], [0x65, 0x00], [0x66, 0xA0],
        [0xFF, 0x01], [0x22, 0x32], [0x47, 0x14], [0x49, 0xFF],
        [0x4A, 0x00], [0xFF, 0x00], [0x7A, 0x0A], [0x7B, 0x00],
        [0x78, 0x21], [0xFF, 0x01], [0x23, 0x34], [0x42, 0x00],
        [0x44, 0xFF], [0x45, 0x26], [0x46, 0x05], [0x40, 0x40],
        [0x0E, 0x06], [0x20, 0x1A], [0x43, 0x40], [0xFF, 0x00],
        [0x34, 0x03], [0x35, 0x44], [0xFF, 0x01], [0x31, 0x04],
        [0x4B, 0x09], [0x4C, 0x05], [0x4D, 0x04], [0xFF, 0x00],
        [0x44, 0x00], [0x45, 0x20], [0x47, 0x08], [0x48, 0x28],
        [0x67, 0x00], [0x70, 0x04], [0x71, 0x01], [0x72, 0xFE],
        [0x76, 0x00], [0x77, 0x00], [0xFF, 0x01], [0x0D, 0x01],
        [0xFF, 0x00], [0x80, 0x01], [0x01, 0xF8], [0xFF, 0x01],
        [0x8E, 0x01], [0x00, 0x01], [0xFF, 0x00], [0x80, 0x00]
    ]

    // Pre-range configuration tables
    const prePhaseTable = [0x18, 0x30, 0x40, 0x50]
    const preRangeTimeoutMacropTable = [0x00AF, 0x0096, 0x0083, 0x0073]
    const preMsrcTimeoutMacropTable = [0x2B, 0x25, 0x20, 0x1C]

    // Final range configuration tables
    const finalValuesTable: number[][] = [
        [0x10, 0x08, 0x02, 0x0C, 0x30],
        [0x28, 0x08, 0x03, 0x09, 0x20],
        [0x38, 0x08, 0x03, 0x08, 0x20],
        [0x48, 0x08, 0x03, 0x07, 0x20]
    ]

    const preAndFinalTimeoutMacropTable: number[][] = [
        [0x02B0, 0x0296, 0x0284, 0x01EF],
        [0x02AA, 0x028F, 0x01FE, 0x01E3],
        [0x02A6, 0x028B, 0x01F2, 0x01D9],
        [0x02A3, 0x0288, 0x01EC, 0x01D3]
    ]

    const intervalTable = [30, 200, 33, 20]

    // Range status lookup table
    const rangeStatusTable = [
        RangeStatus.Unknown, RangeStatus.HardwareFailure,
        RangeStatus.HardwareFailure, RangeStatus.HardwareFailure,
        RangeStatus.SignalFailure, RangeStatus.PhaseFailure,
        RangeStatus.MinRangeFailure, RangeStatus.Unknown,
        RangeStatus.MinRangeFailure, RangeStatus.PhaseFailure,
        RangeStatus.MinRangeFailure, RangeStatus.OK,
        RangeStatus.Unknown, RangeStatus.Unknown,
        RangeStatus.Unknown, RangeStatus.Unknown
    ]

    // Helper functions for I2C communication
    function writeReg8(reg: number, value: number): boolean {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        return pins.i2cWriteBuffer(currentAddress, buf) == 0
    }

    function writeReg16BE(reg: number, value: number): boolean {
        let buf = pins.createBuffer(3)
        buf[0] = reg
        buf[1] = (value >> 8) & 0xFF
        buf[2] = value & 0xFF
        return pins.i2cWriteBuffer(currentAddress, buf) == 0
    }

    function writeReg32BE(reg: number, value: number): boolean {
        let buf = pins.createBuffer(5)
        buf[0] = reg
        buf[1] = (value >> 24) & 0xFF
        buf[2] = (value >> 16) & 0xFF
        buf[3] = (value >> 8) & 0xFF
        buf[4] = value & 0xFF
        return pins.i2cWriteBuffer(currentAddress, buf) == 0
    }

    function readReg8(reg: number): number {
        pins.i2cWriteNumber(currentAddress, reg, NumberFormat.UInt8LE)
        return pins.i2cReadNumber(currentAddress, NumberFormat.UInt8LE)
    }

    function readReg16BE(reg: number): number {
        pins.i2cWriteNumber(currentAddress, reg, NumberFormat.UInt8LE)
        return pins.i2cReadNumber(currentAddress, NumberFormat.UInt16BE)
    }

    function readRegBuffer(reg: number, len: number): Buffer {
        pins.i2cWriteNumber(currentAddress, reg, NumberFormat.UInt8LE)
        return pins.i2cReadBuffer(currentAddress, len)
    }

    function writeDefaultValues(): boolean {
        for (let i = 0; i < defaultValues.length; i++) {
            if (!writeReg8(defaultValues[i][0], defaultValues[i][1])) {
                return false
            }
        }
        return true
    }

    function performSingleRefCalibration(vhv: boolean): boolean {
        let seqCfg = vhv ? 0x01 : 0x02
        let sysStart = (vhv ? 0x40 : 0x00) | 0x01

        if (!writeReg8(SYSTEM_SEQUENCE_CONFIG, seqCfg) ||
            !writeReg8(SYSTEM_RANGE_START, sysStart)) {
            return false
        }

        let timeout = control.millis() + 5000
        while ((readReg8(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
            if (control.millis() > timeout) {
                return false
            }
            basic.pause(1)
        }

        return writeReg8(SYSTEM_INTERRUPT_CLEAR, 0x01) &&
            writeReg8(SYSTEM_RANGE_START, 0x00)
    }

    function writeVcselPeriodRange(prePclk: number, finalPclk: number): boolean {
        let preVcselPeriod = (prePclk >> 1) - 1
        let finalVcselPeriod = (finalPclk >> 1) - 1

        if ((prePclk & 1) || prePclk < 12 || prePclk > 18) {
            return false
        }
        if ((finalPclk & 1) || finalPclk < 8 || finalPclk > 14) {
            return false
        }

        // Pre-range configuration
        let preIdx = (prePclk - 12) >> 1
        if (!writeReg8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, prePhaseTable[preIdx]) ||
            !writeReg8(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08) ||
            !writeReg8(PRE_RANGE_CONFIG_VCSEL_PERIOD, preVcselPeriod) ||
            !writeReg16BE(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, preRangeTimeoutMacropTable[preIdx]) ||
            !writeReg8(MSRC_CONFIG_TIMEOUT_MACROP, preMsrcTimeoutMacropTable[preIdx])) {
            return false
        }

        // Final range configuration
        let finalIdx = (finalPclk - 8) >> 1
        let finalVals = finalValuesTable[finalIdx]
        if (!writeReg8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, finalVals[0]) ||
            !writeReg8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, finalVals[1]) ||
            !writeReg8(GLOBAL_CONFIG_VCSEL_WIDTH, finalVals[2]) ||
            !writeReg8(ALGO_PHASECAL_CONFIG_TIMEOUT, finalVals[3]) ||
            !writeReg8(0xFF, 0x01) ||
            !writeReg8(ALGO_PHASECAL_LIM, finalVals[4]) ||
            !writeReg8(0xFF, 0x00) ||
            !writeReg8(PRE_RANGE_CONFIG_VCSEL_PERIOD, finalVcselPeriod) ||
            !writeReg16BE(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                preAndFinalTimeoutMacropTable[preIdx][finalIdx])) {
            return false
        }

        let scfg = readReg8(SYSTEM_SEQUENCE_CONFIG)
        return performSingleRefCalibration(false) &&
            writeReg8(SYSTEM_SEQUENCE_CONFIG, scfg)
    }

    function writeSignalRateLimit(mcps: number): boolean {
        if (mcps < 0 || mcps >= 512) {
            return false
        }
        let value = Math.floor(mcps * 128)
        return writeReg16BE(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, value)
    }

    function setMode(mode: Mode): boolean {
        if (!writeReg8(0x80, 0x01) || !writeReg8(0xFF, 0x01) ||
            !writeReg8(0x00, 0x00) || !writeReg8(0x91, stopVariable) ||
            !writeReg8(0x00, 0x01) || !writeReg8(0xFF, 0x00) ||
            !writeReg8(0x80, 0x00)) {
            return false
        }

        let success = false
        if (mode == Mode.Default) {
            success = writeSignalRateLimit(0.25) && writeVcselPeriodRange(14, 10)
        } else if (mode == Mode.HighAccuracy) {
            success = writeSignalRateLimit(0.25) &&
                writeVcselPeriodRange(14, 10) &&
                writeReg16BE(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 0x059A)
        } else if (mode == Mode.HighSpeed) {
            success = writeSignalRateLimit(0.25) &&
                writeVcselPeriodRange(14, 10) &&
                writeReg16BE(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 0x00D5)
        } else if (mode == Mode.LongRange) {
            success = writeSignalRateLimit(0.1) && writeVcselPeriodRange(18, 14)
        }

        if (success) {
            let interval = intervalTable[mode]
            if (writeReg32BE(SYSTEM_INTERMEASUREMENT_PERIOD, interval)) {
                currentMode = mode
                return true
            }
        }
        return false
    }

    function writeDefaultSettings(operating: Operating): boolean {
        // Set operating condition
        if (!writeReg8(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
            operating == Operating.Mode2V8 ? 1 : 0)) {
            return false
        }

        // Set I2C standard mode
        stopVariable = 0xFF
        if (!writeReg8(0x88, 0x00) || !writeReg8(0x80, 0x01) ||
            !writeReg8(0xFF, 0x01) || !writeReg8(0x00, 0x00)) {
            return false
        }
        stopVariable = readReg8(0x91)
        if (!writeReg8(0x00, 0x01) || !writeReg8(0xFF, 0x00) ||
            !writeReg8(0x80, 0x00)) {
            return false
        }

        // Configure signal
        let msrcCtrl = readReg8(MSRC_CONFIG_CONTROL)
        if (!writeReg8(MSRC_CONFIG_CONTROL, msrcCtrl | 0x12)) {
            return false
        }

        // Write default values
        if (!writeDefaultValues()) {
            return false
        }

        // Set interrupt config
        let gpioHvMux = readReg8(GPIO_HV_MUX_ACTIVE_HIGH)
        if (!writeReg8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04) ||
            !writeReg8(GPIO_HV_MUX_ACTIVE_HIGH, gpioHvMux & ~0x10) ||
            !writeReg8(SYSTEM_INTERRUPT_CLEAR, 0x01)) {
            return false
        }

        // Specific sequence steps
        if (!writeReg8(SYSTEM_SEQUENCE_CONFIG,
            RANGE_SEQUENCE_STEP_DSS | RANGE_SEQUENCE_STEP_PRE_RANGE |
            RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
            return false
        }

        // Temperature calibration
        if (!performSingleRefCalibration(true) || !performSingleRefCalibration(false)) {
            return false
        }

        // Restore sequence steps
        if (!writeReg8(SYSTEM_SEQUENCE_CONFIG,
            RANGE_SEQUENCE_STEP_DSS | RANGE_SEQUENCE_STEP_PRE_RANGE |
            RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
            return false
        }

        return true
    }

    function isDataReady(): boolean {
        return (readReg8(RESULT_INTERRUPT_STATUS) & 0x07) != 0
    }

    function readMeasurement(): boolean {
        let data = readRegBuffer(RESULT_RANGE_STATUS, 12)
        let statusBits = (data[0] & 0x78) >> 3
        let isValid = statusBits == 11

        lastStatus = rangeStatusTable[statusBits]

        if (isValid) {
            lastRange = (data[10] << 8) | data[11]
            return true
        } else {
            lastRange = -1
            return false
        }
    }

    /**
     * Initialize the ToF sensor
     * @param operating Operating voltage mode, eg: Operating.Mode2V8
     * @param mode Measurement mode, eg: Mode.Default
     */
    //% blockId=m5tof_initialize
    //% block="initialize ToF sensor|operating mode %operating|measurement mode %mode"
    //% operating.defl=Operating.Mode2V8
    //% mode.defl=Mode.Default
    //% weight=100
    //% blockGap=8
    export function initialize(
        operating: Operating = Operating.Mode2V8,
        mode: Mode = Mode.Default
    ): void {
        // Check model ID
        let modelId = readReg8(MODEL_ID)
        if (modelId != VALID_MODEL_ID) {
            return
        }

        // Write default settings
        if (!writeDefaultSettings(operating)) {
            return
        }

        // Set mode
        if (!setMode(mode)) {
            return
        }

        // Start periodic measurement
        if (!startMeasurement()) {
            return
        }

        isInitialized = true
    }

    /**
     * Start continuous distance measurement
     */
    //% blockId=m5tof_start_measurement
    //% block="start ToF measurement"
    //% weight=90
    //% blockGap=8
    export function startMeasurement(): boolean {
        if (!writeReg8(0x80, 0x01) || !writeReg8(0xFF, 0x01) ||
            !writeReg8(0x00, 0x00) || !writeReg8(0x91, stopVariable) ||
            !writeReg8(0x00, 0x01) || !writeReg8(0xFF, 0x00) ||
            !writeReg8(0x80, 0x00) || !writeReg8(SYSTEM_RANGE_START, 0x02) ||
            !writeReg8(SYSTEM_INTERRUPT_CLEAR, 0x01)) {
            return false
        }
        return true
    }

    /**
     * Stop continuous distance measurement
     */
    //% blockId=m5tof_stop_measurement
    //% block="stop ToF measurement"
    //% weight=89
    //% blockGap=8
    export function stopMeasurement(): boolean {
        return writeReg8(SYSTEM_RANGE_START, 0x01) &&
            writeReg8(0xFF, 0x01) &&
            writeReg8(0x00, 0x00) &&
            writeReg8(0x91, 0x00) &&
            writeReg8(0xFF, 0x00)
    }

    /**
     * Update and read the latest distance measurement
     * Call this regularly to get new measurements
     */
    //% blockId=m5tof_update
    //% block="update ToF reading"
    //% weight=88
    //% blockGap=8
    export function update(): void {
        if (!isInitialized) {
            return
        }

        if (isDataReady()) {
            readMeasurement()
            writeReg8(SYSTEM_INTERRUPT_CLEAR, 0x01)
        }
    }

    /**
     * Get the distance in millimeters from the last measurement
     * Returns -1 if measurement is invalid
     */
    //% blockId=m5tof_distance
    //% block="ToF distance (mm)"
    //% weight=80
    //% blockGap=8
    export function distance(): number {
        return lastRange
    }

    /**
     * Get the distance in centimeters from the last measurement
     * Returns -1 if measurement is invalid
     */
    //% blockId=m5tof_distance_cm
    //% block="ToF distance (cm)"
    //% weight=79
    //% blockGap=8
    export function distanceCM(): number {
        return lastRange >= 0 ? Math.floor(lastRange / 10) : -1
    }

    /**
     * Check if the last measurement is valid
     */
    //% blockId=m5tof_is_valid
    //% block="ToF reading is valid"
    //% weight=78
    //% blockGap=8
    export function isValid(): boolean {
        return lastRange >= 0
    }

    /**
     * Get the status of the last measurement
     */
    //% blockId=m5tof_status
    //% block="ToF measurement status"
    //% weight=77
    //% blockGap=8
    export function status(): RangeStatus {
        return lastStatus
    }

    /**
     * Perform a single distance measurement (blocking)
     * @param timeoutMs Maximum time to wait for measurement in milliseconds, eg: 1000
     */
    //% blockId=m5tof_measure_single
    //% block="measure ToF distance once|timeout %timeoutMs ms"
    //% timeoutMs.defl=1000
    //% weight=70
    //% blockGap=8
    export function measureSingle(timeoutMs: number = 1000): number {
        if (!isInitialized) {
            return -1
        }

        // Prepare for single shot
        if (!writeReg8(0x80, 0x01) || !writeReg8(0xFF, 0x01) ||
            !writeReg8(0x00, 0x00) || !writeReg8(0x91, stopVariable) ||
            !writeReg8(0x00, 0x01) || !writeReg8(0xFF, 0x00) ||
            !writeReg8(0x80, 0x00) ||
            !writeReg8(SYSTEM_RANGE_START, 0x01)) {
            return -1
        }

        // Wait for measurement to start
        let timeout = control.millis() + timeoutMs
        while ((readReg8(SYSTEM_RANGE_START) & 0x01) != 0) {
            if (control.millis() > timeout) {
                return -1
            }
            basic.pause(1)
        }

        // Wait for data ready
        timeout = control.millis() + timeoutMs
        while (!isDataReady()) {
            if (control.millis() > timeout) {
                return -1
            }
            basic.pause(1)
        }

        // Read measurement
        readMeasurement()
        writeReg8(SYSTEM_INTERRUPT_CLEAR, 0x01)

        return lastRange
    }

    /**
     * Change the measurement mode
     * @param mode New measurement mode
     */
    //% blockId=m5tof_set_mode
    //% block="set ToF mode to %mode"
    //% weight=60
    //% blockGap=8
    export function changeMode(mode: Mode): boolean {
        return setMode(mode)
    }

    /**
     * Software reset the sensor
     * Requires re-initialization after reset
     */
    //% blockId=m5tof_reset
    //% block="reset ToF sensor"
    //% weight=50
    //% blockGap=8
    export function reset(): boolean {
        if (!writeReg8(SOFT_RESET, 0x00)) {
            return false
        }
        basic.pause(1)
        if (!writeReg8(SOFT_RESET, 0x01)) {
            return false
        }
        basic.pause(1)
        isInitialized = false
        return true
    }

    /**
     * Check if the sensor is properly connected
     */
    //% blockId=m5tof_is_connected
    //% block="ToF sensor is connected"
    //% weight=38
    //% blockGap=8
    //% advanced=true
    export function isConnected(): boolean {
        let modelId = readReg8(MODEL_ID)
        return modelId == VALID_MODEL_ID
    }
}
