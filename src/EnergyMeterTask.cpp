#include "zcl_options.h"

#ifdef SUPPORTS_POWER_METERING

#include "EnergyMeterTask.h"
#include "BasicClusterEndpoint.h"

extern "C"
{
    #include "AppHardwareApi.h"
    #include "dbg.h"
}

// Acquisition tick. Deliberately short: the 16-bit pulse counters must not be
// able to wrap between two reads (worst case is CF1 at ~2.2 kHz for 16 A, so
// ~2.2k pulses per tick against a 65535 range), and Timer 0 wraps after 67 s
static const uint32 SAMPLE_PERIOD_MS = 1000;

// Pulses and timebase ticks are summed over this many acquisition ticks before
// the measured values are refreshed. Precision at low load comes from the
// length of the integration window, not from the length of the hardware
// sampling period: at 1 W, CF runs at ~0.24 Hz, so a 1 s window holds 0 or 1
// pulse and quantises the reading to ~4.5 W steps, while 20 s collects ~5
// pulses and resolves ~0.2 W. Summing into 32 bits is what makes the long
// window safe - sampling the hardware every 20 s would already sit at 44k of
// the counter's 65535 range, and 60 s would wrap it silently at high load
static const uint8 INTEGRATION_TICKS = 20;

// Timer 0 free-runs as the sampling timebase: 16 MHz / 2^14 = 976.5625 Hz.
// The ZTIMER 1 s callback jitters when the main loop is busy (radio storms),
// so the window length is measured, never assumed. 16-bit wrap = 67 s
static const uint8 TIMEBASE_PRESCALE = 14;
// 976.5625 Hz as an exact fraction: freq[Hz] = pulses * 15625 / (16 * ticks)
static const uint32 TIMEBASE_HZ_NUM = 15625;
static const uint32 TIMEBASE_HZ_DEN = 16;

// Conversion constants, scaled by 1e4: value = freq_Hz * K / 10000.
// Power: calibrated 2026-08-02 against an inline power meter (1910 W at
// 424.41 Hz CF over a 3 min pulse-count integration) -> 4.5004 W/Hz,
// +8.8 % over the datasheet-nominal 4.138 (shunt below its marked 2 mOhm).
// Voltage: calibrated against a multimeter at the load terminals under load
// (235.5 V read vs 225.3 V displayed -> +4.53 % over datasheet-nominal).
// Current: derived, not directly measured - all HLW8012 channels share Vref
// and the shunt, so Kc = Kp/Kv (+4.05 % over nominal; PF cross-check 0.966).
// Constants are specimen-calibrated on a QBKG11LM; QBKG11LM and QBKG12LM
// share the same board, so they serve as defaults for both.
static const uint32 METERING_W_PER_HZ_E4 = 45004;
static const uint32 METERING_DV_PER_HZ_E4 = 34176;
static const uint32 METERING_MA_PER_HZ_E4 = 75357;

// ZCL ActivePower is a *signed* 16-bit value, so reporting in 0.1 W units
// (ACPowerDivisor = 10) caps at 3276.7 W. Both QBKG11LM and QBKG12LM are rated
// 10 A / 2500 W in total - the two-gang model meters its single shared mains
// input with one HLW8012 - so the cap sits above anything either switch may
// legally carry
static const uint32 ACTIVE_POWER_DW_MAX = 32767;
static const uint32 CURRENT_MA_MAX = 65535;

// Energy register persistence: save when this many pulses accumulated since
// the last save (~0.1 kWh at the calibrated 4.5 J/pulse), or daily if any
// unsaved energy exists. Keeps EEPROM wear negligible (>=5 years even at a
// continuous 5 kWh/day) while bounding the power-cut loss to ~0.1 kWh.
static const uint32 ENERGY_SAVE_PULSE_DELTA = 80000;
static const uint32 ENERGY_SAVE_MAX_TICKS = 86400;

EnergyMeterTask::EnergyMeterTask()
{
    // Drive SEL low for a deterministic CF1 mode. The signal reaches the
    // HLW8012 inverted through a 2N7002; actual polarity is resolved later
    vAHI_DioSetDirection(0, METERING_SEL_MASK);
    vAHI_DioSetOutput(0, METERING_SEL_MASK);

    // Rising edge, debounce off (debounce would cap counting at 1.2-3.7 kHz),
    // counters kept separate (both channels needed), no interrupts
    bAHI_PulseCounterConfigure(E_AHI_PC_1, 0, 0, E_AHI_PC_COMBINE_OFF, FALSE);
    bAHI_PulseCounterConfigure(E_AHI_PC_0, 0, 0, E_AHI_PC_COMBINE_OFF, FALSE);
    bAHI_StartPulseCounter(E_AHI_PC_1);
    bAHI_StartPulseCounter(E_AHI_PC_0);

    // Timebase timer: no interrupts, no output - and no DIO takeover, its
    // pins overlap CF (DIO8), SEL (DIO9) and the button (DIO10)
    vAHI_TimerDIOControl(E_AHI_TIMER_0, FALSE);
    vAHI_TimerEnable(E_AHI_TIMER_0, TIMEBASE_PRESCALE, FALSE, FALSE, FALSE);
    vAHI_TimerStartRepeat(E_AHI_TIMER_0, 0x0000, 0xFFFF);

    // Baseline the counts after start: bAHI_StartPulseCounter() may bump the
    // count by one even without a pulse
    bAHI_Read16BitCounter(E_AHI_PC_1, &prevCfCount);
    bAHI_Read16BitCounter(E_AHI_PC_0, &prevCf1Count);
    prevTimebaseTicks = u16AHI_TimerReadCount(E_AHI_TIMER_0);
    cfWindow.reset();
    cf1Window.reset();
    windowTicks = 0;
    cfResult.reset();
    voltageResult.reset();
    currentResult.reset();
    cf1Total = 0;
    selCurrentMode = 0;         // constructor drove SEL low = voltage mode
    selSettleTick = false;

    // Restore the lifetime energy register
    persistedEnergyPulses.init((uint64)0, "Energy");
    cfTotal = persistedEnergyPulses.getValue();
    lastSavedPulses = cfTotal;
    ticksSinceSave = 0;
    meteringEndpoint = NULL;

    PeriodicTask::init(SAMPLE_PERIOD_MS);
    startTimer(SAMPLE_PERIOD_MS);
}

EnergyMeterTask * EnergyMeterTask::getInstance()
{
    static EnergyMeterTask instance;
    return &instance;
}

// value = freq[Hz] * kE4 / 10000, scaled by `scale` to reach the reporting
// unit (10 for 0.1 W). A full window holds ~44k pulses and ~20k ticks, so the
// numerator needs 64 bits - but nothing is rounded before the final division
static uint32 calibratedValue(const PulseWindow & window, uint32 kE4, uint32 scale)
{
    if(window.ticks == 0)
        return 0;

    uint64 numerator = (uint64)window.pulses * TIMEBASE_HZ_NUM * kE4 * scale;
    return (uint32)(numerator / ((uint64)window.ticks * TIMEBASE_HZ_DEN * 10000));
}

static uint16 freqDHz(const PulseWindow & window)
{
    if(window.ticks == 0)
        return 0;

    uint64 f = (uint64)window.pulses * TIMEBASE_HZ_NUM * 10 / ((uint64)window.ticks * TIMEBASE_HZ_DEN);
    return (f > 65535) ? 65535 : (uint16)f;
}

uint16 EnergyMeterTask::getCfFreqDHz() const
{
    return freqDHz(cfResult);
}

uint16 EnergyMeterTask::getActivePowerDW() const
{
    uint32 deciWatts = calibratedValue(cfResult, METERING_W_PER_HZ_E4, 10);
    return (deciWatts > ACTIVE_POWER_DW_MAX) ? ACTIVE_POWER_DW_MAX : deciWatts;
}

uint16 EnergyMeterTask::getVoltageDV() const
{
    return calibratedValue(voltageResult, METERING_DV_PER_HZ_E4, 1);
}

uint16 EnergyMeterTask::getCurrentMA() const
{
    uint32 mA = calibratedValue(currentResult, METERING_MA_PER_HZ_E4, 1);
    return (mA > CURRENT_MA_MAX) ? CURRENT_MA_MAX : mA;
}

uint64 EnergyMeterTask::getEnergyWh() const
{
    // Each CF pulse is a fixed energy quantum: Wh = pulses * (W/Hz) / 3600
    return cfTotal * METERING_W_PER_HZ_E4 / 36000000;
}

void EnergyMeterTask::timerCallback()
{
    uint16 cfCount, cf1Count;
    bAHI_Read16BitCounter(E_AHI_PC_1, &cfCount);
    bAHI_Read16BitCounter(E_AHI_PC_0, &cf1Count);
    uint16 nowTicks = u16AHI_TimerReadCount(E_AHI_TIMER_0);

    uint16 cfDelta = (uint16)(cfCount - prevCfCount);
    uint16 cf1Delta = (uint16)(cf1Count - prevCf1Count);
    uint16 tickDelta = (uint16)(nowTicks - prevTimebaseTicks);
    prevCfCount = cfCount;
    prevCf1Count = cf1Count;
    prevTimebaseTicks = nowTicks;

    cfTotal += cfDelta;
    cf1Total += cf1Delta;

    // CF is not multiplexed - every tick belongs to the power window
    cfWindow.add(cfDelta, tickDelta);

    // CF1 is only attributable to a mode that was active for the whole tick
    if(selSettleTick)
        selSettleTick = false;
    else
        cf1Window.add(cf1Delta, tickDelta);

    // Wear-aware persistence of the energy register
    ticksSinceSave++;
    if((cfTotal - lastSavedPulses >= ENERGY_SAVE_PULSE_DELTA) ||
       (cfTotal != lastSavedPulses && ticksSinceSave >= ENERGY_SAVE_MAX_TICKS))
    {
        persistedEnergyPulses.setValue(cfTotal);
        lastSavedPulses = cfTotal;
        ticksSinceSave = 0;
    }

    if(++windowTicks >= INTEGRATION_TICKS)
    {
        // Publish the integrated windows and start the next pair
        cfResult = cfWindow;
        if(selCurrentMode)
            currentResult = cf1Window;
        else
            voltageResult = cf1Window;

        DBG_vPrintf(TRUE, "EnergyMeterTask: CF=%d.%d Hz (%d pulses), CF1=%d.%d Hz mode=%c (%d pulses / %d ticks)\n",
                    getCfFreqDHz() / 10, getCfFreqDHz() % 10, (uint16)cfWindow.pulses,
                    freqDHz(cf1Window) / 10, freqDHz(cf1Window) % 10,
                    selCurrentMode ? 'I' : 'V', (uint16)cf1Window.pulses, (uint16)cf1Window.ticks);

        cfWindow.reset();
        cf1Window.reset();
        windowTicks = 0;

        // Alternate SEL between voltage and current measurement. The next tick
        // spans the mode change, so it is dropped from the CF1 window - only
        // one tick is lost per window, not the whole window
        selCurrentMode ^= 1;
        // DIO9 low = voltage mode, high = current mode (2N7002 inverts on its way to SEL)
        if(selCurrentMode)
            vAHI_DioSetOutput(METERING_SEL_MASK, 0);
        else
            vAHI_DioSetOutput(0, METERING_SEL_MASK);
        selSettleTick = true;
    }

    // Push fresh values into the ZCL cluster structs so both reads and the
    // attribute reporting engine (which samples the structs directly when it
    // evaluates the reportable change) see current data
    if(meteringEndpoint)
        meteringEndpoint->updateMeteringAttributes();
}

#endif // SUPPORTS_POWER_METERING
