#ifndef ENERGY_METER_TASK_H
#define ENERGY_METER_TASK_H

#include "PeriodicTask.h"
#include "PersistedValue.h"
#include "PdmIds.h"

class BasicClusterEndpoint;

// A pulse count and the timebase ticks it was collected over. Frequencies and
// calibrated values are derived from the pair, so the window length never has
// to be assumed - it is measured
struct PulseWindow
{
    uint32 pulses;
    uint32 ticks;

    void reset()
    {
        pulses = 0;
        ticks = 0;
    }

    void add(uint16 pulseDelta, uint16 tickDelta)
    {
        pulses += pulseDelta;
        ticks += tickDelta;
    }
};

// Acquires pulse frequencies from the on-board HLW8012 energy metering IC
// using the JN516x hardware pulse counters (no remapping needed):
// - CF  (active power pulses)                  -> DIO8 -> Pulse Counter 1
// - CF1 (current or voltage pulses, SEL-muxed) -> DIO1 -> Pulse Counter 0
// The counters run free and are sampled every second (uint16 wrap-around
// arithmetic absorbs counter overflow); the deltas are summed into 32-bit
// windows spanning many seconds, which is where low-load precision comes from.
class EnergyMeterTask : public PeriodicTask
{
    uint16 prevCfCount;
    uint16 prevCf1Count;
    uint16 prevTimebaseTicks;

    // Windows in progress. CF counts continuously; CF1 only while SEL has
    // settled on the mode being measured
    PulseWindow cfWindow;
    PulseWindow cf1Window;
    uint8 windowTicks;      // acquisition ticks accumulated in the present window

    // Last completed windows. Calibrated values are derived from these on
    // demand, so no resolution is lost to an intermediate frequency rounding
    PulseWindow cfResult;
    PulseWindow voltageResult;
    PulseWindow currentResult;

    uint64 cfTotal;         // lifetime CF pulses (energy register, PDM-backed)
    uint32 cf1Total;        // cumulative CF1 pulses since boot (mode-mixed, diagnostic)
    uint8 selCurrentMode;   // SEL state: 0 = voltage, 1 = current
    bool selSettleTick;     // tick straddling a SEL toggle - not attributable to either mode

    PersistedValue<uint64, PDM_ID_ENERGY> persistedEnergyPulses;
    uint64 lastSavedPulses;
    uint32 ticksSinceSave;

    BasicClusterEndpoint * meteringEndpoint;

private:
    EnergyMeterTask();

public:
    static EnergyMeterTask * getInstance();

    void setMeteringEndpoint(BasicClusterEndpoint * ep) { meteringEndpoint = ep; }

    // Raw acquisition values (calibration/diagnostic attributes)
    uint16 getCfFreqDHz() const;
    uint64 getCfTotal() const { return cfTotal; }
    uint32 getCf1Total() const { return cf1Total; }

    // Calibrated electrical values (conversion constants live in the task)
    uint16 getActivePowerDW() const;
    uint16 getVoltageDV() const;
    uint16 getCurrentMA() const;
    uint64 getEnergyWh() const;

protected:
    virtual void timerCallback();
};

#endif // ENERGY_METER_TASK_H
