#pragma once

#include <type_traits>
#include <vector>

#include "cnpg/dsp/Common.h"

// TriodeStage -- see docs/plan.md section 2.9. Task P1.1 landed TriodeStageParams only (the
// APVTS-backed P1 parameter surface, already its final shape per the draft). Task P1.7 grows this
// header into the full TriodeStage class: KorenTriodeParams, TransferTableView, and the deferred
// setSupplyVoltage/setHeaterVoltage hooks. Zero JUCE includes.
//
// -----------------------------------------------------------------------------------------------
// What this models, and what it deliberately does NOT model.
// -----------------------------------------------------------------------------------------------
//
// A single fixed classic ECC83 (12AX7) common-cathode gain stage, evaluated as a STATIC
// (memoryless) waveshaper: for every input sample, TriodeStage looks up the same offline-solved
// input-voltage -> output-voltage curve, with no state carried between samples. Locked topology
// (docs/plan.md section 2.9, task brief step 1):
//   - 100k plate load resistor (Rp), to a fixed B+ supply.
//   - Bypassed cathode bias: a cathode resistor sets the DC self-bias operating point, but its own
//     bypass capacitor is assumed to hold the cathode at that fixed DC voltage for AC signals --
//     the cathode does not move with the signal, so the full grid-referred input swings Vgk.
//   - 1M next-stage load, AC-coupled (the following stage's grid-leak resistor, in parallel with
//     Rp for the signal's effective plate load once the coupling cap is treated as a short at
//     signal frequencies -- see "AC load line" below).
//   - A soft grid-current clamp via `rgi`: once the grid swings positive of the cathode, grid
//     conduction begins and softly compresses the effective grid drive rather than hard-clipping
//     it (see "grid-current clamp" below).
// `publishedEcc83()` hardcodes the published Koren ECC83/12AX7 SPICE parameter set (Koren, N.,
// "Improved Vacuum Tube Models for SPICE Simulations," Glass Audio 8(5), 1996; this exact
// (mu, ex, kg1, kp, kvb, rgi) tuple is the parameter set most commonly circulated for the 12AX7 in
// tube-amp SPICE modelling references, e.g. Duncan Amplification's published triode SPICE model
// library) -- see the literal values at `publishedEcc83()`'s definition in TriodeStage.cpp.
//
// Required caveat (user-mandated, RC2, docs/plan.md line 50): a static waveshaper has no bias
// drift (no coupling-cap/cathode-bypass dynamics) -- nothing here depends on signal history. So
// the P1 chain built on TriodeStage is explicitly NOT a voicing reference for drive feel:
// drive-feel voicing conclusions are deferred until a dynamic stage lands (P3+) -- one with real
// state (coupling-cap high-pass behavior, cathode-bypass corner interacting with signal level,
// thermal/bias settling). Anyone tuning `drive`/`outputTrimDb` defaults or writing listening notes
// against this P1 stage should read this paragraph first.
//
// -----------------------------------------------------------------------------------------------
// The Koren equations (evaluated in double throughout; see TriodeStage.cpp for the exact code).
// -----------------------------------------------------------------------------------------------
//
//   E1 = (Vp / kp) * ln(1 + exp(kp * (1/mu + Vgk / sqrt(kvb + Vp^2))))
//   Ip = (E1 > 0) ? 2 * E1^ex / kg1 : 0
//
// `ln(1+exp(x))` (softplus) is evaluated via `log1p` with a linear large-x fallback (`x` itself,
// once `x` is large enough that `exp(x)` would overflow double but softplus(x) - x is already far
// below machine epsilon) so the offline solve never overflows even while probing extreme grid
// voltages during table construction -- see the numerical-care note on `TriodeStage.cpp`.
//
// -----------------------------------------------------------------------------------------------
// DC operating point, AC load line, and the offline table (task brief step 2).
// -----------------------------------------------------------------------------------------------
//
// prepare() solves, once, entirely offline:
//   1. The DC self-bias operating point (Vp0, Ip0, Vk0): with the grid at DC ground (no grid
//      current at idle) and the cathode floating on its own bias resistor, Ip0 = Ip(Vb - Ip0*Rp,
//      -Ip0*Rk) is a single implicit equation in Ip0, solved by bisection (both sides of the
//      equation move monotonically in Ip0, so the root is unique and bisection is safe and
//      allocation-free).
//   2. For a uniform grid of grid-referred input voltages spanning a generous domain, the AC
//      load-line intersection Vp(Vgk): the tube's own Ip(Vp,Vgk) curve against the AC load line
//      Ip = Ip0 + (Vp0-Vp)/Rac through the DC operating point, where Rac = Rp || Rnext is the
//      effective plate load the signal sees once the coupling cap to the next stage's 1M grid-leaf
//      is treated as a short (the "no coupling-cap dynamics" caveat above means this stage does
//      not model the cap's own frequency response/time-constant -- only the resistive loading
//      effect of the next stage, which is a static property). Also solved by bisection (Ip(Vp,*)
//      is increasing in Vp; the load-line term is decreasing in Vp; a single monotone root).
//   3. Grid-current clamp: for Vgk_raw = vin - Vk0 > 0 (grid swinging positive of the bypassed
//      cathode), the effective Vgk used above is softened by a resistive divider between an
//      assumed Thevenin drive-source resistance and `rgi`: Vgk = Vgk_raw * rgi/(rgi+Rsource). This
//      is a genuine (if simplified) circuit reading of grid conduction -- current drawn by the
//      grid through `rgi` against whatever is driving it -- and is "soft" in the sense the brief
//      names: a graceful, continuous gain reduction past the knee, not a hard voltage ceiling.
//      `Rsource` is this module's own implementation choice (not part of the brief's locked
//      circuit values); see TriodeStage.cpp for the exact value and its rationale.
//   4. The resulting raw curve is inverted (common-cathode stage: plate voltage falls as grid
//      voltage rises), has its own value at vin=0 subtracted exactly (task brief step 5's "zero
//      input -> DC-removed zero output": this is provably 0 by construction here, since vin=0 maps
//      to exactly the operating point the load line was built through -- the subtraction only
//      guards residual bisection rounding, forced to an exact 0.0f at that one table node), then
//      normalized so the curve's own small-signal slope at the origin is unity -- i.e. a small
//      grid-voltage swing produces an equal-magnitude normalized-output swing. This means `drive`'s
//      volts-per-full-scale calibration (see below) is the only free gain constant in the whole
//      chain: the table itself never needs re-tuning if the circuit constants change.
// Interpolation is cubic (Catmull-Rom, uniform knot spacing) over this fixed table, evaluated in
// double and rounded to float on output; the fractional table index is computed so that an exact
// vin == 0.0 always lands on an exact integer node (0 * anything finite == 0.0 in IEEE-754,
// preserved through the index arithmetic), so cubic interpolation's exact-node-reproduction
// property at t=0 makes "zero input -> zero output" hold bit-exactly, not just approximately.
//
// -----------------------------------------------------------------------------------------------
// Gain staging: `drive` and the +16 dB summing-headroom convention (task brief step 2).
// -----------------------------------------------------------------------------------------------
//
// `drive` (0..1 nominal, clamped defensively wider) maps to a linear pre-gain of `2 * drive`, so
// drive=0.5 (the documented default) is unity: a single string at the -18 dBFS per-string nominal
// (docs/plan.md section 1.9's pickup calibration) passes through this stage close to its own
// intrinsic small-signal gain (normalized to unity above), landing in the gentle, mostly-linear
// part of the curve -- appropriate coloration for what is topologically an amp's INPUT stage, not
// a lead/overdrive stage. drive=1.0 doubles that swing (+6 dB), and multi-string summing before
// this stage can add up to the ~+16 dB documented headroom budget -- at those levels the grid
// swing approaches and crosses the grid-conduction knee and the plate-current cutoff region,
// producing the intended increasing coloration without needing per-note gain-staging care from the
// player. `outputTrimDb` is a plain post-stage dB trim (0 dB default). `bypass` is an immediate,
// unramped passthrough switch (task brief step 2: "passes input through untouched") -- input is
// copied to output bit-exactly with no gain, table lookup, or ramp-state mutation.
//
// -----------------------------------------------------------------------------------------------
// Deferred hooks (task brief step 3; REQUIRED, RC2): supply/heater THD contracts and offline
// table loading, present at signature level now, audibly inert through P2.
// -----------------------------------------------------------------------------------------------
//
// `setSupplyVoltage`/`setHeaterVoltage` are STORE-ONLY through P2: the value passed is retained in
// a member but never read by prepare(), setParams(), or process() -- the fixed default supply
// voltage baked into publishedEcc83()'s companion circuit constants (TriodeStage.cpp) is always
// what the offline table solve uses. Calling either hook any number of times, in any order,
// produces byte-identical process() output to never having called them at all ("audibly inert" per
// the task brief's [contract] test). P3+ is where a dynamic stage makes these audible (shifting
// operating point / emission-dependent THD).
//
// `loadTransferTable` is a safe validation-only stub through P2: it checks `values != nullptr`,
// `size >= 2`, a finite and strictly-increasing (`inputMin < inputMax`) domain, and that every
// sample is finite, returning false on any violation and true otherwise -- but never stores the
// table or wires it into process() (the internally-solved Koren curve above is always what
// process() runs through P2). Tables are produced only by a future in-repo C++ headless tool
// (docs/plan.md section on tools/ activation) -- never Python or LTspice in P0-P2 -- and only
// consumed for real once a dynamic stage supersedes this static one (P3+).

namespace cnpg::dsp {

// Published Koren ECC83/12AX7 SPICE model parameters. Single authoritative definition: the six
// scalars a Koren-equation triode model needs, always produced together via `publishedEcc83()`,
// never edited by hand elsewhere.
struct KorenTriodeParams {
    double mu;  // amplification factor
    double ex;  // Koren exponent
    double kg1; // grid-1 constant
    double kp;  // knee parameter
    double kvb; // knee volt-boost
    double rgi; // grid-current onset resistance (soft grid clamp)
};

struct TriodeStageParams {
    float drive = 0.5f;        // input gain into the waveshaper, calibrated against +16 dB summing headroom
    float outputTrimDb = 0.0f; // post-stage trim
    bool bypass = false;       // triode-bypass switch: audition the raw string (P1 monitoring chain)
};

static_assert(std::is_trivially_copyable_v<TriodeStageParams>,
              "TriodeStageParams must stay trivially copyable for the realtime APVTS snapshot path.");

// Koren-equation static waveshaper of a fixed classic ECC83 input stage. See the file-level
// comment above for the full model, calibration, and caveat. Block domain, NOT templated: Sample
// is always float (the realtime instantiation) -- there is no offline/double variant of this
// module, unlike the sample-domain physics classes.
class TriodeStage {
  public:
    // Message thread; may allocate. Solves the DC operating point and the AC-load-line transfer
    // curve once (see the file-level comment), sizing and filling the internal cubic-interpolation
    // table -- the only allocation this class ever performs. Calls setParams(TriodeStageParams{})
    // then reset(), so a process() call before any explicit setParams() runs the documented
    // defaults already settled, not a degenerate state.
    void prepare(double sampleRate, int maxBlockSize);

    // Realtime-safe. Collapses any pending drive/outputTrim ramp onto its current target
    // immediately; the transfer table itself (built in prepare()) is untouched -- this is a static
    // waveshaper with no per-sample memory to clear beyond the two smoothed gains.
    void reset() noexcept;

    // Realtime-safe. Retargets the drive/outputTrim ramps (the OutputGain/PickupTap per-block
    // Direct-Form convention: the next process() call ramps linearly from the last settled value to
    // the new target across that call's samples, landing exactly on target at the last sample) and
    // updates the bypass switch immediately (no ramp -- see the file-level comment).
    void setParams(const TriodeStageParams& p) noexcept;

    // The pinned published Koren ECC83/12AX7 parameter set (Koren 1996; see the file-level
    // comment for the citation). Pure function of no state; safe to call before prepare().
    static KorenTriodeParams publishedEcc83() noexcept;

    // Static waveshaping; caller wraps this in Oversampler::processWrapped at the chosen factor
    // (P1.8). Realtime-safe; never allocates. Valid for numSamples in [1, maxBlockSize]; clamps
    // internally like every other module in this repo.
    void process(const Sample* in, Sample* out, int numSamples) noexcept;

    // Offline table-loading contract (signature locked now; tables produced by a future C++
    // headless in-repo tool once tools/ activates in P3 -- never Python/LTspice in P0-P2). Message
    // thread. Validates domain sanity only through P2 (see the file-level comment) -- does not
    // store the table or change process()'s behavior. Returns false on any malformed input.
    struct TransferTableView {
        const float* values; // uniformly sampled transfer curve
        int size;            // number of samples, >= 2
        float inputMin;      // domain lower bound (volts at grid)
        float inputMax;      // domain upper bound
    };
    bool loadTransferTable(const TransferTableView& table);

    // REQUIRED deferred hooks (contract methods, P3+ implementation): supply-voltage and
    // heater-dependent THD behavior. Through P2 these store state and have NO audible effect --
    // see the file-level comment's "Deferred hooks" section.
    void setSupplyVoltage(float plateSupplyVolts) noexcept; // P3+: shifts operating point / THD
    void setHeaterVoltage(float heaterVolts) noexcept;      // P3+: emission-dependent THD contract

  private:
    void buildTransferTable();
    float interpolate(double fractionalIndex) const noexcept;
    Sample waveshapeOne(Sample x, float driveLinear, float outputLinear) const noexcept;

    double sampleRate_ = 44100.0; // kept for lifecycle-contract completeness; the static map itself
                                  // has no sample-rate dependence (see the file-level comment)
    int maxBlockSize_ = 0;

    KorenTriodeParams koren_{};

    std::vector<float> table_;       // uniform LUT built once in prepare()/buildTransferTable()
    double invStep_ = 0.0;           // 1 / (table volts-per-sample step); see waveshapeOne()
    double centerIndexDouble_ = 0.0; // exact-integer table index corresponding to vin == 0.0

    bool bypass_ = false;
    float currentDriveLinear_ = 1.0f;  // last settled input-gain multiplier
    float targetDriveLinear_ = 1.0f;   // requested by the most recent setParams()
    float currentOutputLinear_ = 1.0f; // last settled output-trim multiplier
    float targetOutputLinear_ = 1.0f;  // requested by the most recent setParams()

    // Deferred-hook storage only (task brief step 3): never read by prepare()/setParams()/process()
    // through P2. P3+ is where a dynamic stage reads these.
    float supplyVoltsOverride_ = 0.0f;
    bool supplyVoltsSet_ = false;
    float heaterVoltsOverride_ = 0.0f;
    bool heaterVoltsSet_ = false;
};

} // namespace cnpg::dsp
