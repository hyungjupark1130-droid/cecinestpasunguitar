#include "support/ClickMetric.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cnpg::test {

namespace {

double medianOf(std::vector<double> values) {
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    if ((values.size() % 2) == 1)
        return values[mid];
    return 0.5 * (values[mid - 1] + values[mid]);
}

} // namespace

double ClickMeasurement::metric(const ClickMeasurement& reference) const noexcept {
    if (!(reference.medianAbsDiff > 0.0))
        return std::numeric_limits<double>::infinity();
    return peakWindowAbsDiff / reference.medianAbsDiff;
}

ClickMeasurement measureClick(const float* samples, std::size_t count, double sampleRate, std::size_t begin,
                              std::size_t end) {
    ClickMeasurement result;
    if (samples == nullptr || sampleRate <= 0.0)
        return result;

    const std::size_t last = std::min(end, count);
    if (begin >= last)
        return result;

    // Sample hygiene over the span itself, not over the differences: a single NaN would otherwise
    // be reported only through the two differences it poisons.
    for (std::size_t n = begin; n < last; ++n) {
        const float value = samples[n];
        if (!std::isfinite(value))
            ++result.nonFiniteSamples;
        else if (std::fpclassify(value) == FP_SUBNORMAL)
            ++result.subnormalSamples;
        else
            result.peakAbsSample = std::max(result.peakAbsSample, std::fabs(static_cast<double>(value)));
    }

    if (last - begin < 2)
        return result;

    std::vector<double> absDiff;
    absDiff.reserve(last - begin - 1);
    for (std::size_t n = begin + 1; n < last; ++n)
        absDiff.push_back(std::fabs(static_cast<double>(samples[n]) - static_cast<double>(samples[n - 1])));

    result.medianAbsDiff = medianOf(absDiff);

    // Non-overlapping 10 ms windows, as written. See the header for why the window size cannot
    // change the resulting number, and why it is still worth computing this way.
    const auto windowSamples =
        std::max<std::size_t>(1, static_cast<std::size_t>(kClickMetricWindowSeconds * sampleRate + 0.5));
    for (std::size_t start = 0; start < absDiff.size(); start += windowSamples) {
        const std::size_t stop = std::min(start + windowSamples, absDiff.size());
        double windowPeak = 0.0;
        double windowLevel = 0.0;
        for (std::size_t i = start; i < stop; ++i) {
            windowPeak = std::max(windowPeak, absDiff[i]);
            // absDiff[i] was formed from samples[begin + i] and its predecessor.
            windowLevel = std::max(windowLevel, std::fabs(static_cast<double>(samples[begin + 1 + i])));
        }
        if (windowPeak > result.peakWindowAbsDiff) {
            result.peakWindowAbsDiff = windowPeak;
            result.peakWindowStart = start;
        }
        // The step-to-level reading, taken PER WINDOW so a span that decays across itself cannot
        // hide a jump behind its own loud beginning -- see the second companion in the header.
        if (windowLevel > 0.0)
            result.peakStepToLevel = std::max(result.peakStepToLevel, windowPeak / windowLevel);
    }

    return result;
}

ClickMeasurement measureClick(const std::vector<float>& samples, double sampleRate, std::size_t begin,
                              std::size_t end) {
    return measureClick(samples.data(), samples.size(), sampleRate, begin, end);
}

double clickExcessDb(const ClickMeasurement& test, const ClickMeasurement& reference) {
    const double referenceMetric = reference.metric(reference);
    if (!(referenceMetric > 0.0) || !std::isfinite(referenceMetric))
        return std::numeric_limits<double>::infinity();
    const double testMetric = test.metric(reference);
    if (!(testMetric > 0.0))
        return -std::numeric_limits<double>::infinity();
    return 20.0 * std::log10(testMetric / referenceMetric);
}

double clickExcessAgainstResidualDb(const ClickMeasurement& test, const ClickMeasurement& testResidual,
                                    const ClickMeasurement& reference, const ClickMeasurement& referenceResidual) {
    // The one difference from clickExcessDb: each render's peak is measured against the motion of
    // what REMAINS of THAT render after the change, so the denominators no longer cancel -- which
    // is the entire point (see the header) -- while a level change both renders share still does.
    const double referenceMetric = reference.metric(referenceResidual);
    if (!(referenceMetric > 0.0) || !std::isfinite(referenceMetric))
        return std::numeric_limits<double>::infinity();
    const double testMetric = test.metric(testResidual);
    if (!std::isfinite(testMetric))
        return std::numeric_limits<double>::infinity(); // residual is silence: nothing to be loud against
    if (!(testMetric > 0.0))
        return -std::numeric_limits<double>::infinity();
    return 20.0 * std::log10(testMetric / referenceMetric);
}

double clickExcessAgainstLevelDb(const ClickMeasurement& test, const ClickMeasurement& reference) {
    // Each render's peak |dx| against its OWN peak |x| over the same span -- see the header. Both
    // ratios are dimensionless shape numbers, so a level change of any size divides out and only a
    // change in waveform shape moves the result.
    if (!(reference.peakStepToLevel > 0.0) || !std::isfinite(reference.peakStepToLevel))
        return std::numeric_limits<double>::infinity(); // a degenerate reference gates nothing
    if (!(test.peakStepToLevel > 0.0))
        return -std::numeric_limits<double>::infinity();
    return 20.0 * std::log10(test.peakStepToLevel / reference.peakStepToLevel);
}

} // namespace cnpg::test
