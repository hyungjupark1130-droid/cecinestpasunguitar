#include "MidiFileReader.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iterator>

namespace cnpg::render {

namespace {

// The default tempo the standard mandates when a file carries no set-tempo meta event: 500000
// microseconds per quarter note, i.e. 120 BPM.
constexpr std::uint32_t kDefaultMicrosPerQuarter = 500000u;

// A byte cursor over the whole file image, with bounds checks that report rather than throw.
// Every read goes through this, so "the file is one byte shorter than its chunk header claims"
// surfaces as a parse error naming the offset, not as undefined behaviour.
class ByteReader {
  public:
    ByteReader(const std::uint8_t* data, std::size_t size) : data_(data), size_(size) {}

    std::size_t position() const noexcept { return pos_; }
    std::size_t remaining() const noexcept { return size_ - pos_; }
    bool atEnd() const noexcept { return pos_ >= size_; }

    bool readByte(std::uint8_t& out) noexcept {
        if (pos_ >= size_)
            return false;
        out = data_[pos_++];
        return true;
    }

    bool peekByte(std::uint8_t& out) const noexcept {
        if (pos_ >= size_)
            return false;
        out = data_[pos_];
        return true;
    }

    bool readBigEndian(int numBytes, std::uint32_t& out) noexcept {
        if (remaining() < static_cast<std::size_t>(numBytes))
            return false;
        std::uint32_t value = 0;
        for (int i = 0; i < numBytes; ++i)
            value = (value << 8) | static_cast<std::uint32_t>(data_[pos_++]);
        out = value;
        return true;
    }

    bool skip(std::size_t numBytes) noexcept {
        if (remaining() < numBytes)
            return false;
        pos_ += numBytes;
        return true;
    }

    // Standard MIDI variable-length quantity: up to four 7-bit groups, high bit set on every byte
    // but the last. A fifth continuation byte is malformed (the standard caps VLQs at 4 bytes) and
    // is reported rather than silently wrapping.
    bool readVariableLength(std::uint32_t& out) noexcept {
        std::uint32_t value = 0;
        for (int i = 0; i < 4; ++i) {
            std::uint8_t byte = 0;
            if (!readByte(byte))
                return false;
            value = (value << 7) | (static_cast<std::uint32_t>(byte) & 0x7Fu);
            if ((static_cast<unsigned>(byte) & 0x80u) == 0) {
                out = value;
                return true;
            }
        }
        return false;
    }

  private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t pos_ = 0;
};

// How many data bytes a channel voice status byte carries. Program change (0xC0) and channel
// pressure (0xD0) carry one; everything else carries two.
int dataByteCount(std::uint8_t status) noexcept {
    const std::uint8_t type = static_cast<std::uint8_t>(static_cast<unsigned>(status) & 0xF0u);
    return (type == 0xC0u || type == 0xD0u) ? 1 : 2;
}

// One entry in the merged, cross-track timeline. Meta set-tempo events ride along with the channel
// voice events (rather than being collected separately) so that a tempo change lands at exactly
// the right point in a timeline that may interleave several tracks -- a tempo map applied out of
// order would shift every later event.
struct TimelineEntry {
    std::uint64_t tick = 0;
    int track = 0;
    std::size_t orderInTrack = 0;
    bool isTempo = false;
    std::uint32_t microsPerQuarter = kDefaultMicrosPerQuarter; // valid when isTempo
    MidiFileEvent event{};                                     // valid when !isTempo (sample filled in later)
};

std::string describe(const std::filesystem::path& path, const std::string& what) { return path.string() + ": " + what; }

bool readWholeFile(const std::filesystem::path& path, std::vector<std::uint8_t>& out, std::string& error) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        error = describe(path, "cannot open file for reading");
        return false;
    }
    out.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    if (out.empty()) {
        error = describe(path, "file is empty");
        return false;
    }
    return true;
}

// Parses one MTrk body (already bounded to the chunk's declared length) into `timeline`.
bool parseTrack(ByteReader& reader, std::size_t trackEnd, int trackIndex, const std::filesystem::path& path,
                std::vector<TimelineEntry>& timeline, std::string& error) {
    std::uint64_t tick = 0;
    std::uint8_t runningStatus = 0;
    std::size_t orderInTrack = 0;

    while (reader.position() < trackEnd) {
        std::uint32_t delta = 0;
        if (!reader.readVariableLength(delta)) {
            error = describe(path, "malformed variable-length delta time in track " + std::to_string(trackIndex));
            return false;
        }
        tick += delta;

        std::uint8_t statusByte = 0;
        if (!reader.peekByte(statusByte)) {
            error = describe(path, "track " + std::to_string(trackIndex) + " ends mid-event");
            return false;
        }

        if (statusByte == 0xFFu) { // meta event
            reader.skip(1);
            std::uint8_t metaType = 0;
            std::uint32_t length = 0;
            if (!reader.readByte(metaType) || !reader.readVariableLength(length)) {
                error = describe(path, "malformed meta event in track " + std::to_string(trackIndex));
                return false;
            }
            if (metaType == 0x51u) { // set tempo
                if (length != 3) {
                    error = describe(path, "set-tempo meta event with length " + std::to_string(length) +
                                               " (must be 3) in track " + std::to_string(trackIndex));
                    return false;
                }
                std::uint32_t micros = 0;
                if (!reader.readBigEndian(3, micros)) {
                    error = describe(path, "truncated set-tempo meta event in track " + std::to_string(trackIndex));
                    return false;
                }
                TimelineEntry entry;
                entry.tick = tick;
                entry.track = trackIndex;
                entry.orderInTrack = orderInTrack++;
                entry.isTempo = true;
                // A zero tempo would divide the whole timeline by zero; treat it the way the
                // standard's own "must be > 0" implies and refuse rather than produce infinities.
                if (micros == 0) {
                    error = describe(path, "set-tempo meta event declares 0 microseconds per quarter note");
                    return false;
                }
                entry.microsPerQuarter = micros;
                timeline.push_back(entry);
            } else if (!reader.skip(length)) {
                error = describe(path, "truncated meta event payload in track " + std::to_string(trackIndex));
                return false;
            }
            // Meta events cancel running status (the standard's own recommendation).
            runningStatus = 0;
            continue;
        }

        if (statusByte == 0xF0u || statusByte == 0xF7u) { // SysEx (both forms carry a length)
            reader.skip(1);
            std::uint32_t length = 0;
            if (!reader.readVariableLength(length) || !reader.skip(length)) {
                error = describe(path, "truncated SysEx event in track " + std::to_string(trackIndex));
                return false;
            }
            runningStatus = 0;
            continue;
        }

        std::uint8_t status = 0;
        if (statusByte >= 0x80u) {
            status = statusByte;
            reader.skip(1);
            runningStatus = status;
        } else {
            // Running status: the data byte belongs to the previous channel voice status.
            if (runningStatus == 0) {
                error = describe(path, "running-status data byte with no preceding status byte in track " +
                                           std::to_string(trackIndex));
                return false;
            }
            status = runningStatus;
        }

        const int numData = dataByteCount(status);
        std::uint8_t data1 = 0;
        std::uint8_t data2 = 0;
        if (!reader.readByte(data1) || (numData == 2 && !reader.readByte(data2))) {
            error = describe(path, "truncated channel voice message in track " + std::to_string(trackIndex));
            return false;
        }

        TimelineEntry entry;
        entry.tick = tick;
        entry.track = trackIndex;
        entry.orderInTrack = orderInTrack++;
        entry.isTempo = false;
        entry.event.status = status;
        entry.event.data1 = data1;
        entry.event.data2 = data2;
        timeline.push_back(entry);
    }

    return true;
}

} // namespace

bool readMidiFile(const std::filesystem::path& path, double sampleRate, MidiFileContents& out, std::string& error) {
    out = MidiFileContents{};
    error.clear();

    if (!(sampleRate > 0.0)) {
        error = describe(path, "sample rate must be positive");
        return false;
    }

    std::vector<std::uint8_t> bytes;
    if (!readWholeFile(path, bytes, error))
        return false;

    ByteReader reader(bytes.data(), bytes.size());

    std::uint32_t headerTag = 0;
    std::uint32_t headerLength = 0;
    if (!reader.readBigEndian(4, headerTag) || headerTag != 0x4D546864u) { // "MThd"
        error = describe(path, "not a Standard MIDI File (missing MThd header chunk)");
        return false;
    }
    if (!reader.readBigEndian(4, headerLength) || headerLength < 6) {
        error = describe(path, "MThd chunk is shorter than the required 6 bytes");
        return false;
    }

    std::uint32_t format = 0;
    std::uint32_t declaredTracks = 0;
    std::uint32_t division = 0;
    if (!reader.readBigEndian(2, format) || !reader.readBigEndian(2, declaredTracks) ||
        !reader.readBigEndian(2, division)) {
        error = describe(path, "truncated MThd chunk");
        return false;
    }
    // MThd may legally be longer than 6 bytes; the surplus is reserved and skipped.
    if (headerLength > 6 && !reader.skip(headerLength - 6)) {
        error = describe(path, "MThd chunk declares more bytes than the file contains");
        return false;
    }

    if (format > 1) {
        error = describe(path, "SMF format " + std::to_string(format) +
                                   " is not supported (only 0 and 1 have a single merged timeline)");
        return false;
    }
    if ((division & 0x8000u) != 0) {
        error = describe(path, "SMPTE time division is not supported; the corpus is authored in metrical time");
        return false;
    }
    if (division == 0) {
        error = describe(path, "MThd declares 0 ticks per quarter note");
        return false;
    }

    out.format = static_cast<int>(format);
    out.ticksPerQuarter = static_cast<int>(division);

    std::vector<TimelineEntry> timeline;
    int tracksParsed = 0;

    while (!reader.atEnd()) {
        std::uint32_t chunkTag = 0;
        std::uint32_t chunkLength = 0;
        if (!reader.readBigEndian(4, chunkTag) || !reader.readBigEndian(4, chunkLength)) {
            error = describe(path, "truncated chunk header after track " + std::to_string(tracksParsed));
            return false;
        }
        if (chunkLength > reader.remaining()) {
            error = describe(path, "chunk declares " + std::to_string(chunkLength) + " bytes but only " +
                                       std::to_string(reader.remaining()) + " remain");
            return false;
        }

        if (chunkTag != 0x4D54726Bu) { // not "MTrk": the standard requires readers to skip it
            reader.skip(chunkLength);
            continue;
        }

        const std::size_t trackEnd = reader.position() + chunkLength;
        if (!parseTrack(reader, trackEnd, tracksParsed, path, timeline, error))
            return false;

        // Trust the chunk's declared length over wherever the event stream happened to stop: a
        // track whose last event ends early (or whose End of Track meta is followed by padding)
        // still hands the next chunk header back at the right offset.
        if (reader.position() < trackEnd)
            reader.skip(trackEnd - reader.position());

        ++tracksParsed;
    }

    if (tracksParsed != static_cast<int>(declaredTracks)) {
        error = describe(path, "MThd declares " + std::to_string(declaredTracks) + " track(s) but the file contains " +
                                   std::to_string(tracksParsed));
        return false;
    }
    out.numTracks = tracksParsed;

    // Stable sort by tick only: entries already carry their (track, orderInTrack) position, and a
    // stable sort therefore preserves track-then-file order among simultaneous events without
    // needing those fields in the comparator.
    std::stable_sort(timeline.begin(), timeline.end(),
                     [](const TimelineEntry& a, const TimelineEntry& b) { return a.tick < b.tick; });

    // Walk the merged timeline once, accumulating seconds through the tempo map, and stamp every
    // channel voice event with its sample index.
    double seconds = 0.0;
    std::uint64_t lastTick = 0;
    std::uint32_t microsPerQuarter = kDefaultMicrosPerQuarter;
    const double ticksPerQuarter = static_cast<double>(out.ticksPerQuarter);

    out.events.reserve(timeline.size());
    for (const TimelineEntry& entry : timeline) {
        const double deltaTicks = static_cast<double>(entry.tick - lastTick);
        seconds += deltaTicks * (static_cast<double>(microsPerQuarter) * 1.0e-6) / ticksPerQuarter;
        lastTick = entry.tick;

        if (entry.isTempo) {
            microsPerQuarter = entry.microsPerQuarter;
            continue;
        }

        MidiFileEvent event = entry.event;
        event.sample = static_cast<long long>(std::llround(seconds * sampleRate));
        out.events.push_back(event);
        out.lastEventSample = event.sample;
        out.lastEventSeconds = seconds;
    }

    return true;
}

} // namespace cnpg::render
