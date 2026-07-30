#include "cnpg/dsp/MidiTranslation.h"

#include <catch2/catch_test_macros.hpp>

using cnpg::dsp::RawMidiEvent;
using cnpg::dsp::translateRawMidi;

TEST_CASE("translateRawMidi: copies status/data1/data2/sampleOffset through unchanged", "[contract]") {
    const RawMidiEvent event = translateRawMidi(0x90, 60, 100, 12345);

    REQUIRE(event.status == 0x90);
    REQUIRE(event.data1 == 60);
    REQUIRE(event.data2 == 100);
    REQUIRE(event.sampleOffset == 12345);
}

TEST_CASE("translateRawMidi: derives channel from the status byte's low nibble", "[contract]") {
    REQUIRE(translateRawMidi(0x90, 60, 100, 0).channel == 0);
    REQUIRE(translateRawMidi(0x91, 60, 100, 0).channel == 1);
    REQUIRE(translateRawMidi(0x9F, 60, 100, 0).channel == 15);
    REQUIRE(translateRawMidi(0x80, 60, 0, 0).channel == 0);
    REQUIRE(translateRawMidi(0xB4, 64, 127, 0).channel == 4);
}

TEST_CASE("translateRawMidi: pure and stateless -- identical input always yields identical output", "[contract]") {
    const RawMidiEvent first = translateRawMidi(0x93, 45, 90, 500);
    const RawMidiEvent second = translateRawMidi(0x93, 45, 90, 500);

    REQUIRE(first.status == second.status);
    REQUIRE(first.data1 == second.data1);
    REQUIRE(first.data2 == second.data2);
    REQUIRE(first.sampleOffset == second.sampleOffset);
    REQUIRE(first.channel == second.channel);
}
