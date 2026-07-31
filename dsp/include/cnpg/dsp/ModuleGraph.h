#pragma once

#include <vector>

#include "cnpg/dsp/Common.h"

// ModuleGraph -- see docs/plan.md section 2.13 (this file implements that draft's interface
// verbatim). Zero JUCE includes.
//
// -----------------------------------------------------------------------------------------------
// Status: real code, deliberately not yet in the audio path.
// -----------------------------------------------------------------------------------------------
//
// Through P2 the plugin runs the HARD-WIRED chain, not this graph:
//
//   StringNetwork -> PickupTap -> Oversampler(TriodeStage, bypassable) -> CabFilter (bypassable)
//                 -> OutputGain -> SoftClipLimiter
//
// with the safety clip LAST so "rendered peak <= ceilingDb" stays enforceable (section 2.11).
// ModuleGraph is exercised only by tests/dsp/ModuleGraphTests.cpp until P4 turns free routing on
// (body, transformer and power-amp nodes, and the feedback-bus edge). That is a scheduling
// decision, not a stub: everything declared below is implemented and tested now, including the
// chain-equivalence property that P4's cutover depends on -- routing the P2 chain through the
// graph must produce SAMPLE-IDENTICAL output to calling the same modules directly in sequence, or
// the cutover is a silent voicing change rather than a refactor.
//
// -----------------------------------------------------------------------------------------------
// Threading and the freeze discipline.
// -----------------------------------------------------------------------------------------------
//
// Mutation (addNode/connect/setInputNode/setOutputNode) is MESSAGE-THREAD-ONLY and must be
// followed by freeze(); process() is realtime-safe over the frozen topology. Any mutation
// implicitly UNFREEZES the graph (isFrozen() goes false) -- so a caller that mutates and forgets to
// re-freeze gets an audibly inert graph rather than a graph running a half-built topology over a
// stale buffer plan. The caller is responsible for ensuring no process() call is in flight while it
// mutates (in a plugin: the host's own suspension around a topology change); the freeze flag is a
// discipline check, not a synchronization primitive, and this class contains no locks by design.
//
// -----------------------------------------------------------------------------------------------
// Cycles.
// -----------------------------------------------------------------------------------------------
//
// connect() rejects any edge that would close a cycle, and rejects self-edges, invalid NodeIds and
// duplicate edges. This is not a limitation to be worked around later: docs/plan.md section 2.13
// and the P4 section both state that the feedback path is an explicit DELAY-BEARING node feeding
// StringNetwork::injectFeedback(), never a graph cycle. A cycle in a block-domain graph has no
// defined evaluation order at all -- some node would have to consume a block it has not produced
// yet -- so accepting one would mean silently inventing a block of delay somewhere the caller did
// not ask for it. Rejecting it keeps that delay explicit, in a node with a name.
//
// -----------------------------------------------------------------------------------------------
// Evaluation semantics.
// -----------------------------------------------------------------------------------------------
//
// Every node runs once per process() call, in a topological order fixed at freeze(), whether or not
// it lies on the input -> output path -- a stateful module whose state stopped advancing because it
// was temporarily off the path would resume from stale memory and click when it came back.
//
// A node's input is the SUM of its predecessors' outputs, plus the graph's own input buffer if it
// is the designated input node. A node with neither (no predecessors, not the input node) is fed
// silence. The graph's output is the output node's buffer. Fan-in summing is plain addition with
// no per-branch gain and no latency compensation: totalLatencySamples() REPORTS the path latency
// (the maximum over all input -> output paths, which for the linear P1/P2 chain is exactly the sum
// of the members' latencies), but ModuleGraph does not ALIGN branches. Automatic per-branch delay
// compensation is a P4 problem, arriving with the first topology that actually has two paths of
// different latency; inventing it now would be an untested guess at what P4 needs.
//
// process() before freeze(), or with no valid output node, copies input to output unchanged
// ("audibly inert"). Silence would be the other defensible choice; passthrough is chosen because a
// mis-sequenced host callback then drops no audio it was already carrying.

namespace cnpg::dsp {

// Adapter interface a block module implements to join the graph. The concrete dsp/ modules do NOT
// implement it directly -- they keep their own natural signatures (PickupTap consumes
// StringTapBuffers, Oversampler takes a nonlinearity callback, ...) and a thin adapter per module
// bridges the two. Through P2 those adapters live in the tests; P4 is when they become production
// types alongside the body/transformer/power-amp nodes.
class IBlockModule {
  public:
    virtual ~IBlockModule() = default;

    // Message thread; may allocate.
    virtual void prepare(double sampleRate, int maxBlockSize) = 0;

    // Realtime-safe; clears transient state.
    virtual void reset() noexcept = 0;

    // Realtime-safe; never allocates, locks, throws or performs I/O. `in` and `out` never alias
    // when called by ModuleGraph.
    virtual void process(const Sample* in, Sample* out, int numSamples) noexcept = 0;

    // Bulk delay this module introduces, at the graph's sample rate. Zero for everything that is
    // not an oversampler or an explicit delay.
    virtual int latencySamples() const noexcept = 0;
};

class ModuleGraph {
  public:
    using NodeId = int; // stable for the life of the graph; kInvalidNode is invalid

    static constexpr NodeId kInvalidNode = -1;

    // Message thread; allocates. Stores the settings, prepares every node added so far, and
    // unfreezes (the buffer plan is sized here, so it must be rebuilt by a later freeze()).
    // Calling prepare() before or after addNode() is equivalent: a node added AFTER a prepare() is
    // prepared immediately with the stored settings, so ordering never leaves a node unprepared.
    void prepare(double sampleRate, int maxBlockSize);

    // Realtime-safe. Forwards reset() to every node and zeroes the inter-node buffers, so a reset
    // graph is indistinguishable from a freshly frozen one. Does not change the topology or the
    // frozen flag.
    void reset() noexcept;

    // Message thread; allocates. Returns the new node's id, or kInvalidNode if `module` is already
    // in this graph. The graph stores a reference: the caller owns the module and must keep it
    // alive at least as long as the graph. Unfreezes.
    NodeId addNode(IBlockModule& module);

    // Message thread. Adds the edge from -> to. Returns false (and changes nothing) on an invalid
    // NodeId, a self-edge, a duplicate edge, or any edge that would close a cycle. Unfreezes on
    // success.
    bool connect(NodeId from, NodeId to);

    // Message thread. The node that receives the graph's input buffer. An invalid NodeId is
    // ignored (the current setting is kept). Unfreezes.
    void setInputNode(NodeId node);

    // Message thread. The node whose output becomes the graph's output. An invalid NodeId is
    // ignored (the current setting is kept). Unfreezes.
    void setOutputNode(NodeId node);

    // Message thread; allocates. Computes the topological execution order and the per-node buffer
    // plan, and recomputes totalLatencySamples(). Always succeeds -- the acyclicity that makes a
    // topological order exist is enforced by connect(), one edge at a time, so there is no failure
    // mode left for freeze() to report.
    void freeze();

    // Realtime-safe over the frozen topology; never allocates or locks. Mono in/out through P4's
    // routing expansion. numSamples is clamped to the prepared maxBlockSize like every other module
    // in this repo. In-place use (in == out) is fine. See the file-level comment for what an
    // unfrozen graph does.
    void process(const Sample* in, Sample* out, int numSamples) noexcept;

    // Latency along the frozen input -> output path (the maximum over all such paths; exactly the
    // sum of the members' latencies for a linear chain). Zero when unfrozen or when no path exists.
    int totalLatencySamples() const noexcept { return totalLatencySamples_; }

    bool isFrozen() const noexcept { return frozen_; }

    // Number of nodes added so far. Not part of the section-2.13 draft; present because a graph
    // that cannot report its own size is awkward to assert against.
    int numNodes() const noexcept { return static_cast<int>(nodes_.size()); }

  private:
    struct Node {
        IBlockModule* module = nullptr;
        std::vector<int> predecessors; // node indices feeding this node
        std::vector<int> successors;   // node indices this node feeds
    };

    bool isValidNode(NodeId node) const noexcept;
    bool canReach(int from, int to);            // message thread; used for cycle rejection
    Sample* nodeBuffer(int nodeIndex) noexcept; // frozen-plan storage for one node's output

    double sampleRate_ = 44100.0;
    int maxBlockSize_ = 0;
    bool prepared_ = false;

    std::vector<Node> nodes_;
    NodeId inputNode_ = kInvalidNode;
    NodeId outputNode_ = kInvalidNode;

    bool frozen_ = false;
    std::vector<int> executionOrder_; // topological order, filled by freeze()
    std::vector<Sample> bufferPlan_;  // (numNodes + 1) * maxBlockSize, allocated by freeze()
    std::vector<int> reachVisited_;   // canReach() scratch; message thread only
    std::vector<int> reachStack_;     // canReach() scratch; message thread only
    int totalLatencySamples_ = 0;
};

} // namespace cnpg::dsp
