#include "cnpg/dsp/ModuleGraph.h"

#include <algorithm>
#include <cstddef>

namespace cnpg::dsp {

namespace {

std::size_t asIndex(int value) noexcept { return static_cast<std::size_t>(value); }

} // namespace

void ModuleGraph::prepare(double sampleRate, int maxBlockSize) {
    sampleRate_ = (sampleRate > 0.0) ? sampleRate : 44100.0;
    maxBlockSize_ = std::max(0, maxBlockSize);
    prepared_ = true;

    for (Node& node : nodes_)
        node.module->prepare(sampleRate_, maxBlockSize_);

    // The buffer plan is sized against maxBlockSize_, so it is stale now: force a re-freeze rather
    // than let process() run over buffers planned for a different block size.
    frozen_ = false;
}

void ModuleGraph::reset() noexcept {
    for (Node& node : nodes_)
        node.module->reset();

    std::fill(bufferPlan_.begin(), bufferPlan_.end(), Sample(0));
}

ModuleGraph::NodeId ModuleGraph::addNode(IBlockModule& module) {
    // A module added twice would get two node ids sharing one piece of internal filter state, which
    // is never what a caller means -- reject it rather than let the second id quietly alias.
    for (const Node& existing : nodes_)
        if (existing.module == &module)
            return kInvalidNode;

    Node node;
    node.module = &module;
    nodes_.push_back(std::move(node));

    // Ordering-independence (see the header): if prepare() has already run, the new node is
    // prepared immediately with the stored settings, so no add/prepare sequence leaves it
    // unprepared.
    if (prepared_)
        module.prepare(sampleRate_, maxBlockSize_);

    frozen_ = false;
    return static_cast<NodeId>(nodes_.size()) - 1;
}

bool ModuleGraph::connect(NodeId from, NodeId to) {
    if (!isValidNode(from) || !isValidNode(to))
        return false;

    if (from == to)
        return false; // a self-edge is the shortest possible cycle

    const std::vector<int>& successors = nodes_[asIndex(from)].successors;
    if (std::find(successors.begin(), successors.end(), to) != successors.end())
        return false; // duplicate edge: carries no meaning in a summing graph, so it is a caller bug

    // Adding from -> to closes a cycle exactly when `to` can already reach `from`.
    if (canReach(to, from))
        return false;

    nodes_[asIndex(from)].successors.push_back(to);
    nodes_[asIndex(to)].predecessors.push_back(from);
    frozen_ = false;
    return true;
}

void ModuleGraph::setInputNode(NodeId node) {
    if (!isValidNode(node))
        return;

    inputNode_ = node;
    frozen_ = false;
}

void ModuleGraph::setOutputNode(NodeId node) {
    if (!isValidNode(node))
        return;

    outputNode_ = node;
    frozen_ = false;
}

void ModuleGraph::freeze() {
    const int numNodes = static_cast<int>(nodes_.size());

    // Kahn's algorithm. connect() has already rejected every edge that would close a cycle, so a
    // complete topological order always exists and the queue can never stall with nodes left over.
    executionOrder_.clear();
    executionOrder_.reserve(asIndex(numNodes));

    std::vector<int> remainingPredecessors(asIndex(numNodes), 0);
    for (int i = 0; i < numNodes; ++i)
        remainingPredecessors[asIndex(i)] = static_cast<int>(nodes_[asIndex(i)].predecessors.size());

    for (int i = 0; i < numNodes; ++i)
        if (remainingPredecessors[asIndex(i)] == 0)
            executionOrder_.push_back(i);

    for (std::size_t cursor = 0; cursor < executionOrder_.size(); ++cursor) {
        const int node = executionOrder_[cursor];
        for (int successor : nodes_[asIndex(node)].successors)
            if (--remainingPredecessors[asIndex(successor)] == 0)
                executionOrder_.push_back(successor);
    }

    // One maxBlockSize run per node for its output, plus one shared fan-in mixing scratch. Nodes
    // are evaluated one at a time in executionOrder_, so a single scratch suffices.
    bufferPlan_.assign(asIndex(numNodes + 1) * asIndex(maxBlockSize_), Sample(0));

    // Longest-path latency from the input node, relaxed along the topological order (so every
    // predecessor is final before a node is visited). For the linear P1/P2 chain this is exactly
    // the sum of the members' latencies, which is what the acceptance criterion asserts; for a
    // branching topology it is the maximum over paths -- see the header on why ModuleGraph reports
    // rather than compensates.
    totalLatencySamples_ = 0;
    if (isValidNode(inputNode_) && isValidNode(outputNode_)) {
        constexpr int kUnreachable = -1;
        std::vector<int> latencyTo(asIndex(numNodes), kUnreachable);
        latencyTo[asIndex(inputNode_)] = nodes_[asIndex(inputNode_)].module->latencySamples();

        for (int node : executionOrder_) {
            if (latencyTo[asIndex(node)] == kUnreachable)
                continue;

            for (int successor : nodes_[asIndex(node)].successors) {
                const int candidate = latencyTo[asIndex(node)] + nodes_[asIndex(successor)].module->latencySamples();
                latencyTo[asIndex(successor)] = std::max(latencyTo[asIndex(successor)], candidate);
            }
        }

        if (latencyTo[asIndex(outputNode_)] != kUnreachable)
            totalLatencySamples_ = latencyTo[asIndex(outputNode_)];
    }

    frozen_ = true;
}

void ModuleGraph::process(const Sample* in, Sample* out, int numSamples) noexcept {
    const int count = std::clamp(numSamples, 0, maxBlockSize_);
    if (count <= 0)
        return;

    // Unfrozen, or nothing to read the result from: audibly inert (see the header).
    if (!frozen_ || !isValidNode(outputNode_)) {
        if (in != out)
            std::copy(in, in + count, out);
        return;
    }

    const int numNodes = static_cast<int>(nodes_.size());
    Sample* mixScratch = bufferPlan_.data() + asIndex(numNodes) * asIndex(maxBlockSize_);

    for (int node : executionOrder_) {
        const Node& current = nodes_[asIndex(node)];
        const bool isInputNode = (node == inputNode_);
        const int numPredecessors = static_cast<int>(current.predecessors.size());

        // Choose the node's input without copying wherever a copy would be pointless: the graph's
        // own input buffer when this is the input node with nothing else feeding it, or a single
        // predecessor's output buffer directly. Only genuine fan-in (or the input node with
        // additional predecessors) needs the mixing scratch.
        const Sample* nodeInput = nullptr;
        if (numPredecessors == 0) {
            if (isInputNode) {
                nodeInput = in;
            } else {
                std::fill(mixScratch, mixScratch + count, Sample(0));
                nodeInput = mixScratch;
            }
        } else if (numPredecessors == 1 && !isInputNode) {
            nodeInput = nodeBuffer(current.predecessors[0]);
        } else {
            if (isInputNode)
                std::copy(in, in + count, mixScratch);
            else
                std::fill(mixScratch, mixScratch + count, Sample(0));

            for (int predecessor : current.predecessors) {
                const Sample* source = nodeBuffer(predecessor);
                for (int n = 0; n < count; ++n)
                    mixScratch[n] += source[n];
            }
            nodeInput = mixScratch;
        }

        current.module->process(nodeInput, nodeBuffer(node), count);
    }

    // Written last, so `out` may alias `in` (nothing reads `in` after this point).
    const Sample* result = nodeBuffer(outputNode_);
    std::copy(result, result + count, out);
}

bool ModuleGraph::isValidNode(NodeId node) const noexcept {
    return node >= 0 && node < static_cast<NodeId>(nodes_.size());
}

bool ModuleGraph::canReach(int from, int to) {
    if (from == to)
        return true;

    // Iterative depth-first search over the successor lists. Message thread only: the two scratch
    // vectors are members purely to keep the repeated allocation out of a connect()-heavy setup
    // loop, never for realtime reasons (process() does not call this).
    reachVisited_.assign(nodes_.size(), 0);
    reachStack_.clear();
    reachStack_.push_back(from);
    reachVisited_[asIndex(from)] = 1;

    while (!reachStack_.empty()) {
        const int node = reachStack_.back();
        reachStack_.pop_back();

        for (int successor : nodes_[asIndex(node)].successors) {
            if (successor == to)
                return true;

            if (reachVisited_[asIndex(successor)] == 0) {
                reachVisited_[asIndex(successor)] = 1;
                reachStack_.push_back(successor);
            }
        }
    }

    return false;
}

Sample* ModuleGraph::nodeBuffer(int nodeIndex) noexcept {
    return bufferPlan_.data() + asIndex(nodeIndex) * asIndex(maxBlockSize_);
}

} // namespace cnpg::dsp
