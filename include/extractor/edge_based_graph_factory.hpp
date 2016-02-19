//  This class constructs the edge-expanded routing graph

#ifndef EDGE_BASED_GRAPH_FACTORY_HPP_
#define EDGE_BASED_GRAPH_FACTORY_HPP_

#include "extractor/edge_based_edge.hpp"
#include "extractor/speed_profile.hpp"
#include "extractor/restriction_map.hpp"
#include "extractor/compressed_edge_container.hpp"
#include "extractor/edge_based_node.hpp"
#include "extractor/original_edge_data.hpp"
#include "extractor/query_node.hpp"

#include "engine/guidance/turn_instruction.hpp"

#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"
#include "util/deallocating_vector.hpp"

#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <iosfwd>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

#include <boost/filesystem/fstream.hpp>

struct lua_State;

namespace osrm
{
namespace extractor
{

class EdgeBasedGraphFactory
{
  public:
    EdgeBasedGraphFactory(const EdgeBasedGraphFactory &) = delete;
    EdgeBasedGraphFactory &operator=(const EdgeBasedGraphFactory &) = delete;

    explicit EdgeBasedGraphFactory(std::shared_ptr<util::NodeBasedDynamicGraph> node_based_graph,
                                   const CompressedEdgeContainer &compressed_edge_container,
                                   const std::unordered_set<NodeID> &barrier_nodes,
                                   const std::unordered_set<NodeID> &traffic_lights,
                                   std::shared_ptr<const RestrictionMap> restriction_map,
                                   const std::vector<QueryNode> &node_info_list,
                                   SpeedProfileProperties speed_profile);

#ifdef DEBUG_GEOMETRY
    void Run(const std::string &original_edge_data_filename,
             lua_State *lua_state,
             const std::string &edge_segment_lookup_filename,
             const std::string &edge_penalty_filename,
             const bool generate_edge_lookup,
             const std::string &debug_turns_path);
#else
    void Run(const std::string &original_edge_data_filename,
             lua_State *lua_state,
             const std::string &edge_segment_lookup_filename,
             const std::string &edge_penalty_filename,
             const bool generate_edge_lookup);
#endif

    // The following get access functions destroy the content in the factory
    void GetEdgeBasedEdges(util::DeallocatingVector<EdgeBasedEdge> &edges);
    void GetEdgeBasedNodes(std::vector<EdgeBasedNode> &nodes);
    void GetStartPointMarkers(std::vector<bool> &node_is_startpoint);
    void GetEdgeBasedNodeWeights(std::vector<EdgeWeight> &output_node_weights);

    unsigned GetHighestEdgeID();

    // Basic analysis of a turn (u --(e1)-- v --(e2)-- w)
    // with known angle.
    // Handles special cases like u-turns and roundabouts
    // For basic turns, the turn based on the angle-classification is returned
    engine::guidance::TurnInstruction AnalyzeTurn(const NodeID u,
                                                  const EdgeID e1,
                                                  const NodeID v,
                                                  const EdgeID e2,
                                                  const NodeID w,
                                                  const double angle) const;

    std::int32_t GetTurnPenalty(double angle, lua_State *lua_state) const;

  private:
    using EdgeData = util::NodeBasedDynamicGraph::EdgeData;

    //! maps index from m_edge_based_node_list to ture/false if the node is an entry point to the
    //! graph
    std::vector<bool> m_edge_based_node_is_startpoint;

    //! node weights that indicate the length of the segment (node based) represented by the
    //! edge-based node
    std::vector<EdgeWeight> m_edge_based_node_weights;

    //! list of edge based nodes (compressed segments)
    std::vector<EdgeBasedNode> m_edge_based_node_list;
    util::DeallocatingVector<EdgeBasedEdge> m_edge_based_edge_list;
    unsigned m_max_edge_id;

    const std::vector<QueryNode> &m_node_info_list;
    std::shared_ptr<util::NodeBasedDynamicGraph> m_node_based_graph;
    std::shared_ptr<RestrictionMap const> m_restriction_map;

    const std::unordered_set<NodeID> &m_barrier_nodes;
    const std::unordered_set<NodeID> &m_traffic_lights;
    const CompressedEdgeContainer &m_compressed_edge_container;

    SpeedProfileProperties speed_profile;

    void CompressGeometry();
    unsigned RenumberEdges();
    void GenerateEdgeExpandedNodes();
#ifdef DEBUG_GEOMETRY
    void GenerateEdgeExpandedEdges(const std::string &original_edge_data_filename,
                                   lua_State *lua_state,
                                   const std::string &edge_segment_lookup_filename,
                                   const std::string &edge_fixed_penalties_filename,
                                   const bool generate_edge_lookup,
                                   const std::string &debug_turns_path);
#else
    void GenerateEdgeExpandedEdges(const std::string &original_edge_data_filename,
                                   lua_State *lua_state,
                                   const std::string &edge_segment_lookup_filename,
                                   const std::string &edge_fixed_penalties_filename,
                                   const bool generate_edge_lookup);
#endif

    void InsertEdgeBasedNode(const NodeID u, const NodeID v);

    void FlushVectorToStream(std::ofstream &edge_data_file,
                             std::vector<OriginalEdgeData> &original_edge_data_vector) const;

    struct TurnCandidate
    {
        EdgeID eid; // the id of the arc
        bool valid; // a turn may be relevant to good instructions, even if we cannot take the road
        double angle;                                  // the approximated angle of the turn
        engine::guidance::TurnInstruction instruction; // a proposed instruction
        double confidence;                             // how close to the border is the turn?

        std::string toString() const
        {
            std::string result = "[turn] ";
            result += std::to_string(eid);
            result += " valid: ";
            result += std::to_string(valid);
            result += " angle: ";
            result += std::to_string(angle);
            result += " instruction: ";
            result += std::to_string(static_cast<std::int32_t>(instruction.type)) + " " +
                      std::to_string(static_cast<std::int32_t>(instruction.direction_modifier));
            result += " confidence: ";
            result += std::to_string(confidence);
            return result;
        }
    };

    // Use In Order to generate base turns

    // cannot be const due to the counters...
    std::vector<TurnCandidate> getTurnCandidates(const NodeID from, const EdgeID via_edge);
    std::vector<TurnCandidate> optimizeCandidates(const EdgeID via_edge,
                                                  std::vector<TurnCandidate> turn_candidates) const;

    std::vector<TurnCandidate> optimizeRamps(const EdgeID via_edge,
                                             std::vector<TurnCandidate> turn_candidates) const;

    engine::guidance::TurnType
    checkForkAndEnd(const EdgeID via_edge, const std::vector<TurnCandidate> &turn_candidates) const;
    std::vector<TurnCandidate> handleForkAndEnd(const engine::guidance::TurnType type,
                                                std::vector<TurnCandidate> turn_candidates) const;

    std::vector<TurnCandidate> suppressTurns(const EdgeID via_edge,
                                             std::vector<TurnCandidate> turn_candidates) const;

    bool isObviousChoice(const EdgeID coming_from_eid,
                         const std::size_t turn_index,
                         const std::vector<TurnCandidate> &turn_candidates) const;

    std::size_t restricted_turns_counter;
    std::size_t skipped_uturns_counter;
    std::size_t skipped_barrier_turns_counter;
};
} // namespace extractor
} // namespace osrm

#endif /* EDGE_BASED_GRAPH_FACTORY_HPP_ */
