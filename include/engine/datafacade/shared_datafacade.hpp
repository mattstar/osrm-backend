#ifndef SHARED_DATAFACADE_HPP
#define SHARED_DATAFACADE_HPP

// implements all data storage when shared memory _IS_ used

#include "engine/datafacade/datafacade_base.hpp"
#include "storage/shared_datatype.hpp"
#include "storage/shared_memory.hpp"

#include "engine/geospatial_query.hpp"
#include "util/range_table.hpp"
#include "util/static_graph.hpp"
#include "util/static_rtree.hpp"
#include "util/make_unique.hpp"
#include "util/simple_logger.hpp"

#include <cstddef>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/assert.hpp>
#include <boost/thread/tss.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/lock_guard.hpp>

namespace osrm
{
namespace engine
{
namespace datafacade
{

class SharedDataFacade final : public BaseDataFacade
{

  private:
    using super = BaseDataFacade;
    using QueryGraph = util::StaticGraph<EdgeData, true>;
    using GraphNode = typename QueryGraph::NodeArrayEntry;
    using GraphEdge = typename QueryGraph::EdgeArrayEntry;
    using NameIndexBlock = typename util::RangeTable<16, true>::BlockT;
    using InputEdge = typename QueryGraph::InputEdge;
    using RTreeLeaf = typename super::RTreeLeaf;
    using SharedRTree =
        util::StaticRTree<RTreeLeaf, util::ShM<util::FixedPointCoordinate, true>::vector, true>;
    using SharedGeospatialQuery = GeospatialQuery<SharedRTree>;
    using TimeStampedRTreePair = std::pair<unsigned, std::shared_ptr<SharedRTree>>;
    using RTreeNode = typename SharedRTree::TreeNode;

    storage::SharedDataLayout *data_layout;
    char *shared_memory;
    storage::SharedDataTimestamp *data_timestamp_ptr;

    storage::SharedDataType CURRENT_LAYOUT;
    storage::SharedDataType CURRENT_DATA;
    unsigned CURRENT_TIMESTAMP;

    unsigned m_check_sum;
    std::unique_ptr<QueryGraph> m_query_graph;
    std::unique_ptr<storage::SharedMemory> m_layout_memory;
    std::unique_ptr<storage::SharedMemory> m_large_memory;
    std::string m_timestamp;

    std::shared_ptr<util::ShM<util::FixedPointCoordinate, true>::vector> m_coordinate_list;
    util::ShM<NodeID, true>::vector m_via_node_list;
    util::ShM<unsigned, true>::vector m_name_ID_list;
    util::ShM<extractor::TurnInstruction, true>::vector m_turn_instruction_list;
    util::ShM<extractor::TravelMode, true>::vector m_travel_mode_list;
    util::ShM<char, true>::vector m_names_char_list;
    util::ShM<unsigned, true>::vector m_name_begin_indices;
    util::ShM<bool, true>::vector m_edge_is_compressed;
    util::ShM<unsigned, true>::vector m_geometry_indices;
    util::ShM<unsigned, true>::vector m_geometry_list;
    util::ShM<bool, true>::vector m_is_core_node;

    boost::thread_specific_ptr<std::pair<unsigned, std::shared_ptr<SharedRTree>>> m_static_rtree;
    boost::thread_specific_ptr<SharedGeospatialQuery> m_geospatial_query;
    boost::filesystem::path file_index_path;

    std::shared_ptr<util::RangeTable<16, true>> m_name_table;

    void LoadChecksum()
    {
        m_check_sum = *data_layout->GetBlockPtr<unsigned>(shared_memory,
                                                          storage::SharedDataLayout::HSGR_CHECKSUM);
        util::SimpleLogger().Write() << "set checksum: " << m_check_sum;
    }

    void LoadTimestamp()
    {
        auto timestamp_ptr =
            data_layout->GetBlockPtr<char>(shared_memory, storage::SharedDataLayout::TIMESTAMP);
        m_timestamp.resize(data_layout->GetBlockSize(storage::SharedDataLayout::TIMESTAMP));
        std::copy(timestamp_ptr,
                  timestamp_ptr + data_layout->GetBlockSize(storage::SharedDataLayout::TIMESTAMP),
                  m_timestamp.begin());
    }

    void LoadRTree()
    {
        BOOST_ASSERT_MSG(!m_coordinate_list->empty(), "coordinates must be loaded before r-tree");

        auto tree_ptr = data_layout->GetBlockPtr<RTreeNode>(
            shared_memory, storage::SharedDataLayout::R_SEARCH_TREE);
        m_static_rtree.reset(new TimeStampedRTreePair(
            CURRENT_TIMESTAMP,
            util::make_unique<SharedRTree>(
                tree_ptr, data_layout->num_entries[storage::SharedDataLayout::R_SEARCH_TREE],
                file_index_path, m_coordinate_list)));
        m_geospatial_query.reset(
            new SharedGeospatialQuery(*m_static_rtree->second, m_coordinate_list));
    }

    void LoadGraph()
    {
        auto graph_nodes_ptr = data_layout->GetBlockPtr<GraphNode>(
            shared_memory, storage::SharedDataLayout::GRAPH_NODE_LIST);

        auto graph_edges_ptr = data_layout->GetBlockPtr<GraphEdge>(
            shared_memory, storage::SharedDataLayout::GRAPH_EDGE_LIST);

        typename util::ShM<GraphNode, true>::vector node_list(
            graph_nodes_ptr, data_layout->num_entries[storage::SharedDataLayout::GRAPH_NODE_LIST]);
        typename util::ShM<GraphEdge, true>::vector edge_list(
            graph_edges_ptr, data_layout->num_entries[storage::SharedDataLayout::GRAPH_EDGE_LIST]);
        m_query_graph.reset(new QueryGraph(node_list, edge_list));
    }

    void LoadNodeAndEdgeInformation()
    {
        auto coordinate_list_ptr = data_layout->GetBlockPtr<util::FixedPointCoordinate>(
            shared_memory, storage::SharedDataLayout::COORDINATE_LIST);
        m_coordinate_list = util::make_unique<util::ShM<util::FixedPointCoordinate, true>::vector>(
            coordinate_list_ptr,
            data_layout->num_entries[storage::SharedDataLayout::COORDINATE_LIST]);

        auto travel_mode_list_ptr = data_layout->GetBlockPtr<extractor::TravelMode>(
            shared_memory, storage::SharedDataLayout::TRAVEL_MODE);
        typename util::ShM<extractor::TravelMode, true>::vector travel_mode_list(
            travel_mode_list_ptr, data_layout->num_entries[storage::SharedDataLayout::TRAVEL_MODE]);
        m_travel_mode_list = std::move(travel_mode_list);

        auto turn_instruction_list_ptr = data_layout->GetBlockPtr<extractor::TurnInstruction>(
            shared_memory, storage::SharedDataLayout::TURN_INSTRUCTION);
        typename util::ShM<extractor::TurnInstruction, true>::vector turn_instruction_list(
            turn_instruction_list_ptr,
            data_layout->num_entries[storage::SharedDataLayout::TURN_INSTRUCTION]);
        m_turn_instruction_list = std::move(turn_instruction_list);

        auto name_id_list_ptr = data_layout->GetBlockPtr<unsigned>(
            shared_memory, storage::SharedDataLayout::NAME_ID_LIST);
        typename util::ShM<unsigned, true>::vector name_id_list(
            name_id_list_ptr, data_layout->num_entries[storage::SharedDataLayout::NAME_ID_LIST]);
        m_name_ID_list = std::move(name_id_list);
    }

    void LoadViaNodeList()
    {
        auto via_node_list_ptr = data_layout->GetBlockPtr<NodeID>(
            shared_memory, storage::SharedDataLayout::VIA_NODE_LIST);
        typename util::ShM<NodeID, true>::vector via_node_list(
            via_node_list_ptr, data_layout->num_entries[storage::SharedDataLayout::VIA_NODE_LIST]);
        m_via_node_list = std::move(via_node_list);
    }

    void LoadNames()
    {
        auto offsets_ptr = data_layout->GetBlockPtr<unsigned>(
            shared_memory, storage::SharedDataLayout::NAME_OFFSETS);
        auto blocks_ptr = data_layout->GetBlockPtr<NameIndexBlock>(
            shared_memory, storage::SharedDataLayout::NAME_BLOCKS);
        typename util::ShM<unsigned, true>::vector name_offsets(
            offsets_ptr, data_layout->num_entries[storage::SharedDataLayout::NAME_OFFSETS]);
        typename util::ShM<NameIndexBlock, true>::vector name_blocks(
            blocks_ptr, data_layout->num_entries[storage::SharedDataLayout::NAME_BLOCKS]);

        auto names_list_ptr = data_layout->GetBlockPtr<char>(
            shared_memory, storage::SharedDataLayout::NAME_CHAR_LIST);
        typename util::ShM<char, true>::vector names_char_list(
            names_list_ptr, data_layout->num_entries[storage::SharedDataLayout::NAME_CHAR_LIST]);
        m_name_table = util::make_unique<util::RangeTable<16, true>>(
            name_offsets, name_blocks, static_cast<unsigned>(names_char_list.size()));

        m_names_char_list = std::move(names_char_list);
    }

    void LoadCoreInformation()
    {
        if (data_layout->num_entries[storage::SharedDataLayout::CORE_MARKER] <= 0)
        {
            return;
        }

        auto core_marker_ptr = data_layout->GetBlockPtr<unsigned>(
            shared_memory, storage::SharedDataLayout::CORE_MARKER);
        typename util::ShM<bool, true>::vector is_core_node(
            core_marker_ptr, data_layout->num_entries[storage::SharedDataLayout::CORE_MARKER]);
        m_is_core_node = std::move(is_core_node);
    }

    void LoadGeometries()
    {
        auto geometries_compressed_ptr = data_layout->GetBlockPtr<unsigned>(
            shared_memory, storage::SharedDataLayout::GEOMETRIES_INDICATORS);
        typename util::ShM<bool, true>::vector edge_is_compressed(
            geometries_compressed_ptr,
            data_layout->num_entries[storage::SharedDataLayout::GEOMETRIES_INDICATORS]);
        m_edge_is_compressed = std::move(edge_is_compressed);

        auto geometries_index_ptr = data_layout->GetBlockPtr<unsigned>(
            shared_memory, storage::SharedDataLayout::GEOMETRIES_INDEX);
        typename util::ShM<unsigned, true>::vector geometry_begin_indices(
            geometries_index_ptr,
            data_layout->num_entries[storage::SharedDataLayout::GEOMETRIES_INDEX]);
        m_geometry_indices = std::move(geometry_begin_indices);

        auto geometries_list_ptr = data_layout->GetBlockPtr<unsigned>(
            shared_memory, storage::SharedDataLayout::GEOMETRIES_LIST);
        typename util::ShM<unsigned, true>::vector geometry_list(
            geometries_list_ptr,
            data_layout->num_entries[storage::SharedDataLayout::GEOMETRIES_LIST]);
        m_geometry_list = std::move(geometry_list);
    }

  public:
    virtual ~SharedDataFacade() {}

    boost::shared_mutex data_mutex;

    SharedDataFacade()
    {
        if (!storage::SharedMemory::RegionExists(storage::CURRENT_REGIONS))
        {
            throw util::exception(
                "No shared memory blocks found, have you forgotten to run osrm-datastore?");
        }
        data_timestamp_ptr = static_cast<storage::SharedDataTimestamp *>(
            storage::makeSharedMemory(storage::CURRENT_REGIONS,
                                      sizeof(storage::SharedDataTimestamp), false, false)->Ptr());
        CURRENT_LAYOUT = storage::LAYOUT_NONE;
        CURRENT_DATA = storage::DATA_NONE;
        CURRENT_TIMESTAMP = 0;

        // load data
        CheckAndReloadFacade();
    }

    void CheckAndReloadFacade()
    {
        if (CURRENT_LAYOUT != data_timestamp_ptr->layout ||
            CURRENT_DATA != data_timestamp_ptr->data ||
            CURRENT_TIMESTAMP != data_timestamp_ptr->timestamp)
        {
            // Get exclusive lock
            util::SimpleLogger().Write(logDEBUG) << "Updates available, getting exclusive lock";
            const boost::lock_guard<boost::shared_mutex> lock(data_mutex);

            if (CURRENT_LAYOUT != data_timestamp_ptr->layout ||
                CURRENT_DATA != data_timestamp_ptr->data)
            {
                // release the previous shared memory segments
                storage::SharedMemory::Remove(CURRENT_LAYOUT);
                storage::SharedMemory::Remove(CURRENT_DATA);

                CURRENT_LAYOUT = data_timestamp_ptr->layout;
                CURRENT_DATA = data_timestamp_ptr->data;
                CURRENT_TIMESTAMP = 0; // Force trigger a reload

                util::SimpleLogger().Write(logDEBUG)
                    << "Current layout was different to new layout, swapping";
            }
            else
            {
                util::SimpleLogger().Write(logDEBUG)
                    << "Current layout was same to new layout, not swapping";
            }

            if (CURRENT_TIMESTAMP != data_timestamp_ptr->timestamp)
            {
                CURRENT_TIMESTAMP = data_timestamp_ptr->timestamp;

                util::SimpleLogger().Write(logDEBUG) << "Performing data reload";
                m_layout_memory.reset(storage::makeSharedMemory(CURRENT_LAYOUT));

                data_layout = static_cast<storage::SharedDataLayout *>(m_layout_memory->Ptr());

                m_large_memory.reset(storage::makeSharedMemory(CURRENT_DATA));
                shared_memory = (char *)(m_large_memory->Ptr());

                const auto file_index_ptr = data_layout->GetBlockPtr<char>(
                    shared_memory, storage::SharedDataLayout::FILE_INDEX_PATH);
                file_index_path = boost::filesystem::path(file_index_ptr);
                if (!boost::filesystem::exists(file_index_path))
                {
                    util::SimpleLogger().Write(logDEBUG) << "Leaf file name "
                                                         << file_index_path.string();
                    throw util::exception("Could not load leaf index file. "
                                          "Is any data loaded into shared memory?");
                }

                LoadGraph();
                LoadChecksum();
                LoadNodeAndEdgeInformation();
                LoadGeometries();
                LoadTimestamp();
                LoadViaNodeList();
                LoadNames();
                LoadCoreInformation();

                util::SimpleLogger().Write()
                    << "number of geometries: " << m_coordinate_list->size();
                for (unsigned i = 0; i < m_coordinate_list->size(); ++i)
                {
                    if (!GetCoordinateOfNode(i).IsValid())
                    {
                        util::SimpleLogger().Write() << "coordinate " << i << " not valid";
                    }
                }
            }
            util::SimpleLogger().Write(logDEBUG) << "Releasing exclusive lock";
        }
    }

    // search graph access
    unsigned GetNumberOfNodes() const override final { return m_query_graph->GetNumberOfNodes(); }

    unsigned GetNumberOfEdges() const override final { return m_query_graph->GetNumberOfEdges(); }

    unsigned GetOutDegree(const NodeID n) const override final
    {
        return m_query_graph->GetOutDegree(n);
    }

    NodeID GetTarget(const EdgeID e) const override final { return m_query_graph->GetTarget(e); }

    EdgeData &GetEdgeData(const EdgeID e) const override final
    {
        return m_query_graph->GetEdgeData(e);
    }

    EdgeID BeginEdges(const NodeID n) const override final { return m_query_graph->BeginEdges(n); }

    EdgeID EndEdges(const NodeID n) const override final { return m_query_graph->EndEdges(n); }

    EdgeRange GetAdjacentEdgeRange(const NodeID node) const override final
    {
        return m_query_graph->GetAdjacentEdgeRange(node);
    };

    // searches for a specific edge
    EdgeID FindEdge(const NodeID from, const NodeID to) const override final
    {
        return m_query_graph->FindEdge(from, to);
    }

    EdgeID FindEdgeInEitherDirection(const NodeID from, const NodeID to) const override final
    {
        return m_query_graph->FindEdgeInEitherDirection(from, to);
    }

    EdgeID
    FindEdgeIndicateIfReverse(const NodeID from, const NodeID to, bool &result) const override final
    {
        return m_query_graph->FindEdgeIndicateIfReverse(from, to, result);
    }

    // node and edge information access
    util::FixedPointCoordinate GetCoordinateOfNode(const NodeID id) const override final
    {
        return m_coordinate_list->at(id);
    };

    virtual bool EdgeIsCompressed(const unsigned id) const override final
    {
        return m_edge_is_compressed.at(id);
    }

    virtual void GetUncompressedGeometry(const unsigned id,
                                         std::vector<unsigned> &result_nodes) const override final
    {
        const unsigned begin = m_geometry_indices.at(id);
        const unsigned end = m_geometry_indices.at(id + 1);

        result_nodes.clear();
        result_nodes.insert(result_nodes.begin(), m_geometry_list.begin() + begin,
                            m_geometry_list.begin() + end);
    }

    virtual unsigned GetGeometryIndexForEdgeID(const unsigned id) const override final
    {
        return m_via_node_list.at(id);
    }

    extractor::TurnInstruction GetTurnInstructionForEdgeID(const unsigned id) const override final
    {
        return m_turn_instruction_list.at(id);
    }

    extractor::TravelMode GetTravelModeForEdgeID(const unsigned id) const override final
    {
        return m_travel_mode_list.at(id);
    }

    std::vector<PhantomNodeWithDistance>
    NearestPhantomNodesInRange(const util::FixedPointCoordinate input_coordinate,
                               const float max_distance) override final
    {
        if (!m_static_rtree.get() || CURRENT_TIMESTAMP != m_static_rtree->first)
        {
            LoadRTree();
            BOOST_ASSERT(m_geospatial_query.get());
        }

        return m_geospatial_query->NearestPhantomNodesInRange(input_coordinate, max_distance);
    }

    std::vector<PhantomNodeWithDistance>
    NearestPhantomNodesInRange(const util::FixedPointCoordinate input_coordinate,
                               const float max_distance,
                               const int bearing,
                               const int bearing_range) override final
    {
        if (!m_static_rtree.get() || CURRENT_TIMESTAMP != m_static_rtree->first)
        {
            LoadRTree();
            BOOST_ASSERT(m_geospatial_query.get());
        }

        return m_geospatial_query->NearestPhantomNodesInRange(input_coordinate, max_distance,
                                                              bearing, bearing_range);
    }

    std::vector<PhantomNodeWithDistance>
    NearestPhantomNodes(const util::FixedPointCoordinate input_coordinate,
                        const unsigned max_results,
                        const int bearing = 0,
                        const int bearing_range = 180) override final
    {
        if (!m_static_rtree.get() || CURRENT_TIMESTAMP != m_static_rtree->first)
        {
            LoadRTree();
            BOOST_ASSERT(m_geospatial_query.get());
        }

        return m_geospatial_query->NearestPhantomNodes(input_coordinate, max_results, bearing,
                                                       bearing_range);
    }

    std::pair<PhantomNode, PhantomNode> NearestPhantomNodeWithAlternativeFromBigComponent(
        const util::FixedPointCoordinate input_coordinate) override final
    {
        if (!m_static_rtree.get() || CURRENT_TIMESTAMP != m_static_rtree->first)
        {
            LoadRTree();
            BOOST_ASSERT(m_geospatial_query.get());
        }

        return m_geospatial_query->NearestPhantomNodeWithAlternativeFromBigComponent(
            input_coordinate);
    }

    std::pair<PhantomNode, PhantomNode> NearestPhantomNodeWithAlternativeFromBigComponent(
        const util::FixedPointCoordinate input_coordinate, const double max_distance) override final
    {
        if (!m_static_rtree.get() || CURRENT_TIMESTAMP != m_static_rtree->first)
        {
            LoadRTree();
            BOOST_ASSERT(m_geospatial_query.get());
        }

        return m_geospatial_query->NearestPhantomNodeWithAlternativeFromBigComponent(
            input_coordinate, max_distance);
    }

    std::pair<PhantomNode, PhantomNode> NearestPhantomNodeWithAlternativeFromBigComponent(
        const util::FixedPointCoordinate input_coordinate,
        const double max_distance,
        const int bearing,
        const int bearing_range) override final
    {
        if (!m_static_rtree.get() || CURRENT_TIMESTAMP != m_static_rtree->first)
        {
            LoadRTree();
            BOOST_ASSERT(m_geospatial_query.get());
        }

        return m_geospatial_query->NearestPhantomNodeWithAlternativeFromBigComponent(
            input_coordinate, max_distance, bearing, bearing_range);
    }

    std::pair<PhantomNode, PhantomNode> NearestPhantomNodeWithAlternativeFromBigComponent(
        const util::FixedPointCoordinate input_coordinate,
        const int bearing,
        const int bearing_range) override final
    {
        if (!m_static_rtree.get() || CURRENT_TIMESTAMP != m_static_rtree->first)
        {
            LoadRTree();
            BOOST_ASSERT(m_geospatial_query.get());
        }

        return m_geospatial_query->NearestPhantomNodeWithAlternativeFromBigComponent(
            input_coordinate, bearing, bearing_range);
    }


    unsigned GetCheckSum() const override final { return m_check_sum; }

    unsigned GetNameIndexFromEdgeID(const unsigned id) const override final
    {
        return m_name_ID_list.at(id);
    };

    std::string get_name_for_id(const unsigned name_id) const override final
    {
        if (std::numeric_limits<unsigned>::max() == name_id)
        {
            return "";
        }
        auto range = m_name_table->GetRange(name_id);

        std::string result;
        result.reserve(range.size());
        if (range.begin() != range.end())
        {
            result.resize(range.back() - range.front() + 1);
            std::copy(m_names_char_list.begin() + range.front(),
                      m_names_char_list.begin() + range.back() + 1, result.begin());
        }
        return result;
    }

    bool IsCoreNode(const NodeID id) const override final
    {
        if (m_is_core_node.size() > 0)
        {
            return m_is_core_node.at(id);
        }

        return false;
    }

    virtual std::size_t GetCoreSize() const override final { return m_is_core_node.size(); }

    std::string GetTimestamp() const override final { return m_timestamp; }
};
}
}
}

#endif // SHARED_DATAFACADE_HPP
