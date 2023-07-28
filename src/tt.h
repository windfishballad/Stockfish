/*
  Stockfish, a UCI chess playing engine derived from Glaurung 2.1
  Copyright (C) 2004-2023 The Stockfish developers (see AUTHORS file)

  Stockfish is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Stockfish is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TT_H_INCLUDED
#define TT_H_INCLUDED

#include "misc.h"
#include "types.h"

namespace Stockfish {

/// TTEntry struct is a wrapper to 85 bits of a transposition table cluster of 256 = 3 * 85 bits + 1 unused.
///
/// key        16 bit
/// value      16 bit
/// eval       16 bit
/// depth       8 bit
/// generation  4 bit
/// pv node     1 bit
/// bound type  2 bit
/// move       11 bits
/// move2      11 bits




extern uint16_t moveMapping[1 << 16];
extern Move inverseMoveMapping[1 << 12];

constexpr uint32_t KEY_INDEX_0 = 0;
constexpr uint32_t VALUE_INDEX_0 = 0;
constexpr uint32_t EVAL_INDEX_0 = 1;
constexpr uint32_t PV_INDEX_0 = 1;
constexpr uint32_t GEN_INDEX_0 = 1;
constexpr uint32_t MOVE_INDEX_0 = 1;
constexpr uint32_t DEPTH_INDEX_0 = 2;
constexpr uint32_t BOUND_INDEX_0 = 2;
constexpr uint32_t MOVE2_INDEX_0 = 2;


constexpr uint32_t KEY_SHIFT_0 = 16;
constexpr uint32_t VALUE_SHIFT_0 = 0;
constexpr uint32_t EVAL_SHIFT_0 = 16;
constexpr uint32_t PV_SHIFT_0 = 15;
constexpr uint32_t GEN_SHIFT_0 = 11;
constexpr uint32_t MOVE_SHIFT_0 = 0;
constexpr uint32_t DEPTH_SHIFT_0 = 24;
constexpr uint32_t BOUND_SHIFT_0 = 22;
constexpr uint32_t MOVE2_SHIFT_0 = 11;


constexpr uint32_t KEY_MASK_0 = 0xFFFF << KEY_SHIFT_0;
constexpr uint32_t VALUE_MASK_0 = 0xFFFF << VALUE_SHIFT_0;
constexpr uint32_t EVAL_MASK_0 = 0xFFFF << EVAL_SHIFT_0;
constexpr uint32_t PV_MASK_0 = 1 << PV_SHIFT_0;
constexpr uint32_t GEN_MASK_0 = 0xF << GEN_SHIFT_0;
constexpr uint32_t MOVE_MASK_0 = 0x7FF << MOVE_SHIFT_0;
constexpr uint32_t DEPTH_MASK_0 = 0xFF << DEPTH_SHIFT_0;
constexpr uint32_t BOUND_MASK_0 = 3 << BOUND_SHIFT_0;
constexpr uint32_t MOVE2_MASK_0 = 0x7FF << MOVE2_SHIFT_0;

constexpr uint32_t KEY_INDEX_1 = 3;
constexpr uint32_t VALUE_INDEX_1 = 3;
constexpr uint32_t EVAL_INDEX_1 = 4;
constexpr uint32_t PV_INDEX_1 = 4;
constexpr uint32_t GEN_INDEX_1 = 4;
constexpr uint32_t MOVE_INDEX_1 = 4;
constexpr uint32_t DEPTH_INDEX_1 = 5;
constexpr uint32_t BOUND_INDEX_1 = 5;
constexpr uint32_t MOVE2_INDEX_1 = 2;


constexpr uint32_t KEY_SHIFT_1 = 16;
constexpr uint32_t VALUE_SHIFT_1 = 0;
constexpr uint32_t EVAL_SHIFT_1 = 16;
constexpr uint32_t PV_SHIFT_1 = 15;
constexpr uint32_t GEN_SHIFT_1 = 11;
constexpr uint32_t MOVE_SHIFT_1 = 0;
constexpr uint32_t DEPTH_SHIFT_1 = 24;
constexpr uint32_t BOUND_SHIFT_1 = 22;
constexpr uint32_t MOVE2_SHIFT_1 = 0;


constexpr uint32_t KEY_MASK_1 = 0xFFFF << KEY_SHIFT_1;
constexpr uint32_t VALUE_MASK_1 = 0xFFFF << VALUE_SHIFT_1;
constexpr uint32_t EVAL_MASK_1 = 0xFFFF << EVAL_SHIFT_1;
constexpr uint32_t PV_MASK_1 = 1 << PV_SHIFT_1;
constexpr uint32_t GEN_MASK_1 = 0xF << GEN_SHIFT_1;
constexpr uint32_t MOVE_MASK_1 = 0x7FF << MOVE_SHIFT_1;
constexpr uint32_t DEPTH_MASK_1 = 0xFF << DEPTH_SHIFT_1;
constexpr uint32_t BOUND_MASK_1 = 3 << BOUND_SHIFT_1;
constexpr uint32_t MOVE2_MASK_1 = 0x7FF << MOVE2_SHIFT_1;

constexpr uint32_t KEY_INDEX_2 = 6;
constexpr uint32_t VALUE_INDEX_2 = 6;
constexpr uint32_t EVAL_INDEX_2 = 7;
constexpr uint32_t PV_INDEX_2 = 7;
constexpr uint32_t GEN_INDEX_2 = 7;
constexpr uint32_t MOVE_INDEX_2 = 5;
constexpr uint32_t DEPTH_INDEX_2 = 7;
constexpr uint32_t BOUND_INDEX_2 = 7;
constexpr uint32_t MOVE2_INDEX_2 = 5;


constexpr uint32_t KEY_SHIFT_2 = 16;
constexpr uint32_t VALUE_SHIFT_2 = 0;
constexpr uint32_t EVAL_SHIFT_2 = 15;
constexpr uint32_t PV_SHIFT_2 = 14;
constexpr uint32_t GEN_SHIFT_2 = 10;
constexpr uint32_t MOVE_SHIFT_2 = 0;
constexpr uint32_t DEPTH_SHIFT_2 = 0;
constexpr uint32_t BOUND_SHIFT_2 = 8;
constexpr uint32_t MOVE2_SHIFT_2 = 11;


constexpr uint32_t KEY_MASK_2 = 0xFFFF << KEY_SHIFT_2;
constexpr uint32_t VALUE_MASK_2 = 0xFFFF << VALUE_SHIFT_2;
constexpr uint32_t EVAL_MASK_2 = 0xFFFF << EVAL_SHIFT_2;
constexpr uint32_t PV_MASK_2 = 1 << PV_SHIFT_2;
constexpr uint32_t GEN_MASK_2 = 0xF << GEN_SHIFT_2;
constexpr uint32_t MOVE_MASK_2 = 0x7FF << MOVE_SHIFT_2;
constexpr uint32_t DEPTH_MASK_2 = 0xFF << DEPTH_SHIFT_2;
constexpr uint32_t BOUND_MASK_2 = 3 << BOUND_SHIFT_2;
constexpr uint32_t MOVE2_MASK_2 = 0x7FF << MOVE2_SHIFT_2;


namespace Transposition {

void init();

}

struct Cluster {

	int getDepth(int rank) const;
	int getGeneration(int rank) const;
	uint16_t getKey(int rank) const;
	void setGeneration(int rank);

  uint32_t data[8];
};


struct TTEntry {

	template<bool first>
	Move  move(Position& pos)  const;
	Value value() const { return (Value) (m_value - VALUE_INFINITE); }
	Value eval()  const { return (Value) (m_eval - VALUE_INFINITE); }
	Depth depth() const { return (Depth) (m_depth + DEPTH_OFFSET); }
	bool is_pv()  const { return (bool) m_is_pv; }
	Bound bound() const { return (Bound) m_bound; }
	void save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev);



private:
	friend class TranspositionTable;

	uint32_t  key;
	uint32_t  m_move;
	uint32_t m_move2;
	uint32_t m_depth;
	uint32_t m_is_pv;
	uint32_t m_bound;
	uint32_t m_value;
	uint32_t m_eval;
	uint32_t generation;

	Cluster* data;
	int rank;
	void copyData();
	void copyDataLight();

};



/// A TranspositionTable is an array of Cluster, of size clusterCount. Each
/// cluster consists of ClusterSize number of TTEntry. Each non-empty TTEntry
/// contains information on exactly one position. The size of a Cluster should
/// divide the size of a cache line for best performance, as the cacheline is
/// prefetched when possible.

class TranspositionTable {

  static constexpr int ClusterSize = 3;

  static_assert(sizeof(Cluster) == 32, "Unexpected Cluster size");

  // Constants used to refresh the hash table periodically
  static constexpr int      GENERATION_CYCLE = 16;     // cycle length

public:
 ~TranspositionTable() { aligned_large_pages_free(table); }
  void new_search() { generation = (generation + 1) % GENERATION_CYCLE; } // Lower bits are used for other things
  void probe(TTEntry& tte, const Key key, bool& found) const;
  int hashfull() const;
  void resize(size_t mbSize);
  void clear();

  Cluster* first_entry(const Key key) const {
    return &table[mul_hi64(key, clusterCount)];
  }

private:
  friend struct TTEntry;
  friend struct Cluster;

  size_t clusterCount;
  Cluster* table;
  uint8_t generation; // Size must be not bigger than TTEntry::genBound8
};

extern TranspositionTable TT;

} // namespace Stockfish

#endif // #ifndef TT_H_INCLUDED
