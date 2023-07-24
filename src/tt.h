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

extern uint16_t moveMapping[1 << 16];
extern Move inverseMoveMapping[1 << 12];

namespace Transposition {
	void init();
} // namespace Transposition

extern const int MOVE_MASK;
extern const int BOUND_MASK;
extern const int PV_MASK;
extern const int GEN_MASK;


extern const int GEN_MASK_COMPLEMENTARY;
extern const int MOVE_MASK_COMPLEMENTARY;

extern const int EVAL_MASK;
extern const int VALUE_MASK;
extern const int DEPTH_MASK;



/// TTEntry struct is the 8 bytes transposition table entry, defined as below:
///
/// key        16 bit
/// depth       6 bits
/// generation  2 bits
/// pv node     1 bit
/// bound type  2 bit
/// move       11 bit
/// value      13 bit
/// eval value 13 bit

struct TTEntry {

  Move  move()  const { return (Move ) inverseMoveMapping[(moveBoundPVGen & MOVE_MASK) >> 5]; }
  Value value() const { return (Value) ((evalValueDepth & VALUE_MASK) >> 4); }
  Value eval()  const { return (Value) ((evalValueDepth & EVAL_MASK) >> 15); }
  Depth depth() const { return (Depth) ((evalValueDepth & DEPTH_MASK) + DEPTH_OFFSET); }
  bool is_pv()  const { return (bool)(moveBoundPVGen & PV_MASK); }
  Bound bound() const { return (Bound)(moveBoundPVGen & BOUND_MASK); }
  void save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev);

private:
  friend class TranspositionTable;

  uint16_t key16;
  uint16_t moveBoundPVGen;
  uint32_t evalValueDepth;

};


/// A TranspositionTable is an array of Cluster, of size clusterCount. Each
/// cluster consists of ClusterSize number of TTEntry. Each non-empty TTEntry
/// contains information on exactly one position. The size of a Cluster should
/// divide the size of a cache line for best performance, as the cacheline is
/// prefetched when possible.

class TranspositionTable {

  static constexpr int ClusterSize = 4;

  struct Cluster {
    TTEntry entry[ClusterSize];
  };

  static_assert(sizeof(Cluster) == 32, "Unexpected Cluster size");

  static constexpr int      GENERATION_CYCLE = 4;     // cycle length

public:
 ~TranspositionTable() { aligned_large_pages_free(table); }
  void new_search() { generation = (generation + 1) & GEN_MASK; }
  TTEntry* probe(const Key key, bool& found) const;
  int hashfull() const;
  void resize(size_t mbSize);
  void clear();

  TTEntry* first_entry(const Key key) const {
    return &table[mul_hi64(key, clusterCount)].entry[0];
  }

private:
  friend struct TTEntry;

  size_t clusterCount;
  Cluster* table;
  uint8_t generation; // Size must be not bigger than TTEntry::genBound8
};

extern TranspositionTable TT;

} // namespace Stockfish

#endif // #ifndef TT_H_INCLUDED
