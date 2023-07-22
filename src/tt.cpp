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

#include <cstring>   // For std::memset
#include <iostream>
#include <thread>

#include "bitboard.h"
#include "misc.h"
#include "thread.h"
#include "tt.h"
#include "uci.h"

namespace Stockfish {

TranspositionTable TT; // Our global transposition table

/// TTEntry::save() populates the TTEntry with a new node's data, possibly
/// overwriting an old position. Update is not atomic and can be racy.

void TTWrapper::save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev, Value improvement) {

	//New position

	bool newPosition= (uint16_t) k != tte->key16;

	assert(0<=improvement && improvement <= 31);


	if(newPosition) //save improvement
	{
			extraInfo->info = (extraInfo-> info & ~ improvementMask[rank]) | (improvement << 5*rank);
	}
	else //save improvement if improves on previous result
	{
		if(improvement > 0 && improvement > (extraInfo->info & improvementMask[rank]) >> 5) //first redundant check saves compute since most tt saves pass 0
			extraInfo->info = (extraInfo-> info & ~ improvementMask[rank]) | (improvement << 5*rank);
	}

	tte->save(k, v, pv, b, d, m, ev, newPosition);
}

void TTEntry::save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev, bool newPosition) {

  // Preserve any existing move for the same position
  if (m || newPosition)
      move16 = (uint16_t)m;

  // Overwrite less valuable entries (cheapest checks first)
  if ( b == BOUND_EXACT
      || newPosition
      || d - DEPTH_OFFSET + 2 * pv > depth8 - 4)
  {
      assert(d > DEPTH_OFFSET);
      assert(d < 256 + DEPTH_OFFSET);

      key16     = (uint16_t)k;
      depth8    = (uint8_t)(d - DEPTH_OFFSET);
      genBound8 = (uint8_t)(TT.generation8 | uint8_t(pv) << 2 | b);
      value16   = (int16_t)v;
      eval16    = (int16_t)ev;

  }

}


/// TranspositionTable::resize() sets the size of the transposition table,
/// measured in megabytes. Transposition table consists of a power of 2 number
/// of clusters and each cluster consists of ClusterSize number of TTEntry.

void TranspositionTable::resize(size_t mbSize) {

  Threads.main()->wait_for_search_finished();

  aligned_large_pages_free(table);

  clusterCount = mbSize * 1024 * 1024 / sizeof(Cluster);

  table = static_cast<Cluster*>(aligned_large_pages_alloc(clusterCount * sizeof(Cluster)));
  if (!table)
  {
      std::cerr << "Failed to allocate " << mbSize
                << "MB for transposition table." << std::endl;
      exit(EXIT_FAILURE);
  }

  clear();
}


/// TranspositionTable::clear() initializes the entire transposition table to zero,
//  in a multi-threaded way.

void TranspositionTable::clear() {

  std::vector<std::thread> threads;

  for (size_t idx = 0; idx < size_t(Options["Threads"]); ++idx)
  {
      threads.emplace_back([this, idx]() {

          // Thread binding gives faster search on systems with a first-touch policy
          if (Options["Threads"] > 8)
              WinProcGroup::bindThisThread(idx);

          // Each thread will zero its part of the hash table
          const size_t stride = size_t(clusterCount / Options["Threads"]),
                       start  = size_t(stride * idx),
                       len    = idx != size_t(Options["Threads"]) - 1 ?
                                stride : clusterCount - start;

          std::memset(&table[start], 0, len * sizeof(Cluster));
      });
  }

  for (std::thread& th : threads)
      th.join();
}


/// TranspositionTable::probe() looks up the current position in the transposition
/// table. It returns true and a pointer to the TTEntry if the position is found.
/// Otherwise, it returns false and a pointer to an empty or least valuable TTEntry
/// to be replaced later. The replace value of an entry is calculated as its depth
/// minus 8 times its relative age. TTEntry t1 is considered more valuable than
/// TTEntry t2 if its replace value is greater than that of t2.

void TranspositionTable::probe(TTWrapper& ttw, const Key key, bool& found) const {

  Cluster* const cluster = first_entry(key);
  TTEntry* const tte = &cluster->entry[0];

  const uint16_t key16 = (uint16_t)key;  // Use the low 16 bits as key inside the cluster

  for (int i = 0; i < ClusterSize; ++i)
      if (tte[i].key16 == key16 || !tte[i].depth8)
      {
          tte[i].genBound8 = uint8_t(generation8 | (tte[i].genBound8 & (GENERATION_DELTA - 1))); // Refresh

          ttw.tte = &tte[i];
          ttw.extraInfo = &(cluster->extra);
          ttw.rank = i;

          found = (bool)tte[i].depth8;

          return;
      }

  // Find an entry to be replaced according to the replacement strategy
  TTEntry* replace = tte;
  int r = 0;
  for (int i = 1; i < ClusterSize; ++i)
      // Due to our packed storage format for generation and its cyclic
      // nature we add GENERATION_CYCLE (256 is the modulus, plus what
      // is needed to keep the unrelated lowest n bits from affecting
      // the result) to calculate the entry age correctly even after
      // generation8 overflows into the next cycle.
      if (  replace->depth8 - ((GENERATION_CYCLE + generation8 - replace->genBound8) & GENERATION_MASK)
          >   tte[i].depth8 - ((GENERATION_CYCLE + generation8 -   tte[i].genBound8) & GENERATION_MASK))
      {
          replace = &tte[i];
          r = i;
      }

  ttw.tte = replace;
  ttw.extraInfo = &(cluster->extra);
  ttw.rank = r;

  found = false;

  return;
}


/// TranspositionTable::hashfull() returns an approximation of the hashtable
/// occupation during a search. The hash is x permill full, as per UCI protocol.

int TranspositionTable::hashfull() const {

  int cnt = 0;
  for (int i = 0; i < 1000; ++i)
      for (int j = 0; j < ClusterSize; ++j)
          cnt += table[i].entry[j].depth8 && (table[i].entry[j].genBound8 & GENERATION_MASK) == generation8;

  return cnt / ClusterSize;
}

} // namespace Stockfish
