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

uint16_t moveMapping[1 << 16];
Move inverseMoveMapping[1 << 12];

const int MOVE_MASK = (0x7FF << 5);
const int BOUND_MASK = (0x3 << 3);
const int PV_MASK = (1 << 2);
const int GEN_MASK  = 0x3;


const int GEN_MASK_COMPLEMENTARY = (1>>16) - 1 - GEN_MASK;
const int MOVE_MASK_COMPLEMENTARY = (1>>16) - 1 - MOVE_MASK;

const int EVAL_MASK = (0x1FFF << 17);
const int VALUE_MASK = (0x1FFF << 6);
const int DEPTH_MASK = 0x3F;


void Transposition::init() {

	int counter=0;

	moveMapping[MOVE_NONE] = 0;
	inverseMoveMapping[counter++] = MOVE_NONE;


	//from - to moves
	for(int i = 0; i < 64; i++)
	{
		for(PieceType pieceType : {QUEEN, KNIGHT})
		{
			Bitboard attacks=PseudoAttacks[pieceType][i];
			while(attacks)
			{
				int j = pop_lsb(attacks);
				moveMapping[(i << 6) + j] = counter;
				inverseMoveMapping[counter++] = (Move) ((i << 6) + j);
			}
		}
	}


	//Entries for castling

	for(int i = SQ_B1; i <= SQ_G1; i++)
	{
		for(int j = SQ_A1; j < i; j++)
		{
			moveMapping[CASTLING + (i >> 6) + j] = counter;
			inverseMoveMapping[counter++] = (Move) (CASTLING + (i >> 6) + j);
		}
		for(int j = i+1; j <= SQ_H1; j++)
		{
			moveMapping[CASTLING + (i >> 6) + j ]= counter;
			inverseMoveMapping[counter++] = (Move) (CASTLING + (i >> 6) + j);
		}
	}

	for(int i = SQ_B8; i <= SQ_G8; i++)
	{
		for(int j = SQ_A8; j < i; j++)
				{
					moveMapping[CASTLING + (i >> 6) + j] = counter;
					inverseMoveMapping[counter++] = (Move) (CASTLING + (i >> 6) + j);
				}
		for(int j = i+1; j <= SQ_H8; j++)
		{
			moveMapping[CASTLING + (i >> 6) + j] = counter;
			inverseMoveMapping[counter++] = (Move) (CASTLING + (i >> 6) + j);
		}
	}


	//Entries for promotions

	for(int i = SQ_A7; i <= SQ_H7; i++)
		for(PieceType pieceType : {KNIGHT, BISHOP, ROOK, QUEEN})
		{
			moveMapping[PROMOTION + ((pieceType - KNIGHT) << 12) + (i << 6) + i + 8] = counter;
			inverseMoveMapping[counter++] = (Move) (PROMOTION + ((pieceType - KNIGHT) << 12) + (i << 6) + i + 8);
		}

	for(int i = SQ_A2; i <= SQ_H2; i++)
			for(PieceType pieceType : {KNIGHT, BISHOP, ROOK, QUEEN})
			{
				moveMapping[PROMOTION + ((pieceType - KNIGHT) << 12) + (i << 6) + i - 8] = counter;
				inverseMoveMapping[counter++] = (Move) (PROMOTION + ((pieceType - KNIGHT) << 12) + (i << 6) + i - 8);
			}



	//Entries for en passant

	for(int i = SQ_A5; i<= SQ_G5; i++)
	{
		moveMapping[EN_PASSANT + (i << 6) + i + 9] = counter;
		inverseMoveMapping[counter++] = (Move) (EN_PASSANT + (i << 6) + i + 9);
	}
	for(int i = SQ_B5; i<= SQ_H5; i++)
	{
		moveMapping[EN_PASSANT + (i << 6) + i + 7] = counter;
		inverseMoveMapping[counter++] = (Move) (EN_PASSANT + (i << 6) + i + 7);
	}
	for(int i = SQ_A4; i<= SQ_G4; i++)
	{
		moveMapping[EN_PASSANT + (i << 6) + i - 7] = counter;
		inverseMoveMapping[counter++] = (Move) (EN_PASSANT + (i << 6) + i - 7);
	}
	for(int i = SQ_B4; i<= SQ_H4; i++)
	{
		moveMapping[EN_PASSANT + (i << 6) + i - 9] = counter;
		inverseMoveMapping[counter++] = (Move) (EN_PASSANT + (i << 6) + i - 9);
	}



}

TranspositionTable TT; // Our global transposition table

/// TTEntry::save() populates the TTEntry with a new node's data, possibly
/// overwriting an old position. Update is not atomic and can be racy.

void TTEntry::save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev) {

  // Preserve any existing move for the same position
  if (m || (uint16_t)k != key16)
  {
      moveBoundPVGen = (moveBoundPVGen & MOVE_MASK_COMPLEMENTARY) | (moveMapping[m] << 5);
      std::cout << m << " - " << move() << "\n";
      assert (move() == m);
  }

  // Overwrite less valuable entries (cheapest checks first)
  if (   b == BOUND_EXACT
      || (uint16_t)k != key16
      || d + 2 * pv >  depth() - 4)
  {
      assert(d > DEPTH_OFFSET);
      assert(d < 256 + DEPTH_OFFSET);

      uint32_t value = (uint32_t) std::clamp(v, Value(-0x7FFF), Value(0x7FFF)) / 4;
      uint32_t eval = (uint32_t) std::clamp(ev, Value(-0x7FFF), Value(0x7FFF)) / 4;
      uint32_t depth =(uint32_t) std::min(d - DEPTH_OFFSET, 0x3F);

      evalValueDepth = (eval << 17) + (value << 6) + depth;

      uint16_t gen = TT.generation;
      uint16_t newPv = (uint16_t) pv;
      uint16_t bound = (uint16_t) (b & (v > Value(0x7FFF)) ? BOUND_LOWER : (v < Value(-0x7FFF)) ? BOUND_UPPER : BOUND_EXACT);

      moveBoundPVGen = (moveBoundPVGen & MOVE_MASK) | ((bound << 3) + (newPv << 2) + gen);
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

TTEntry* TranspositionTable::probe(const Key key, bool& found) const {

  TTEntry* const tte = first_entry(key);
  const uint16_t key16 = (uint16_t)key;  // Use the low 16 bits as key inside the cluster
  uint8_t depth[ClusterSize];
  uint8_t gen[ClusterSize];

  for (int i = 0; i < ClusterSize; ++i)
  {
	  depth[i] = (uint8_t) (tte[i].evalValueDepth & DEPTH_MASK);
	  gen[i] = (tte[i].moveBoundPVGen & GEN_MASK);
      if (tte[i].key16 == key16 || !depth[i])
      {
          tte[i].moveBoundPVGen = uint8_t (generation | (tte[i].moveBoundPVGen & GEN_MASK_COMPLEMENTARY)); // Refresh

          return found = (bool) depth[i], &tte[i];
      }
  }

  // Find an entry to be replaced according to the replacement strategy
  int replace=0;
  for (int i = 1; i < ClusterSize; ++i)
      // Due to our packed storage format for generation and its cyclic
      // nature we add GENERATION_CYCLE (256 is the modulus, plus what
      // is needed to keep the unrelated lowest n bits from affecting
      // the result) to calculate the entry age correctly even after
      // generation8 overflows into the next cycle.
      if (  depth[replace] - ((GENERATION_CYCLE + generation - gen[replace]) & GEN_MASK)
          >   depth[i] - ((GENERATION_CYCLE + generation -   gen[i]) & GEN_MASK))
          replace = i;

  return found = false, &tte[replace];
}


/// TranspositionTable::hashfull() returns an approximation of the hashtable
/// occupation during a search. The hash is x permill full, as per UCI protocol.

int TranspositionTable::hashfull() const {

  int cnt = 0;
  for (int i = 0; i < 1000; ++i)
      for (int j = 0; j < ClusterSize; ++j)
          cnt += (table[i].entry[j].evalValueDepth & DEPTH_MASK) && (table[i].entry[j].moveBoundPVGen & GEN_MASK) == generation;

  return cnt / ClusterSize;
}

} // namespace Stockfish
