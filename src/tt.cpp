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

void Transposition::init() {

	int counter=0;

	moveMapping[MOVE_NONE] = 0;
	inverseMoveMapping[counter++] = MOVE_NONE;

	assert(popcount(KEY_MASK_0)==16);
	assert(popcount(VALUE_MASK_0)==16);
	assert(popcount(EVAL_MASK_0)==16);
	assert(popcount(PV_MASK_0)==1);
	assert(popcount(GEN_MASK_0)==4);
	assert(popcount(MOVE_MASK_0)==11);
	assert(popcount(DEPTH_MASK_0)==8);
	assert(popcount(BOUND_MASK_0)==2);
	assert(popcount(MOVE2_MASK_0)==11);
	assert(popcount(KEY_MASK_0)==16);
	assert(popcount(VALUE_MASK_1)==16);
	assert(popcount(EVAL_MASK_1)==16);
	assert(popcount(PV_MASK_1)==1);
	assert(popcount(GEN_MASK_1)==4);
	assert(popcount(MOVE_MASK_1)==11);
	assert(popcount(DEPTH_MASK_1)==8);
	assert(popcount(BOUND_MASK_1)==2);
	assert(popcount(MOVE2_MASK_1)==11);
	assert(popcount(KEY_MASK_0)==16);
	assert(popcount(VALUE_MASK_2)==16);
	assert(popcount(EVAL_MASK_2)==16);
	assert(popcount(PV_MASK_2)==1);
	assert(popcount(GEN_MASK_2)==4);
	assert(popcount(MOVE_MASK_2)==11);
	assert(popcount(DEPTH_MASK_2)==8);
	assert(popcount(BOUND_MASK_2)==2);
	assert(popcount(MOVE2_MASK_2)==11);
	assert(KEY_MASK_0 + VALUE_MASK_0==0xFFFFFFFF);
	assert(EVAL_MASK_0 + PV_MASK_0 + GEN_MASK_0 + MOVE_MASK_0==0xFFFFFFFF);
	assert(DEPTH_MASK_0+BOUND_MASK_0+MOVE2_MASK_0+MOVE2_MASK_1==0xFFFFFFFF);
	assert(KEY_MASK_1 + VALUE_MASK_1==0xFFFFFFFF);
	assert(EVAL_MASK_1 + PV_MASK_1 + GEN_MASK_1 + MOVE_MASK_1==0xFFFFFFFF);
	assert(DEPTH_MASK_1+BOUND_MASK_1+MOVE2_MASK_2+MOVE_MASK_2==0xFFFFFFFF);
	assert(KEY_MASK_2 + VALUE_MASK_2==0xFFFFFFFF);
	assert((EVAL_MASK_2+PV_MASK_2+GEN_MASK_2+BOUND_MASK_2+DEPTH_MASK_2)==0x7FFFFFFF);






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
				moveMapping[EN_PASSANT + (i << 6) + j]=counter;
				inverseMoveMapping[counter++] = (Move) ((i << 6) + j);
			}
		}
	}


	//Entries for castling

	for(int i = SQ_B1; i <= SQ_G1; i++)
	{
		for(int j = SQ_A1; j < i; j++)
		{
			moveMapping[CASTLING + (i << 6) + j] = counter;
			inverseMoveMapping[counter] = (Move) (CASTLING + 1);
		}
		for(int j = i+1; j <= SQ_H1; j++)
		{
			moveMapping[CASTLING + (i << 6) + j ]= counter+1;
			inverseMoveMapping[counter+1] = (Move) (CASTLING);
		}
	}

	for(int i = SQ_B8; i <= SQ_G8; i++)
	{
		for(int j = SQ_A8; j < i; j++)
		{
			moveMapping[CASTLING + (i << 6) + j] = counter;
			inverseMoveMapping[counter] = (Move) (CASTLING + 1);
		}
		for(int j = i+1; j <= SQ_H8; j++)
		{
			moveMapping[CASTLING + (i << 6) + j] = counter+1;
			inverseMoveMapping[counter+1] = (Move) (CASTLING);
		}
	}

	counter+=2;

	//Entries for promotions

	for(int i = SQ_B7; i <= SQ_G7; i++)
		for(PieceType pieceType : {KNIGHT, BISHOP, ROOK, QUEEN})
		{
			for(int k : {7, 8, 9})
			{
				moveMapping[PROMOTION + ((pieceType - KNIGHT) << 12) + (i << 6) + i + k] = counter;
				inverseMoveMapping[counter++] = (Move) (PROMOTION + ((pieceType - KNIGHT) << 12) + (i << 6) + i + k);
			}
		}

	int l = SQ_A7;
	for(PieceType pieceType : {KNIGHT, BISHOP, ROOK, QUEEN})
		for(int k : {8, 9})
			{
				moveMapping[PROMOTION + ((pieceType - KNIGHT) << 12) + (l << 6) + l + k] = counter;
				inverseMoveMapping[counter++] = (Move) (PROMOTION + ((pieceType - KNIGHT) << 12) + (l << 6) + l + k);
			}

	l = SQ_H7;
	for(PieceType pieceType : {KNIGHT, BISHOP, ROOK, QUEEN})
		for(int k : {7, 8})
			{
				moveMapping[PROMOTION + ((pieceType - KNIGHT) << 12) + (l << 6) + l + k] = counter;
				inverseMoveMapping[counter++] = (Move) (PROMOTION + ((pieceType - KNIGHT) << 12) + (l << 6) + l + k);
			}

	for(int i = SQ_B2; i <= SQ_G2; i++)
			for(PieceType pieceType : {KNIGHT, BISHOP, ROOK, QUEEN})
			{
				for(int k: {7, 8, 9})
				{
					moveMapping[PROMOTION + ((pieceType - KNIGHT) << 12) + (i << 6) + i - k] = counter;
					inverseMoveMapping[counter++] = (Move) (PROMOTION + ((pieceType - KNIGHT) << 12) + (i << 6) + i - k);
				}
			}

	l = SQ_A2;
	for(PieceType pieceType : {KNIGHT, BISHOP, ROOK, QUEEN})
		for(int k : {7, 8})
			{
				moveMapping[PROMOTION + ((pieceType - KNIGHT) << 12) + (l << 6) + l - k] = counter;
				inverseMoveMapping[counter++] = (Move) (PROMOTION + ((pieceType - KNIGHT) << 12) + (l << 6) + l - k);
			}

	l = SQ_H2;

	for(PieceType pieceType : {KNIGHT, BISHOP, ROOK, QUEEN})
		for(int k : {8, 9})
			{
				moveMapping[PROMOTION + ((pieceType - KNIGHT) << 12) + (l << 6) + l - k] = counter;
				inverseMoveMapping[counter++] = (Move) (PROMOTION + ((pieceType - KNIGHT) << 12) + (l << 6) + l - k);
			}

}

TranspositionTable TT; // Our global transposition table

/// TTEntry::save() populates the TTEntry with a new node's data, possibly
/// overwriting an old position. Update is not atomic and can be racy.

void TTEntry::copyData() {

	switch(rank)
	{
	case 0:
		key = (data->data[KEY_INDEX_0] & KEY_MASK_0) >> KEY_SHIFT_0;
		m_move = (data->data[MOVE_INDEX_0] & MOVE_MASK_0) >> MOVE_SHIFT_0;
		m_move2 = (data->data[MOVE2_INDEX_0] & MOVE2_MASK_0) >> MOVE2_SHIFT_0;
		m_depth = (data->data[DEPTH_INDEX_0] & DEPTH_MASK_0) >> DEPTH_SHIFT_0;
		m_is_pv = (data->data[PV_INDEX_0] & PV_MASK_0) >> PV_SHIFT_0;
		m_bound = (data->data[BOUND_INDEX_0] & BOUND_MASK_0) >> BOUND_SHIFT_0;
		m_value = (data->data[VALUE_INDEX_0] & VALUE_MASK_0) >> VALUE_SHIFT_0;
		m_eval = (data->data[EVAL_INDEX_0] & EVAL_MASK_0) >> EVAL_SHIFT_0;
		generation = (data->data[GEN_INDEX_0] & GEN_MASK_0) >> GEN_SHIFT_0;

		return;

	case 1:
		key = (data->data[KEY_INDEX_1] & KEY_MASK_1) >> KEY_SHIFT_1;
		m_move = (data->data[MOVE_INDEX_1] & MOVE_MASK_1) >> MOVE_SHIFT_1;
		m_move2 = (data->data[MOVE2_INDEX_1] & MOVE2_MASK_1) >> MOVE2_SHIFT_1;
		m_depth = (data->data[DEPTH_INDEX_1] & DEPTH_MASK_1) >> DEPTH_SHIFT_1;
		m_is_pv = (data->data[PV_INDEX_1] & PV_MASK_1) >> PV_SHIFT_1;
		m_bound = (data->data[BOUND_INDEX_1] & BOUND_MASK_1) >> BOUND_SHIFT_1;
		m_value = (data->data[VALUE_INDEX_1] & VALUE_MASK_1) >> VALUE_SHIFT_1;
		m_eval = (data->data[EVAL_INDEX_1] & EVAL_MASK_1) >> EVAL_SHIFT_1;
		generation = (data->data[GEN_INDEX_1] & GEN_MASK_1) >> GEN_SHIFT_1;

		return;

	case 2:
		key = (data->data[KEY_INDEX_2] & KEY_MASK_2) >> KEY_SHIFT_2;
		m_move = (data->data[MOVE_INDEX_2] & MOVE_MASK_2) >> MOVE_SHIFT_2;
		m_move2 = (data->data[MOVE2_INDEX_2] & MOVE2_MASK_2) >> MOVE2_SHIFT_2;
		m_depth = (data->data[DEPTH_INDEX_2] & DEPTH_MASK_2) >> DEPTH_SHIFT_2;
		m_is_pv = (data->data[PV_INDEX_2] & PV_MASK_2) >> PV_SHIFT_2;
		m_bound = (data->data[BOUND_INDEX_2] & BOUND_MASK_2) >> BOUND_SHIFT_2;
		m_value = (data->data[VALUE_INDEX_2] & VALUE_MASK_2) >> VALUE_SHIFT_2;
		m_eval = (data->data[EVAL_INDEX_2] & EVAL_MASK_2) >> EVAL_SHIFT_2;
		generation = (data->data[GEN_INDEX_2] & GEN_MASK_2) >> GEN_SHIFT_2;
	}
}

void TTEntry::copyDataLight() {

	switch(rank)
	{
	case 0:
		key = (data->data[KEY_INDEX_0] & KEY_MASK_0) >> KEY_SHIFT_0;
		m_move = (data->data[MOVE_INDEX_0] & MOVE_MASK_0) >> MOVE_SHIFT_0;
		m_move2 = (data->data[MOVE2_INDEX_0] & MOVE2_MASK_0) >> MOVE2_SHIFT_0;
		m_depth = (data->data[DEPTH_INDEX_0] & DEPTH_MASK_0) >> DEPTH_SHIFT_0;

		return;

	case 1:
		key = (data->data[KEY_INDEX_1] & KEY_MASK_1) >> KEY_SHIFT_1;
		m_move = (data->data[MOVE_INDEX_1] & MOVE_MASK_1) >> MOVE_SHIFT_1;
		m_move2 = (data->data[MOVE2_INDEX_1] & MOVE2_MASK_1) >> MOVE2_SHIFT_1;
		m_depth = (data->data[DEPTH_INDEX_1] & DEPTH_MASK_1) >> DEPTH_SHIFT_1;

		return;

	case 2:
		key = (data->data[KEY_INDEX_2] & KEY_MASK_2) >> KEY_SHIFT_2;
		m_move = (data->data[MOVE_INDEX_2] & MOVE_MASK_2) >> MOVE_SHIFT_2;
		m_move2 = (data->data[MOVE2_INDEX_2] & MOVE2_MASK_2) >> MOVE2_SHIFT_2;
		m_depth = (data->data[DEPTH_INDEX_2] & DEPTH_MASK_2) >> DEPTH_SHIFT_2;

	}
}

int Cluster::getDepth(int rank) const
{
	switch(rank)
	{
	case 0:
		return (int) ((data[DEPTH_INDEX_0] & DEPTH_MASK_0) >> DEPTH_SHIFT_0);

	case 1:
		return (int) ((data[DEPTH_INDEX_1] & DEPTH_MASK_1) >> DEPTH_SHIFT_1);

	case 2:
		return (int) ((data[DEPTH_INDEX_2] & DEPTH_MASK_2) >> DEPTH_SHIFT_2);
	}

	return 0;
}

int Cluster::getGeneration(int rank) const
{
	switch(rank)
	{
	case 0:
		return (int) ((data[GEN_INDEX_0] & GEN_MASK_0) >> GEN_SHIFT_0);

	case 1:
		return (int) ((data[GEN_INDEX_1] & GEN_MASK_1) >> GEN_SHIFT_1);

	case 2:
		return (int) ((data[GEN_INDEX_2] & GEN_MASK_2) >> GEN_SHIFT_2);
	}

	return 0;
}

uint16_t Cluster::getKey(int rank) const
{
	switch(rank)
	{
	case 0:
		return (uint16_t) ((data[KEY_INDEX_0] & KEY_MASK_0) >> KEY_SHIFT_0);

	case 1:
		return (uint16_t) ((data[KEY_INDEX_1] & KEY_MASK_1) >> KEY_SHIFT_1);

	case 2:
		return (uint16_t) ((data[KEY_INDEX_2] & KEY_MASK_2) >> KEY_SHIFT_2);
	}

	return 0;
}

void Cluster::setGeneration(int rank)
{
	switch(rank)
	{
	case 0:
		data[GEN_INDEX_0] = (data[GEN_INDEX_0] & ~ GEN_MASK_0) | (TT.generation << GEN_SHIFT_0);
		return;

	case 1:
		data[GEN_INDEX_1] = (data[GEN_INDEX_1] & ~ GEN_MASK_1) | (TT.generation << GEN_SHIFT_1);
		return;

	case 2:
		data[GEN_INDEX_2] = (data[GEN_INDEX_2] & ~ GEN_MASK_2) | (TT.generation << GEN_SHIFT_2);
		return;
	}
}

void TTEntry::save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev, Move m2) {

	// Preserve any existing m_move for the same position

	copyDataLight();

	bool update=false;
	bool updateMoves=false;

	if ((uint16_t) k == key)
	{
	  if(m && moveMapping[m] != m_move)
	  {
		  m_move2 = m_move;
		  m_move=moveMapping[m];
		  updateMoves=true;
	  }
	}
	else
	{
		m_move = moveMapping[m];
		m_move2 = 0;
		updateMoves=true;
	}

	if(m2)
	{
		m_move2 = m2 == MOVE_NULL ? 0 : moveMapping[m2];
		updateMoves=true;
	}

	if(updateMoves)
	{
		assert(m_move <= 0x7FF);
		assert(m_move2 <= 0x7FF);
	}

	// Overwrite less valuable entries (cheapest checks first)
	if (   b == BOUND_EXACT
	  || (uint16_t)k != key
	  || d - DEPTH_OFFSET + 2 * pv > (int) m_depth - 4)
	{
	  assert(d > DEPTH_OFFSET);
	  assert(d < 256 + DEPTH_OFFSET);

	  update=true;

	  key			= (uint16_t) k;
	  m_depth     	= (uint32_t) (d - DEPTH_OFFSET);
	  generation   	= (uint32_t) TT.generation;
	  m_value     	= (uint32_t) VALUE_INFINITE + v;
	  m_eval      	= (uint32_t) VALUE_INFINITE + ev;
	  m_bound     	= (uint32_t) b;
	  m_is_pv      	= (uint32_t) pv;

	  assert(generation <= 0xF);
	  assert(key <= 0xFFFF);
	  assert(m_eval <= 0xFFFF);
	  assert(m_value <= 0xFFFF);
	  assert(m_is_pv <= 1);
	  assert(m_bound <= 3);
	  assert(m_depth <= 0xFF);

	}

	if(update)
	{
		switch(rank)
		{
		case 0:
			data->data[0] = (key << KEY_SHIFT_0) + (m_value << VALUE_SHIFT_0);
			data->data[1] = (m_eval << EVAL_SHIFT_0) + (m_is_pv << PV_SHIFT_0) + (generation << GEN_SHIFT_0) + (m_move << MOVE_SHIFT_0);
			data->data[2] = (data->data[2] & ~ (DEPTH_MASK_0 + BOUND_MASK_0 + MOVE2_MASK_0)) | ((m_depth << DEPTH_SHIFT_0) + (m_bound << BOUND_SHIFT_0) + (m_move2 << MOVE2_SHIFT_0));
			return;

		case 1:
			data->data[2] = (data->data[2] & ~ (MOVE2_MASK_1)) | (m_move2 << MOVE2_SHIFT_1);
			data->data[3] = (key << KEY_SHIFT_1) + (m_value << VALUE_SHIFT_1);
			data->data[4] = (m_eval << EVAL_SHIFT_1) + (m_is_pv << PV_SHIFT_1) + (generation << GEN_SHIFT_1) + (m_move << MOVE_SHIFT_1);
			data->data[5] = (data->data[5] & ~ (DEPTH_MASK_1 + BOUND_MASK_1)) | ((m_depth << DEPTH_SHIFT_1) + (m_bound << BOUND_SHIFT_1));
			return;

		case 2:
			data->data[5] = (data->data[5] & ~ (MOVE2_MASK_2 + MOVE_MASK_2)) | ((m_move2 << MOVE2_SHIFT_2) + (m_move << MOVE_SHIFT_2));
			data->data[6] = (key << KEY_SHIFT_2) + (m_value << VALUE_SHIFT_2);
			data->data[7] = (m_eval << EVAL_SHIFT_2) + (m_is_pv << PV_SHIFT_2) + (generation << GEN_SHIFT_2) + (m_bound << BOUND_SHIFT_2) + (m_depth << DEPTH_SHIFT_2);
			return;
		}
	}
	else if(updateMoves)
	{
		switch(rank)
		{
		case 0:
			data->data[1] = (data->data[1] & ~ MOVE_MASK_0) | (m_move << MOVE_SHIFT_0);
			data->data[2] =( data->data[2] & ~ MOVE2_MASK_0) | (m_move2 << MOVE2_SHIFT_0);
			return;

		case 1:
			data->data[2] = (data->data[2] & ~ MOVE2_MASK_1) | (m_move2 << MOVE2_SHIFT_1);
			data->data[4] = (data->data[4] & ~ MOVE_MASK_1) | (m_move << MOVE_SHIFT_1);
			return;

		case 2:
			data->data[5] = (data->data[5] & ~ (MOVE2_MASK_2 + MOVE_MASK_2)) | ((m_move2 << MOVE2_SHIFT_2) + (m_move << MOVE_SHIFT_2));
			return;
		}
	}

}

template<bool first>
Move TTEntry::move(Position& pos) const {
	Move move = inverseMoveMapping[first ? m_move : m_move2];

	if(!move)
		return MOVE_NONE;

	if(pos.ep_square() && pos.ep_square() == to_sq(move) && type_of(pos.piece_on(from_sq(move))) == PAWN)
			return (Move) (move + EN_PASSANT);

	if(type_of(move) == CASTLING)
	{
		const Color sideToMove = pos.side_to_move();
		const CastlingRights cr = (Color) sideToMove & (CastlingRights) ((move & 1) ? QUEEN_SIDE : KING_SIDE);
		const Square ksq = pos.square<KING>(sideToMove);
		const Square rsq = pos.castling_rook_square(cr);
		return make<CASTLING>(ksq,rsq);
	}

	return move;

}

template Move TTEntry::move<true>(Position& pos) const;
template Move TTEntry::move<false>(Position& pos) const;



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
/// to be replaced later. The replace m_value of an entry is calculated as its depth
/// minus 8 times its relative age. TTEntry t1 is considered more valuable than
/// TTEntry t2 if its replace m_value is greater than that of t2.

void TranspositionTable::probe(TTEntry& tte, const Key key, bool& found) const {

  tte.data = first_entry(key);
  int depth[ClusterSize];
  found=false;

  const uint16_t key16 = (uint16_t)key;  // Use the low 16 bits as key inside the cluster

  for (int i = 0; i < ClusterSize; ++i)
  {
	  depth[i] = tte.data->getDepth(i);
      if (tte.data->getKey(i) == key16 || !depth[i])
      {
    	  tte.rank = i;
          tte.data->setGeneration(i); // Refresh

          assert(tte.data->getDepth(i) == depth[i]);
          assert(tte.data->getGeneration(i) == TT.generation);

          if(depth[i]){
        	  found=true;
        	  tte.copyData();
          }


          return;
      }
  }

  // Find an entry to be replaced according to the replacement strategy

  int replace=0;
  int oldness[ClusterSize];
  oldness[0] = (GENERATION_CYCLE + generation - tte.data->getGeneration(0)) & (GENERATION_CYCLE-1);


  if(oldness[0] == GENERATION_CYCLE - 1)
  {
	  tte.rank=0;
	  return;
  }



  for (int i = 1; i < ClusterSize; ++i)
  {

	  oldness[i] = (GENERATION_CYCLE + generation - tte.data->getGeneration(i)) & (GENERATION_CYCLE-1);


  	  if(oldness[i] == GENERATION_CYCLE - 1)
  	  {
  		  tte.rank = i;
  		  return;
  	  }



      if (  depth[replace] - 16*oldness[replace]
          >   depth[i] - 16*oldness[i])
          replace = i;
  }
  tte.rank = replace;

}


/// TranspositionTable::hashfull() returns an approximation of the hashtable
/// occupation during a search. The hash is x permill full, as per UCI protocol.

int TranspositionTable::hashfull() const {

  int cnt = 0;
  for (int i = 0; i < 1000; ++i)
      for (int j = 0; j < ClusterSize; ++j)
          cnt += table[i].getDepth(j) && table[i].getGeneration(j) == generation;

  return cnt / ClusterSize;
}

} // namespace Stockfish
