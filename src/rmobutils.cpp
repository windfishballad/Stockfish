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

#include <cassert>
#include <cmath>

#include "rmobutils.h"
#include "types.h"
#include "uci.h"

namespace Stockfish {

//Copy of UCI::win_rate_model analytics with appropriate grain

double rMob::valueToGScore(Value v, int ply) {
    // The model only captures up to 240 plies, so limit the input and then rescale
    double m = std::min(240, ply) / 64.0;

    // The coefficients of a third-order polynomial fit is based on the fishtest data
    // for two parameters that need to transform eval to the argument of a logistic
    // function.
    constexpr double as[] = {   0.38036525,   -2.82015070,   23.17882135,  307.36768407};
    constexpr double bs[] = {  -2.29434733,   13.27689788,  -14.26828904,   63.45318330 };

    // Enforce that NormalizeToPawnValue corresponds to a 50% win rate at ply 64
    static_assert(UCI::NormalizeToPawnValue == int(as[0] + as[1] + as[2] + as[3]));

    double a = (((as[0] * m + as[1]) * m + as[2]) * m) + as[3];
    double b = (((bs[0] * m + bs[1]) * m + bs[2]) * m) + bs[3];

    // Transform the eval to centipawns with limited range
    double x = std::clamp(double(v), -4000.0, 4000.0);

    // Transform w/l to score and invert it to gScore
    double score =  1.0 / (1 + std::exp((a - x) / b))- 1.0 / (1 + std::exp((a + x) / b));

    return v > 0 ? -std::log2(score) : v < 0 ? std::log2(-score) : std::numeric_limits<double>::infinity();

 }

//Return the highest value (in absolute) that the given g-Score will be preferred to. It depends on ply because it relies on SF's WDL model to convert values into expected score.
Value rMob::gScoreToValue(GScore g, int ply) {

    // Process to a simple dichotomy
    Value minValue = Value(0), maxValue=Value(2048);

    for(int i=0;i<20;++i) {
    	Value v = (minValue + maxValue)/2;
    	if (valueToGScore(v,ply)>std::abs(g))
    			minValue=v;
    	else
    		maxValue=v;
    }
    return g>0 ? minValue:-minValue;
 }

Value rMob::gScoreConversion[MAX_G_SCORE-1][241];

void rMob::init() {
	for (GScore g=G05; g<MAX_G_SCORE; ++g)
		for(int ply=0;ply<241;ply++)
			gScoreConversion[g-1][ply]=gScoreToValue(g,ply);
}



} //namespace Stockfish
