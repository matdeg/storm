//
// Created by foxnbk on 16.03.16.
//

#include "UniformExpression.h"

namespace storm {
    namespace pgcl {
        UniformExpression::UniformExpression(int_fast64_t begin, int_fast64_t end) : begin(begin), end(end) {
            // Intentionally left empty.
        }

        int_fast64_t UniformExpression::getBegin() const {
            return this->begin;
        }

        int_fast64_t UniformExpression::getEnd() const {
            return this->end;
        }
    }
}

#include "UniformExpression.h"
