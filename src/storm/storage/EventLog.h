#pragma once

#include <vector>
#include "storm/storage/jani/Property.h"
#include "storm/storage/jani/Model.h"
#include "storm/storage/Trace.h"
#include "storm/storage/expressions/ExpressionManager.h"

namespace storm {
namespace storage {
class EventLog {
   public:

    void addTrace(storm::storage::Trace trace);
    storm::storage::Trace getTrace(uint_fast64_t k) const;
    std::vector<storm::storage::Trace> getTraces();

    uint_fast64_t size() const;
    uint_fast64_t getInvalidTracesSize() const;
   private:
    std::vector<storm::storage::Trace> traces; 
    uint_fast64_t invalidTracesSize = 0;
};

}  // namespace storage
}  // namespace storm