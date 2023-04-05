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
    storm::storage::Trace getTrace(uint_fast64_t k);
    std::vector<storm::storage::Trace> getTraces();

    uint_fast64_t size();

   private:
    std::vector<storm::storage::Trace> traces; 
};

}  // namespace storage
}  // namespace storm