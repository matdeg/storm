#include <vector>
#include "storm/storage/jani/Property.h"
#include "storm/storage/jani/Model.h"
#include "storm/storage/EventLog.h"
#include "storm/storage/Trace.h"
#include "storm/utility/Stopwatch.h"

namespace storm {
namespace storage {

void EventLog::addTrace(storm::storage::Trace trace) {
    this->traces.emplace_back(trace);
}

storm::storage::Trace EventLog::getTrace(uint_fast64_t k) const {
    return this->traces[k];
}

std::vector<storm::storage::Trace> EventLog::getTraces() {
    return this->traces;
}

uint_fast64_t EventLog::size() const {
    return traces.size();
}


}  // namespace storage
}  // namespace storm