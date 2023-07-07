#pragma once

#include <vector>
#include <algorithm>
#include "storm/storage/jani/Property.h"
#include "storm/storage/jani/Model.h"
#include "storm/storage/Trace.h"
#include "storm/storage/expressions/ExpressionManager.h"

namespace storm {
namespace storage {


template<typename ValueType>
class EventLog {
   public:
    void addTrace(std::vector<uint_fast64_t> trace, bool isValid);
    void updateTracesLength();
    std::vector<std::vector<uint_fast64_t>> getTraces() const;
    uint_fast64_t size() const;
    uint_fast64_t totalSize() const;
    uint_fast64_t getId(std::vector<uint_fast64_t> trace) ;
    void addProbability(ValueType p, int id);
    uint_fast64_t getTraceCount(std::vector<uint_fast64_t> trace) ; 
    bool isValid(uint_fast64_t id) const;
    void printInformation() ;
    void printDetailledInformation() ;
    ValueType score() ;
    uint_fast64_t numberOfZeroTraces() ;
    uint_fast64_t numberOfValidTraces() ;


   private:
    std::map<std::vector<uint_fast64_t>,int> tracesCount; 
    std::map<std::vector<uint_fast64_t>,int> traceToId;
    std::vector<std::vector<uint_fast64_t>> idToTrace;
    std::vector<uint_fast64_t> tracesLength;
    std::vector<uint_fast64_t> cumulativeTracesLength;
    double meanLength;
    uint_fast64_t minLength = std::numeric_limits<int>::max();
    uint_fast64_t maxLength = 0;
    std::vector<bool> validityVector;
    std::vector<ValueType> tracesProbability; 
    int Id = 0;
    
};

}  // namespace storage
}  // namespace storm