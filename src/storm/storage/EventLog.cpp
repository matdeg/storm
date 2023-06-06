#include <vector>
#include <math.h>
#include "storm/storage/jani/Property.h"
#include "storm/storage/jani/Model.h"
#include "storm/storage/EventLog.h"
#include "storm/storage/Trace.h"
#include "storm/utility/Stopwatch.h"
#include "storm/exceptions/OptionParserException.h"

namespace storm {
namespace storage {

template<typename ValueType>
void EventLog<ValueType>::addTrace(std::vector<uint_fast64_t> trace, bool isValid) {
    if (tracesCount.count(trace) > 0) {
        tracesCount[trace] += 1;
    } else {
        tracesCount[trace] = 1;
        idToTrace.emplace_back(trace);
        traceToId[trace] = Id++;
        validityVector.emplace_back(isValid);
        if (trace.size() > maxLength) {
            maxLength = trace.size();
        }
        if (trace.size() < minLength) {
            minLength = trace.size();
        }
    }
}

template<typename ValueType>
void EventLog<ValueType>::updateTracesLength() {
    tracesLength = std::vector<uint_fast64_t>(maxLength + 1,0);
    cumulativeTracesLength = std::vector<uint_fast64_t>(maxLength + 1,0);
    for (auto x : this->tracesCount) {
        tracesLength[x.first.size()] += x.second;
    }
    for (int k = 1; k < maxLength + 1; k++) {
        cumulativeTracesLength[k] = tracesLength[k] + cumulativeTracesLength[k-1];
    }
    double sumLengths = 0;
    for (int k = 1; k < maxLength + 1; k++) {
        sumLengths += k * tracesLength[k];
    }
    meanLength = sumLengths/(double)size();
}

template<typename ValueType>
uint_fast64_t EventLog<ValueType>::getTraceCount(std::vector<uint_fast64_t> trace) {
    return this->tracesCount[trace]; 
}

template<typename ValueType>
uint_fast64_t EventLog<ValueType>::getId(std::vector<uint_fast64_t> trace) {
    return this->traceToId[trace];
}

template<typename ValueType>
void EventLog<ValueType>::addProbability(ValueType p) {
    this->tracesProbability.emplace_back(p);
}

template<typename ValueType>
std::vector<std::vector<uint_fast64_t>> EventLog<ValueType>::getTraces() const {
    std::vector<std::vector<uint_fast64_t>> result;
    for (auto x : this->tracesCount) {
        result.emplace_back(x.first);
    }
    return result;
}

template<typename ValueType>
uint_fast64_t EventLog<ValueType>::size() const {
    return tracesCount.size();
}

template<typename ValueType>
uint_fast64_t EventLog<ValueType>::totalSize() const {
    uint_fast64_t n = 0;
    for (auto x : this->tracesCount) {
        n += x.second;
    }
    return n;
}
template<typename ValueType>
bool EventLog<ValueType>::isValid(uint_fast64_t id) const{
    return this->validityVector[id];
}

template<typename ValueType>
ValueType EventLog<ValueType>::score(){
    int N = totalSize();
    if constexpr (std::is_same<ValueType,double>::value) {
        ValueType sc = 1;
        for (int k = 0; k < size(); k++) {
            sc += fabs(tracesProbability[k] - (double)tracesCount[idToTrace[k]]/(double)N) - tracesProbability[k];
        }
        return sc/2;
    } else if constexpr (std::is_same<ValueType,storm::RationalFunction>::value) {
        STORM_LOG_THROW(false,storm::exceptions::OptionParserException, "score cannot be computed with Rational Functions");
    } else if constexpr (std::is_same<ValueType,storm::RationalNumber>::value) { 
        ValueType sc = 1;
        for (int k = 0; k < size(); k++) {
            if (tracesProbability[k] > (double)tracesCount[idToTrace[k]]/(double)N) {
                sc -= (double)tracesCount[idToTrace[k]]/(double)N;
            } else {
                sc += (double)tracesCount[idToTrace[k]]/(double)N - 2*tracesProbability[k];
            }
        }
        return sc/2;
    }
}


template<typename ValueType>
uint_fast64_t EventLog<ValueType>::numberOfZeroTraces() {
    if constexpr (!std::is_same<ValueType,storm::RationalFunction>::value) {
        uint_fast64_t n = 0;
        for (int k = 0; k < size(); k++) {
            if (tracesProbability[k] == 0) {
                n += tracesCount[idToTrace[k]];
            }
        }
        return n;
    } else {
        STORM_LOG_THROW(false,storm::exceptions::OptionParserException, "cannot compare 0 with Rational Functions");
    }
    
}

template<typename ValueType>
uint_fast64_t EventLog<ValueType>::numberOfValidTraces() {
    uint_fast64_t n = 0;
    for (int k = 0; k < size(); k++) {
        if (isValid(k)) {
            n += tracesCount[idToTrace[k]];
        }
    }
    return n;
}

template<typename ValueType>
void EventLog<ValueType>::printInformation() {
    updateTracesLength();
    int n = size();
    int N = totalSize();
    std::cout << "\n------------------------------------\n";
    std::cout << "EVENT LOG INFO : \n";
    std::cout << "number of distinct trace : \t" << n << "\n";
    std::cout << "number of traces : \t" << N << "\n";
    std::cout << "Percentage of traces with probability 0 : \t" << 100 * (double)numberOfZeroTraces()/(double)N << "%\n";
    std::cout << "Percentage of valid traces : \t" << 100 * (double)numberOfValidTraces()/(double)N << "%\n";
    std::cout << "Minimum trace's length : \t" << minLength << "\n";
    std::cout << "Maximum trace's length : \t" << maxLength << "\n";
    std::cout << "Mean length of traces: \t" << meanLength << "\n";
    std::cout << "score : \t" << score() << "\n";
    std::cout << "------------------------------------\n";
}

template<typename ValueType>
void EventLog<ValueType>::printDetailledInformation() {
    int n = size();
    int N = totalSize();
    for (int k = 0; k < n; k++) {
        std::cout << "trace number " << k << " has probability " << tracesProbability[k] << " and frequency " << (double)tracesCount[idToTrace[k]]/(double)N<< "\n";
    }
    printInformation();
}

template class EventLog<double>;
template class EventLog<storm::RationalFunction>;
template class EventLog<storm::RationalNumber>;

}  // namespace storage
}  // namespace storm