#include "ExplicitTraceParser.h"
#include "storm-config.h"
#include <algorithm>
#include <iostream>

#include "storm/storage/jani/Model.h"
#include "storm/storage/EventLog.h"
#include "storm/utility/Stopwatch.h"
#include "storm/utility/macros.h"
#include "storm/io/file.h"


namespace storm {
namespace parser {

template <typename ValueType>
ExplicitTraceParser<ValueType>::ExplicitTraceParser(storm::jani::Model const& model) : model(model) {}

template <typename ValueType>
storm::storage::EventLog<ValueType>& ExplicitTraceParser<ValueType>::parseTraces(std::string const& filename) {
    
    storm::utility::Stopwatch modelParsingWatch(true);

    std::ifstream file(filename);
    std::string l;
    std::vector<uint_fast64_t> trace;
    bool isValidTrace = true;


    while (storm::utility::getline(file,l)) {
        if (l.substr(0,3) != "---") {
            int index = -1;
            if (getModel().hasAction(l)) {
                index = getModel().getActionIndex(l);
            } else {
                isValidTrace = false;
            }
            trace.emplace_back(index);
        } else {
            eventLog.addTrace(trace,isValidTrace);
            isValidTrace = true;
            trace.clear();
        }
    }

    modelParsingWatch.stop();
    STORM_PRINT("Time for traces input parsing: " << modelParsingWatch << ".\n\n");
    return getEventLog();
    
}

template <typename ValueType>
storm::storage::EventLog<ValueType>& ExplicitTraceParser<ValueType>::getEventLog() {
    return this->eventLog;
}

template <typename ValueType>
storm::jani::Model const& ExplicitTraceParser<ValueType>::getModel() {
    return this->model;
}

template class ExplicitTraceParser<double>;
template class ExplicitTraceParser<storm::RationalFunction>;
template class ExplicitTraceParser<storm::RationalNumber>;

} //namespace parser
} //namespace storm