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

ExplicitTraceParser::ExplicitTraceParser(storm::jani::Model const& model) : model(model) {}

storm::storage::EventLog& ExplicitTraceParser::parseTraces(std::string const& filename) {
    
    storm::utility::Stopwatch modelParsingWatch(true);

    std::ifstream file(filename);
    std::string l;
    storm::storage::Trace trace(traceID++);
    bool isValidTrace = true;


    while (storm::utility::getline(file,l)) {
        if (l.substr(0,3) != "---") {
            int index = -1;
            if (getModel().hasAction(l)) {
                index = getModel().getActionIndex(l);
            } else {
                isValidTrace = false;
            }
            trace.addEvent(index);
        } else {
            if (isValidTrace) {
                eventLog.addTrace(trace);
            }
            trace = storm::storage::Trace(traceID++);
            isValidTrace = true;
        }
    }

    modelParsingWatch.stop();
    STORM_PRINT("Time for traces input parsing: " << modelParsingWatch << ".\n\n");
    return getEventLog();
    
}


storm::storage::EventLog& ExplicitTraceParser::getEventLog() {
    return this->eventLog;
}


storm::jani::Model const& ExplicitTraceParser::getModel() {
    return this->model;
}

} //namespace parser
} //namespace storm