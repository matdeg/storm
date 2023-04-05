#pragma once

#include "storm-config.h"
#include <string>

#include "storm/storage/EventLog.h"
#include "storm/storage/jani/Model.h"
#include "storm/settings/SettingsManager.h"
#include "storm/settings/modules/IOSettings.h"

namespace storm {
namespace parser {
class ExplicitTraceParser {
   public:

   ExplicitTraceParser(storm::jani::Model const& model);

    /*!
     * Parses the given file into the GSPN.
     *
     * @param filename The name of the file to parse.
     * @return The resulting GSPN.
     */
    storm::storage::EventLog& parseTraces(std::string const& filename);

    storm::storage::EventLog& getEventLog();
    storm::jani::Model const& getModel();

   private:
    // the constructed Traces
    int limit = std::numeric_limits<int>::max();
    storm::storage::EventLog eventLog;
    storm::jani::Model model;
    uint_fast64_t traceID = 0;
};
} //namespace parser
} //namespace storm