#pragma once

#include "storm-config.h"
#ifdef STORM_HAVE_XERCES
#include <string>

#include "storm-gspn/adapters/XercesAdapter.h"
#include "storm/storage/EventLog.h"
#include "storm/storage/jani/Model.h"
#include <xercesc/parsers/XercesDOMParser.hpp>
#include "storm/settings/SettingsManager.h"
#include "storm/settings/modules/IOSettings.h"

namespace storm {
namespace parser {

template <typename ValueType>
class XesParser {
   public:

   XesParser(storm::jani::Model const& model);

    /*!
     * Parses the given file into the GSPN.
     *
     * @param filename The name of the file to parse.
     * @return The resulting GSPN.
     */
    storm::storage::EventLog<ValueType>& parseXesTraces(std::string const& filename);

    void traverseProjectElement(xercesc::DOMNode const* const node);
    void traverseTraceElement(xercesc::DOMNode const* const node);
    boost::optional<std::string> traverseEventElement(xercesc::DOMNode const* const node);
    std::string traverseStringElement(xercesc::DOMNode const* const node);
    bool isConceptName(xercesc::DOMNode const* const node);

    storm::storage::EventLog<ValueType>& getEventLog();
    storm::jani::Model const& getModel();

   private:
    // the constructed Traces
    int limit = std::numeric_limits<int>::max();
    storm::storage::EventLog<ValueType> eventLog;
    storm::jani::Model model;
    uint_fast64_t traceID = 0;
};
} //namespace parser
} //namespace storm
#endif