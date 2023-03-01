#pragma once

#include "storm-config.h"
#ifdef STORM_HAVE_XERCES
#include <string>

#include "storm-gspn/adapters/XercesAdapter.h"
#include "storm/storage/Trace.h"
#include "storm/storage/jani/Model.h"
#include <xercesc/parsers/XercesDOMParser.hpp>

namespace storm {
namespace parser {
class XesParser {
   public:

   XesParser(storm::jani::Model const& model);

    /*!
     * Parses the given file into the GSPN.
     *
     * @param filename The name of the file to parse.
     * @return The resulting GSPN.
     */
    std::vector<storm::storage::Trace> parseXesTraces(std::string const& filename);

    void traverseProjectElement(xercesc::DOMNode const* const node);
    void traverseTraceElement(xercesc::DOMNode const* const node);
    std::string traverseEventElement(xercesc::DOMNode const* const node);
    std::string traverseStringElement(xercesc::DOMNode const* const node);
    bool isConceptName(xercesc::DOMNode const* const node);

    void addTrace(storm::storage::Trace const& trace);
    std::vector<storm::storage::Trace> getTraces();
    storm::jani::Model getModel();

   private:
    // the constructed Traces
    std::vector<storm::storage::Trace> traces;
    storm::jani::Model model;
};
} //namespace parser
} //namespace storm
#endif