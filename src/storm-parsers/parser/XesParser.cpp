#include "XesParser.h"
#include "storm-config.h"
#include <algorithm>
#include <iostream>

#include "storm-gspn/adapters/XercesAdapter.h"
#include "storm/storage/jani/Model.h"
#include "storm/storage/Trace.h"
#include "storm/utility/Stopwatch.h"
#include "storm/exceptions/BaseException.h"
#include "storm/exceptions/NotSupportedException.h"
#include "storm/exceptions/UnexpectedException.h"
#include "storm/exceptions/WrongFormatException.h"
#include "storm/utility/macros.h"


namespace storm {
namespace parser {

XesParser::XesParser(storm::jani::Model const& model) : model(model) {
    // Intentionally left empty.
}

std::vector<storm::storage::Trace> XesParser::parseXesTraces(std::string const& filename) {
    #ifdef STORM_HAVE_XERCES
    // initialize xercesc
    storm::utility::Stopwatch modelParsingWatch(true);

    try {
        xercesc::XMLPlatformUtils::Initialize();
    } catch (xercesc::XMLException const& toCatch) {
        // Error occurred during the initialization process. Abort parsing since it is not possible.
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Failed to initialize xercesc\n");
    }

    auto parser = new xercesc::XercesDOMParser();
    parser->setValidationScheme(xercesc::XercesDOMParser::Val_Always);
    parser->setDoNamespaces(false);
    parser->setDoSchema(false);
    parser->setLoadExternalDTD(false);
    parser->setIncludeIgnorableWhitespace(false);

    auto errHandler = (xercesc::ErrorHandler*)new xercesc::HandlerBase();
    parser->setErrorHandler(errHandler);

    // parse file
    try {
        parser->parse(filename.c_str());
    } catch (xercesc::XMLException const& toCatch) {
        auto message = xercesc::XMLString::transcode(toCatch.getMessage());
        // Error occurred while parsing the file. Abort constructing the gspn since the input file is not valid
        // or the parser run into a problem.
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, message);
        xercesc::XMLString::release(&message);
    } catch (const xercesc::DOMException& toCatch) {
        auto message = xercesc::XMLString::transcode(toCatch.msg);
        // Error occurred while parsing the file. Abort constructing the gspn since the input file is not valid
        // or the parser run into a problem.
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, message);
        xercesc::XMLString::release(&message);
    } catch (...) {
        // Error occurred while parsing the file. Abort constructing the gspn since the input file is not valid
        // or the parser run into a problem.
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Failed to parse pnml file.\n");
    } 

    // build gspn by traversing the DOM object
    parser->getDocument()->normalizeDocument();
    xercesc::DOMElement* elementRoot = parser->getDocument()->getDocumentElement();

    if (storm::adapters::XMLtoString(elementRoot->getTagName()) == "log") {
        traverseProjectElement(elementRoot);
        modelParsingWatch.stop();
        STORM_PRINT("Time for traces input parsing: " << modelParsingWatch << ".\n\n");
        return getTraces();
    }
    // clean up
    delete parser;
    delete errHandler;
    xercesc::XMLPlatformUtils::Terminate();
#else
    STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Storm is not compiled with XML support: " << filename << " can not be parsed");
#endif
}

void XesParser::traverseProjectElement(xercesc::DOMNode const* const node) {
    // traverse children
    for (uint_fast64_t i = 0; i < node->getChildNodes()->getLength(); ++i) {
        auto child = node->getChildNodes()->item(i);
        auto name = storm::adapters::getName(child);
        if (name.compare("trace") == 0) {
            traverseTraceElement(child);
        }
    }
}

void XesParser::traverseTraceElement(xercesc::DOMNode const* const node) {
    storm::storage::Trace trace;

    // traverse children
    for (uint_fast64_t i = 0; i < node->getChildNodes()->getLength(); ++i) {
        auto child = node->getChildNodes()->item(i);
        auto name = storm::adapters::getName(child);
        if (name.compare("event") == 0) {
            std::string event = traverseEventElement(child);
            int index = -1;
            if (getModel().hasAction(event)) {
                index = getModel().getActionIndex(event);
            }
            trace.addEvent(index);
        }
    }
    trace.setModel(getModel());
    addTrace(trace);
}

std::string XesParser::traverseEventElement(xercesc::DOMNode const* const node) {
    // traverse children
    for (uint_fast64_t i = 0; i < node->getChildNodes()->getLength(); ++i) {
        auto child = node->getChildNodes()->item(i);
        auto name = storm::adapters::getName(child);
        if (name.compare("string") == 0 && isConceptName(child)) {
            return traverseStringElement(child);
        }
    }
}

std::string XesParser::traverseStringElement(xercesc::DOMNode const* const node) {
    for (uint_fast64_t i = 0; i < node->getAttributes()->getLength(); ++i) {
        auto attr = node->getAttributes()->item(i);
        auto name = storm::adapters::getName(attr);    
        if (name.compare("value") == 0) {
            return storm::adapters::XMLtoString(attr->getNodeValue());
        }
    }      
}

bool XesParser::isConceptName(xercesc::DOMNode const* const node) {
    for (uint_fast64_t i = 0; i < node->getAttributes()->getLength(); ++i) {
        auto attr = node->getAttributes()->item(i);
        auto name = storm::adapters::getName(attr);    
        if (name.compare("key") == 0 & storm::adapters::XMLtoString(attr->getNodeValue()) == "concept:name") {
            return true;
        }
    }
    return false;
}

std::vector<storm::storage::Trace> XesParser::getTraces() {
    return this->traces;
}

void XesParser::addTrace(storm::storage::Trace const& trace) {
    this->traces.emplace_back(trace);
}

storm::jani::Model XesParser::getModel() {
    return this->model;
}

} //namespace parser
} //namespace storm