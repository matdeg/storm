
#include "XesParser.h"
#include "storm-config.h"
#include <algorithm>
#include <iostream>

#include "storm-gspn/adapters/XercesAdapter.h"
#include "storm/storage/jani/Model.h"
#include "storm/storage/EventLog.h"
#include "storm/utility/Stopwatch.h"
#include "storm/exceptions/BaseException.h"
#include "storm/exceptions/NotSupportedException.h"
#include "storm/exceptions/UnexpectedException.h"
#include "storm/exceptions/WrongFormatException.h"
#include "storm/utility/macros.h"
#include "storm/io/file.h"


namespace storm {
namespace parser {

template<typename ValueType>
XesParser<ValueType>::XesParser(storm::jani::Model const& model) : model(model) {}

template<typename ValueType>
storm::storage::EventLog<ValueType>& XesParser<ValueType>::parseXesTraces(std::string const& filename) {
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

    bool other = false;

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
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "unabled to parse the file " + filename);
    } 

    // build gspn by traversing the DOM object
    parser->getDocument()->normalizeDocument();
    xercesc::DOMElement* elementRoot = parser->getDocument()->getDocumentElement();
    if(storm::settings::getModule<storm::settings::modules::IOSettings>().hasTracesLimit()) {
        limit = storm::settings::getModule<storm::settings::modules::IOSettings>().getTracesLimit();
    }
    if (storm::adapters::XMLtoString(elementRoot->getTagName()) == "log") {
        traverseProjectElement(elementRoot);
        modelParsingWatch.stop();
        STORM_PRINT("Time for traces input parsing: " << modelParsingWatch << ".\n\n");
        return getEventLog();

    } else {
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "no \"log\" root name in the .xes file ");
    }
    
    // clean up
    delete parser;
    delete errHandler;
    xercesc::XMLPlatformUtils::Terminate();
#else
    STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Storm is not compiled with XML support: " << filename << " can not be parsed");
#endif
}

template<typename ValueType>
void XesParser<ValueType>::traverseProjectElement(xercesc::DOMNode const* const node) {
    // traverse children
    for (uint_fast64_t i = 0; i < node->getChildNodes()->getLength() && traceID < limit; ++i) {
        auto child = node->getChildNodes()->item(i);
        auto name = storm::adapters::getName(child);
        if (name.compare("trace") == 0) {
            traverseTraceElement(child);
        }
    }
}

template<typename ValueType>
void XesParser<ValueType>::traverseTraceElement(xercesc::DOMNode const* const node) {
    std::vector<uint_fast64_t> trace;
    bool isValidTrace = true;
    uint_fast64_t i = 0;
    // traverse children
    while (i < node->getChildNodes()->getLength()) {
        auto child = node->getChildNodes()->item(i);
        auto name = storm::adapters::getName(child);
        if (name.compare("event") == 0) {
            auto p = traverseEventElement(child);
            if (p) {
                std::string event = *p;
                int index = -1;
                if (getModel().hasAction(event)) {
                    index = getModel().getActionIndex(event);
                } else {
                    isValidTrace = false;
                }
                trace.emplace_back(index);
            }
        }
        i++;
    }
    if (trace.size() != 0) {
        eventLog.addTrace(trace,isValidTrace);
    }
    
}

template<typename ValueType>
boost::optional<std::string> XesParser<ValueType>::traverseEventElement(xercesc::DOMNode const* const node) {
    // traverse children
    for (uint_fast64_t i = 0; i < node->getChildNodes()->getLength(); ++i) {
        auto child = node->getChildNodes()->item(i);
        auto name = storm::adapters::getName(child);
        if (name.compare("string") == 0 && isConceptName(child)) {
            return traverseStringElement(child);
        }
    }
    return boost::none;
}

template<typename ValueType>
std::string XesParser<ValueType>::traverseStringElement(xercesc::DOMNode const* const node) {
    for (uint_fast64_t i = 0; i < node->getAttributes()->getLength(); ++i) {
        auto attr = node->getAttributes()->item(i);
        auto name = storm::adapters::getName(attr);    
        if (name.compare("value") == 0) {
            return storm::adapters::XMLtoString(attr->getNodeValue());
        }
    }      
    STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "No value has been found in the string element in the .xes file");
}

template<typename ValueType>
bool XesParser<ValueType>::isConceptName(xercesc::DOMNode const* const node) {
    for (uint_fast64_t i = 0; i < node->getAttributes()->getLength(); ++i) {
        auto attr = node->getAttributes()->item(i);
        auto name = storm::adapters::getName(attr);    
        if (name.compare("key") == 0 & storm::adapters::XMLtoString(attr->getNodeValue()) == "concept:name") {
            return true;
        }
    }
    return false;
}

template<typename ValueType>
storm::storage::EventLog<ValueType>& XesParser<ValueType>::getEventLog() {
    return this->eventLog;
}

template<typename ValueType>
storm::jani::Model const& XesParser<ValueType>::getModel() {
    return this->model;
}

template class XesParser<double>;
template class XesParser<storm::RationalFunction>;
template class XesParser<storm::RationalNumber>;

} //namespace parser
} //namespace storm