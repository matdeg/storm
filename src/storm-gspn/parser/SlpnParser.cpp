#include "SlpnParser.h"

#include <algorithm>
#include <iostream>

#include "storm/exceptions/NotSupportedException.h"
#include "storm/exceptions/UnexpectedException.h"
#include "storm/exceptions/WrongFormatException.h"
#include "storm/utility/macros.h"
#include "storm/io/file.h"

namespace storm {
namespace parser {

SlpnParser::SlpnParser(std::string const& constantDefinitionString)
    : manager(std::make_shared<storm::expressions::ExpressionManager>()) {
    if (constantDefinitionString != "") {
        std::vector<std::string> constDefs;
        boost::split(constDefs, constantDefinitionString, boost::is_any_of(","));
        for (auto const& pair : constDefs) {
            std::vector<std::string> keyvaluepair;
            boost::split(keyvaluepair, pair, boost::is_any_of("="));
            STORM_LOG_THROW(keyvaluepair.size() == 2, storm::exceptions::WrongFormatException,
                            "Expected a constant definition of the form N=42 but got " << pair);
            constantDefinitions.emplace(keyvaluepair.at(0), keyvaluepair.at(1));
        }
    }
}

int SlpnParser::getNumberPlaces() {
    return this->numberPlaces;
}

void SlpnParser::setNumberPlaces(int numberPlaces) {
    this->numberPlaces = numberPlaces;
}

int SlpnParser::getNumberTransitions() {
    return this->numberTransitions;
}

void SlpnParser::setNumberTransitions(int numberTransitions) {
    this->numberTransitions = numberTransitions;
}

int SlpnParser::getNumberInputPlaces() {
    return this->numberInputPlaces;
}

void SlpnParser::setNumberInputPlaces(int numberInputPlaces) {
    this->numberInputPlaces = numberInputPlaces;
}

int SlpnParser::getNumberOutputPlaces() {
    return this->numberOutputPlaces;
}

void SlpnParser::setNumberOutputPlaces(int numberOutputPlaces) {
    this->numberOutputPlaces = numberOutputPlaces;
}

int SlpnParser::getTransitionID() {
    return this->currentTransitionID;
}

void SlpnParser::incrementTransitionID() {
    this->currentTransitionID++;
}

storm::gspn::GSPN* SlpnParser::parse(std::string const& filename) {
    std::ifstream o(filename);
    std::string l;
    if (o) {
        storm::utility::getline(o,l);
        if (!(l == "# number of places")) {
            STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Wrong format for the file " + filename + ", expected line : \"# number of places\"\n");
        } 
        storm::utility::getline(o,l);
        setNumberPlaces(std::stoi(l));
        traverseMarkings(filename,o);
        builder.setGspnName("Gspn_Name");
        return builder.buildGspn(manager,constantsSubstitution);
    } else {
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Can't read the file " + filename + "\n");
    }
}

void SlpnParser::traverseMarkings(std::string const& filename, std::ifstream& o) {
    std::string l;
    storm::utility::getline(o,l);
    if (!(l == "# initial marking")) {
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Wrong format for the file " + filename + ", expected line : \"# initial marking\"\n");
    }
    for (int i = 0;i < SlpnParser::getNumberPlaces();i++) {
        storm::utility::getline(o,l);
        builder.addPlace(boost::none, std::stoi(l), "P" + std::to_string(i));
    }
    traverseTransitions(filename,o);
}

void SlpnParser::traverseTransitions(std::string const& filename, std::ifstream& o) {
    std::string l;
    storm::utility::getline(o,l);
    if (!(l == "# number of transitions")) {
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Wrong format for the file " + filename + ", expected line : \"# number of transitions\"\n");
    } 
    storm::utility::getline(o,l);
    setNumberTransitions(std::stoi(l));
    traverseTransition(filename,o);
}

void SlpnParser::traverseTransition(std::string const& filename, std::ifstream& o) {
    std::string l;
    for (int j = 0; j < getNumberTransitions();j++) {
        double weight;
        std::string tag;
        storm::utility::getline(o,l);
        if (!(l == "# transition " + std::to_string(getTransitionID()))) {
            STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Wrong format for the file " + filename + ", expected line : \"# transition " + std::to_string(getTransitionID()) + "\"\n");
        } 
        storm::utility::getline(o,l);
        if (l.substr(0,5) == "label") {
            tag = l.substr(6,l.length()-6);
        } else if (l == "silent") {
            tag = "";
        } else {
            STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Wrong format for the file " + filename + ", expected transition " + std::to_string(getTransitionID()) + " label to be \"silent\" or \"label [something]\"\n");
        }

        storm::utility::getline(o,l);
        if (!(l == "# weight ")) {
            STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Wrong format for the file " + filename + ", expected line : \"# weight\"\n");
        } 
        storm::utility::getline(o,l);
        weight = std::stod(l);
        builder.addImmediateTransition(1, weight, "T" + std::to_string(j), tag);

        storm::utility::getline(o,l);
        if (!(l == "# number of input places")) {
            STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Wrong format for the file " + filename + ", expected line : \"# number of input places\"\n");
        } 
        storm::utility::getline(o,l);
        SlpnParser::setNumberInputPlaces(std::stoi(l));
        for (int i = 0;i < SlpnParser::getNumberInputPlaces();i++) {
            storm::utility::getline(o,l);
            builder.addInputArc("P" + l, "T" + std::to_string(j), 1);
        }

        storm::utility::getline(o,l);
        if (!(l == "# number of output places")) {
            STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Wrong format for the file " + filename + ", expected line : \"# number of output places\"\n");
        } 
        storm::utility::getline(o,l);
        SlpnParser::setNumberOutputPlaces(std::stoi(l));
        for (int i = 0;i < getNumberOutputPlaces();i++) {
            storm::utility::getline(o,l);
            builder.addOutputArc("T" + std::to_string(j), "P" + l, 1);
        }
        incrementTransitionID();
    }
}

} //namespace parser
} //namespace storm