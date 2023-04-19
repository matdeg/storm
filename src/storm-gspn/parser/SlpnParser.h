#pragma once

#include "storm-config.h"
#include <string>

#include "storm-parsers/parser/ExpressionParser.h"
#include "storm/storage/expressions/ExpressionManager.h"

#include "storm-gspn/storage/gspn/GSPN.h"

#include "storm-gspn/storage/gspn/GspnBuilder.h"

namespace storm {
namespace parser {
class SlpnParser {
   public:

    SlpnParser(std::string const& constantDefinitionString);

    /*!
     * Parses the given file into the GSPN.
     *
     * @param filename The name of the file to parse.
     * @return The resulting GSPN.
     */
    storm::gspn::GSPN* parse(std::string const& filename);
    void traverseMarkings(std::string const& filename, std::ifstream& o);
    void traverseTransitions(std::string const& filename, std::ifstream& o);
    void traverseTransition(std::string const& filename, std::ifstream& o);

    int getNumberPlaces();
    void setNumberPlaces(int numberPlaces);
    int getNumberTransitions();
    void setNumberTransitions(int numberTransitions);
    int getNumberInputPlaces();
    void setNumberInputPlaces(int numberInputPlaces);
    int getNumberOutputPlaces();
    void setNumberOutputPlaces(int numberOutputPlaces);
    int getTransitionID();
    void incrementTransitionID();


   private:
    int numberPlaces;
    int numberTransitions;
    int numberInputPlaces;
    int numberOutputPlaces;
    int currentTransitionID = 0;

    // the constructed gspn
    storm::gspn::GspnBuilder builder;
    std::shared_ptr<storm::expressions::ExpressionManager> manager;
    std::shared_ptr<storm::parser::ExpressionParser> expressionParser;
    std::unordered_map<std::string, std::string> constantDefinitions;
    std::map<storm::expressions::Variable, storm::expressions::Expression> constantsSubstitution;
};
} //namespace parser
} //namespace storm