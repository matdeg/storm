#include <vector>
#include "storm/storage/jani/Property.h"
#include "storm/storage/jani/Model.h"
#include "storm/storage/EventLog.h"
#include "storm/storage/Trace.h"

namespace storm {
namespace storage {

EventLog::EventLog() {}
EventLog::EventLog(storm::jani::Model const& model) : model(model) {}

void EventLog::addTrace(storm::storage::Trace trace) {
    this->traces.emplace_back(trace);
}

storm::storage::Trace EventLog::getTrace(uint_fast64_t k) {
    return this->traces[k];
}

storm::jani::Model EventLog::getModel() {
    return this->model;
}

uint_fast64_t EventLog::size() {
    return traces.size();
}
std::vector<storm::jani::Property> EventLog::getProperties() {
    return this->standardProperties;
}

void EventLog::updateModel() {

    std::cout << "flag 0\n";
    auto & expressionManager = model.getManager();
    for (storm::storage::Trace trace : traces) {
        std::cout << "flag 0\n";
        uint_fast64_t id = trace.getID();
        std::cout << "flag 1\n";
        std::string idStr = std::to_string(id);
        std::cout << "flag 0\n";
        storm::expressions::Variable finalExpression = expressionManager.declareBooleanVariable("final_" + idStr);
        std::cout << "flag 4\n";
        std::shared_ptr<storm::jani::Variable> janiVar = storm::jani::Variable::makeBooleanVariable("final_" + idStr, finalExpression, expressionManager.boolean(false),false);
        storm::jani::Variable const& finalVar = model.addVariable(*janiVar);
        std::cout << "flag 8\n";
        storm::jani::Automaton automaton("trace_automaton_" + idStr,expressionManager.declareIntegerVariable("loc_0_" + idStr));
        for (uint_fast64_t i = 0; i <= trace.size(); i++) {
            automaton.addLocation(storm::jani::Location("loc_" + std::to_string(i) + "_" + idStr));
        }
        std::cout << "flag 0\n";
        automaton.addLocation(storm::jani::Location("loc_sink_" + idStr));
        automaton.addInitialLocation("loc_0_" + idStr);

        storm::expressions::Expression guard = expressionManager.boolean(true);
        std::vector<storm::expressions::Expression> probabilities;
        probabilities.emplace_back(expressionManager.rational(1.0));
        std::cout << "flag 9\n";
        std::vector<storm::jani::Assignment> assignments;
        uint_fast64_t locIndexSink = automaton.getLocationIndex("loc_sink_" + idStr);
        std::vector<uint64_t> destinationLocationsSink;
        destinationLocationsSink.emplace_back(locIndexSink);


        for (uint_fast64_t i = 0; i <= trace.size(); i++) {
            // event transition
            uint_fast64_t actionIndex = -1;
            uint_fast64_t locIndex = automaton.getLocationIndex("loc_" + std::to_string(i) + "_" + idStr);
            std::vector<uint64_t> destinationLocations;

            if (i < trace.size()) {
                actionIndex = trace.get()[i];
                destinationLocations.emplace_back(automaton.getLocationIndex("loc_" + std::to_string(i+1) + "_" + idStr));
                if (i+1 == trace.size()) {
                    assignments.emplace_back(storm::jani::LValue(finalVar),expressionManager.boolean(true));
                }
                std::shared_ptr<storm::jani::TemplateEdge> templateEdge = std::make_shared<storm::jani::TemplateEdge>(guard.simplify());
                automaton.registerTemplateEdge(templateEdge);
                templateEdge->addDestination(storm::jani::TemplateEdgeDestination(assignments));
                storm::jani::Edge e(locIndex, actionIndex, boost::none, templateEdge, destinationLocations, probabilities);
                automaton.addEdge(e);
                assignments.clear();
            }
            
            // sink transition
            assignments.emplace_back(storm::jani::LValue(finalVar),expressionManager.boolean(false));
            for (uint_fast64_t actionIndexToSink : model.getNonsilentActionIndices()) {
                if (actionIndexToSink != actionIndex) {
                    std::shared_ptr<storm::jani::TemplateEdge> templateEdgeSink = std::make_shared<storm::jani::TemplateEdge>(guard.simplify());
                    automaton.registerTemplateEdge(templateEdgeSink);
                    templateEdgeSink->addDestination(storm::jani::TemplateEdgeDestination(assignments));
                    storm::jani::Edge eSink(locIndex, actionIndexToSink, boost::none, templateEdgeSink, destinationLocationsSink, probabilities);
                    automaton.addEdge(eSink);
                }
            }
            assignments.clear();
        }
        model.addAutomaton(automaton);

        auto finalFormula = std::make_shared<storm::logic::AtomicExpressionFormula>(finalVar.getExpressionVariable().getExpression() && model.getGlobalVariable("deadl").getExpressionVariable().getExpression());
        storm::solver::OptimizationDirection optimizationDirection = storm::solver::OptimizationDirection::Maximize;
        std::set<storm::expressions::Variable> emptySet;

        // Build reachability property
        auto reachFormula = std::make_shared<storm::logic::ProbabilityOperatorFormula>(
            std::make_shared<storm::logic::EventuallyFormula>(finalFormula, storm::logic::FormulaContext::Probability),
            storm::logic::OperatorInformation(optimizationDirection));
        standardProperties.emplace_back("MaxPrReachFinal", reachFormula, emptySet,
                                        "The maximal probability to eventually reach a final state.");
    }
}

   

}  // namespace storage
}  // namespace storm