#include <vector>
#include "storm/storage/jani/Property.h"
#include "storm/storage/jani/Model.h"
#include "storm/storage/Trace.h"
#include "storm/logic/BinaryBooleanOperatorType.h"

namespace storm {
namespace storage {

Trace::Trace(storm::jani::Model model, uint_fast64_t id) : model(model), id(id) {}
Trace::Trace(std::vector<uint_fast64_t> const& trace) : trace(trace) {}

void Trace::addEvent(uint_fast64_t k) {
    this->trace.emplace_back(k);
}

std::vector<uint_fast64_t> Trace::get() {
    return this->trace;
}

storm::jani::Model Trace::getModel() {
    return this->model;
}

uint_fast64_t Trace::size() {
    return get().size();
}

void Trace::setEmptyActionIndex(uint_fast64_t k) {
    this->emptyActionIndex = k;
}

std::vector<storm::jani::Property> Trace::getProperty() {
    return this->standardProperties;
}


void Trace::updateModel() {

    auto & expressionManager = model.getManager();

    storm::expressions::Variable finalExpression = expressionManager.declareBooleanVariable("final_" + std::to_string(id));
    std::shared_ptr<storm::jani::Variable> janiFinalVar = storm::jani::Variable::makeBooleanVariable("final_" + std::to_string(id), finalExpression, expressionManager.boolean(false),true);
    storm::jani::Variable const& finalVar = model.addVariable(*janiFinalVar);
    storm::expressions::Variable sinkExpression = expressionManager.declareBooleanVariable("sink_" + std::to_string(id));
    std::shared_ptr<storm::jani::Variable> janiSinkVar = storm::jani::Variable::makeBooleanVariable("sink_" + std::to_string(id), sinkExpression, expressionManager.boolean(false),true);
    storm::jani::Variable const& sinkVar = model.addVariable(*janiSinkVar);

    storm::jani::Automaton automaton("trace_automaton",expressionManager.declareIntegerVariable("loc_0_" + std::to_string(id)));
    for (uint_fast64_t i = 0; i <= size(); i++) {
        automaton.addLocation(storm::jani::Location("loc_" + std::to_string(i)));
    }
    automaton.addLocation(storm::jani::Location("loc_sink"));
    automaton.addInitialLocation("loc_0");

    storm::expressions::Expression guard = expressionManager.boolean(true);
    std::vector<storm::expressions::Expression> probabilities;
    probabilities.emplace_back(expressionManager.rational(1.0));
    std::vector<storm::jani::Assignment> assignments;
    
    uint_fast64_t locIndexSink = automaton.getLocationIndex("loc_sink");
    std::vector<uint64_t> destinationLocationsSink;
    destinationLocationsSink.emplace_back(locIndexSink);
    assignments.emplace_back(storm::jani::LValue(sinkVar),expressionManager.boolean(true));
    automaton.getLocation(locIndexSink).addTransientAssignment(assignments.front());
    assignments.clear();

    auto last_index = automaton.getLocationIndex("loc_" + std::to_string(size()));
    assignments.emplace_back(storm::jani::LValue(finalVar),expressionManager.boolean(true));
    automaton.getLocation(last_index).addTransientAssignment(assignments.front());
    assignments.clear();
    for (uint_fast64_t i = 0; i <= size(); i++) {
        // event transition
        uint_fast64_t actionIndex = -1;
        uint_fast64_t locIndex = automaton.getLocationIndex("loc_" + std::to_string(i));
        std::vector<uint64_t> destinationLocations;

        if (i < size()) {
            actionIndex = get()[i];
            destinationLocations.emplace_back(automaton.getLocationIndex("loc_" + std::to_string(i+1)));
            std::shared_ptr<storm::jani::TemplateEdge> templateEdge = std::make_shared<storm::jani::TemplateEdge>(guard.simplify());
            automaton.registerTemplateEdge(templateEdge);
            templateEdge->addDestination(storm::jani::TemplateEdgeDestination(assignments));
            storm::jani::Edge e(locIndex, actionIndex, boost::none, templateEdge, destinationLocations, probabilities);
            automaton.addEdge(e);
        }

        /*
        // self-loop transition
        setEmptyActionIndex(getModel().getActionIndex(" "));
        std::vector<uint64_t> destinationLocationsLoop;
        destinationLocationsLoop.emplace_back(locIndex);
        if (i == size()) {
            assignments.emplace_back(storm::jani::LValue(finalVar),expressionManager.boolean(true));
        }
        std::shared_ptr<storm::jani::TemplateEdge> templateEdgeLoop = std::make_shared<storm::jani::TemplateEdge>(guard.simplify());
        automaton.registerTemplateEdge(templateEdgeLoop);
        templateEdgeLoop->addDestination(storm::jani::TemplateEdgeDestination(assignments));
        storm::jani::Edge eLoop(locIndex, emptyActionIndex, boost::none, templateEdgeLoop, destinationLocationsLoop, probabilities);
        automaton.addEdge(eLoop);
        assignments.clear();
        */

        // sink transition
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
    /*
    // sink self-loop
    for (auto actionIndex : model.getNonsilentActionIndices()) {
        std::shared_ptr<storm::jani::TemplateEdge> templateEdgeSink = std::make_shared<storm::jani::TemplateEdge>(guard.simplify());
        automaton.registerTemplateEdge(templateEdgeSink);
        templateEdgeSink->addDestination(storm::jani::TemplateEdgeDestination(assignments));
        storm::jani::Edge eSink(locIndexSink, actionIndex, boost::none, templateEdgeSink, destinationLocationsSink, probabilities);
        automaton.addEdge(eSink);
    }*/
    model.addAutomaton(automaton);

    auto leftFormula = std::make_shared<storm::logic::AtomicExpressionFormula>(!sinkVar.getExpressionVariable().getExpression());
    auto rightFormula = std::make_shared<storm::logic::AtomicExpressionFormula>(finalVar.getExpressionVariable().getExpression() && model.getGlobalVariable("deadl").getExpressionVariable().getExpression());
    storm::solver::OptimizationDirection optimizationDirection = storm::solver::OptimizationDirection::Maximize;
    std::set<storm::expressions::Variable> emptySet;

    // Build reachability property
    auto reachFormula = std::make_shared<storm::logic::ProbabilityOperatorFormula>(
        std::make_shared<storm::logic::UntilFormula>(leftFormula, rightFormula),
        storm::logic::OperatorInformation(optimizationDirection));
    standardProperties.emplace_back("MaxPrReachFinal", reachFormula, emptySet,
                                    "The maximal probability to eventually reach a final state.");
}

   

}  // namespace storage
}  // namespace storm