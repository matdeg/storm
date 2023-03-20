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

bool EventLog::smaller(storm::storage::Trace trace1, storm::storage::Trace trace2){
    auto a = trace1.get();
    auto b = trace2.get();
    int na = a.size();
    int nb = b.size();
    int l = (na<nb)?na:nb;
    for (int i = 0; i < l; i++) {
        if (a[i] < b[i]) {
            return true;
        } else if (a[i] > b[i]) {
            return false;
        }
    }
    return (na<nb)?true:false;
}

storm::jani::Model EventLog::getModel() {
    return this->model;
}

void EventLog::setCuts() {
    std::cout << size() << "\n";
    std::vector<uint_fast64_t> cuts;
    cuts.emplace_back(0);
    uint_fast64_t actionTemp = traces[0].get()[0];
    uint_fast64_t action;
    for (uint_fast64_t nTrace = 0; nTrace < size(); nTrace++) {
        action = traces[nTrace].get()[0];
        if (action != actionTemp){
            actionTemp = action;
            cuts.emplace_back(nTrace);
        }
    }
    cuts.emplace_back(size()+1);
    bool canReduce = cuts.size() > 2;
    uint_fast64_t n = cuts.size();
    uint_fast64_t k = 0;
    while (k < n-2) {
        if (cuts[k+2] - cuts[k] < minAutomata) {
            cuts.erase(cuts.begin() + k+1);
            n--;
        } else {
            k++;
        }
    }
    for (auto k : cuts) {
        std::cout << k << ", ";
    }
    std::cout << "\n";
}

uint_fast64_t EventLog::size() {
    return traces.size();
}
std::vector<storm::jani::Property> EventLog::getProperties() {
    return this->standardProperties;
}

void EventLog::initialize() {
    std::sort (traces.begin(), traces.end(), smaller);
    setCuts(); 
}

void EventLog::updateModel(uint_fast_64 t) {
    storm::jani::Automaton automaton("trace_automaton_" + std::to_string(t),expressionManager.declareIntegerVariable("loc_0_" + std::to_string(t)));
    automaton.addLocation(storm::jani::Location("loc_sink_" + std::to_string(t)));
    uint_fast64_t sinkIndex = automaton.getLocation("loc_sink_" + std::to_string(t));
    uint_fast64_t start = cuts[t];
    uint_fast64_t end = cuts[t+1]-1;
    auto & expressionManager = model.getManager();
    for (uint_fast64_t nTrace = start; nTrace <= end; nTrace++) {
        storm::storage::Trace trace = getTrace(nTrace);
        uint_fast64_t current_loc = automaton.getLocationIndex("loc_0_" + to_string(t));
        uint_fast64_t n = trace.size();
        for (uint_fast64_t i = 0; i < n,i++) {
            actionIndex = trace.get()[i];
            auto next_loc_set = automaton.getEdgesFromLocation(current_loc, actionIndex);
            if (next_loc_set.empty()) {
                std::string next_loc_string = "loc_" + std::to_string(current_loc) + "_" + std::to_string(actionIndex);
                automaton.addLocation(storm::jani::Location(next_loc_string));
                auto next_loc = automaton.getLocationIndex(next_loc_string);
                storm::expressions::Expression guard = expressionManager.boolean(true);
                std::vector<storm::expressions::Expression> probabilities;
                probabilities.emplace_back(expressionManager.rational(1.0));
                std::vector<storm::jani::Assignment> assignments;
                std::vector<uint64_t> destinationLocations;
                destinationLocations.emplace_back(next_loc);
                std::shared_ptr<storm::jani::TemplateEdge> templateEdge = std::make_shared<storm::jani::TemplateEdge>(guard.simplify());
                automaton.registerTemplateEdge(templateEdge);
                templateEdge->addDestination(storm::jani::TemplateEdgeDestination(assignments));
                storm::jani::Edge e(current_loc, actionIndex, boost::none, templateEdge, destinationLocations, probabilities);
                automaton.addEdge(e);
            } else if (next_loc_set.size() == 1) {
                storm::jani::Edge edge = *(next_loc_set.begin());
                storm::jani::std::vector<storm::jani::EdgeDestination> edgeDestinations = edge.getDestinations();
                if (edgeDestinations.size() == 1) {
                    storm::jani::EdgeDestination dest = edgeDestinations.front();
                    current_loc = dest.getLocationIndex();
                } else {
                    STORM_LOG_THROW(false, storm::exceptions::BaseException,"Edges should have only one destination");
                }
            } else {
                STORM_LOG_THROW(false, storm::exceptions::BaseException,"Non determinism of the built automaton");
            }
        }
        uint_fast64_t id = trace.getID();
        std::string idStr = std::to_string(id);
        storm::expressions::Variable finalExpression = expressionManager.declareBooleanVariable("final_" + idStr);
        std::shared_ptr<storm::jani::Variable> janiVar = storm::jani::Variable::makeBooleanVariable("final_" + idStr, finalExpression, expressionManager.boolean(false),false);
        storm::jani::Variable const& finalVar = model.addVariable(*janiVar);
        storm::jani::Assignment assign = storm::jani::LValue(finalVar),expressionManager.boolean(true);
        automaton.getLocation(current_loc).addTransientAssignment(assign);
    }
    // this loop could be changed using map from location to index or sth
    for (auto location : automaton.getLocations()) {
        uint_fast64_t locationIndex = automaton.getLocationIndex(location.getName());
        for (uint_fast64_t actionIndex : model.getNonsilentActionIndices()) {
            if (automaton.getEdgesFromLocation(locationIndex, actionIndex).empty()) {
                storm::expressions::Expression guard = expressionManager.boolean(true);
                std::vector<storm::expressions::Expression> probabilities;
                probabilities.emplace_back(expressionManager.rational(1.0));
                std::vector<storm::jani::Assignment> assignments;
                std::vector<uint64_t> destinationLocations;
                destinationLocations.emplace_back(sinkIndex);
                std::shared_ptr<storm::jani::TemplateEdge> templateEdge = std::make_shared<storm::jani::TemplateEdge>(guard.simplify());
                automaton.registerTemplateEdge(templateEdge);
                templateEdge->addDestination(storm::jani::TemplateEdgeDestination(assignments));
                storm::jani::Edge e(locationIndex, actionIndex, boost::none, templateEdge, destinationLocations, probabilities);
                automaton.addEdge(e);
            }
        }
    }

    model.addAutomaton(automaton);

    auto leftFormula = std::make_shared<storm::logic::AtomicExpressionFormula>(!model.getGlobalVariable("deadl").getExpressionVariable().getExpression());
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

   

}  // namespace storage
}  // namespace storm