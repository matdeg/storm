#include "storm/modelchecker/trace/TraceMdpModelChecker.h"
#include "storm/modelchecker/prctl/SparseMdpPrctlModelChecker.h"
#include "storm/modelchecker/prctl/SparseDtmcPrctlModelChecker.h"
#include "storm/models/sparse/Ctmc.h"
#include "storm/modelchecker/CheckTask.h"

#include "cpphoafparser/consumer/hoa_consumer.hh"

#include "storm/utility/FilteredRewardModel.h"
#include "storm/utility/constants.h"
#include "storm/utility/graph.h"
#include "storm/utility/macros.h"
#include "storm/utility/vector.h"

#include "storm/modelchecker/results/ExplicitParetoCurveCheckResult.h"
#include "storm/modelchecker/results/ExplicitQualitativeCheckResult.h"
#include "storm/modelchecker/results/ExplicitQuantitativeCheckResult.h"
#include "storm/modelchecker/results/LexicographicCheckResult.h"
#include "storm/modelchecker/AbstractModelChecker.h"

#include "storm/logic/FragmentSpecification.h"
#include "storm/logic/Formulas.h"

#include "storm/models/sparse/StandardRewardModel.h"

#include "storm/modelchecker/helper/finitehorizon/SparseNondeterministicStepBoundedHorizonHelper.h"
#include "storm/modelchecker/helper/infinitehorizon/SparseNondeterministicInfiniteHorizonHelper.h"
#include "storm/modelchecker/helper/ltl/SparseLTLHelper.h"
#include "storm/modelchecker/helper/utility/SetInformationFromCheckTask.h"
#include "storm/modelchecker/lexicographic/lexicographicModelChecking.h"
#include "storm/modelchecker/prctl/helper/SparseMdpPrctlHelper.h"

#include "storm/modelchecker/multiobjective/multiObjectiveModelChecking.h"
#include "storm/modelchecker/prctl/helper/rewardbounded/QuantileHelper.h"

#include "storm/storage/EventLog.h"
#include "storm/storage/StronglyConnectedComponentDecomposition.h"

#include "storm/solver/SolveGoal.h"

#include "storm/exceptions/OptionParserException.h"
#include "storm/storage/expressions/Expressions.h"

#include "storm/exceptions/InvalidPropertyException.h"

#include "storm/automata/DeterministicAutomaton.h"

#ifdef STORM_HAVE_SPOT
#include "spot/tl/formula.hh"
#include "spot/tl/parse.hh"
#include "spot/tl/print.hh"
#include "spot/parseaut/public.hh"
#include "spot/twaalgos/neverclaim.hh"
#include "spot/twaalgos/dot.hh"
#include "spot/twaalgos/hoa.hh"
#include "spot/twaalgos/totgba.hh"
#include "spot/twaalgos/translate.hh"
#endif

namespace storm {
namespace modelchecker {

template<typename SparseMdpModelType>
TraceMdpModelChecker<SparseMdpModelType>::TraceMdpModelChecker(SparseMdpModelType const& model) 
    : SparsePropositionalModelChecker<SparseMdpModelType>(model) {
    // Intentionally left empty.
}

template<typename SparseMdpModelType>
std::pair<std::shared_ptr<storm::models::sparse::Dtmc<typename SparseMdpModelType::ValueType>>, std::shared_ptr<storm::logic::Formula>>  TraceMdpModelChecker<SparseMdpModelType>::checkPsl(Environment const& env, std::string stringPsl) {
    spot::parsed_formula pf = spot::parse_infix_psl(stringPsl);
    spot::translator trans;
    trans.set_type(spot::postprocessor::Generic);
    trans.set_pref(spot::postprocessor::Deterministic | spot::postprocessor::Complete | spot::postprocessor::SBAcc);
    spot::twa_graph_ptr aut = trans.run(pf.f);
    std::stringstream autStream;
    spot::print_hoa(autStream, aut, "is");
    storm::automata::DeterministicAutomaton::ptr da = storm::automata::DeterministicAutomaton::parse(autStream);

    storm::storage::SparseMatrixBuilder<ValueType> transitionMatrixBuilderTrace(0, 0, 0, false, true, 0);
    SparseMdpModelType const& model = this->getModel(); 
    auto transitionMatrix = model.getTransitionMatrix();
    uint_fast64_t initialStateAutomaton = da->getInitialState();
    std::deque<std::pair<StateType,uint_fast64_t>> statesToExploreTrace;
    for (auto a : model.getInitialStates()) {
        std::pair<StateType,uint_fast64_t> k (a,initialStateAutomaton);
        statesToExploreTrace.emplace_back(k);
    }

    auto data = model.getChoiceOrigins();
    auto stateLabels = model.getStateLabeling();
    std::vector<uint_fast64_t> deadlocks;
    for (int k = 0; k < transitionMatrix.getRowGroupCount(); k++) {
        if (transitionMatrix.getRowGroupSize(k) == 1) {
            int i = transitionMatrix.getRowGroupIndices()[k];
            if (transitionMatrix.getRow(i).getNumberOfEntries() == 1) {
                if (transitionMatrix.getRow(i).begin()->getColumnValuePair().first == k) {
                    deadlocks.emplace_back(k);
                }
            }
        }
    }
    std::vector<uint_fast64_t> edgeToAction = data->edgeIndexToActionIndex();

    // Now explore the current state until there is no more reachable state.
    uint_fast64_t currentRowGroup = 0;
    uint_fast64_t currentRow = 0;
    uint_fast64_t indexIncrement = 0;

    auto timeOfStart = std::chrono::high_resolution_clock::now();
    auto timeOfLastMessage = std::chrono::high_resolution_clock::now();
    uint64_t numberOfExploredStates = 0;
    uint64_t numberOfExploredStatesSinceLastMessage = 0;
    
    std::map<std::pair<StateType,uint_fast64_t>, uint_fast64_t> coupleToIndex = {{std::make_pair(0,initialStateAutomaton),0}};
    // Perform a search through the model.
    while (!statesToExploreTrace.empty()) {
        // Get the first state in the queue.
        uint_fast64_t automatonIndex = statesToExploreTrace.front().second;
        StateType currentIndex = statesToExploreTrace.front().first;
        currentRowGroup = currentIndex;
        statesToExploreTrace.pop_front();

        bool deadl = false;
        for (auto k : deadlocks) {
            if (k == currentIndex) {
                deadl = true;
            }
        }

        transitionMatrixBuilderTrace.newRowGroup(currentRow);

        if(!deadl) {
            double totalMass = 0;
            for (int i = transitionMatrix.getRowGroupIndices()[currentRowGroup]; i < transitionMatrix.getRowGroupIndices()[currentRowGroup + 1]; i++) {
                totalMass += data->getMass(i).evaluateAsDouble();
            }
            std::map<uint_fast64_t,ValueType> indexToValue;

            for (int i = transitionMatrix.getRowGroupIndices()[currentRowGroup]; i < transitionMatrix.getRowGroupIndices()[currentRowGroup + 1]; i++) {
                double Mass = data->getMass(i).evaluateAsDouble();
                std::string actionName = data->getActionName(i);
                uint_fast64_t actionIndex = edgeToAction[data->testFunction(i)[0]];
                for (auto b : transitionMatrix.getRow(i)) {
                    auto a = b.getColumnValuePair();
                    auto nextStateIndex = a.first;
                    auto proba = a.second;
                    std::pair<StateType,uint_fast64_t> nextCouple;
                    if (actionIndex == 0) {
                        nextCouple = std::make_pair(nextStateIndex,automatonIndex);
                    } else {
                        auto& apSet = da->getAPSet();
                        uint_fast64_t labelIndex = 0;
                        if (apSet.contains(actionName)) {
                            uint_fast64_t indexAP = apSet.getIndex(actionName);
                            labelIndex = 1L << indexAP;
                        } 
                        uint_fast64_t nextAutomatonIndex = da->getSuccessor(automatonIndex,labelIndex);
                        nextCouple = std::make_pair(nextStateIndex,nextAutomatonIndex);
                    }
                    if (coupleToIndex.find(nextCouple) == coupleToIndex.end()) {
                        statesToExploreTrace.emplace_back(nextCouple);
                        coupleToIndex[nextCouple] = ++indexIncrement;
                    }
                    uint_fast64_t index = coupleToIndex[nextCouple];
                    if (indexToValue.count(index) == 0) {
                        indexToValue[index] = proba* Mass / totalMass ;
                    } else {
                        indexToValue[index] = indexToValue[index] + proba* Mass / totalMass;
                    }
                }
            }
            for (const auto& [key, value] : indexToValue) {
                transitionMatrixBuilderTrace.addNextValue(currentRow, key,value);
            }

        } else {
            transitionMatrixBuilderTrace.addNextValue(currentRow, currentRow,storm::utility::one<ValueType>());
        }
        currentRow++;
    }

    transitionMatrix = transitionMatrixBuilderTrace.build(0, transitionMatrixBuilderTrace.getCurrentRowGroupCount());
    storm::models::sparse::StateLabeling newStateLabels(transitionMatrixBuilderTrace.getCurrentRowGroupCount());
    for (std::string s : stateLabels.getLabels()) {
        newStateLabels.addLabel(s);
    }
    for (unsigned int i = 0; i < da->getAcceptance()->getNumberOfAcceptanceSets(); i++) {
        newStateLabels.addLabel(std::to_string(i));
    }
    for (auto it = coupleToIndex.begin(); it != coupleToIndex.end(); it++) {
        uint_fast64_t state = it->first.first;
        uint_fast64_t automatonState = it->first.second;
        auto labels = stateLabels.getLabelsOfState(state);
        for (std::string s : labels) {
            if (s == "init") {
                if (automatonState == initialStateAutomaton) {
                    newStateLabels.addLabelToState(s,it->second);
                }
            } else {
                newStateLabels.addLabelToState(s,it->second);
            }
            
        }
        for (unsigned int i = 0; i < da->getAcceptance()->getNumberOfAcceptanceSets(); i++) {
            if (da->getAcceptance()->getAcceptanceSet(i).get(automatonState)) {
                newStateLabels.addLabelToState(std::to_string(i),it->second);
            }
        }
    }

    storm::storage::StronglyConnectedComponentDecomposition<ValueType> bottomSccs(transitionMatrix, storage::StronglyConnectedComponentDecompositionOptions().onlyBottomSccs().dropNaiveSccs());
    storm::storage::BitVector acceptingStates(transitionMatrix.getRowGroupCount(), false);

    std::size_t checkedBSCCs = 0, acceptingBSCCs = 0, acceptingBSCCStates = 0;
    for (auto& scc : bottomSccs) {
        checkedBSCCs++;
        if (isAccepting(*da->getAcceptance()->getAcceptanceExpression(), newStateLabels, scc)) {
            acceptingBSCCs++;
            for (auto& state : scc) {
                acceptingStates.set(state);
                acceptingBSCCStates++;
            }
        }
    }

    newStateLabels.addLabel("accepting");
    for (auto state : acceptingStates) {
        newStateLabels.addLabelToState("accepting", state);
    }

    storm::storage::sparse::ModelComponents<ValueType, RewardModelType> modelComponents(
        transitionMatrix, newStateLabels,
        std::unordered_map<std::string, RewardModelType>());
    modelComponents.transitionMatrix.makeRowGroupingTrivial();
    auto dtmc = std::make_shared<storm::models::sparse::Dtmc<ValueType, RewardModelType>>(std::move(modelComponents));

    auto acceptingFormula = std::make_shared<storm::logic::AtomicLabelFormula>(storm::logic::AtomicLabelFormula("accepting"));
    auto formula = std::make_shared<storm::logic::EventuallyFormula>(storm::logic::EventuallyFormula(acceptingFormula));
    return std::make_pair(dtmc,formula);

}

template<typename SparseMdpModelType>
bool TraceMdpModelChecker<SparseMdpModelType>::isAccepting(cpphoafparser::HOAConsumer::acceptance_expr& accExpr, storm::models::sparse::StateLabeling const& stateLabeling, const storm::storage::StateBlock& scc) {
    switch (accExpr.getType()) {
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_AND:
            return isAccepting(*accExpr.getLeft(), stateLabeling, scc) && isAccepting(*accExpr.getRight(), stateLabeling, scc);
            break;
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_OR:
            return isAccepting(*accExpr.getLeft(), stateLabeling, scc) || isAccepting(*accExpr.getRight(), stateLabeling, scc);            
            break;
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_NOT:
            return !isAccepting(*accExpr.getLeft(), stateLabeling, scc);
            break;
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_TRUE:
            return true;
            break;
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_FALSE:
            return false;
            break;
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_ATOM:
            const cpphoafparser::AtomAcceptance& atom = accExpr.getAtom();
            storm::storage::BitVector const& acceptanceSet = stateLabeling.getStates(std::to_string(atom.getAcceptanceSet()));
            bool negated = atom.isNegated();
            bool rv;
            switch (atom.getType()) {
                case cpphoafparser::AtomAcceptance::TEMPORAL_INF:
                    rv = false;
                    for (auto& state : scc) {
                        if (acceptanceSet.get(state)) {
                            rv = true;
                            break;
                        }
                    }
                    break;
                case cpphoafparser::AtomAcceptance::TEMPORAL_FIN:
                    rv = true;
                    for (auto& state : scc) {
                        if (acceptanceSet.get(state)) {
                            rv = false;
                            break;
                        }
                    }
                    break;
            }
            return (negated ? !rv : rv);
    }
    STORM_LOG_THROW(false,storm::exceptions::InvalidPropertyException, "unexpected acceptance expression");
}

template<typename SparseMdpModelType>
std::shared_ptr<storm::logic::Formula> TraceMdpModelChecker<SparseMdpModelType>::buildFormulaFromAcceptance(cpphoafparser::HOAConsumer::acceptance_expr& accExpr) {
    switch (accExpr.getType()) {
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_AND:
            return std::make_shared<storm::logic::BinaryBooleanPathFormula>(storm::logic::BinaryBooleanPathFormula(storm::logic::BinaryBooleanOperatorType::And,buildFormulaFromAcceptance(*accExpr.getLeft()),buildFormulaFromAcceptance(*accExpr.getRight())));
            break;
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_OR:
            return std::make_shared<storm::logic::BinaryBooleanPathFormula>(storm::logic::BinaryBooleanPathFormula(storm::logic::BinaryBooleanOperatorType::Or,buildFormulaFromAcceptance(*accExpr.getLeft()),buildFormulaFromAcceptance(*accExpr.getRight())));
            break;
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_NOT:
            return std::make_shared<storm::logic::UnaryBooleanPathFormula>(storm::logic::UnaryBooleanPathFormula(storm::logic::UnaryBooleanOperatorType::Not,buildFormulaFromAcceptance(*accExpr.getLeft())));
            break;
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_TRUE:
            return std::make_shared<storm::logic::BooleanLiteralFormula>(true);
            break;
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_FALSE:
            return std::make_shared<storm::logic::BooleanLiteralFormula>(false);
            break;
        case cpphoafparser::BooleanExpression<cpphoafparser::AtomAcceptance>::OperatorType::EXP_ATOM:
            const cpphoafparser::AtomAcceptance& atom = accExpr.getAtom();
            if (atom.getType() == cpphoafparser::AtomAcceptance::AtomType::TEMPORAL_FIN) {
                auto formulaINFState = std::make_shared<storm::logic::AtomicLabelFormula>(storm::logic::AtomicLabelFormula(std::to_string(atom.getAcceptanceSet())));
                auto formulaFINFState = std::make_shared<storm::logic::EventuallyFormula>(storm::logic::EventuallyFormula(formulaINFState));
                auto formulaFGINFState = std::make_shared<storm::logic::GloballyFormula>(storm::logic::GloballyFormula(formulaFINFState));
                return std::make_shared<storm::logic::UnaryBooleanPathFormula>(storm::logic::UnaryBooleanPathFormula(storm::logic::UnaryBooleanOperatorType::Not,formulaFGINFState));
            } else {
                auto formulaINFState = std::make_shared<storm::logic::AtomicLabelFormula>(storm::logic::AtomicLabelFormula(std::to_string(atom.getAcceptanceSet())));
                auto formulaFINFState = std::make_shared<storm::logic::EventuallyFormula>(storm::logic::EventuallyFormula(formulaINFState));
                return std::make_shared<storm::logic::GloballyFormula>(storm::logic::GloballyFormula(formulaFINFState));
            }
            break;
    }
    STORM_LOG_THROW(false,storm::exceptions::NotSupportedException, "unexpected acceptance expression");
}

template<typename SparseMdpModelType>
std::shared_ptr<storm::models::sparse::Dtmc<typename SparseMdpModelType::ValueType>> TraceMdpModelChecker<SparseMdpModelType>::buildProductAsDtmc(Environment const& env, std::vector<uint_fast64_t> const trace) {


    storm::storage::SparseMatrixBuilder<ValueType> transitionMatrixBuilderTrace(0, 0, 0, false, true, 0);
    SparseMdpModelType const& model = this->getModel(); 
    auto transitionMatrix = model.getTransitionMatrix();
        
    std::deque<std::pair<StateType,uint_fast64_t>> statesToExploreTrace;
    for (auto a : model.getInitialStates()) {
        std::pair<StateType,uint_fast64_t> k (a,0);
        statesToExploreTrace.emplace_back(k);
    }

    auto data = model.getChoiceOrigins();
    auto stateLabels = model.getStateLabeling();
    std::vector<uint_fast64_t> deadlocks;
    for (int k = 0; k < transitionMatrix.getRowGroupCount(); k++) {
        if (transitionMatrix.getRowGroupSize(k) == 1) {
            int i = transitionMatrix.getRowGroupIndices()[k];
            if (transitionMatrix.getRow(i).getNumberOfEntries() == 1) {
                if (transitionMatrix.getRow(i).begin()->getColumnValuePair().first == k) {
                    deadlocks.emplace_back(k);
                }
            }
        }
    }
    std::vector<uint_fast64_t> edgeToAction = data->edgeIndexToActionIndex();

    // Now explore the current state until there is no more reachable state.
    uint_fast64_t currentRowGroup = 0;
    uint_fast64_t currentRow = 0;
    uint_fast64_t indexIncrement = 0;

    auto timeOfStart = std::chrono::high_resolution_clock::now();
    auto timeOfLastMessage = std::chrono::high_resolution_clock::now();
    uint64_t numberOfExploredStates = 0;
    uint64_t numberOfExploredStatesSinceLastMessage = 0;

    
    uint_fast64_t sinkIndex = transitionMatrix.getRowCount();
    std::map<std::pair<StateType,uint_fast64_t>, uint_fast64_t> coupleToIndex = {{std::make_pair(0,0),0}};
    // Perform a search through the model.
    while (!statesToExploreTrace.empty()) {
        // Get the first state in the queue.
        uint_fast64_t depth = statesToExploreTrace.front().second;
        StateType currentIndex = statesToExploreTrace.front().first;
        currentRowGroup = currentIndex;
        statesToExploreTrace.pop_front();

        bool deadl = false;
        for (auto k : deadlocks) {
            if (k == currentIndex) {
                deadl = true;
            }
        }

        transitionMatrixBuilderTrace.newRowGroup(currentRow);

        if(!deadl && !(currentIndex == sinkIndex)) {
            double totalMass = 0.0;
            for (int i = transitionMatrix.getRowGroupIndices()[currentRowGroup]; i < transitionMatrix.getRowGroupIndices()[currentRowGroup + 1]; i++) {
                totalMass += data->getMass(i).evaluateAsDouble();
            }
            std::map<uint_fast64_t,ValueType> indexToValue;

            for (int i = transitionMatrix.getRowGroupIndices()[currentRowGroup]; i < transitionMatrix.getRowGroupIndices()[currentRowGroup + 1]; i++) {
                double Mass = data->getMass(i).evaluateAsDouble();
                uint_fast64_t actionIndex = edgeToAction[data->testFunction(i)[0]];
                for (auto b : transitionMatrix.getRow(i)) {
                    auto a = b.getColumnValuePair();
                    auto nextStateIndex = a.first;
                    auto proba = a.second;
                    std::pair<StateType,uint_fast64_t> nextCouple;
                    if (actionIndex == 0) {
                        nextCouple = std::make_pair(nextStateIndex,depth);
                    } else if (actionIndex == trace[depth]) {
                        nextCouple = std::make_pair(nextStateIndex,depth + 1);
                    } else {
                        nextCouple = std::make_pair(sinkIndex,0);
                    }
                    if (coupleToIndex.find(nextCouple) == coupleToIndex.end()) {
                        statesToExploreTrace.emplace_back(nextCouple);
                        coupleToIndex[nextCouple] = ++indexIncrement;
                    }
                    uint_fast64_t index = coupleToIndex[nextCouple];
                    if (indexToValue.count(index) == 0) {
                        indexToValue[index] = proba* Mass / totalMass ;
                    } else {
                        indexToValue[index] = indexToValue[index] + proba* Mass / totalMass;
                    }
                }
            }
            for (const auto& [key, value] : indexToValue) {
                transitionMatrixBuilderTrace.addNextValue(currentRow, key,value);
            }

        } else {
            transitionMatrixBuilderTrace.addNextValue(currentRow, currentRow,storm::utility::one<ValueType>());
        }
        currentRow++;
    }
    storm::models::sparse::StateLabeling newStateLabels(transitionMatrixBuilderTrace.getCurrentRowGroupCount());
    newStateLabels.addLabel("init");
    newStateLabels.addLabel("final");
    for (auto it = coupleToIndex.begin(); it != coupleToIndex.end(); it++) {
        uint_fast64_t state = it->first.first;
        uint_fast64_t depth = it->first.second;
        if (state != sinkIndex) {
            auto labels = stateLabels.getLabelsOfState(state);
            if (labels.count("init") != 0 && depth == 0) {
                newStateLabels.addLabelToState("init",it->second);
            }
            bool deadl = false;
            for (auto k : deadlocks) {
                if (k == state) {
                    deadl = true;
                }
            }
            if (deadl && it->first.second == trace.size()) {
                newStateLabels.addLabelToState("final",it->second);
            }
        }
    }
    storm::storage::sparse::ModelComponents<ValueType, RewardModelType> modelComponents(
        transitionMatrixBuilderTrace.build(0, transitionMatrixBuilderTrace.getCurrentRowGroupCount()), newStateLabels,
        std::unordered_map<std::string, RewardModelType>());
    modelComponents.transitionMatrix.makeRowGroupingTrivial();
    auto dtmc = std::make_shared<storm::models::sparse::Dtmc<ValueType, RewardModelType>>(std::move(modelComponents));
    return dtmc;
}

template<typename SparseMdpModelType>
std::shared_ptr<storm::models::sparse::Ctmc<typename SparseMdpModelType::ValueType>> TraceMdpModelChecker<SparseMdpModelType>::buildProductAsCtmc(Environment const& env, std::vector<uint_fast64_t> const trace) {


    storm::storage::SparseMatrixBuilder<ValueType> transitionMatrixBuilderTrace(0, 0, 0, false, true, 0);
    SparseMdpModelType const& model = this->getModel(); 
    auto transitionMatrix = model.getTransitionMatrix();
    

    std::deque<std::pair<StateType,uint_fast64_t>> statesToExploreTrace;
    for (auto a : model.getInitialStates()) {
        std::pair<StateType,uint_fast64_t> k (a,0);
        statesToExploreTrace.emplace_back(k);
    }

    auto data = model.getChoiceOrigins();
    auto stateLabels = model.getStateLabeling();
    std::vector<uint_fast64_t> deadlocks;
    for (int k = 0; k < transitionMatrix.getRowGroupCount(); k++) {
        if (transitionMatrix.getRowGroupSize(k) == 1) {
            int i = transitionMatrix.getRowGroupIndices()[k];
            if (transitionMatrix.getRow(i).getNumberOfEntries() == 1) {
                if (transitionMatrix.getRow(i).begin()->getColumnValuePair().first == k) {
                    deadlocks.emplace_back(k);
                }
            }
        }
    }
    std::vector<uint_fast64_t> edgeToAction = data->edgeIndexToActionIndex();

    // Now explore the current state until there is no more reachable state.
    uint_fast64_t currentRowGroup = 0;
    uint_fast64_t currentRow = 0;
    uint_fast64_t indexIncrement = 0;

    auto timeOfStart = std::chrono::high_resolution_clock::now();
    auto timeOfLastMessage = std::chrono::high_resolution_clock::now();
    uint64_t numberOfExploredStates = 0;
    uint64_t numberOfExploredStatesSinceLastMessage = 0;

    
    uint_fast64_t sinkIndex = transitionMatrix.getRowCount();
    std::map<std::pair<StateType,uint_fast64_t>, uint_fast64_t> coupleToIndex = {{std::make_pair(0,0),0}};
    // Perform a search through the model.
    while (!statesToExploreTrace.empty()) {
        // Get the first state in the queue.
        uint_fast64_t depth = statesToExploreTrace.front().second;
        StateType currentIndex = statesToExploreTrace.front().first;
        currentRowGroup = currentIndex;
        statesToExploreTrace.pop_front();

        bool deadl = false;
        for (auto k : deadlocks) {
            if (k == currentIndex) {
                deadl = true;
            }
        }

        transitionMatrixBuilderTrace.newRowGroup(currentRow);

        if(!deadl && !(currentIndex == sinkIndex)) {
            std::map<uint_fast64_t,ValueType> indexToValue;

            for (int i = transitionMatrix.getRowGroupIndices()[currentRowGroup]; i < transitionMatrix.getRowGroupIndices()[currentRowGroup + 1]; i++) {
                double Mass = data->getMass(i).evaluateAsDouble();
                uint_fast64_t actionIndex = edgeToAction[data->testFunction(i)[0]];
                for (auto b : transitionMatrix.getRow(i)) {
                    auto a = b.getColumnValuePair();
                    auto nextStateIndex = a.first;
                    auto proba = a.second;
                    std::pair<StateType,uint_fast64_t> nextCouple;
                    if (actionIndex == 0) {
                        nextCouple = std::make_pair(nextStateIndex,depth);
                    } else if (actionIndex == trace[depth]) {
                        nextCouple = std::make_pair(nextStateIndex,depth + 1);
                    } else {
                        nextCouple = std::make_pair(sinkIndex,0);
                    }
                    if (coupleToIndex.find(nextCouple) == coupleToIndex.end()) {
                        statesToExploreTrace.emplace_back(nextCouple);
                        coupleToIndex[nextCouple] = ++indexIncrement;
                    }
                    uint_fast64_t index = coupleToIndex[nextCouple];
                    if (indexToValue.count(index) == 0) {
                        indexToValue[index] = proba* Mass;
                    } else {
                        indexToValue[index] = indexToValue[index] + proba* Mass;
                    }
                }
            }
            for (const auto& [key, value] : indexToValue) {
                transitionMatrixBuilderTrace.addNextValue(currentRow, key,value);
            }

        } else {
            transitionMatrixBuilderTrace.addNextValue(currentRow, currentRow,storm::utility::one<ValueType>());
        }
        currentRow++;
    }
    storm::models::sparse::StateLabeling newStateLabels(transitionMatrixBuilderTrace.getCurrentRowGroupCount());
    newStateLabels.addLabel("init");
    newStateLabels.addLabel("final");
    for (auto it = coupleToIndex.begin(); it != coupleToIndex.end(); it++) {
        uint_fast64_t state = it->first.first;
        uint_fast64_t depth = it->first.second;
        if (state != sinkIndex) {
            auto labels = stateLabels.getLabelsOfState(state);
            if (labels.count("init") != 0 && depth == 0) {
                newStateLabels.addLabelToState("init",it->second);
            }
            bool deadl = false;
            for (auto k : deadlocks) {
                if (k == state) {
                    deadl = true;
                }
            }
            if (deadl && it->first.second == trace.size()) {
                newStateLabels.addLabelToState("final",it->second);
            }
        }
    }
    storm::storage::sparse::ModelComponents<ValueType, RewardModelType> modelComponents(
        transitionMatrixBuilderTrace.build(0, transitionMatrixBuilderTrace.getCurrentRowGroupCount()), newStateLabels,
        std::unordered_map<std::string, RewardModelType>(),true);
    modelComponents.transitionMatrix.makeRowGroupingTrivial();
    auto ctmc = std::make_shared<storm::models::sparse::Ctmc<ValueType, RewardModelType>>(std::move(modelComponents));
    return ctmc;
}

template<typename SparseMdpModelType>
std::shared_ptr<storm::models::sparse::Ctmc<typename SparseMdpModelType::ValueType>> TraceMdpModelChecker<SparseMdpModelType>::buildAsCtmc(Environment const& env) {


    storm::storage::SparseMatrixBuilder<ValueType> transitionMatrixBuilderTrace(0, 0, 0, false, true, 0);
    SparseMdpModelType const& model = this->getModel(); 
    auto transitionMatrix = model.getTransitionMatrix();

    auto data = model.getChoiceOrigins();
    std::vector<uint_fast64_t> edgeToAction = data->edgeIndexToActionIndex();

    std::vector<uint_fast64_t> deadlocks;
    for (int k = 0; k < transitionMatrix.getRowGroupCount(); k++) {
        if (transitionMatrix.getRowGroupSize(k) == 1) {
            int i = transitionMatrix.getRowGroupIndices()[k];
            if (transitionMatrix.getRow(i).getNumberOfEntries() == 1) {
                if (transitionMatrix.getRow(i).begin()->getColumnValuePair().first == k) {
                    deadlocks.emplace_back(k);
                }
            }
        }
    }

    // Now explore the current state until there is no more reachable state.
    uint_fast64_t currentRowGroup = 0;
    uint_fast64_t currentRow = 0;
    uint_fast64_t indexIncrement = 0;

    for (uint_fast64_t k = 0; k < transitionMatrix.getRowGroupCount(); k++) {
        transitionMatrixBuilderTrace.newRowGroup(currentRow);
        bool deadl = false;
        for (auto i : deadlocks) {
            if (k == i) {
                deadl = true;
            }
        }
        if (!deadl) {
            std::map<uint_fast64_t,ValueType> indexToValue;
            for (int i = transitionMatrix.getRowGroupIndices()[k]; i < transitionMatrix.getRowGroupIndices()[k + 1]; i++) {
                double Mass = data->getMass(i).evaluateAsDouble();
                for (auto b : transitionMatrix.getRow(i)) {
                    auto a = b.getColumnValuePair();
                    auto index = a.first;
                    if (indexToValue.count(index) == 0) {
                        indexToValue[index] = a.second * Mass;
                    } else {
                        indexToValue[index] = indexToValue[index] + a.second * Mass;
                    }
                }
            }
            for (const auto& [key, value] : indexToValue) {
                transitionMatrixBuilderTrace.addNextValue(currentRow, key,value);
            }
        } else {
            transitionMatrixBuilderTrace.addNextValue(currentRow, currentRow, storm::utility::one<ValueType>());
        }
        currentRow++;
    }

    auto stateLabels = model.getStateLabeling();
    storm::storage::sparse::ModelComponents<ValueType, RewardModelType> modelComponents(
        transitionMatrixBuilderTrace.build(0, transitionMatrixBuilderTrace.getCurrentRowGroupCount()), stateLabels,
        std::unordered_map<std::string, RewardModelType>(),true);

    modelComponents.transitionMatrix.makeRowGroupingTrivial();
    auto ctmc = std::make_shared<storm::models::sparse::Ctmc<ValueType, RewardModelType>>(std::move(modelComponents));
    return ctmc;
}

template class TraceMdpModelChecker<storm::models::sparse::Mdp<double>>;

#ifdef STORM_HAVE_CARL
template class TraceMdpModelChecker<storm::models::sparse::Mdp<storm::RationalNumber>>;
#endif
}  // namespace modelchecker
}  // namespace storm
