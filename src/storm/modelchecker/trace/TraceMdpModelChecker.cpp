#include "storm/modelchecker/trace/TraceMdpModelChecker.h"
#include "storm/modelchecker/prctl/SparseMdpPrctlModelChecker.h"
#include "storm/modelchecker/CheckTask.h"

#include "storm/utility/FilteredRewardModel.h"
#include "storm/utility/constants.h"
#include "storm/utility/graph.h"
#include "storm/utility/macros.h"
#include "storm/utility/vector.h"

#include "storm/modelchecker/results/ExplicitParetoCurveCheckResult.h"
#include "storm/modelchecker/results/ExplicitQualitativeCheckResult.h"
#include "storm/modelchecker/results/ExplicitQuantitativeCheckResult.h"
#include "storm/modelchecker/results/LexicographicCheckResult.h"

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

#include "storm/solver/SolveGoal.h"

#include "storm/exceptions/InvalidPropertyException.h"
#include "storm/exceptions/InvalidStateException.h"
#include "storm/storage/expressions/Expressions.h"

#include "storm/exceptions/InvalidPropertyException.h"

namespace storm {
namespace modelchecker {

template<typename SparseMdpModelType>
TraceMdpModelChecker<SparseMdpModelType>::TraceMdpModelChecker(SparseMdpModelType const& model) 
    : SparsePropositionalModelChecker<SparseMdpModelType>(model) {
    // Intentionally left empty.
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::check(Environment const& env, std::vector<uint_fast64_t> const trace) {


    storm::storage::SparseMatrixBuilder<ValueType> transitionMatrixBuilderTrace(0, 0, 0, false, true, 0);
    SparseMdpModelType const& model = this->getModel(); 
    auto transitionMatrix = model.getTransitionMatrix();

    std::deque<std::pair<StateType,uint_fast64_t>> statesToExploreTrace;
    for (auto a : model.getInitialStates()) {
        std::pair<StateType,uint_fast64_t> k (a,0);
        statesToExploreTrace.emplace_back(k);
    }

    auto stateLabels = model.getStateLabeling();
    auto deadlocks = stateLabels.getStates("deadl");
    auto data = model.getChoiceOrigins();

    /* for (auto k : deadlocks) {
            std::cout << "deadl : " << k << "\n";
        } */

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
        
        /* std::cout << "\ncurrent index is " << currentIndex << "\n";
        std::cout << "current depth is " << depth << "\n";
        std::cout << "current row group is " << currentRowGroup << "\n"; */

        bool deadl = false;
        for (auto k : deadlocks) {
            if (k == currentIndex) {
                deadl = true;
            }
        }

        transitionMatrixBuilderTrace.newRowGroup(currentRow);

        if(!deadl && !(currentIndex == sinkIndex)) {
            /* std::cout << "not trivial \n"; */
            double totalMass = 0.0;
            for (int i = transitionMatrix.getRowGroupIndices()[currentRowGroup]; i < transitionMatrix.getRowGroupIndices()[currentRowGroup + 1]; i++) {
                totalMass += data->getMass(i);
            }
            /* std::cout << "total Mass is " << totalMass << "\n"; */
            std::map<uint_fast64_t,ValueType> indexToValue;

            for (int i = transitionMatrix.getRowGroupIndices()[currentRowGroup]; i < transitionMatrix.getRowGroupIndices()[currentRowGroup + 1]; i++) {
                double Mass = data->getMass(i);
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
                    /* std::cout << "add next couple " << nextCouple.first << "," << nextCouple.second << " in line " << currentRowGroup << " with proba " << proba * Mass / totalMass <<  "\n"; */
                    //transitionMatrixBuilderTrace.addNextValue(currentRow, coupleToIndex[nextCouple],proba * Mass / totalMass);
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
            /* std::cout << "trivial \n"; */
            transitionMatrixBuilderTrace.addNextValue(currentRow, currentRow,storm::utility::one<ValueType>());
        }
        currentRow++;
    }
    /* for (auto it = coupleToIndex.begin(); it != coupleToIndex.end(); it++) {
        std::cout << "(" << it->first.first << "," << it->first.second << ") -> " << it->second << "\n"; 
    } */


    storm::models::sparse::StateLabeling newStateLabels(transitionMatrixBuilderTrace.getCurrentRowGroupCount());
    newStateLabels.addLabel("init");
    newStateLabels.addLabel("final");
    for (auto it = coupleToIndex.begin(); it != coupleToIndex.end(); it++) {
        uint_fast64_t state = it->first.first;
        if (state != sinkIndex) {
            auto labels = stateLabels.getLabelsOfState(state);
            if (labels.count("init") != 0) {
                newStateLabels.addLabelToState("init",it->second);
            }
            if (labels.count("deadl") != 0 && it->first.second == trace.size()) {
                newStateLabels.addLabelToState("final",it->second);
            }

        }
    }

    storm::storage::sparse::ModelComponents<ValueType, RewardModelType> modelComponents(
        transitionMatrixBuilderTrace.build(0, transitionMatrixBuilderTrace.getCurrentRowGroupCount()), newStateLabels,
        std::unordered_map<std::string, RewardModelType>());


    /* std::cout << modelComponents.transitionMatrix; */

    auto leftFormula = storm::logic::Formula::getTrueFormula();
    auto rightFormula = std::make_shared<storm::logic::AtomicLabelFormula>(storm::logic::AtomicLabelFormula("final"));
    auto formula = std::make_shared<storm::logic::UntilFormula>(storm::logic::UntilFormula(leftFormula, rightFormula));
    storm::solver::OptimizationDirection optimizationDirection = storm::solver::OptimizationDirection::Maximize;
    auto probFormula = storm::logic::ProbabilityOperatorFormula(formula, storm::logic::OperatorInformation(optimizationDirection));
    auto task = storm::modelchecker::CheckTask<storm::logic::Formula, ValueType>(probFormula,true);

    auto mdp = std::make_shared<storm::models::sparse::Mdp<ValueType, RewardModelType>>(std::move(modelComponents));
    storm::modelchecker::SparseMdpPrctlModelChecker<storm::models::sparse::Mdp<ValueType>> modelchecker(*mdp);
    std::unique_ptr<storm::modelchecker::CheckResult> result;
    result = modelchecker.check(env,task);
    auto filter = std::make_unique<storm::modelchecker::ExplicitQualitativeCheckResult>(model.getInitialStates());
    result->filter(filter->asQualitativeCheckResult());
    return result;
}

template class TraceMdpModelChecker<storm::models::sparse::Mdp<double>>;

#ifdef STORM_HAVE_CARL
template class TraceMdpModelChecker<storm::models::sparse::Mdp<storm::RationalNumber>>;
#endif
}  // namespace modelchecker
}  // namespace storm
