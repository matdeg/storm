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
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::check2(Environment const& env, std::vector<uint_fast64_t> const trace) {


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


template<typename SparseMdpModelType>
bool TraceMdpModelChecker<SparseMdpModelType>::canHandleStatic(CheckTask<storm::logic::Formula, ValueType> const& checkTask,
                                                                     bool* requiresSingleInitialState) {
    storm::logic::Formula const& formula = checkTask.getFormula();
    if (formula.isInFragment(storm::logic::prctlstar()
                                 .setLongRunAverageRewardFormulasAllowed(true)
                                 .setLongRunAverageProbabilitiesAllowed(true)
                                 .setConditionalProbabilityFormulasAllowed(true)
                                 .setOnlyEventuallyFormuluasInConditionalFormulasAllowed(true)
                                 .setTotalRewardFormulasAllowed(true)
                                 .setRewardBoundedUntilFormulasAllowed(true)
                                 .setRewardBoundedCumulativeRewardFormulasAllowed(true)
                                 .setMultiDimensionalBoundedUntilFormulasAllowed(true)
                                 .setMultiDimensionalCumulativeRewardFormulasAllowed(true)
                                 .setTimeOperatorsAllowed(true)
                                 .setReachbilityTimeFormulasAllowed(true)
                                 .setRewardAccumulationAllowed(true))) {
        return true;
    } else if (checkTask.isOnlyInitialStatesRelevantSet()) {
        auto multiObjectiveFragment = storm::logic::multiObjective()
                                          .setTimeAllowed(true)
                                          .setCumulativeRewardFormulasAllowed(true)
                                          .setTimeBoundedCumulativeRewardFormulasAllowed(true)
                                          .setStepBoundedCumulativeRewardFormulasAllowed(true)
                                          .setRewardBoundedCumulativeRewardFormulasAllowed(true)
                                          .setTimeBoundedUntilFormulasAllowed(true)
                                          .setStepBoundedUntilFormulasAllowed(true)
                                          .setRewardBoundedUntilFormulasAllowed(true)
                                          .setMultiDimensionalBoundedUntilFormulasAllowed(true)
                                          .setMultiDimensionalCumulativeRewardFormulasAllowed(true)
                                          .setRewardAccumulationAllowed(true);
        auto lexObjectiveFragment = storm::logic::lexObjective()
                                        .setHOAPathFormulasAllowed(true)
                                        .setCumulativeRewardFormulasAllowed(true)
                                        .setTimeBoundedCumulativeRewardFormulasAllowed(true)
                                        .setStepBoundedCumulativeRewardFormulasAllowed(true)
                                        .setRewardBoundedCumulativeRewardFormulasAllowed(true)
                                        .setTimeBoundedUntilFormulasAllowed(true)
                                        .setStepBoundedUntilFormulasAllowed(true)
                                        .setRewardBoundedUntilFormulasAllowed(true)
                                        .setMultiDimensionalBoundedUntilFormulasAllowed(true)
                                        .setMultiDimensionalCumulativeRewardFormulasAllowed(true)
                                        .setRewardAccumulationAllowed(true);

        if (formula.isInFragment(multiObjectiveFragment) || formula.isInFragment(storm::logic::quantiles()) || formula.isInFragment(lexObjectiveFragment)) {
            if (requiresSingleInitialState) {
                *requiresSingleInitialState = true;
            }
            return true;
        }
    }
    return false;
}

template<typename SparseMdpModelType>
bool TraceMdpModelChecker<SparseMdpModelType>::canHandle(CheckTask<storm::logic::Formula, ValueType> const& checkTask) const {
    bool requiresSingleInitialState = false;
    if (canHandleStatic(checkTask, &requiresSingleInitialState)) {
        return !requiresSingleInitialState || this->getModel().getInitialStates().getNumberOfSetBits() == 1;
    } else {
        return false;
    }
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeBoundedUntilProbabilities(
    Environment const& env, CheckTask<storm::logic::BoundedUntilFormula, ValueType> const& checkTask) {
    storm::logic::BoundedUntilFormula const& pathFormula = checkTask.getFormula();
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    if (pathFormula.isMultiDimensional() || pathFormula.getTimeBoundReference().isRewardBound()) {
        STORM_LOG_THROW(checkTask.isOnlyInitialStatesRelevantSet(), storm::exceptions::InvalidOperationException,
                        "Checking non-trivial bounded until probabilities can only be computed for the initial states of the model.");
        STORM_LOG_WARN_COND(!checkTask.isQualitativeSet(), "Checking non-trivial bounded until formulas is not optimized w.r.t. qualitative queries");
        storm::logic::OperatorInformation opInfo(checkTask.getOptimizationDirection());
        if (checkTask.isBoundSet()) {
            opInfo.bound = checkTask.getBound();
        }
        auto formula = std::make_shared<storm::logic::ProbabilityOperatorFormula>(checkTask.getFormula().asSharedPointer(), opInfo);
        helper::rewardbounded::MultiDimensionalRewardUnfolding<ValueType, true> rewardUnfolding(this->getModel(), formula);
        auto numericResult = storm::modelchecker::helper::SparseMdpPrctlHelper<ValueType>::computeRewardBoundedValues(
            env, checkTask.getOptimizationDirection(), rewardUnfolding, this->getModel().getInitialStates());
        return std::unique_ptr<CheckResult>(new ExplicitQuantitativeCheckResult<ValueType>(std::move(numericResult)));
    } else {
        STORM_LOG_THROW(pathFormula.hasUpperBound(), storm::exceptions::InvalidPropertyException, "Formula needs to have (a single) upper step bound.");
        STORM_LOG_THROW(pathFormula.hasIntegerLowerBound(), storm::exceptions::InvalidPropertyException, "Formula lower step bound must be discrete/integral.");
        STORM_LOG_THROW(pathFormula.hasIntegerUpperBound(), storm::exceptions::InvalidPropertyException, "Formula needs to have discrete upper time bound.");
        std::unique_ptr<CheckResult> leftResultPointer = this->check(env, pathFormula.getLeftSubformula());
        std::unique_ptr<CheckResult> rightResultPointer = this->check(env, pathFormula.getRightSubformula());
        ExplicitQualitativeCheckResult const& leftResult = leftResultPointer->asExplicitQualitativeCheckResult();
        ExplicitQualitativeCheckResult const& rightResult = rightResultPointer->asExplicitQualitativeCheckResult();
        storm::modelchecker::helper::SparseNondeterministicStepBoundedHorizonHelper<ValueType> helper;
        std::vector<ValueType> numericResult =
            helper.compute(env, storm::solver::SolveGoal<ValueType>(this->getModel(), checkTask), this->getModel().getTransitionMatrix(),
                           this->getModel().getBackwardTransitions(), leftResult.getTruthValuesVector(), rightResult.getTruthValuesVector(),
                           pathFormula.getNonStrictLowerBound<uint64_t>(), pathFormula.getNonStrictUpperBound<uint64_t>(), checkTask.getHint());
        return std::unique_ptr<CheckResult>(new ExplicitQuantitativeCheckResult<ValueType>(std::move(numericResult)));
    }
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeNextProbabilities(
    Environment const& env, CheckTask<storm::logic::NextFormula, ValueType> const& checkTask) {
    storm::logic::NextFormula const& pathFormula = checkTask.getFormula();
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    std::unique_ptr<CheckResult> subResultPointer = this->check(env, pathFormula.getSubformula());
    ExplicitQualitativeCheckResult const& subResult = subResultPointer->asExplicitQualitativeCheckResult();
    std::vector<ValueType> numericResult = storm::modelchecker::helper::SparseMdpPrctlHelper<ValueType>::computeNextProbabilities(
        env, checkTask.getOptimizationDirection(), this->getModel().getTransitionMatrix(), subResult.getTruthValuesVector());
    return std::unique_ptr<CheckResult>(new ExplicitQuantitativeCheckResult<ValueType>(std::move(numericResult)));
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeUntilProbabilities(
    Environment const& env, CheckTask<storm::logic::UntilFormula, ValueType> const& checkTask) {

    storm::logic::UntilFormula const& pathFormula = checkTask.getFormula();
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    std::unique_ptr<CheckResult> leftResultPointer = this->check(env, pathFormula.getLeftSubformula());
    std::unique_ptr<CheckResult> rightResultPointer = this->check(env, pathFormula.getRightSubformula());
    ExplicitQualitativeCheckResult const& leftResult = leftResultPointer->asExplicitQualitativeCheckResult();
    ExplicitQualitativeCheckResult const& rightResult = rightResultPointer->asExplicitQualitativeCheckResult();
    auto ret = storm::modelchecker::helper::SparseMdpPrctlHelper<ValueType>::computeUntilProbabilities(
        env, storm::solver::SolveGoal<ValueType>(this->getModel(), checkTask), this->getModel().getTransitionMatrix(),
        this->getModel().getBackwardTransitions(), leftResult.getTruthValuesVector(), rightResult.getTruthValuesVector(), checkTask.isQualitativeSet(),
        checkTask.isProduceSchedulersSet(), checkTask.getHint());
    std::unique_ptr<CheckResult> result(new ExplicitQuantitativeCheckResult<ValueType>(std::move(ret.values)));
    if (checkTask.isProduceSchedulersSet() && ret.scheduler) {
        result->asExplicitQuantitativeCheckResult<ValueType>().setScheduler(std::move(ret.scheduler));
    }
    return result;
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeGloballyProbabilities(
    Environment const& env, CheckTask<storm::logic::GloballyFormula, ValueType> const& checkTask) {
    storm::logic::GloballyFormula const& pathFormula = checkTask.getFormula();
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    std::unique_ptr<CheckResult> subResultPointer = this->check(env, pathFormula.getSubformula());
    ExplicitQualitativeCheckResult const& subResult = subResultPointer->asExplicitQualitativeCheckResult();
    auto ret = storm::modelchecker::helper::SparseMdpPrctlHelper<ValueType>::computeGloballyProbabilities(
        env, storm::solver::SolveGoal<ValueType>(this->getModel(), checkTask), this->getModel().getTransitionMatrix(),
        this->getModel().getBackwardTransitions(), subResult.getTruthValuesVector(), checkTask.isQualitativeSet(), checkTask.isProduceSchedulersSet());
    std::unique_ptr<CheckResult> result(new ExplicitQuantitativeCheckResult<ValueType>(std::move(ret.values)));
    if (checkTask.isProduceSchedulersSet() && ret.scheduler) {
        result->asExplicitQuantitativeCheckResult<ValueType>().setScheduler(std::move(ret.scheduler));
    }
    return result;
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeHOAPathProbabilities(
    Environment const& env, CheckTask<storm::logic::HOAPathFormula, ValueType> const& checkTask) {
    storm::logic::HOAPathFormula const& pathFormula = checkTask.getFormula();

    storm::modelchecker::helper::SparseLTLHelper<ValueType, true> helper(this->getModel().getTransitionMatrix());
    storm::modelchecker::helper::setInformationFromCheckTaskNondeterministic(helper, checkTask, this->getModel());

    auto formulaChecker = [&](storm::logic::Formula const& formula) {
        return this->check(env, formula)->asExplicitQualitativeCheckResult().getTruthValuesVector();
    };
    auto apSets = helper.computeApSets(pathFormula.getAPMapping(), formulaChecker);
    std::vector<ValueType> numericResult = helper.computeDAProductProbabilities(env, *pathFormula.readAutomaton(), apSets);

    std::unique_ptr<CheckResult> result(new ExplicitQuantitativeCheckResult<ValueType>(std::move(numericResult)));
    if (checkTask.isProduceSchedulersSet()) {
        result->asExplicitQuantitativeCheckResult<ValueType>().setScheduler(
            std::make_unique<storm::storage::Scheduler<ValueType>>(helper.extractScheduler(this->getModel())));
    }

    return result;
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeLTLProbabilities(
    Environment const& env, CheckTask<storm::logic::PathFormula, ValueType> const& checkTask) {
    storm::logic::PathFormula const& pathFormula = checkTask.getFormula();

    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");

    storm::modelchecker::helper::SparseLTLHelper<ValueType, true> helper(this->getModel().getTransitionMatrix());
    storm::modelchecker::helper::setInformationFromCheckTaskNondeterministic(helper, checkTask, this->getModel());

    auto formulaChecker = [&](storm::logic::Formula const& formula) {
        return this->check(env, formula)->asExplicitQualitativeCheckResult().getTruthValuesVector();
    };
    std::vector<ValueType> numericResult = helper.computeLTLProbabilities(env, pathFormula, formulaChecker);

    std::unique_ptr<CheckResult> result(new ExplicitQuantitativeCheckResult<ValueType>(std::move(numericResult)));
    if (checkTask.isProduceSchedulersSet()) {
        result->asExplicitQuantitativeCheckResult<ValueType>().setScheduler(
            std::make_unique<storm::storage::Scheduler<ValueType>>(helper.extractScheduler(this->getModel())));
    }

    return result;
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeConditionalProbabilities(
    Environment const& env, CheckTask<storm::logic::ConditionalFormula, ValueType> const& checkTask) {
    storm::logic::ConditionalFormula const& conditionalFormula = checkTask.getFormula();
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    STORM_LOG_THROW(this->getModel().getInitialStates().getNumberOfSetBits() == 1, storm::exceptions::InvalidPropertyException,
                    "Cannot compute conditional probabilities on MDPs with more than one initial state.");
    STORM_LOG_THROW(conditionalFormula.getSubformula().isEventuallyFormula(), storm::exceptions::InvalidPropertyException,
                    "Illegal conditional probability formula.");
    STORM_LOG_THROW(conditionalFormula.getConditionFormula().isEventuallyFormula(), storm::exceptions::InvalidPropertyException,
                    "Illegal conditional probability formula.");

    std::unique_ptr<CheckResult> leftResultPointer = this->check(env, conditionalFormula.getSubformula().asEventuallyFormula().getSubformula());
    std::unique_ptr<CheckResult> rightResultPointer = this->check(env, conditionalFormula.getConditionFormula().asEventuallyFormula().getSubformula());
    ExplicitQualitativeCheckResult const& leftResult = leftResultPointer->asExplicitQualitativeCheckResult();
    ExplicitQualitativeCheckResult const& rightResult = rightResultPointer->asExplicitQualitativeCheckResult();

    return storm::modelchecker::helper::SparseMdpPrctlHelper<ValueType>::computeConditionalProbabilities(
        env, storm::solver::SolveGoal<ValueType>(this->getModel(), checkTask), this->getModel().getTransitionMatrix(),
        this->getModel().getBackwardTransitions(), leftResult.getTruthValuesVector(), rightResult.getTruthValuesVector());
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeCumulativeRewards(
    Environment const& env, storm::logic::RewardMeasureType, CheckTask<storm::logic::CumulativeRewardFormula, ValueType> const& checkTask) {
    storm::logic::CumulativeRewardFormula const& rewardPathFormula = checkTask.getFormula();
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    if (rewardPathFormula.isMultiDimensional() || rewardPathFormula.getTimeBoundReference().isRewardBound()) {
        STORM_LOG_THROW(checkTask.isOnlyInitialStatesRelevantSet(), storm::exceptions::InvalidOperationException,
                        "Checking reward bounded cumulative reward formulas can only be done for the initial states of the model.");
        STORM_LOG_THROW(!checkTask.getFormula().hasRewardAccumulation(), storm::exceptions::InvalidOperationException,
                        "Checking reward bounded cumulative reward formulas is not supported if reward accumulations are given.");
        STORM_LOG_WARN_COND(!checkTask.isQualitativeSet(), "Checking reward bounded until formulas is not optimized w.r.t. qualitative queries");
        storm::logic::OperatorInformation opInfo(checkTask.getOptimizationDirection());
        if (checkTask.isBoundSet()) {
            opInfo.bound = checkTask.getBound();
        }
        auto formula = std::make_shared<storm::logic::RewardOperatorFormula>(checkTask.getFormula().asSharedPointer(), checkTask.getRewardModel(), opInfo);
        helper::rewardbounded::MultiDimensionalRewardUnfolding<ValueType, true> rewardUnfolding(this->getModel(), formula);
        auto numericResult = storm::modelchecker::helper::SparseMdpPrctlHelper<ValueType>::computeRewardBoundedValues(
            env, checkTask.getOptimizationDirection(), rewardUnfolding, this->getModel().getInitialStates());
        return std::unique_ptr<CheckResult>(new ExplicitQuantitativeCheckResult<ValueType>(std::move(numericResult)));
    } else {
        STORM_LOG_THROW(rewardPathFormula.hasIntegerBound(), storm::exceptions::InvalidPropertyException, "Formula needs to have a discrete time bound.");
        auto rewardModel = storm::utility::createFilteredRewardModel(this->getModel(), checkTask);
        std::vector<ValueType> numericResult = storm::modelchecker::helper::SparseMdpPrctlHelper<ValueType>::computeCumulativeRewards(
            env, storm::solver::SolveGoal<ValueType>(this->getModel(), checkTask), this->getModel().getTransitionMatrix(), rewardModel.get(),
            rewardPathFormula.getNonStrictBound<uint64_t>());
        return std::unique_ptr<CheckResult>(new ExplicitQuantitativeCheckResult<ValueType>(std::move(numericResult)));
    }
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeInstantaneousRewards(
    Environment const& env, storm::logic::RewardMeasureType, CheckTask<storm::logic::InstantaneousRewardFormula, ValueType> const& checkTask) {
    storm::logic::InstantaneousRewardFormula const& rewardPathFormula = checkTask.getFormula();
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    STORM_LOG_THROW(rewardPathFormula.hasIntegerBound(), storm::exceptions::InvalidPropertyException, "Formula needs to have a discrete time bound.");
    std::vector<ValueType> numericResult = storm::modelchecker::helper::SparseMdpPrctlHelper<ValueType>::computeInstantaneousRewards(
        env, storm::solver::SolveGoal<ValueType>(this->getModel(), checkTask), this->getModel().getTransitionMatrix(),
        checkTask.isRewardModelSet() ? this->getModel().getRewardModel(checkTask.getRewardModel()) : this->getModel().getRewardModel(""),
        rewardPathFormula.getBound<uint64_t>());
    return std::unique_ptr<CheckResult>(new ExplicitQuantitativeCheckResult<ValueType>(std::move(numericResult)));
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeReachabilityRewards(
    Environment const& env, storm::logic::RewardMeasureType, CheckTask<storm::logic::EventuallyFormula, ValueType> const& checkTask) {
    storm::logic::EventuallyFormula const& eventuallyFormula = checkTask.getFormula();
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    std::unique_ptr<CheckResult> subResultPointer = this->check(env, eventuallyFormula.getSubformula());
    ExplicitQualitativeCheckResult const& subResult = subResultPointer->asExplicitQualitativeCheckResult();
    auto rewardModel = storm::utility::createFilteredRewardModel(this->getModel(), checkTask);
    auto ret = storm::modelchecker::helper::SparseMdpPrctlHelper<ValueType>::computeReachabilityRewards(
        env, storm::solver::SolveGoal<ValueType>(this->getModel(), checkTask), this->getModel().getTransitionMatrix(),
        this->getModel().getBackwardTransitions(), rewardModel.get(), subResult.getTruthValuesVector(), checkTask.isQualitativeSet(),
        checkTask.isProduceSchedulersSet(), checkTask.getHint());
    std::unique_ptr<CheckResult> result(new ExplicitQuantitativeCheckResult<ValueType>(std::move(ret.values)));
    if (checkTask.isProduceSchedulersSet() && ret.scheduler) {
        result->asExplicitQuantitativeCheckResult<ValueType>().setScheduler(std::move(ret.scheduler));
    }
    return result;
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeReachabilityTimes(
    Environment const& env, storm::logic::RewardMeasureType, CheckTask<storm::logic::EventuallyFormula, ValueType> const& checkTask) {
    storm::logic::EventuallyFormula const& eventuallyFormula = checkTask.getFormula();
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    std::unique_ptr<CheckResult> subResultPointer = this->check(env, eventuallyFormula.getSubformula());
    ExplicitQualitativeCheckResult const& subResult = subResultPointer->asExplicitQualitativeCheckResult();
    auto ret = storm::modelchecker::helper::SparseMdpPrctlHelper<ValueType>::computeReachabilityTimes(
        env, storm::solver::SolveGoal<ValueType>(this->getModel(), checkTask), this->getModel().getTransitionMatrix(),
        this->getModel().getBackwardTransitions(), subResult.getTruthValuesVector(), checkTask.isQualitativeSet(), checkTask.isProduceSchedulersSet(),
        checkTask.getHint());
    std::unique_ptr<CheckResult> result(new ExplicitQuantitativeCheckResult<ValueType>(std::move(ret.values)));
    if (checkTask.isProduceSchedulersSet() && ret.scheduler) {
        result->asExplicitQuantitativeCheckResult<ValueType>().setScheduler(std::move(ret.scheduler));
    }
    return result;
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeTotalRewards(
    Environment const& env, storm::logic::RewardMeasureType, CheckTask<storm::logic::TotalRewardFormula, ValueType> const& checkTask) {
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    auto rewardModel = storm::utility::createFilteredRewardModel(this->getModel(), checkTask);
    auto ret = storm::modelchecker::helper::SparseMdpPrctlHelper<ValueType>::computeTotalRewards(
        env, storm::solver::SolveGoal<ValueType>(this->getModel(), checkTask), this->getModel().getTransitionMatrix(),
        this->getModel().getBackwardTransitions(), rewardModel.get(), checkTask.isQualitativeSet(), checkTask.isProduceSchedulersSet(), checkTask.getHint());
    std::unique_ptr<CheckResult> result(new ExplicitQuantitativeCheckResult<ValueType>(std::move(ret.values)));
    if (checkTask.isProduceSchedulersSet() && ret.scheduler) {
        result->asExplicitQuantitativeCheckResult<ValueType>().setScheduler(std::move(ret.scheduler));
    }
    return result;
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeLongRunAverageProbabilities(
    Environment const& env, CheckTask<storm::logic::StateFormula, ValueType> const& checkTask) {
    storm::logic::StateFormula const& stateFormula = checkTask.getFormula();
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    std::unique_ptr<CheckResult> subResultPointer = this->check(env, stateFormula);
    ExplicitQualitativeCheckResult const& subResult = subResultPointer->asExplicitQualitativeCheckResult();

    storm::modelchecker::helper::SparseNondeterministicInfiniteHorizonHelper<ValueType> helper(this->getModel().getTransitionMatrix());
    storm::modelchecker::helper::setInformationFromCheckTaskNondeterministic(helper, checkTask, this->getModel());
    auto values = helper.computeLongRunAverageProbabilities(env, subResult.getTruthValuesVector());

    std::unique_ptr<CheckResult> result(new ExplicitQuantitativeCheckResult<ValueType>(std::move(values)));
    if (checkTask.isProduceSchedulersSet()) {
        result->asExplicitQuantitativeCheckResult<ValueType>().setScheduler(std::make_unique<storm::storage::Scheduler<ValueType>>(helper.extractScheduler()));
    }
    return result;
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::computeLongRunAverageRewards(
    Environment const& env, storm::logic::RewardMeasureType rewardMeasureType,
    CheckTask<storm::logic::LongRunAverageRewardFormula, ValueType> const& checkTask) {
    STORM_LOG_THROW(checkTask.isOptimizationDirectionSet(), storm::exceptions::InvalidPropertyException,
                    "Formula needs to specify whether minimal or maximal values are to be computed on nondeterministic model.");
    auto rewardModel = storm::utility::createFilteredRewardModel(this->getModel(), checkTask);
    storm::modelchecker::helper::SparseNondeterministicInfiniteHorizonHelper<ValueType> helper(this->getModel().getTransitionMatrix());
    storm::modelchecker::helper::setInformationFromCheckTaskNondeterministic(helper, checkTask, this->getModel());
    auto values = helper.computeLongRunAverageRewards(env, rewardModel.get());
    std::unique_ptr<CheckResult> result(new ExplicitQuantitativeCheckResult<ValueType>(std::move(values)));
    if (checkTask.isProduceSchedulersSet()) {
        result->asExplicitQuantitativeCheckResult<ValueType>().setScheduler(std::make_unique<storm::storage::Scheduler<ValueType>>(helper.extractScheduler()));
    }
    return result;
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::checkMultiObjectiveFormula(
    Environment const& env, CheckTask<storm::logic::MultiObjectiveFormula, ValueType> const& checkTask) {
    return multiobjective::performMultiObjectiveModelChecking(env, this->getModel(), checkTask.getFormula());
}

template<class SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::checkLexObjectiveFormula(
    const Environment& env, const CheckTask<storm::logic::MultiObjectiveFormula, ValueType>& checkTask) {
    auto formulaChecker = [&](storm::logic::Formula const& formula) {
        return this->check(env, formula)->asExplicitQualitativeCheckResult().getTruthValuesVector();
    };
    auto ret = lexicographic::check(env, this->getModel(), checkTask, formulaChecker);
    std::unique_ptr<CheckResult> result(new LexicographicCheckResult<ValueType>(ret.values, *this->getModel().getInitialStates().begin()));
    return result;
}

template<typename SparseMdpModelType>
std::unique_ptr<CheckResult> TraceMdpModelChecker<SparseMdpModelType>::checkQuantileFormula(
    Environment const& env, CheckTask<storm::logic::QuantileFormula, ValueType> const& checkTask) {
    STORM_LOG_THROW(checkTask.isOnlyInitialStatesRelevantSet(), storm::exceptions::InvalidOperationException,
                    "Computing quantiles is only supported for the initial states of a model.");
    STORM_LOG_THROW(this->getModel().getInitialStates().getNumberOfSetBits() == 1, storm::exceptions::InvalidOperationException,
                    "Quantiles not supported on models with multiple initial states.");
    uint64_t initialState = *this->getModel().getInitialStates().begin();

    helper::rewardbounded::QuantileHelper<SparseMdpModelType> qHelper(this->getModel(), checkTask.getFormula());
    auto res = qHelper.computeQuantile(env);

    if (res.size() == 1 && res.front().size() == 1) {
        return std::unique_ptr<CheckResult>(new ExplicitQuantitativeCheckResult<ValueType>(initialState, std::move(res.front().front())));
    } else {
        return std::unique_ptr<CheckResult>(new ExplicitParetoCurveCheckResult<ValueType>(initialState, std::move(res)));
    }
}

template class TraceMdpModelChecker<storm::models::sparse::Mdp<double>>;

#ifdef STORM_HAVE_CARL
template class TraceMdpModelChecker<storm::models::sparse::Mdp<storm::RationalNumber>>;
#endif
}  // namespace modelchecker
}  // namespace storm
