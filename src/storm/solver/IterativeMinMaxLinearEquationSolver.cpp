#include <functional>

#include "storm/solver/IterativeMinMaxLinearEquationSolver.h"

#include "storm/utility/ConstantsComparator.h"

#include "storm/environment/solver/MinMaxSolverEnvironment.h"

#include "storm/utility/KwekMehlhorn.h"
#include "storm/utility/NumberTraits.h"

#include "storm/utility/Stopwatch.h"
#include "storm/utility/vector.h"
#include "storm/utility/macros.h"
#include "storm/exceptions/InvalidEnvironmentException.h"
#include "storm/exceptions/InvalidStateException.h"
#include "storm/exceptions/UnmetRequirementException.h"
#include "storm/exceptions/NotSupportedException.h"
#include "storm/exceptions/PrecisionExceededException.h"

namespace storm {
    namespace solver {
        
        template<typename ValueType>
        IterativeMinMaxLinearEquationSolver<ValueType>::IterativeMinMaxLinearEquationSolver(std::unique_ptr<LinearEquationSolverFactory<ValueType>>&& linearEquationSolverFactory) : linearEquationSolverFactory(std::move(linearEquationSolverFactory)) {
            // Intentionally left empty
        }
        
        template<typename ValueType>
        IterativeMinMaxLinearEquationSolver<ValueType>::IterativeMinMaxLinearEquationSolver(storm::storage::SparseMatrix<ValueType> const& A, std::unique_ptr<LinearEquationSolverFactory<ValueType>>&& linearEquationSolverFactory) : StandardMinMaxLinearEquationSolver<ValueType>(A), linearEquationSolverFactory(std::move(linearEquationSolverFactory)) {
            // Intentionally left empty.
        }
        
        template<typename ValueType>
        IterativeMinMaxLinearEquationSolver<ValueType>::IterativeMinMaxLinearEquationSolver(storm::storage::SparseMatrix<ValueType>&& A, std::unique_ptr<LinearEquationSolverFactory<ValueType>>&& linearEquationSolverFactory) : StandardMinMaxLinearEquationSolver<ValueType>(std::move(A)), linearEquationSolverFactory(std::move(linearEquationSolverFactory)) {
            // Intentionally left empty.
        }
        
        template<typename ValueType>
        MinMaxMethod IterativeMinMaxLinearEquationSolver<ValueType>::getMethod(Environment const& env, bool isExactMode) const {
            // Adjust the method if none was specified and we want exact or sound computations.
            auto method = env.solver().minMax().getMethod();
            
            if (isExactMode && method != MinMaxMethod::PolicyIteration && method != MinMaxMethod::RationalSearch) {
                if (env.solver().minMax().isMethodSetFromDefault()) {
                    STORM_LOG_INFO("Selecting 'Policy iteration' as the solution technique to guarantee exact results. If you want to override this, please explicitly specify a different method.");
                    method = MinMaxMethod::PolicyIteration;
                } else {
                    STORM_LOG_WARN("The selected solution method does not guarantee exact results.");
                }
            } else if (env.solver().isForceSoundness() && method != MinMaxMethod::SoundValueIteration && method != MinMaxMethod::IntervalIteration && method != MinMaxMethod::PolicyIteration && method != MinMaxMethod::RationalSearch) {
                if (env.solver().minMax().isMethodSetFromDefault()) {
                    STORM_LOG_INFO("Selecting 'sound value iteration' as the solution technique to guarantee sound results. If you want to override this, please explicitly specify a different method.");
                    method = MinMaxMethod::SoundValueIteration;
                } else {
                    STORM_LOG_WARN("The selected solution method does not guarantee sound results.");
                }
            }
            STORM_LOG_THROW(method == MinMaxMethod::ValueIteration || method == MinMaxMethod::PolicyIteration || method == MinMaxMethod::RationalSearch || method == MinMaxMethod::SoundValueIteration || method == MinMaxMethod::IntervalIteration, storm::exceptions::InvalidEnvironmentException, "This solver does not support the selected method.");
            return method;
        }
        
        template<typename ValueType>
        bool IterativeMinMaxLinearEquationSolver<ValueType>::internalSolveEquations(Environment const& env, OptimizationDirection dir, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            bool result = false;
            switch (getMethod(env, storm::NumberTraits<ValueType>::IsExact)) {
                case MinMaxMethod::ValueIteration:
                    result = solveEquationsValueIteration(env, dir, x, b);
                    break;
                case MinMaxMethod::PolicyIteration:
                    result = solveEquationsPolicyIteration(env, dir, x, b);
                    break;
                case MinMaxMethod::RationalSearch:
                    result = solveEquationsRationalSearch(env, dir, x, b);
                    break;
                case MinMaxMethod::IntervalIteration:
                    result = solveEquationsIntervalIteration(env, dir, x, b);
                    break;
                case MinMaxMethod::SoundValueIteration:
                    result = solveEquationsSoundValueIteration(env, dir, x, b);
                    break;
                default:
                    STORM_LOG_THROW(false, storm::exceptions::InvalidEnvironmentException, "This solver does not implement the selected solution method");
            }
            
            return result;
        }

        template<typename ValueType>
        bool IterativeMinMaxLinearEquationSolver<ValueType>::solveInducedEquationSystem(Environment const& env, std::unique_ptr<LinearEquationSolver<ValueType>>& linearEquationSolver, std::vector<uint64_t> const& scheduler, std::vector<ValueType>& x, std::vector<ValueType>& subB, std::vector<ValueType> const& originalB) const {
            assert(subB.size() == x.size());
            
            // Resolve the nondeterminism according to the given scheduler.
            bool convertToEquationSystem = this->linearEquationSolverFactory->getEquationProblemFormat(env) == LinearEquationSolverProblemFormat::EquationSystem;
            storm::storage::SparseMatrix<ValueType> submatrix = this->A->selectRowsFromRowGroups(scheduler, convertToEquationSystem);
            if (convertToEquationSystem) {
                submatrix.convertToEquationSystem();
            }
            storm::utility::vector::selectVectorValues<ValueType>(subB, scheduler, this->A->getRowGroupIndices(), originalB);
            
            // Check whether the linear equation solver is already initialized
            if (!linearEquationSolver) {
                // Initialize the equation solver
                linearEquationSolver = this->linearEquationSolverFactory->create(env, std::move(submatrix));
                linearEquationSolver->setBoundsFromOtherSolver(*this);
                linearEquationSolver->setCachingEnabled(true);
            } else {
                // If the equation solver is already initialized, it suffices to update the matrix
                linearEquationSolver->setMatrix(std::move(submatrix));
            }
            // Solve the equation system for the 'DTMC' and return true upon success
            return linearEquationSolver->solveEquations(env, x, subB);
        }
        
        template<typename ValueType>
        bool IterativeMinMaxLinearEquationSolver<ValueType>::solveEquationsPolicyIteration(Environment const& env, OptimizationDirection dir, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            // Create the initial scheduler.
            std::vector<storm::storage::sparse::state_type> scheduler = this->hasInitialScheduler() ? this->getInitialScheduler() : std::vector<storm::storage::sparse::state_type>(this->A->getRowGroupCount());
            
            // Get a vector for storing the right-hand side of the inner equation system.
            if (!auxiliaryRowGroupVector) {
                auxiliaryRowGroupVector = std::make_unique<std::vector<ValueType>>(this->A->getRowGroupCount());
            }
            std::vector<ValueType>& subB = *auxiliaryRowGroupVector;

            // The solver that we will use throughout the procedure.
            std::unique_ptr<storm::solver::LinearEquationSolver<ValueType>> solver;
            // The linear equation solver should be at least as precise as this solver
            std::unique_ptr<storm::Environment> environmentOfSolverStorage;
            auto precOfSolver = env.solver().getPrecisionOfLinearEquationSolver(env.solver().getLinearEquationSolverType());
            if (!storm::NumberTraits<ValueType>::IsExact) {
                bool changePrecision = precOfSolver.first && precOfSolver.first.get() > env.solver().minMax().getPrecision();
                bool changeRelative = precOfSolver.second && !precOfSolver.second.get() && env.solver().minMax().getRelativeTerminationCriterion();
                if (changePrecision || changeRelative) {
                    environmentOfSolverStorage = std::make_unique<storm::Environment>(env);
                    boost::optional<storm::RationalNumber> newPrecision;
                    boost::optional<bool> newRelative;
                    if (changePrecision) {
                        newPrecision = env.solver().minMax().getPrecision();
                    }
                    if (changeRelative) {
                        newRelative = true;
                    }
                    environmentOfSolverStorage->solver().setLinearEquationSolverPrecision(newPrecision, newRelative);
                }
            }
            storm::Environment const& environmentOfSolver = environmentOfSolverStorage ? *environmentOfSolverStorage : env;

            SolverStatus status = SolverStatus::InProgress;
            uint64_t iterations = 0;
            this->startMeasureProgress();
            do {
                // Solve the equation system for the 'DTMC'.
                solveInducedEquationSystem(environmentOfSolver, solver, scheduler, x, subB, b);
                
                // Go through the multiplication result and see whether we can improve any of the choices.
                bool schedulerImproved = false;
                for (uint_fast64_t group = 0; group < this->A->getRowGroupCount(); ++group) {
                    uint_fast64_t currentChoice = scheduler[group];
                    for (uint_fast64_t choice = this->A->getRowGroupIndices()[group]; choice < this->A->getRowGroupIndices()[group + 1]; ++choice) {
                        // If the choice is the currently selected one, we can skip it.
                        if (choice - this->A->getRowGroupIndices()[group] == currentChoice) {
                            continue;
                        }
                        
                        // Create the value of the choice.
                        ValueType choiceValue = storm::utility::zero<ValueType>();
                        for (auto const& entry : this->A->getRow(choice)) {
                            choiceValue += entry.getValue() * x[entry.getColumn()];
                        }
                        choiceValue += b[choice];
                        
                        // If the value is strictly better than the solution of the inner system, we need to improve the scheduler.
                        // TODO: If the underlying solver is not precise, this might run forever (i.e. when a state has two choices where the (exact) values are equal).
                        // only changing the scheduler if the values are not equal (modulo precision) would make this unsound.
                        if (valueImproved(dir, x[group], choiceValue)) {
                            schedulerImproved = true;
                            scheduler[group] = choice - this->A->getRowGroupIndices()[group];
                            x[group] = std::move(choiceValue);
                        }
                    }
                }
                
                // If the scheduler did not improve, we are done.
                if (!schedulerImproved) {
                    status = SolverStatus::Converged;
                }
                
                // Update environment variables.
                ++iterations;
                status = updateStatusIfNotConverged(status, x, iterations, env.solver().minMax().getMaximalNumberOfIterations(), dir == storm::OptimizationDirection::Minimize ? SolverGuarantee::GreaterOrEqual : SolverGuarantee::LessOrEqual);

                // Potentially show progress.
                this->showProgressIterative(iterations);
            } while (status == SolverStatus::InProgress);
            
            reportStatus(status, iterations);
            
            // If requested, we store the scheduler for retrieval.
            if (this->isTrackSchedulerSet()) {
                this->schedulerChoices = std::move(scheduler);
            }
            
            if (!this->isCachingEnabled()) {
                clearCache();
            }

            return status == SolverStatus::Converged || status == SolverStatus::TerminatedEarly;
        }
        
        template<typename ValueType>
        bool IterativeMinMaxLinearEquationSolver<ValueType>::valueImproved(OptimizationDirection dir, ValueType const& value1, ValueType const& value2) const {
            if (dir == OptimizationDirection::Minimize) {
                return value2 < value1;
            } else {
                return value2 > value1;
            }
        }

        template<typename ValueType>
        MinMaxLinearEquationSolverRequirements IterativeMinMaxLinearEquationSolver<ValueType>::getRequirements(Environment const& env, boost::optional<storm::solver::OptimizationDirection> const& direction, bool const& hasInitialScheduler) const {
            auto method = getMethod(env, storm::NumberTraits<ValueType>::IsExact);
            
            // Check whether a linear equation solver is needed and potentially start with its requirements
            bool needsLinEqSolver = false;
            needsLinEqSolver |= method == MinMaxMethod::PolicyIteration;
            needsLinEqSolver |= method == MinMaxMethod::ValueIteration && (this->hasInitialScheduler() || hasInitialScheduler);
            MinMaxLinearEquationSolverRequirements requirements = needsLinEqSolver ? MinMaxLinearEquationSolverRequirements(this->linearEquationSolverFactory->getRequirements(env)) : MinMaxLinearEquationSolverRequirements();

            if (method == MinMaxMethod::ValueIteration) {
                if (!this->hasUniqueSolution()) { // Traditional value iteration has no requirements if the solution is unique.
                    // Computing a scheduler is only possible if the solution is unique
                    if (this->isTrackSchedulerSet()) {
                        requirements.requireNoEndComponents();
                    } else {
                        // As we want the smallest (largest) solution for maximizing (minimizing) equation systems, we have to approach the solution from below (above).
                        if (!direction || direction.get() == OptimizationDirection::Maximize) {
                            requirements.requireLowerBounds();
                        }
                        if (!direction || direction.get() == OptimizationDirection::Minimize) {
                            requirements.requireUpperBounds();
                        }
                    }
                }
            } else if (method == MinMaxMethod::IntervalIteration) {
                // Interval iteration requires a unique solution and lower+upper bounds
                if (!this->hasUniqueSolution()) {
                    requirements.requireNoEndComponents();
                }
                requirements.requireBounds();
            } else if (method == MinMaxMethod::RationalSearch) {
                // Rational search needs to approach the solution from below.
                requirements.requireLowerBounds();
                // The solution needs to be unique in case of minimizing or in cases where we want a scheduler.
                if (!this->hasUniqueSolution() && (!direction || direction.get() == OptimizationDirection::Minimize || this->isTrackSchedulerSet())) {
                    requirements.requireNoEndComponents();
                }
            } else if (method == MinMaxMethod::PolicyIteration) {
                if (!this->hasUniqueSolution()) {
                    requirements.requireValidInitialScheduler();
                }
            } else if (method == MinMaxMethod::SoundValueIteration) {
                if (!this->hasUniqueSolution()) {
                    requirements.requireNoEndComponents();
                }
            } else {
                STORM_LOG_THROW(false, storm::exceptions::InvalidEnvironmentException, "Unsupported technique for iterative MinMax linear equation solver.");
            }
        
            if (env.solver().minMax().isForceBoundsSet()) {
                requirements.requireBounds();
            }
            return requirements;
        }

        template<typename ValueType>
        typename IterativeMinMaxLinearEquationSolver<ValueType>::ValueIterationResult IterativeMinMaxLinearEquationSolver<ValueType>::performValueIteration(Environment const& env, OptimizationDirection dir, std::vector<ValueType>*& currentX, std::vector<ValueType>*& newX, std::vector<ValueType> const& b, ValueType const& precision, bool relative, SolverGuarantee const& guarantee, uint64_t currentIterations, uint64_t maximalNumberOfIterations, storm::solver::MultiplicationStyle const& multiplicationStyle) const {
            
            STORM_LOG_ASSERT(currentX != newX, "Vectors must not be aliased.");
            
            // Get handle to multiplier.
            storm::solver::Multiplier<ValueType> const& multiplier = *this->multiplierA;
            
            // Allow aliased multiplications.
            bool useGaussSeidelMultiplication = multiplicationStyle == storm::solver::MultiplicationStyle::GaussSeidel;
            
            // Proceed with the iterations as long as the method did not converge or reach the maximum number of iterations.
            uint64_t iterations = currentIterations;
            
            std::vector<ValueType>* originalX = currentX;
            
            SolverStatus status = SolverStatus::InProgress;
            while (status == SolverStatus::InProgress) {
                // Compute x' = min/max(A*x + b).
                if (useGaussSeidelMultiplication) {
                    // Copy over the current vector so we can modify it in-place.
                    *newX = *currentX;
                    multiplier.multiplyAndReduceGaussSeidel(env, dir, *newX, &b);
                } else {
                    multiplier.multiplyAndReduce(env, dir, *currentX, &b, *newX);
                }
                
                // Determine whether the method converged.
                if (storm::utility::vector::equalModuloPrecision<ValueType>(*currentX, *newX, precision, relative)) {
                    status = SolverStatus::Converged;
                }
                
                // Update environment variables.
                std::swap(currentX, newX);
                ++iterations;
                status = updateStatusIfNotConverged(status, *currentX, iterations, maximalNumberOfIterations, guarantee);

                // Potentially show progress.
                this->showProgressIterative(iterations);
            }
            
            // Swap the pointers so that the output is always in currentX.
            if (originalX == newX) {
                std::swap(currentX, newX);
            }
            
            return ValueIterationResult(iterations - currentIterations, status);
        }
        
        template<typename ValueType>
        bool IterativeMinMaxLinearEquationSolver<ValueType>::solveEquationsValueIteration(Environment const& env, OptimizationDirection dir, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            if (!this->multiplierA) {
                this->multiplierA = storm::solver::MultiplierFactory<ValueType>().create(env, *this->A);
            }
            
            if (!auxiliaryRowGroupVector) {
                auxiliaryRowGroupVector = std::make_unique<std::vector<ValueType>>(this->A->getRowGroupCount());
            }
            
            // By default, we can not provide any guarantee
            SolverGuarantee guarantee = SolverGuarantee::None;
            
            if (this->hasInitialScheduler()) {
                // Solve the equation system induced by the initial scheduler.
                std::unique_ptr<storm::solver::LinearEquationSolver<ValueType>> linEqSolver;
                // The linear equation solver should be at least as precise as this solver
                std::unique_ptr<storm::Environment> environmentOfSolverStorage;
                auto precOfSolver = env.solver().getPrecisionOfLinearEquationSolver(env.solver().getLinearEquationSolverType());
                if (!storm::NumberTraits<ValueType>::IsExact) {
                    bool changePrecision = precOfSolver.first && precOfSolver.first.get() > env.solver().minMax().getPrecision();
                    bool changeRelative = precOfSolver.second && !precOfSolver.second.get() && env.solver().minMax().getRelativeTerminationCriterion();
                    if (changePrecision || changeRelative) {
                        environmentOfSolverStorage = std::make_unique<storm::Environment>(env);
                        boost::optional<storm::RationalNumber> newPrecision;
                        boost::optional<bool> newRelative;
                        if (changePrecision) {
                            newPrecision = env.solver().minMax().getPrecision();
                        }
                        if (changeRelative) {
                            newRelative = true;
                        }
                        environmentOfSolverStorage->solver().setLinearEquationSolverPrecision(newPrecision, newRelative);
                    }
                }
                storm::Environment const& environmentOfSolver = environmentOfSolverStorage ? *environmentOfSolverStorage : env;

                solveInducedEquationSystem(environmentOfSolver, linEqSolver, this->getInitialScheduler(), x, *auxiliaryRowGroupVector, b);
                // If we were given an initial scheduler and are maximizing (minimizing), our current solution becomes
                // always less-or-equal (greater-or-equal) than the actual solution.
                guarantee = maximize(dir) ? SolverGuarantee::LessOrEqual : SolverGuarantee::GreaterOrEqual;
            } else if (!this->hasUniqueSolution()) {
                if (maximize(dir)) {
                    this->createLowerBoundsVector(x);
                    guarantee = SolverGuarantee::LessOrEqual;
                } else {
                    this->createUpperBoundsVector(x);
                    guarantee = SolverGuarantee::GreaterOrEqual;
                }
            } else if (this->hasCustomTerminationCondition()) {
                if (this->getTerminationCondition().requiresGuarantee(SolverGuarantee::LessOrEqual) && this->hasLowerBound()) {
                    this->createLowerBoundsVector(x);
                    guarantee = SolverGuarantee::LessOrEqual;
                } else if (this->getTerminationCondition().requiresGuarantee(SolverGuarantee::GreaterOrEqual) && this->hasUpperBound()) {
                    this->createUpperBoundsVector(x);
                    guarantee = SolverGuarantee::GreaterOrEqual;
                }
            }

            std::vector<ValueType>* newX = auxiliaryRowGroupVector.get();
            std::vector<ValueType>* currentX = &x;
            
            this->startMeasureProgress();
            ValueIterationResult result = performValueIteration(env, dir, currentX, newX, b, storm::utility::convertNumber<ValueType>(env.solver().minMax().getPrecision()), env.solver().minMax().getRelativeTerminationCriterion(), guarantee, 0, env.solver().minMax().getMaximalNumberOfIterations(), env.solver().minMax().getMultiplicationStyle());

            // Swap the result into the output x.
            if (currentX == auxiliaryRowGroupVector.get()) {
                std::swap(x, *currentX);
            }
            
            reportStatus(result.status, result.iterations);
            
            // If requested, we store the scheduler for retrieval.
            if (this->isTrackSchedulerSet()) {
                this->schedulerChoices = std::vector<uint_fast64_t>(this->A->getRowGroupCount());
                this->multiplierA->multiplyAndReduce(env, dir, x, &b, *auxiliaryRowGroupVector.get(), &this->schedulerChoices.get());
            }
            
            if (!this->isCachingEnabled()) {
                clearCache();
            }
            
            return result.status == SolverStatus::Converged || result.status == SolverStatus::TerminatedEarly;
        }
        
        template<typename ValueType>
        void preserveOldRelevantValues(std::vector<ValueType> const& allValues, storm::storage::BitVector const& relevantValues, std::vector<ValueType>& oldValues) {
            storm::utility::vector::selectVectorValues(oldValues, relevantValues, allValues);
        }
        
        template<typename ValueType>
        ValueType computeMaxAbsDiff(std::vector<ValueType> const& allValues, storm::storage::BitVector const& relevantValues, std::vector<ValueType> const& oldValues) {
            ValueType result = storm::utility::zero<ValueType>();
            auto oldValueIt = oldValues.begin();
            for (auto value : relevantValues) {
                result = storm::utility::max<ValueType>(result, storm::utility::abs<ValueType>(allValues[value] - *oldValueIt));
            }
            return result;
        }
        
        template<typename ValueType>
        ValueType computeMaxAbsDiff(std::vector<ValueType> const& allOldValues, std::vector<ValueType> const& allNewValues, storm::storage::BitVector const& relevantValues) {
            ValueType result = storm::utility::zero<ValueType>();
            for (auto value : relevantValues) {
                result = storm::utility::max<ValueType>(result, storm::utility::abs<ValueType>(allNewValues[value] - allOldValues[value]));
            }
            return result;
        }
        
        /*!
         * This version of value iteration is sound, because it approaches the solution from below and above. This
         * technique is due to Haddad and Monmege (Interval iteration algorithm for MDPs and IMDPs, TCS 2017) and was
         * extended to rewards by Baier, Klein, Leuschner, Parker and Wunderlich (Ensuring the Reliability of Your
         * Model Checker: Interval Iteration for Markov Decision Processes, CAV 2017).
         */
        template<typename ValueType>
        bool IterativeMinMaxLinearEquationSolver<ValueType>::solveEquationsIntervalIteration(Environment const& env, OptimizationDirection dir, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            STORM_LOG_THROW(this->hasUpperBound(), storm::exceptions::UnmetRequirementException, "Solver requires upper bound, but none was given.");

            if (!this->multiplierA) {
                this->multiplierA = storm::solver::MultiplierFactory<ValueType>().create(env, *this->A);
            }
            
            if (!auxiliaryRowGroupVector) {
                auxiliaryRowGroupVector = std::make_unique<std::vector<ValueType>>(this->A->getRowGroupCount());
            }
            
            // Allow aliased multiplications.
            bool useGaussSeidelMultiplication = env.solver().minMax().getMultiplicationStyle() == storm::solver::MultiplicationStyle::GaussSeidel;
            
            std::vector<ValueType>* lowerX = &x;
            this->createLowerBoundsVector(*lowerX);
            this->createUpperBoundsVector(this->auxiliaryRowGroupVector, this->A->getRowGroupCount());
            std::vector<ValueType>* upperX = this->auxiliaryRowGroupVector.get();
            
            std::vector<ValueType>* tmp = nullptr;
            if (!useGaussSeidelMultiplication) {
                auxiliaryRowGroupVector2 = std::make_unique<std::vector<ValueType>>(lowerX->size());
                tmp = auxiliaryRowGroupVector2.get();
            }
            
            // Proceed with the iterations as long as the method did not converge or reach the maximum number of iterations.
            uint64_t iterations = 0;
            
            SolverStatus status = SolverStatus::InProgress;
            bool doConvergenceCheck = true;
            bool useDiffs = this->hasRelevantValues() && !env.solver().minMax().isSymmetricUpdatesSet();
            std::vector<ValueType> oldValues;
            if (useGaussSeidelMultiplication && useDiffs) {
                oldValues.resize(this->getRelevantValues().getNumberOfSetBits());
            }
            ValueType maxLowerDiff = storm::utility::zero<ValueType>();
            ValueType maxUpperDiff = storm::utility::zero<ValueType>();
            bool relative = env.solver().minMax().getRelativeTerminationCriterion();
            ValueType precision = storm::utility::convertNumber<ValueType>(env.solver().minMax().getPrecision());
            if (!relative) {
                precision *= storm::utility::convertNumber<ValueType>(2.0);
            }
            this->startMeasureProgress();
            while (status == SolverStatus::InProgress && iterations < env.solver().minMax().getMaximalNumberOfIterations()) {
                // Remember in which directions we took steps in this iteration.
                bool lowerStep = false;
                bool upperStep = false;

                // In every thousandth iteration, we improve both bounds.
                if (iterations % 1000 == 0 || maxLowerDiff == maxUpperDiff) {
                    lowerStep = true;
                    upperStep = true;
                    if (useGaussSeidelMultiplication) {
                        if (useDiffs) {
                            preserveOldRelevantValues(*lowerX, this->getRelevantValues(), oldValues);
                        }
                        this->multiplierA->multiplyAndReduceGaussSeidel(env, dir, *lowerX, &b);
                        if (useDiffs) {
                            maxLowerDiff = computeMaxAbsDiff(*lowerX, this->getRelevantValues(), oldValues);
                            preserveOldRelevantValues(*upperX, this->getRelevantValues(), oldValues);
                        }
                        this->multiplierA->multiplyAndReduceGaussSeidel(env, dir, *upperX, &b);
                        if (useDiffs) {
                            maxUpperDiff = computeMaxAbsDiff(*upperX, this->getRelevantValues(), oldValues);
                        }
                    } else {
                        this->multiplierA->multiplyAndReduce(env, dir, *lowerX, &b, *tmp);
                        if (useDiffs) {
                            maxLowerDiff = computeMaxAbsDiff(*lowerX, *tmp, this->getRelevantValues());
                        }
                        std::swap(lowerX, tmp);
                        this->multiplierA->multiplyAndReduce(env, dir, *upperX, &b, *tmp);
                        if (useDiffs) {
                            maxUpperDiff = computeMaxAbsDiff(*upperX, *tmp, this->getRelevantValues());
                        }
                        std::swap(upperX, tmp);
                    }
                } else {
                    // In the following iterations, we improve the bound with the greatest difference.
                    if (useGaussSeidelMultiplication) {
                        if (maxLowerDiff >= maxUpperDiff) {
                            if (useDiffs) {
                                preserveOldRelevantValues(*lowerX, this->getRelevantValues(), oldValues);
                            }
                            this->multiplierA->multiplyAndReduceGaussSeidel(env, dir, *lowerX, &b);
                            if (useDiffs) {
                                maxLowerDiff = computeMaxAbsDiff(*lowerX, this->getRelevantValues(), oldValues);
                            }
                            lowerStep = true;
                        } else {
                            if (useDiffs) {
                                preserveOldRelevantValues(*upperX, this->getRelevantValues(), oldValues);
                            }
                            this->multiplierA->multiplyAndReduceGaussSeidel(env, dir, *upperX, &b);
                            if (useDiffs) {
                                maxUpperDiff = computeMaxAbsDiff(*upperX, this->getRelevantValues(), oldValues);
                            }
                            upperStep = true;
                        }
                    } else {
                        if (maxLowerDiff >= maxUpperDiff) {
                            this->multiplierA->multiplyAndReduce(env, dir, *lowerX, &b, *tmp);
                            if (useDiffs) {
                                maxLowerDiff = computeMaxAbsDiff(*lowerX, *tmp, this->getRelevantValues());
                            }
                            std::swap(tmp, lowerX);
                            lowerStep = true;
                        } else {
                            this->multiplierA->multiplyAndReduce(env, dir, *upperX, &b, *tmp);
                            if (useDiffs) {
                                maxUpperDiff = computeMaxAbsDiff(*upperX, *tmp, this->getRelevantValues());
                            }
                            std::swap(tmp, upperX);
                            upperStep = true;
                        }
                    }
                }
                STORM_LOG_ASSERT(maxLowerDiff >= storm::utility::zero<ValueType>(), "Expected non-negative lower diff.");
                STORM_LOG_ASSERT(maxUpperDiff >= storm::utility::zero<ValueType>(), "Expected non-negative upper diff.");
                if (iterations % 1000 == 0) {
                    STORM_LOG_TRACE("Iteration " << iterations << ": lower difference: " << maxLowerDiff << ", upper difference: " << maxUpperDiff << ".");
                }

                if (doConvergenceCheck) {
                    // Determine whether the method converged.
                    if (this->hasRelevantValues()) {
                        status = storm::utility::vector::equalModuloPrecision<ValueType>(*lowerX, *upperX, this->getRelevantValues(), precision, relative) ? SolverStatus::Converged : status;
                    } else {
                        status = storm::utility::vector::equalModuloPrecision<ValueType>(*lowerX, *upperX, precision, relative) ? SolverStatus::Converged : status;
                    }
                }
                
                // Update environment variables.
                ++iterations;
                doConvergenceCheck = !doConvergenceCheck;
                if (lowerStep) {
                    status = updateStatusIfNotConverged(status, *lowerX, iterations, env.solver().minMax().getMaximalNumberOfIterations(), SolverGuarantee::LessOrEqual);
                }
                if (upperStep) {
                    status = updateStatusIfNotConverged(status, *upperX, iterations, env.solver().minMax().getMaximalNumberOfIterations(), SolverGuarantee::GreaterOrEqual);
                }

                // Potentially show progress.
                this->showProgressIterative(iterations);
            }
            
            reportStatus(status, iterations);
            this->overallPerformedIterations += iterations;

            // We take the means of the lower and upper bound so we guarantee the desired precision.
            ValueType two = storm::utility::convertNumber<ValueType>(2.0);
            storm::utility::vector::applyPointwise<ValueType, ValueType, ValueType>(*lowerX, *upperX, *lowerX, [&two] (ValueType const& a, ValueType const& b) -> ValueType { return (a + b) / two; });
            
            // Since we shuffled the pointer around, we need to write the actual results to the input/output vector x.
            if (&x == tmp) {
                std::swap(x, *tmp);
            } else if (&x == this->auxiliaryRowGroupVector.get()) {
                std::swap(x, *this->auxiliaryRowGroupVector);
            }
            
            // If requested, we store the scheduler for retrieval.
            if (this->isTrackSchedulerSet()) {
                this->schedulerChoices = std::vector<uint_fast64_t>(this->A->getRowGroupCount());
                this->multiplierA->multiplyAndReduce(env, dir, x, &b, *this->auxiliaryRowGroupVector, &this->schedulerChoices.get());
            }
            
            if (!this->isCachingEnabled()) {
                clearCache();
            }
            
            return status == SolverStatus::Converged;
        }
        
        template<typename ValueType>
        class SoundValueIterationHelper {
        public:
            SoundValueIterationHelper(std::vector<ValueType>& x, std::vector<ValueType>& y, bool relative, ValueType const& precision, uint64_t sizeOfLargestRowGroup) : x(x), y(y), hasLowerBound(false), hasUpperBound(false), minIndex(0), maxIndex(0), relative(relative), precision(precision) {
                xTmp.resize(sizeOfLargestRowGroup);
                yTmp.resize(sizeOfLargestRowGroup);
                x.assign(x.size(), storm::utility::zero<ValueType>());
                y.assign(x.size(), storm::utility::one<ValueType>());
                hasDecisionValue = false;
                decisionValueBlocks = false;
                convergencePhase1 = true;
                firstIndexViolatingConvergence = 0;
            }
            
            
            inline void setLowerBound(ValueType const& value) {
                hasLowerBound = true;
                lowerBound = value;
            }
            
            inline void setUpperBound(ValueType const& value) {
                hasUpperBound = true;
                upperBound = value;
            }
            
            template<OptimizationDirection dir>
            inline bool better(ValueType const& val1, ValueType const& val2) {
                return maximize(dir) ? val1 > val2 : val1 < val2;
            }
            
            template<OptimizationDirection dir>
            inline ValueType& getPrimaryBound() {
                return maximize(dir) ? upperBound : lowerBound;
            }
            
            template<OptimizationDirection dir>
            inline bool& hasPrimaryBound() {
                return maximize(dir) ? hasUpperBound : hasLowerBound;
            }
            
            template<OptimizationDirection dir>
            inline ValueType& getSecondaryBound() {
                return maximize(dir) ? lowerBound : upperBound;
            }
            
            template<OptimizationDirection dir>
            inline uint64_t& getPrimaryIndex() {
                return maximize(dir) ? maxIndex : minIndex;
            }

            template<OptimizationDirection dir>
            inline uint64_t& getSecondaryIndex() {
                return maximize(dir) ? minIndex : maxIndex;
            }
            
            void multiplyRow(uint64_t const& row, storm::storage::SparseMatrix<ValueType> const& A, storm::solver::Multiplier<ValueType> const& multiplier, ValueType const& bi, ValueType& xi, ValueType& yi) {
                ValueType xRes = bi;
                ValueType yRes = storm::utility::zero<ValueType>();
                multiplier.multiplyRow2(row, x, xRes, y, yRes);
                xi = std::move(xRes);
                yi = std::move(yRes);
            }
            
            template<OptimizationDirection dir>
            void performIterationStep(storm::storage::SparseMatrix<ValueType> const& A, storm::solver::Multiplier<ValueType> const& multiplier, std::vector<ValueType> const& b) {
                if (!decisionValueBlocks) {
                    performIterationStepUpdateDecisionValue<dir>(A, multiplier, b);
                } else {
                    assert(decisionValue == getPrimaryBound<dir>());
                    auto xIt = x.rbegin();
                    auto yIt = y.rbegin();
                    auto groupStartIt = A.getRowGroupIndices().rbegin();
                    uint64_t groupEnd = *groupStartIt;
                    ++groupStartIt;
                    for (auto groupStartIte = A.getRowGroupIndices().rend(); groupStartIt != groupStartIte; groupEnd = *(groupStartIt++), ++xIt, ++yIt) {
                        // Perform the iteration for the first row in the group
                        uint64_t row = *groupStartIt;
                        ValueType xBest, yBest;
                        multiplyRow(row, A, multiplier, b[row], xBest, yBest);
                        ++row;
                        // Only do more work if there are still rows in this row group
                        if (row != groupEnd) {
                            ValueType xi, yi;
                            ValueType bestValue = xBest + yBest * getPrimaryBound<dir>();
                            for (;row < groupEnd; ++row) {
                                // Get the multiplication results
                                multiplyRow(row, A, multiplier, b[row], xi, yi);
                                ValueType currentValue = xi + yi * getPrimaryBound<dir>();
                                // Check if the current row is better then the previously found one
                                if (better<dir>(currentValue, bestValue)) {
                                    xBest = std::move(xi);
                                    yBest = std::move(yi);
                                    bestValue = std::move(currentValue);
                                } else if (currentValue == bestValue && yBest > yi) {
                                    // If the value for this row is not strictly better, it might still be equal and have a better y value
                                    xBest = std::move(xi);
                                    yBest = std::move(yi);
                                }
                            }
                        }
                        *xIt = std::move(xBest);
                        *yIt = std::move(yBest);
                    }
                }
            }
            
            template<OptimizationDirection dir>
            void performIterationStepUpdateDecisionValue(storm::storage::SparseMatrix<ValueType> const& A, storm::solver::Multiplier<ValueType> const& multiplier, std::vector<ValueType> const& b) {
                auto xIt = x.rbegin();
                auto yIt = y.rbegin();
                auto groupStartIt = A.getRowGroupIndices().rbegin();
                uint64_t groupEnd = *groupStartIt;
                ++groupStartIt;
                for (auto groupStartIte = A.getRowGroupIndices().rend(); groupStartIt != groupStartIte; groupEnd = *(groupStartIt++), ++xIt, ++yIt) {
                    // Perform the iteration for the first row in the group
                    uint64_t row = *groupStartIt;
                    ValueType xBest, yBest;
                    multiplyRow(row, A, multiplier, b[row], xBest, yBest);
                    ++row;
                    // Only do more work if there are still rows in this row group
                    if (row != groupEnd) {
                        ValueType xi, yi;
                        uint64_t xyTmpIndex = 0;
                        if (hasPrimaryBound<dir>()) {
                            ValueType bestValue = xBest + yBest * getPrimaryBound<dir>();
                            for (;row < groupEnd; ++row) {
                                // Get the multiplication results
                                multiplyRow(row, A, multiplier, b[row], xi, yi);
                                ValueType currentValue = xi + yi * getPrimaryBound<dir>();
                                // Check if the current row is better then the previously found one
                                if (better<dir>(currentValue, bestValue)) {
                                    if (yBest < yi) {
                                        // We need to store the 'old' best value as it might be relevant for the decision value
                                        xTmp[xyTmpIndex] = std::move(xBest);
                                        yTmp[xyTmpIndex] = std::move(yBest);
                                        ++xyTmpIndex;
                                    }
                                    xBest = std::move(xi);
                                    yBest = std::move(yi);
                                    bestValue = std::move(currentValue);
                                } else if (yBest > yi) {
                                    // If the value for this row is not strictly better, it might still be equal and have a better y value
                                    if (currentValue == bestValue) {
                                        xBest = std::move(xi);
                                        yBest = std::move(yi);
                                    } else {
                                        xTmp[xyTmpIndex] = std::move(xi);
                                        yTmp[xyTmpIndex] = std::move(yi);
                                        ++xyTmpIndex;
                                    }
                                }
                            }
                        } else {
                            for (;row < groupEnd; ++row) {
                                multiplyRow(row, A, multiplier, b[row], xi, yi);
                                // Update the best choice
                                if (yi > yBest || (yi == yBest && better<dir>(xi, xBest))) {
                                        xTmp[xyTmpIndex] = std::move(xBest);
                                        yTmp[xyTmpIndex] = std::move(yBest);
                                        ++xyTmpIndex;
                                    xBest = std::move(xi);
                                    yBest = std::move(yi);
                                } else {
                                    xTmp[xyTmpIndex] = std::move(xi);
                                    yTmp[xyTmpIndex] = std::move(yi);
                                    ++xyTmpIndex;
                                }
                            }
                        }
                        
                        // Update the decision value
                        for (uint64_t i = 0; i < xyTmpIndex; ++i) {
                            ValueType deltaY = yBest - yTmp[i];
                            if (deltaY > storm::utility::zero<ValueType>()) {
                                ValueType newDecisionValue = (xTmp[i] - xBest) / deltaY;
                                if (!hasDecisionValue || better<dir>(newDecisionValue, decisionValue)) {
                                    decisionValue = std::move(newDecisionValue);
                                    hasDecisionValue = true;
                                }
                            }
                        }
                    }
                    *xIt = std::move(xBest);
                    *yIt = std::move(yBest);
                }
            }

            template<OptimizationDirection dir>
            bool checkConvergenceUpdateBounds(storm::storage::BitVector const* relevantValues = nullptr) {
                
                if (convergencePhase1) {
                    if (checkConvergencePhase1()) {
                        firstIndexViolatingConvergence = 0;
                        if (relevantValues != nullptr) {
                            firstIndexViolatingConvergence = relevantValues->getNextSetIndex(firstIndexViolatingConvergence);
                        }
                    } else {
                        return false;
                    }
                }
                STORM_LOG_ASSERT(!std::any_of(y.begin(), y.end(), [](ValueType value){return storm::utility::isOne(value);}), "Did not expect staying-probability 1 at this point.");
                
                // Reaching this point means that we are in Phase 2:
                // The difference between lower and upper bound has to be < precision at every (relevant) value
                
                // For efficiency reasons we first check whether it is worth to compute the actual bounds. We do so by considering possibly too tight bounds
                ValueType lowerBoundCandidate, upperBoundCandidate;
                if (preliminaryConvergenceCheck<dir>(lowerBoundCandidate, upperBoundCandidate)) {
                    updateLowerUpperBound<dir>(lowerBoundCandidate, upperBoundCandidate);
                    checkIfDecisionValueBlocks<dir>();
                    return checkConvergencePhase2<dir>(relevantValues);
                }
                return false;
            }
            
            void setSolutionVector() {
                STORM_LOG_WARN_COND(hasLowerBound && hasUpperBound, "No lower or upper result bound could be computed within the given number of Iterations.");
            
                ValueType meanBound = (upperBound + lowerBound) / storm::utility::convertNumber<ValueType>(2.0);
                storm::utility::vector::applyPointwise(x, y, x, [&meanBound] (ValueType const& xi, ValueType const& yi) { return xi + yi * meanBound; });
                
                STORM_LOG_INFO("Sound Value Iteration terminated with lower value bound "
                                       << (hasLowerBound ? lowerBound : storm::utility::zero<ValueType>()) << (hasLowerBound ? "" : "(none)")
                                       << " and upper value bound "
                                       << (hasUpperBound ? upperBound : storm::utility::zero<ValueType>()) << (hasUpperBound ? "" : "(none)")
                                       << ". Decision value is "
                                       << (hasDecisionValue ? decisionValue : storm::utility::zero<ValueType>()) << (hasDecisionValue ? "" : "(none)")
                                       << ".");
    
                }
                
        private:
            
            bool checkConvergencePhase1() {
                // Return true if y ('the probability to stay within the matrix') is  < 1 at every entry
                for (; firstIndexViolatingConvergence != y.size(); ++firstIndexViolatingConvergence) {
                    static_assert(NumberTraits<ValueType>::IsExact || std::is_same<ValueType, double>::value, "Considered ValueType not handled.");
                    if (NumberTraits<ValueType>::IsExact) {
                        if (storm::utility::isOne(y[firstIndexViolatingConvergence])) {
                            return false;
                        }
                    } else {
                        if (storm::utility::isAlmostOne(storm::utility::convertNumber<double>(y[firstIndexViolatingConvergence]))) {
                            return false;
                        }
                    }
                }
                convergencePhase1 = false;
                return true;
            }
            
            
            bool isPreciseEnough(ValueType const& xi, ValueType const& yi, ValueType const& lb, ValueType const& ub) {
                return yi * (ub - lb) <= storm::utility::abs<ValueType>((relative ? (precision * xi) : (precision * storm::utility::convertNumber<ValueType>(2.0))));
            }
            
            template<OptimizationDirection dir>
            bool preliminaryConvergenceCheck(ValueType& lowerBoundCandidate, ValueType& upperBoundCandidate) {
                lowerBoundCandidate = x[minIndex] / (storm::utility::one<ValueType>() - y[minIndex]);
                upperBoundCandidate = x[maxIndex] / (storm::utility::one<ValueType>() - y[maxIndex]);
                // Make sure that these candidates are at least as tight as the already known bounds
                if (hasLowerBound && lowerBoundCandidate < lowerBound) {
                    lowerBoundCandidate = lowerBound;
                }
                if (hasUpperBound && upperBoundCandidate > upperBound) {
                    upperBoundCandidate = upperBound;
                }
                if (isPreciseEnough(x[firstIndexViolatingConvergence], y[firstIndexViolatingConvergence], lowerBoundCandidate, upperBoundCandidate)) {
                    return true;
                }
                if (!decisionValueBlocks) {
                    return hasDecisionValue && better<dir>(decisionValue, getPrimaryBound<dir>());
                }
                return false;
            }
            
            template<OptimizationDirection dir>
            void updateLowerUpperBound(ValueType& lowerBoundCandidate, ValueType& upperBoundCandidate) {
                auto xIt = x.begin();
                auto xIte = x.end();
                auto yIt = y.begin();
                for (uint64_t index = 0; xIt != xIte; ++xIt, ++yIt, ++index) {
                    ValueType currentBound = *xIt / (storm::utility::one<ValueType>() - *yIt);
                    if (decisionValueBlocks) {
                        if (better<dir>(getSecondaryBound<dir>(), currentBound)) {
                            getSecondaryIndex<dir>() = index;
                            getSecondaryBound<dir>() = std::move(currentBound);
                        }
                    } else {
                        if (currentBound < lowerBoundCandidate) {
                            minIndex = index;
                            lowerBoundCandidate = std::move(currentBound);
                        } else if (currentBound > upperBoundCandidate) {
                            maxIndex = index;
                            upperBoundCandidate = std::move(currentBound);
                        }
                    }
                }
                if ((maximize(dir) || !decisionValueBlocks) && (!hasLowerBound || lowerBoundCandidate > lowerBound)) {
                    setLowerBound(lowerBoundCandidate);
                }
                if ((minimize(dir) || !decisionValueBlocks) && (!hasUpperBound || upperBoundCandidate < upperBound)) {
                    setUpperBound(upperBoundCandidate);
                }
            }
            
            template<OptimizationDirection dir>
            void checkIfDecisionValueBlocks() {
                // Check whether the decision value blocks now (i.e. further improvement of the primary bound would lead to a non-optimal scheduler).
                if (!decisionValueBlocks && hasDecisionValue && better<dir>(decisionValue, getPrimaryBound<dir>())) {
                    getPrimaryBound<dir>() = decisionValue;
                    decisionValueBlocks = true;
                }
            }
            
            template<OptimizationDirection dir>
            bool checkConvergencePhase2(storm::storage::BitVector const* relevantValues = nullptr) {
                // Check whether the desired precision is reached
                if (isPreciseEnough(x[firstIndexViolatingConvergence], y[firstIndexViolatingConvergence], lowerBound, upperBound)) {
                    // The current index satisfies the desired bound. We now move to the next index that violates it
                    while (true) {
                        ++firstIndexViolatingConvergence;
                        if (relevantValues != nullptr) {
                            firstIndexViolatingConvergence = relevantValues->getNextSetIndex(firstIndexViolatingConvergence);
                        }
                        if (firstIndexViolatingConvergence == x.size()) {
                            // Converged!
                            return true;
                        } else {
                            if (!isPreciseEnough(x[firstIndexViolatingConvergence], y[firstIndexViolatingConvergence], lowerBound, upperBound)) {
                                // not converged yet
                                return false;
                            }
                        }
                    }
                }
                return false;
            }
            
            std::vector<ValueType>& x;
            std::vector<ValueType>& y;
            std::vector<ValueType> xTmp, yTmp;
            
            ValueType lowerBound, upperBound, decisionValue;
            bool hasLowerBound, hasUpperBound, hasDecisionValue;
            uint64_t minIndex, maxIndex;
            bool convergencePhase1;
            bool decisionValueBlocks;
            uint64_t firstIndexViolatingConvergence;
            
            bool relative;
            ValueType precision;
        };
        
        template<typename ValueType>
        bool IterativeMinMaxLinearEquationSolver<ValueType>::solveEquationsSoundValueIteration(Environment const& env, OptimizationDirection dir, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {

            // Prepare the solution vectors.
            assert(x.size() == this->A->getRowGroupCount());
            if (!this->auxiliaryRowGroupVector) {
                this->auxiliaryRowGroupVector = std::make_unique<std::vector<ValueType>>();
            }
            
            if (!this->multiplierA) {
                this->multiplierA = storm::solver::MultiplierFactory<ValueType>().create(env, *this->A);
            }
            
            SoundValueIterationHelper<ValueType> helper(x, *this->auxiliaryRowGroupVector, env.solver().minMax().getRelativeTerminationCriterion(), storm::utility::convertNumber<ValueType>(env.solver().minMax().getPrecision()), this->A->getSizeOfLargestRowGroup());

            // Prepare initial bounds for the solution (if given)
            if (this->hasLowerBound()) {
                helper.setLowerBound(this->getLowerBound(true));
            }
            if (this->hasUpperBound()) {
                helper.setUpperBound(this->getUpperBound(true));
            }
            
            storm::storage::BitVector const* relevantValuesPtr = nullptr;
            if (this->hasRelevantValues()) {
                relevantValuesPtr = &this->getRelevantValues();
            }
            
            SolverStatus status = SolverStatus::InProgress;
            this->startMeasureProgress();
            uint64_t iterations = 0;
            
            while (status == SolverStatus::InProgress && iterations < env.solver().minMax().getMaximalNumberOfIterations()) {
                if (minimize(dir)) {
                    helper.template performIterationStep<OptimizationDirection::Minimize>(*this->A, *this->multiplierA, b);
                    if (helper.template checkConvergenceUpdateBounds<OptimizationDirection::Minimize>(relevantValuesPtr)) {
                        status = SolverStatus::Converged;
                    }
                } else {
                    assert(maximize(dir));
                    helper.template performIterationStep<OptimizationDirection::Maximize>(*this->A, *this->multiplierA, b);
                    if (helper.template checkConvergenceUpdateBounds<OptimizationDirection::Maximize>(relevantValuesPtr)) {
                        status = SolverStatus::Converged;
                    }
                }

                // Update environment variables.
                ++iterations;
                // TODO: Implement custom termination criterion. We would need to add our errors to the stepBoundedX values (only if in second phase)
                status = updateStatusIfNotConverged(status, x, iterations, env.solver().minMax().getMaximalNumberOfIterations(), SolverGuarantee::None);
                
                // Potentially show progress.
                this->showProgressIterative(iterations);
            }
            helper.setSolutionVector();
            
            // If requested, we store the scheduler for retrieval.
            if (this->isTrackSchedulerSet()) {
                this->schedulerChoices = std::vector<uint_fast64_t>(this->A->getRowGroupCount());
                this->A->multiplyAndReduce(dir, this->A->getRowGroupIndices(), x, &b, *this->auxiliaryRowGroupVector, &this->schedulerChoices.get());
            }

            reportStatus(status, iterations);
            
            this->overallPerformedIterations += iterations;
            
            if (!this->isCachingEnabled()) {
                clearCache();
            }
            
            return status == SolverStatus::Converged;
        }
        
        template<typename ValueType>
        bool IterativeMinMaxLinearEquationSolver<ValueType>::isSolution(storm::OptimizationDirection dir, storm::storage::SparseMatrix<ValueType> const& matrix, std::vector<ValueType> const& values, std::vector<ValueType> const& b) {
            storm::utility::ConstantsComparator<ValueType> comparator;
            
            auto valueIt = values.begin();
            auto bIt = b.begin();
            for (uint64_t group = 0; group < matrix.getRowGroupCount(); ++group, ++valueIt) {
                ValueType groupValue = *bIt;
                uint64_t row = matrix.getRowGroupIndices()[group];
                groupValue += matrix.multiplyRowWithVector(row, values);

                ++row;
                ++bIt;

                for (auto endRow = matrix.getRowGroupIndices()[group + 1]; row < endRow; ++row, ++bIt) {
                    ValueType newValue = *bIt;
                    newValue += matrix.multiplyRowWithVector(row, values);
                    
                    if ((dir == storm::OptimizationDirection::Minimize && newValue < groupValue) || (dir == storm::OptimizationDirection::Maximize && newValue > groupValue)) {
                        groupValue = newValue;
                    }
                }
                
                // If the value does not match the one in the values vector, the given vector is not a solution.
                if (!comparator.isEqual(groupValue, *valueIt)) {
                    return false;
                }
            }
            
            // Checked all values at this point.
            return true;
        }

        template<typename ValueType>
        template<typename RationalType, typename ImpreciseType>
        bool IterativeMinMaxLinearEquationSolver<ValueType>::sharpen(storm::OptimizationDirection dir, uint64_t precision, storm::storage::SparseMatrix<RationalType> const& A, std::vector<ImpreciseType> const& x, std::vector<RationalType> const& b, std::vector<RationalType>& tmp) {
            
            for (uint64_t p = 0; p <= precision; ++p) {
                storm::utility::kwek_mehlhorn::sharpen(p, x, tmp);

                if (IterativeMinMaxLinearEquationSolver<RationalType>::isSolution(dir, A, tmp, b)) {
                    return true;
                }
            }
            return false;
        }

        template<typename ValueType>
        template<typename ImpreciseType>
        typename std::enable_if<std::is_same<ValueType, ImpreciseType>::value && !NumberTraits<ValueType>::IsExact, bool>::type IterativeMinMaxLinearEquationSolver<ValueType>::solveEquationsRationalSearchHelper(Environment const& env, OptimizationDirection dir, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            // Version for when the overall value type is imprecise.

            // Create a rational representation of the input so we can check for a proper solution later.
            storm::storage::SparseMatrix<storm::RationalNumber> rationalA = this->A->template toValueType<storm::RationalNumber>();
            std::vector<storm::RationalNumber> rationalX(x.size());
            std::vector<storm::RationalNumber> rationalB = storm::utility::vector::convertNumericVector<storm::RationalNumber>(b);
            
            if (!this->multiplierA) {
                this->multiplierA = storm::solver::MultiplierFactory<ValueType>().create(env, *this->A);
            }
            
            if (!auxiliaryRowGroupVector) {
                auxiliaryRowGroupVector = std::make_unique<std::vector<ValueType>>(this->A->getRowGroupCount());
            }
            
            // Forward the call to the core rational search routine.
            bool converged = solveEquationsRationalSearchHelper<storm::RationalNumber, ImpreciseType>(env, dir, *this, rationalA, rationalX, rationalB, *this->A, x, b, *auxiliaryRowGroupVector);
            
            // Translate back rational result to imprecise result.
            auto targetIt = x.begin();
            for (auto it = rationalX.begin(), ite = rationalX.end(); it != ite; ++it, ++targetIt) {
                *targetIt = storm::utility::convertNumber<ValueType>(*it);
            }

            if (!this->isCachingEnabled()) {
                this->clearCache();
            }
            
            return converged;
        }
        
        template<typename ValueType>
        template<typename ImpreciseType>
        typename std::enable_if<std::is_same<ValueType, ImpreciseType>::value && NumberTraits<ValueType>::IsExact, bool>::type IterativeMinMaxLinearEquationSolver<ValueType>::solveEquationsRationalSearchHelper(Environment const& env, OptimizationDirection dir, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            // Version for when the overall value type is exact and the same type is to be used for the imprecise part.
            
            if (!this->multiplierA) {
                this->multiplierA = storm::solver::MultiplierFactory<ValueType>().create(env, *this->A);
            }
            
            if (!auxiliaryRowGroupVector) {
                auxiliaryRowGroupVector = std::make_unique<std::vector<ValueType>>(this->A->getRowGroupCount());
            }
            
            // Forward the call to the core rational search routine.
            bool converged = solveEquationsRationalSearchHelper<ValueType, ImpreciseType>(env, dir, *this, *this->A, x, b, *this->A, *auxiliaryRowGroupVector, b, x);

            if (!this->isCachingEnabled()) {
                this->clearCache();
            }
            
            return converged;
        }

        template<typename ValueType>
        template<typename ImpreciseType>
        typename std::enable_if<!std::is_same<ValueType, ImpreciseType>::value, bool>::type IterativeMinMaxLinearEquationSolver<ValueType>::solveEquationsRationalSearchHelper(Environment const& env, OptimizationDirection dir, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            // Version for when the overall value type is exact and the imprecise one is not. We first try to solve the
            // problem using the imprecise data type and fall back to the exact type as needed.
            
            // Translate A to its imprecise version.
            storm::storage::SparseMatrix<ImpreciseType> impreciseA = this->A->template toValueType<ImpreciseType>();
            
            // Translate x to its imprecise version.
            std::vector<ImpreciseType> impreciseX(x.size());
            {
                std::vector<ValueType> tmp(x.size());
                this->createLowerBoundsVector(tmp);
                auto targetIt = impreciseX.begin();
                for (auto sourceIt = tmp.begin(); targetIt != impreciseX.end(); ++targetIt, ++sourceIt) {
                    *targetIt = storm::utility::convertNumber<ImpreciseType, ValueType>(*sourceIt);
                }
            }
            
            // Create temporary storage for an imprecise x.
            std::vector<ImpreciseType> impreciseTmpX(x.size());
            
            // Translate b to its imprecise version.
            std::vector<ImpreciseType> impreciseB(b.size());
            auto targetIt = impreciseB.begin();
            for (auto sourceIt = b.begin(); targetIt != impreciseB.end(); ++targetIt, ++sourceIt) {
                *targetIt = storm::utility::convertNumber<ImpreciseType, ValueType>(*sourceIt);
            }
            
            // Create imprecise solver from the imprecise data.
            IterativeMinMaxLinearEquationSolver<ImpreciseType> impreciseSolver(std::make_unique<storm::solver::GeneralLinearEquationSolverFactory<ImpreciseType>>());
            impreciseSolver.setMatrix(impreciseA);
            impreciseSolver.setCachingEnabled(true);
            impreciseSolver.multiplierA = storm::solver::MultiplierFactory<ImpreciseType>().create(env, impreciseA);
            
            bool converged = false;
            try {
                // Forward the call to the core rational search routine.
                converged = solveEquationsRationalSearchHelper<ValueType, ImpreciseType>(env, dir, impreciseSolver, *this->A, x, b, impreciseA, impreciseX, impreciseB, impreciseTmpX);
                impreciseSolver.clearCache();
            } catch (storm::exceptions::PrecisionExceededException const& e) {
                STORM_LOG_WARN("Precision of value type was exceeded, trying to recover by switching to rational arithmetic.");
                
                if (!auxiliaryRowGroupVector) {
                    auxiliaryRowGroupVector = std::make_unique<std::vector<ValueType>>(this->A->getRowGroupCount());
                }

                // Translate the imprecise value iteration result to the one we are going to use from now on.
                auto targetIt = auxiliaryRowGroupVector->begin();
                for (auto it = impreciseX.begin(), ite = impreciseX.end(); it != ite; ++it, ++targetIt) {
                    *targetIt = storm::utility::convertNumber<ValueType>(*it);
                }
                
                // Get rid of the superfluous data structures.
                impreciseX = std::vector<ImpreciseType>();
                impreciseTmpX = std::vector<ImpreciseType>();
                impreciseB = std::vector<ImpreciseType>();
                impreciseA = storm::storage::SparseMatrix<ImpreciseType>();

                if (!this->multiplierA) {
                    this->multiplierA = storm::solver::MultiplierFactory<ValueType>().create(env, *this->A);
                }
                
                // Forward the call to the core rational search routine, but now with our value type as the imprecise value type.
                converged = solveEquationsRationalSearchHelper<ValueType, ValueType>(env, dir, *this, *this->A, x, b, *this->A, *auxiliaryRowGroupVector, b, x);
            }
            
            if (!this->isCachingEnabled()) {
                this->clearCache();
            }
            
            return converged;
        }

        template<typename RationalType, typename ImpreciseType>
        struct TemporaryHelper {
            static std::vector<RationalType>* getTemporary(std::vector<RationalType>& rationalX, std::vector<ImpreciseType>*& currentX, std::vector<ImpreciseType>*& newX) {
                return &rationalX;
            }
            
            static void swapSolutions(std::vector<RationalType>& rationalX, std::vector<RationalType>*& rationalSolution, std::vector<ImpreciseType>& x, std::vector<ImpreciseType>*& currentX, std::vector<ImpreciseType>*& newX) {
                // Nothing to do.
            }
        };
        
        template<typename RationalType>
        struct TemporaryHelper<RationalType, RationalType> {
            static std::vector<RationalType>* getTemporary(std::vector<RationalType>& rationalX, std::vector<RationalType>*& currentX, std::vector<RationalType>*& newX) {
                return newX;
            }

            static void swapSolutions(std::vector<RationalType>& rationalX, std::vector<RationalType>*& rationalSolution, std::vector<RationalType>& x, std::vector<RationalType>*& currentX, std::vector<RationalType>*& newX) {
                if (&rationalX == rationalSolution) {
                    // In this case, the rational solution is in place.
                    
                    // However, since the rational solution is no alias to current x, the imprecise solution is stored
                    // in current x and and rational x is not an alias to x, we can swap the contents of currentX to x.
                    std::swap(x, *currentX);
                } else {
                    // Still, we may assume that the rational solution is not current x and is therefore new x.
                    std::swap(rationalX, *rationalSolution);
                    std::swap(x, *currentX);
                }
            }
        };
        
        template<typename ValueType>
        template<typename RationalType, typename ImpreciseType>
        bool IterativeMinMaxLinearEquationSolver<ValueType>::solveEquationsRationalSearchHelper(Environment const& env, OptimizationDirection dir, IterativeMinMaxLinearEquationSolver<ImpreciseType> const& impreciseSolver, storm::storage::SparseMatrix<RationalType> const& rationalA, std::vector<RationalType>& rationalX, std::vector<RationalType> const& rationalB, storm::storage::SparseMatrix<ImpreciseType> const& A, std::vector<ImpreciseType>& x, std::vector<ImpreciseType> const& b, std::vector<ImpreciseType>& tmpX) const {
            
            std::vector<ImpreciseType>* currentX = &x;
            std::vector<ImpreciseType>* newX = &tmpX;

            SolverStatus status = SolverStatus::InProgress;
            uint64_t overallIterations = 0;
            uint64_t valueIterationInvocations = 0;
            ValueType precision = storm::utility::convertNumber<ValueType>(env.solver().minMax().getPrecision());
            impreciseSolver.startMeasureProgress();
            while (status == SolverStatus::InProgress && overallIterations < env.solver().minMax().getMaximalNumberOfIterations()) {
                // Perform value iteration with the current precision.
                typename IterativeMinMaxLinearEquationSolver<ImpreciseType>::ValueIterationResult result = impreciseSolver.performValueIteration(env, dir, currentX, newX, b, storm::utility::convertNumber<ImpreciseType, ValueType>(precision), env.solver().minMax().getRelativeTerminationCriterion(), SolverGuarantee::LessOrEqual, overallIterations, env.solver().minMax().getMaximalNumberOfIterations(), env.solver().minMax().getMultiplicationStyle());
                
                // At this point, the result of the imprecise value iteration is stored in the (imprecise) current x.
                
                ++valueIterationInvocations;
                STORM_LOG_TRACE("Completed " << valueIterationInvocations << " value iteration invocations, the last one with precision " << precision << " completed in " << result.iterations << " iterations.");
                
                // Count the iterations.
                overallIterations += result.iterations;
                
                // Compute maximal precision until which to sharpen.
                uint64_t p = storm::utility::convertNumber<uint64_t>(storm::utility::ceil(storm::utility::log10<ValueType>(storm::utility::one<ValueType>() / precision)));
                
                // Make sure that currentX and rationalX are not aliased.
                std::vector<RationalType>* temporaryRational = TemporaryHelper<RationalType, ImpreciseType>::getTemporary(rationalX, currentX, newX);
                
                // Sharpen solution and place it in the temporary rational.
                bool foundSolution = sharpen(dir, p, rationalA, *currentX, rationalB, *temporaryRational);
                
                // After sharpen, if a solution was found, it is contained in the free rational.
                
                if (foundSolution) {
                    status = SolverStatus::Converged;
                    
                    TemporaryHelper<RationalType, ImpreciseType>::swapSolutions(rationalX, temporaryRational, x, currentX, newX);
                } else {
                    // Increase the precision.
                    precision /= storm::utility::convertNumber<ValueType>(static_cast<uint64_t>(10));
                }
            }
            
            if (status == SolverStatus::InProgress && overallIterations == env.solver().minMax().getMaximalNumberOfIterations()) {
                status = SolverStatus::MaximalIterationsExceeded;
            }
            
            reportStatus(status, overallIterations);
            
            return status == SolverStatus::Converged || status == SolverStatus::TerminatedEarly;
        }

        template<typename ValueType>
        bool IterativeMinMaxLinearEquationSolver<ValueType>::solveEquationsRationalSearch(Environment const& env, OptimizationDirection dir, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            return solveEquationsRationalSearchHelper<double>(env, dir, x, b);
        }
        
        template<typename ValueType>
        void IterativeMinMaxLinearEquationSolver<ValueType>::computeOptimalValueForRowGroup(uint_fast64_t group, OptimizationDirection dir, std::vector<ValueType>& x, std::vector<ValueType> const& b, uint_fast64_t* choice) const {
            uint64_t row = this->A->getRowGroupIndices()[group];
            uint64_t groupEnd = this->A->getRowGroupIndices()[group + 1];
            assert(row != groupEnd);
            
            auto bIt = b.begin() + row;
            ValueType& xi = x[group];
            xi = this->A->multiplyRowWithVector(row, x) + *bIt;
            uint64_t optimalRow = row;
            
            for (++row, ++bIt; row < groupEnd; ++row, ++bIt) {
                ValueType choiceVal = this->A->multiplyRowWithVector(row, x) + *bIt;
                if (minimize(dir)) {
                    if (choiceVal < xi) {
                        xi = choiceVal;
                        optimalRow = row;
                    }
                } else {
                    if (choiceVal > xi) {
                        xi = choiceVal;
                        optimalRow = row;
                    }
                }
            }
            if (choice != nullptr) {
                *choice = optimalRow - this->A->getRowGroupIndices()[group];
            }
        }

        template<typename ValueType>
        SolverStatus IterativeMinMaxLinearEquationSolver<ValueType>::updateStatusIfNotConverged(SolverStatus status, std::vector<ValueType> const& x, uint64_t iterations, uint64_t maximalNumberOfIterations, SolverGuarantee const& guarantee) const {
            if (status != SolverStatus::Converged) {
                if (this->hasCustomTerminationCondition() && this->getTerminationCondition().terminateNow(x, guarantee)) {
                    status = SolverStatus::TerminatedEarly;
                } else if (iterations >= maximalNumberOfIterations) {
                    status = SolverStatus::MaximalIterationsExceeded;
                }
            }
            return status;
        }
        
        template<typename ValueType>
        void IterativeMinMaxLinearEquationSolver<ValueType>::reportStatus(SolverStatus status, uint64_t iterations) {
            switch (status) {
                case SolverStatus::Converged: STORM_LOG_INFO("Iterative solver converged after " << iterations << " iterations."); break;
                case SolverStatus::TerminatedEarly: STORM_LOG_INFO("Iterative solver terminated early after " << iterations << " iterations."); break;
                case SolverStatus::MaximalIterationsExceeded: STORM_LOG_WARN("Iterative solver did not converge after " << iterations << " iterations."); break;
                default:
                    STORM_LOG_THROW(false, storm::exceptions::InvalidStateException, "Iterative solver terminated unexpectedly.");
            }
        }
        
        template<typename ValueType>
        void IterativeMinMaxLinearEquationSolver<ValueType>::clearCache() const {
            multiplierA.reset();
            auxiliaryRowGroupVector.reset();
            auxiliaryRowGroupVector2.reset();
            StandardMinMaxLinearEquationSolver<ValueType>::clearCache();
        }
        
        template class IterativeMinMaxLinearEquationSolver<double>;
        
#ifdef STORM_HAVE_CARL
        template class IterativeMinMaxLinearEquationSolver<storm::RationalNumber>;
#endif
    }
}
