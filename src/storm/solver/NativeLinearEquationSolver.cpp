#include "storm/solver/NativeLinearEquationSolver.h"

#include "storm/environment/solver/NativeSolverEnvironment.h"

#include "storm/utility/ConstantsComparator.h"
#include "storm/utility/KwekMehlhorn.h"
#include "storm/utility/NumberTraits.h"
#include "storm/utility/constants.h"
#include "storm/utility/vector.h"
#include "storm/solver/Multiplier.h"
#include "storm/exceptions/InvalidStateException.h"
#include "storm/exceptions/InvalidEnvironmentException.h"
#include "storm/exceptions/UnmetRequirementException.h"
#include "storm/exceptions/PrecisionExceededException.h"


namespace storm {
    namespace solver {

        template<typename ValueType>
        NativeLinearEquationSolver<ValueType>::NativeLinearEquationSolver() : localA(nullptr), A(nullptr) {
            // Intentionally left empty.
        }

        template<typename ValueType>
        NativeLinearEquationSolver<ValueType>::NativeLinearEquationSolver(storm::storage::SparseMatrix<ValueType> const& A) : localA(nullptr), A(nullptr) {
            this->setMatrix(A);
        }

        template<typename ValueType>
        NativeLinearEquationSolver<ValueType>::NativeLinearEquationSolver(storm::storage::SparseMatrix<ValueType>&& A) : localA(nullptr), A(nullptr) {
            this->setMatrix(std::move(A));
        }
        
        template<typename ValueType>
        void NativeLinearEquationSolver<ValueType>::setMatrix(storm::storage::SparseMatrix<ValueType> const& A) {
            localA.reset();
            this->A = &A;
            clearCache();
        }

        template<typename ValueType>
        void NativeLinearEquationSolver<ValueType>::setMatrix(storm::storage::SparseMatrix<ValueType>&& A) {
            localA = std::make_unique<storm::storage::SparseMatrix<ValueType>>(std::move(A));
            this->A = localA.get();
            clearCache();
        }

        template<typename ValueType>
        bool NativeLinearEquationSolver<ValueType>::solveEquationsSOR(Environment const& env, std::vector<ValueType>& x, std::vector<ValueType> const& b, ValueType const& omega) const {
            STORM_LOG_INFO("Solving linear equation system (" << x.size() << " rows) with NativeLinearEquationSolver (Gauss-Seidel, SOR omega = " << omega << ")");

            if (!this->cachedRowVector) {
                this->cachedRowVector = std::make_unique<std::vector<ValueType>>(getMatrixRowCount());
            }
            
            ValueType precision = storm::utility::convertNumber<ValueType>(env.solver().native().getPrecision());
            uint64_t maxIter = env.solver().native().getMaximalNumberOfIterations();
            bool relative = env.solver().native().getRelativeTerminationCriterion();
            
            // Set up additional environment variables.
            uint_fast64_t iterations = 0;
            bool converged = false;
            bool terminate = false;
            
            this->startMeasureProgress();
            while (!converged && !terminate && iterations < maxIter) {
                A->performSuccessiveOverRelaxationStep(omega, x, b);
                
                // Now check if the process already converged within our precision.
                converged = storm::utility::vector::equalModuloPrecision<ValueType>(*this->cachedRowVector, x, precision, relative);
                terminate = this->terminateNow(x, SolverGuarantee::None);
                
                // If we did not yet converge, we need to backup the contents of x.
                if (!converged) {
                    *this->cachedRowVector = x;
                }
                
                // Potentially show progress.
                this->showProgressIterative(iterations);

                // Increase iteration count so we can abort if convergence is too slow.
                ++iterations;
            }
            
            if (!this->isCachingEnabled()) {
                clearCache();
            }
            
            this->logIterations(converged, terminate, iterations);
            
            return converged;
        }
    
        template<typename ValueType>
        NativeLinearEquationSolver<ValueType>::JacobiDecomposition::JacobiDecomposition(Environment const& env, storm::storage::SparseMatrix<ValueType> const& A) {
            auto decomposition = A.getJacobiDecomposition();
            this->LUMatrix = std::move(decomposition.first);
            this->DVector = std::move(decomposition.second);
            this->multiplier = storm::solver::MultiplierFactory<ValueType>().create(env, this->LUMatrix);
        }
        
        template<typename ValueType>
        bool NativeLinearEquationSolver<ValueType>::solveEquationsJacobi(Environment const& env, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            STORM_LOG_INFO("Solving linear equation system (" << x.size() << " rows) with NativeLinearEquationSolver (Jacobi)");
            
            if (!this->cachedRowVector) {
                this->cachedRowVector = std::make_unique<std::vector<ValueType>>(getMatrixRowCount());
            }
            
            // Get a Jacobi decomposition of the matrix A.
            if (!jacobiDecomposition) {
                jacobiDecomposition = std::make_unique<JacobiDecomposition>(env, *A);
            }
            
            ValueType precision = storm::utility::convertNumber<ValueType>(env.solver().native().getPrecision());
            uint64_t maxIter = env.solver().native().getMaximalNumberOfIterations();
            bool relative = env.solver().native().getRelativeTerminationCriterion();

            std::vector<ValueType>* currentX = &x;
            std::vector<ValueType>* nextX = this->cachedRowVector.get();
            
            // Set up additional environment variables.
            uint_fast64_t iterations = 0;
            bool converged = false;
            bool terminate = false;

            this->startMeasureProgress();
            while (!converged && !terminate && iterations < maxIter) {
                // Compute D^-1 * (b - LU * x) and store result in nextX.
                jacobiDecomposition->multiplier->multiply(env, *currentX, nullptr, *nextX);
                storm::utility::vector::subtractVectors(b, *nextX, *nextX);
                storm::utility::vector::multiplyVectorsPointwise(jacobiDecomposition->DVector, *nextX, *nextX);
                
                // Now check if the process already converged within our precision.
                converged = storm::utility::vector::equalModuloPrecision<ValueType>(*currentX, *nextX, precision, relative);
                terminate = this->terminateNow(*currentX, SolverGuarantee::None);
                
                // Swap the two pointers as a preparation for the next iteration.
                std::swap(nextX, currentX);
                
                // Potentially show progress.
                this->showProgressIterative(iterations);
                
                // Increase iteration count so we can abort if convergence is too slow.
                ++iterations;
            }
            
            // If the last iteration did not write to the original x we have to swap the contents, because the
            // output has to be written to the input parameter x.
            if (currentX == this->cachedRowVector.get()) {
                std::swap(x, *currentX);
            }
            
            if (!this->isCachingEnabled()) {
                clearCache();
            }
            
            this->logIterations(converged, terminate, iterations);

            return converged;
        }
        
        template<typename ValueType>
        NativeLinearEquationSolver<ValueType>::WalkerChaeData::WalkerChaeData(Environment const& env, storm::storage::SparseMatrix<ValueType> const& originalMatrix, std::vector<ValueType> const& originalB) : t(storm::utility::convertNumber<ValueType>(1000.0)) {
            computeWalkerChaeMatrix(originalMatrix);
            computeNewB(originalB);
            precomputeAuxiliaryData();
            multiplier = storm::solver::MultiplierFactory<ValueType>().create(env, this->matrix);
        }
        
        template<typename ValueType>
        void NativeLinearEquationSolver<ValueType>::WalkerChaeData::computeWalkerChaeMatrix(storm::storage::SparseMatrix<ValueType> const& originalMatrix) {
            storm::storage::BitVector columnsWithNegativeEntries(originalMatrix.getColumnCount());
            ValueType zero = storm::utility::zero<ValueType>();
            for (auto const& e : originalMatrix) {
                if (e.getValue() < zero) {
                    columnsWithNegativeEntries.set(e.getColumn());
                }
            }
            std::vector<uint64_t> columnsWithNegativeEntriesBefore = columnsWithNegativeEntries.getNumberOfSetBitsBeforeIndices();
            
            // We now build an extended equation system matrix that only has non-negative coefficients.
            storm::storage::SparseMatrixBuilder<ValueType> builder;
            
            uint64_t row = 0;
            for (; row < originalMatrix.getRowCount(); ++row) {
                for (auto const& entry : originalMatrix.getRow(row)) {
                    if (entry.getValue() < zero) {
                        builder.addNextValue(row, originalMatrix.getRowCount() + columnsWithNegativeEntriesBefore[entry.getColumn()], -entry.getValue());
                    } else {
                        builder.addNextValue(row, entry.getColumn(), entry.getValue());
                    }
                }
            }
            ValueType one = storm::utility::one<ValueType>();
            for (auto column : columnsWithNegativeEntries) {
                builder.addNextValue(row, column, one);
                builder.addNextValue(row, originalMatrix.getRowCount() + columnsWithNegativeEntriesBefore[column], one);
                ++row;
            }
            
            matrix = builder.build();
        }
        
        template<typename ValueType>
        void NativeLinearEquationSolver<ValueType>::WalkerChaeData::computeNewB(std::vector<ValueType> const& originalB) {
            b = std::vector<ValueType>(originalB);
            b.resize(matrix.getRowCount());
        }
        
        template<typename ValueType>
        void NativeLinearEquationSolver<ValueType>::WalkerChaeData::precomputeAuxiliaryData() {
            columnSums = std::vector<ValueType>(matrix.getColumnCount());
            for (auto const& e : matrix) {
                columnSums[e.getColumn()] += e.getValue();
            }
            
            newX.resize(matrix.getRowCount());
        }
        
        template<typename ValueType>
        bool NativeLinearEquationSolver<ValueType>::solveEquationsWalkerChae(Environment const& env, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            STORM_LOG_INFO("Solving linear equation system (" << x.size() << " rows) with NativeLinearEquationSolver (WalkerChae)");
            
            // (1) Compute an equivalent equation system that has only non-negative coefficients.
            if (!walkerChaeData) {
                walkerChaeData = std::make_unique<WalkerChaeData>(env, *this->A, b);
            }

            // (2) Enlarge the vectors x and b to account for additional variables.
            x.resize(walkerChaeData->matrix.getRowCount());
            
            // Square the error bound, so we can use it to check for convergence. We take the squared error, because we
            // do not want to compute the root in the 2-norm computation.
            ValueType squaredErrorBound = storm::utility::pow(storm::utility::convertNumber<ValueType>(env.solver().native().getPrecision()), 2);
            
            uint64_t maxIter = env.solver().native().getMaximalNumberOfIterations();
            
            // Set up references to the x-vectors used in the iteration loop.
            std::vector<ValueType>* currentX = &x;
            std::vector<ValueType>* nextX = &walkerChaeData->newX;

            std::vector<ValueType> tmp = walkerChaeData->matrix.getRowSumVector();
            storm::utility::vector::applyPointwise(tmp, walkerChaeData->b, walkerChaeData->b, [this] (ValueType const& first, ValueType const& second) { return walkerChaeData->t * first + second; } );
            
            // Add t to all entries of x.
            storm::utility::vector::applyPointwise(x, x, [this] (ValueType const& value) { return value + walkerChaeData->t; });

            // Create a vector that always holds Ax.
            std::vector<ValueType> currentAx(x.size());
            walkerChaeData->multiplier->multiply(env, *currentX, nullptr, currentAx);
            
            // (3) Perform iterations until convergence.
            bool converged = false;
            uint64_t iterations = 0;
            this->startMeasureProgress();
            while (!converged && iterations < maxIter) {
                // Perform one Walker-Chae step.
                walkerChaeData->matrix.performWalkerChaeStep(*currentX, walkerChaeData->columnSums, walkerChaeData->b, currentAx, *nextX);
                
                // Compute new Ax.
                walkerChaeData->multiplier->multiply(env, *nextX, nullptr, currentAx);
                
                // Check for convergence.
                converged = storm::utility::vector::computeSquaredNorm2Difference(currentAx, walkerChaeData->b) <= squaredErrorBound;
                
                // Swap the x vectors for the next iteration.
                std::swap(currentX, nextX);
                
                // Potentially show progress.
                this->showProgressIterative(iterations);

                // Increase iteration count so we can abort if convergence is too slow.
                ++iterations;
            }
            
            // If the last iteration did not write to the original x we have to swap the contents, because the
            // output has to be written to the input parameter x.
            if (currentX == &walkerChaeData->newX) {
                std::swap(x, *currentX);
            }

            // Resize the solution to the right size.
            x.resize(this->A->getRowCount());
            
            // Finalize solution vector.
            storm::utility::vector::applyPointwise(x, x, [this] (ValueType const& value) { return value - walkerChaeData->t; } );
            
            if (!this->isCachingEnabled()) {
                clearCache();
            }

            if (converged) {
                STORM_LOG_INFO("Iterative solver converged in " << iterations << " iterations.");
            } else {
                STORM_LOG_WARN("Iterative solver did not converge in " << iterations << " iterations.");
            }

            return converged;
        }
        
        template<typename ValueType>
        typename NativeLinearEquationSolver<ValueType>::PowerIterationResult NativeLinearEquationSolver<ValueType>::performPowerIteration(Environment const& env, std::vector<ValueType>*& currentX, std::vector<ValueType>*& newX, std::vector<ValueType> const& b, ValueType const& precision, bool relative, SolverGuarantee const& guarantee, uint64_t currentIterations, uint64_t maxIterations, storm::solver::MultiplicationStyle const& multiplicationStyle) const {

            bool useGaussSeidelMultiplication = multiplicationStyle == storm::solver::MultiplicationStyle::GaussSeidel;
            
            std::vector<ValueType>* originalX = currentX;
            
            bool converged = false;
            bool terminate = this->terminateNow(*currentX, guarantee);
            uint64_t iterations = currentIterations;
            while (!converged && !terminate && iterations < maxIterations) {
                if (useGaussSeidelMultiplication) {
                    *newX = *currentX;
                    this->multiplier->multiplyGaussSeidel(env, *newX, &b);
                } else {
                    this->multiplier->multiply(env, *currentX, &b, *newX);
                }
                
                // Now check for termination.
                converged = storm::utility::vector::equalModuloPrecision<ValueType>(*currentX, *newX, precision, relative);
                terminate = this->terminateNow(*currentX, guarantee);
                
                // Potentially show progress.
                this->showProgressIterative(iterations);
                
                // Set up next iteration.
                std::swap(currentX, newX);
                ++iterations;
            }
            
            // Swap the pointers so that the output is always in currentX.
            if (originalX == newX) {
                std::swap(currentX, newX);
            }
            
            return PowerIterationResult(iterations - currentIterations, converged ? SolverStatus::Converged : (terminate ? SolverStatus::TerminatedEarly : SolverStatus::MaximalIterationsExceeded));
        }
        
        template<typename ValueType>
        bool NativeLinearEquationSolver<ValueType>::solveEquationsPower(Environment const& env, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            STORM_LOG_INFO("Solving linear equation system (" << x.size() << " rows) with NativeLinearEquationSolver (Power)");

            // Prepare the solution vectors.
            if (!this->cachedRowVector) {
                this->cachedRowVector = std::make_unique<std::vector<ValueType>>(getMatrixRowCount());
            }
            if (!this->multiplier) {
                this->multiplier = storm::solver::MultiplierFactory<ValueType>().create(env, *A);
            }
            std::vector<ValueType>* currentX = &x;
            SolverGuarantee guarantee = SolverGuarantee::None;
            if (this->hasCustomTerminationCondition()) {
                if (this->getTerminationCondition().requiresGuarantee(SolverGuarantee::LessOrEqual) && this->hasLowerBound()) {
                    this->createLowerBoundsVector(*currentX);
                    guarantee = SolverGuarantee::LessOrEqual;
                } else if (this->getTerminationCondition().requiresGuarantee(SolverGuarantee::GreaterOrEqual) && this->hasUpperBound()) {
                    this->createUpperBoundsVector(*currentX);
                    guarantee = SolverGuarantee::GreaterOrEqual;
                }
            }
            std::vector<ValueType>* newX = this->cachedRowVector.get();
            
            // Forward call to power iteration implementation.
            this->startMeasureProgress();
            ValueType precision = storm::utility::convertNumber<ValueType>(env.solver().native().getPrecision());
            PowerIterationResult result = this->performPowerIteration(env, currentX, newX, b, precision, env.solver().native().getRelativeTerminationCriterion(), guarantee, 0, env.solver().native().getMaximalNumberOfIterations(), env.solver().native().getPowerMethodMultiplicationStyle());

            // Swap the result in place.
            if (currentX == this->cachedRowVector.get()) {
                std::swap(x, *newX);
            }
            
            if (!this->isCachingEnabled()) {
                clearCache();
            }
            
            this->logIterations(result.status == SolverStatus::Converged, result.status == SolverStatus::TerminatedEarly, result.iterations);

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
        
        template<typename ValueType>
        bool NativeLinearEquationSolver<ValueType>::solveEquationsIntervalIteration(Environment const& env, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            STORM_LOG_THROW(this->hasLowerBound(), storm::exceptions::UnmetRequirementException, "Solver requires lower bound, but none was given.");
            STORM_LOG_THROW(this->hasUpperBound(), storm::exceptions::UnmetRequirementException, "Solver requires upper bound, but none was given.");
            STORM_LOG_INFO("Solving linear equation system (" << x.size() << " rows) with NativeLinearEquationSolver (IntervalIteration)");

            std::vector<ValueType>* lowerX = &x;
            this->createLowerBoundsVector(*lowerX);
            this->createUpperBoundsVector(this->cachedRowVector, this->getMatrixRowCount());
            std::vector<ValueType>* upperX = this->cachedRowVector.get();
            
            bool useGaussSeidelMultiplication = env.solver().native().getPowerMethodMultiplicationStyle() == storm::solver::MultiplicationStyle::GaussSeidel;
            std::vector<ValueType>* tmp;
            if (!useGaussSeidelMultiplication) {
                cachedRowVector2 = std::make_unique<std::vector<ValueType>>(x.size());
                tmp = cachedRowVector2.get();
            }
            
            if (!this->multiplier) {
                this->multiplier = storm::solver::MultiplierFactory<ValueType>().create(env, *A);
            }
            
            bool converged = false;
            bool terminate = false;
            uint64_t iterations = 0;
            bool doConvergenceCheck = true;
            bool useDiffs = this->hasRelevantValues() && !env.solver().native().isSymmetricUpdatesSet();
            std::vector<ValueType> oldValues;
            if (useGaussSeidelMultiplication && useDiffs) {
                oldValues.resize(this->getRelevantValues().getNumberOfSetBits());
            }
            ValueType maxLowerDiff = storm::utility::zero<ValueType>();
            ValueType maxUpperDiff = storm::utility::zero<ValueType>();
            ValueType precision = storm::utility::convertNumber<ValueType>(env.solver().native().getPrecision());
            bool relative = env.solver().native().getRelativeTerminationCriterion();
            if (!relative) {
                precision *= storm::utility::convertNumber<ValueType>(2.0);
            }
            uint64_t maxIter = env.solver().native().getMaximalNumberOfIterations();
            this->startMeasureProgress();
            while (!converged && !terminate && iterations < maxIter) {
                // Remember in which directions we took steps in this iteration.
                bool lowerStep = false;
                bool upperStep = false;
                
                // In every thousandth iteration or if the differences are the same, we improve both bounds.
                if (iterations % 1000 == 0 || maxLowerDiff == maxUpperDiff) {
                    lowerStep = true;
                    upperStep = true;
                    if (useGaussSeidelMultiplication) {
                        if (useDiffs) {
                            preserveOldRelevantValues(*lowerX, this->getRelevantValues(), oldValues);
                        }
                        this->multiplier->multiplyGaussSeidel(env, *lowerX, &b);
                        if (useDiffs) {
                            maxLowerDiff = computeMaxAbsDiff(*lowerX, this->getRelevantValues(), oldValues);
                            preserveOldRelevantValues(*upperX, this->getRelevantValues(), oldValues);
                        }
                        this->multiplier->multiplyGaussSeidel(env, *upperX, &b);
                        if (useDiffs) {
                            maxUpperDiff = computeMaxAbsDiff(*upperX, this->getRelevantValues(), oldValues);
                        }
                    } else {
                        this->multiplier->multiply(env, *lowerX, &b, *tmp);
                        if (useDiffs) {
                            maxLowerDiff = computeMaxAbsDiff(*lowerX, *tmp, this->getRelevantValues());
                        }
                        std::swap(tmp, lowerX);
                        this->multiplier->multiply(env, *upperX, &b, *tmp);
                        if (useDiffs) {
                            maxUpperDiff = computeMaxAbsDiff(*upperX, *tmp, this->getRelevantValues());
                        }
                        std::swap(tmp, upperX);
                    }
                } else {
                    // In the following iterations, we improve the bound with the greatest difference.
                    if (useGaussSeidelMultiplication) {
                        if (maxLowerDiff >= maxUpperDiff) {
                            if (useDiffs) {
                                preserveOldRelevantValues(*lowerX, this->getRelevantValues(), oldValues);
                            }
                            this->multiplier->multiplyGaussSeidel(env, *lowerX, &b);
                            if (useDiffs) {
                                maxLowerDiff = computeMaxAbsDiff(*lowerX, this->getRelevantValues(), oldValues);
                            }
                            lowerStep = true;
                        } else {
                            if (useDiffs) {
                                preserveOldRelevantValues(*upperX, this->getRelevantValues(), oldValues);
                            }
                           this->multiplier->multiplyGaussSeidel(env, *upperX, &b);
                            if (useDiffs) {
                                maxUpperDiff = computeMaxAbsDiff(*upperX, this->getRelevantValues(), oldValues);
                            }
                            upperStep = true;
                        }
                    } else {
                        if (maxLowerDiff >= maxUpperDiff) {
                            this->multiplier->multiply(env, *lowerX, &b, *tmp);
                            if (useDiffs) {
                                maxLowerDiff = computeMaxAbsDiff(*lowerX, *tmp, this->getRelevantValues());
                            }
                            std::swap(tmp, lowerX);
                            lowerStep = true;
                        } else {
                            this->multiplier->multiply(env, *upperX, &b, *tmp);
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
                    // Now check if the process already converged within our precision. Note that we double the target
                    // precision here. Doing so, we need to take the means of the lower and upper values later to guarantee
                    // the original precision.
                    if (this->hasRelevantValues()) {
                        converged = storm::utility::vector::equalModuloPrecision<ValueType>(*lowerX, *upperX, this->getRelevantValues(), precision, relative);
                    } else {
                        converged = storm::utility::vector::equalModuloPrecision<ValueType>(*lowerX, *upperX, precision, relative);
                    }
                    if (lowerStep) {
                        terminate |= this->terminateNow(*lowerX, SolverGuarantee::LessOrEqual);
                    }
                    if (upperStep) {
                        terminate |= this->terminateNow(*upperX, SolverGuarantee::GreaterOrEqual);
                    }
                }
                
                // Potentially show progress.
                this->showProgressIterative(iterations);
                
                // Set up next iteration.
                ++iterations;
                doConvergenceCheck = !doConvergenceCheck;
            }
            
            // We take the means of the lower and upper bound so we guarantee the desired precision.
            storm::utility::vector::applyPointwise(*lowerX, *upperX, *lowerX, [] (ValueType const& a, ValueType const& b) { return (a + b) / storm::utility::convertNumber<ValueType>(2.0); });

            // Since we shuffled the pointer around, we need to write the actual results to the input/output vector x.
            if (&x == tmp) {
                std::swap(x, *tmp);
            } else if (&x == this->cachedRowVector.get()) {
                std::swap(x, *this->cachedRowVector);
            }
            
            if (!this->isCachingEnabled()) {
                clearCache();
            }
            this->overallPerformedIterations += iterations;

            this->logIterations(converged, terminate, iterations);

            return converged;
        }
        
        template<typename ValueType>
        class SoundPowerHelper {
        public:
            SoundPowerHelper(std::vector<ValueType>& x, std::vector<ValueType>& y, bool relative, ValueType const& precision) : x(x), y(y), hasLowerBound(false), hasUpperBound(false), minIndex(0), maxIndex(0), relative(relative), precision(precision) {
                x.assign(x.size(), storm::utility::zero<ValueType>());
                y.assign(x.size(), storm::utility::one<ValueType>());
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
            
            void multiplyRow(uint64_t const& row, storm::storage::SparseMatrix<ValueType> const& A, storm::solver::Multiplier<ValueType> const& multiplier, ValueType const& bi, ValueType& xi, ValueType& yi) {
                ValueType xRes = bi;
                ValueType yRes = storm::utility::zero<ValueType>();
                multiplier.multiplyRow2(row, x, xRes, y, yRes);
                xi = std::move(xRes);
                yi = std::move(yRes);
            }
            
            void performIterationStep(storm::storage::SparseMatrix<ValueType> const& A, storm::solver::Multiplier<ValueType> const& multiplier, std::vector<ValueType> const& b) {
                auto xIt = x.rbegin();
                auto yIt = y.rbegin();
                uint64_t row = A.getRowCount();
                while (row > 0) {
                    --row;
                    multiplyRow(row, A, multiplier, b[row], *xIt, *yIt);
                    ++xIt;
                    ++yIt;
                }
            }
            
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
                if (preliminaryConvergenceCheck(lowerBoundCandidate, upperBoundCandidate)) {
                    updateLowerUpperBound(lowerBoundCandidate, upperBoundCandidate);
                    return checkConvergencePhase2(relevantValues);
                }
                return false;
            }
            
            void setSolutionVector() {
                STORM_LOG_WARN_COND(hasLowerBound && hasUpperBound, "No lower or upper result bound could be computed within the given number of Iterations.");
            
                ValueType meanBound = (upperBound + lowerBound) / storm::utility::convertNumber<ValueType>(2.0);
                storm::utility::vector::applyPointwise(x, y, x, [&meanBound] (ValueType const& xi, ValueType const& yi) { return xi + yi * meanBound; });
                
                STORM_LOG_INFO("Sound Power Iteration terminated with lower value bound "
                                       << (hasLowerBound ? lowerBound : storm::utility::zero<ValueType>()) << (hasLowerBound ? "" : "(none)")
                                       << " and upper value bound "
                                       << (hasUpperBound ? upperBound : storm::utility::zero<ValueType>()) << (hasUpperBound ? "" : "(none)")
                                       << ".");
                }
                
        private:
            
            bool checkConvergencePhase1() {
                // Return true if y ('the probability to stay within the 'maybestates') is  < 1 at every entry
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
                return false;
            }
            
            void updateLowerUpperBound(ValueType& lowerBoundCandidate, ValueType& upperBoundCandidate) {
                auto xIt = x.begin();
                auto xIte = x.end();
                auto yIt = y.begin();
                for (uint64_t index = 0; xIt != xIte; ++xIt, ++yIt, ++index) {
                    ValueType currentBound = *xIt / (storm::utility::one<ValueType>() - *yIt);
                    if (currentBound < lowerBoundCandidate) {
                        minIndex = index;
                        lowerBoundCandidate = std::move(currentBound);
                    } else if (currentBound > upperBoundCandidate) {
                        maxIndex = index;
                        upperBoundCandidate = std::move(currentBound);
                    }
                }
                if (!hasLowerBound || lowerBoundCandidate > lowerBound) {
                    setLowerBound(lowerBoundCandidate);
                }
                if (!hasUpperBound || upperBoundCandidate < upperBound) {
                    setUpperBound(upperBoundCandidate);
                }
            }
            
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
            
            ValueType lowerBound, upperBound, decisionValue;
            bool hasLowerBound, hasUpperBound, hasDecisionValue;
            uint64_t minIndex, maxIndex;
            bool convergencePhase1;
            uint64_t firstIndexViolatingConvergence;
            
            bool relative;
            ValueType precision;
        };
        
        template<typename ValueType>
        bool NativeLinearEquationSolver<ValueType>::solveEquationsSoundPower(Environment const& env, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {

            // Prepare the solution vectors.
            assert(x.size() == this->A->getRowCount());
            if (!this->cachedRowVector) {
                this->cachedRowVector = std::make_unique<std::vector<ValueType>>();
            }
            
            if (!this->multiplier) {
                this->multiplier = storm::solver::MultiplierFactory<ValueType>().create(env, *this->A);
            }
            
            SoundPowerHelper<ValueType> helper(x, *this->cachedRowVector, env.solver().native().getRelativeTerminationCriterion(), storm::utility::convertNumber<ValueType>(env.solver().native().getPrecision()));

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
            
            bool converged = false;
            bool terminate = false;
            this->startMeasureProgress();
            uint64_t iterations = 0;
            
            while (!converged && iterations < env.solver().native().getMaximalNumberOfIterations()) {
                helper.performIterationStep(*this->A, *this->multiplier, b);
                if (helper.checkConvergenceUpdateBounds(relevantValuesPtr)) {
                    converged = true;
                }

                // todo: custom termination check
                // terminate = ....
                
                // Update environment variables.
                ++iterations;
                
                // Potentially show progress.
                this->showProgressIterative(iterations);
            }
            helper.setSolutionVector();
            
            this->logIterations(converged, terminate, iterations);
            
            this->overallPerformedIterations += iterations;
            
            if (!this->isCachingEnabled()) {
                clearCache();
            }
            
            return converged;
        }
        
        template<typename ValueType>
        bool NativeLinearEquationSolver<ValueType>::solveEquationsRationalSearch(Environment const& env, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            return solveEquationsRationalSearchHelper<double>(env, x, b);
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
        bool NativeLinearEquationSolver<ValueType>::solveEquationsRationalSearchHelper(Environment const& env, NativeLinearEquationSolver<ImpreciseType> const& impreciseSolver, storm::storage::SparseMatrix<RationalType> const& rationalA, std::vector<RationalType>& rationalX, std::vector<RationalType> const& rationalB, storm::storage::SparseMatrix<ImpreciseType> const& A, std::vector<ImpreciseType>& x, std::vector<ImpreciseType> const& b, std::vector<ImpreciseType>& tmpX) const {
            
            ValueType precision = storm::utility::convertNumber<ValueType>(env.solver().native().getPrecision());
            uint64_t maxIter = env.solver().native().getMaximalNumberOfIterations();
            bool relative = env.solver().native().getRelativeTerminationCriterion();
            auto multiplicationStyle = env.solver().native().getPowerMethodMultiplicationStyle();
            
            std::vector<ImpreciseType>* currentX = &x;
            std::vector<ImpreciseType>* newX = &tmpX;

            SolverStatus status = SolverStatus::InProgress;
            uint64_t overallIterations = 0;
            uint64_t valueIterationInvocations = 0;
            impreciseSolver.startMeasureProgress();
            while (status == SolverStatus::InProgress && overallIterations < maxIter) {
                // Perform value iteration with the current precision.
                typename NativeLinearEquationSolver<ImpreciseType>::PowerIterationResult result = impreciseSolver.performPowerIteration(env, currentX, newX, b, storm::utility::convertNumber<ImpreciseType, ValueType>(precision), relative, SolverGuarantee::LessOrEqual, overallIterations, maxIter, multiplicationStyle);
                
                // At this point, the result of the imprecise value iteration is stored in the (imprecise) current x.

                ++valueIterationInvocations;
                STORM_LOG_TRACE("Completed " << valueIterationInvocations << " power iteration invocations, the last one with precision " << precision << " completed in " << result.iterations << " iterations.");
                
                // Count the iterations.
                overallIterations += result.iterations;
                
                // Compute maximal precision until which to sharpen.
                uint64_t p = storm::utility::convertNumber<uint64_t>(storm::utility::ceil(storm::utility::log10<ValueType>(storm::utility::one<ValueType>() / precision)));
                
                // Make sure that currentX and rationalX are not aliased.
                std::vector<RationalType>* temporaryRational = TemporaryHelper<RationalType, ImpreciseType>::getTemporary(rationalX, currentX, newX);
                
                // Sharpen solution and place it in the temporary rational.
                bool foundSolution = sharpen(p, rationalA, *currentX, rationalB, *temporaryRational);
                
                // After sharpen, if a solution was found, it is contained in the free rational.
                
                if (foundSolution) {
                    status = SolverStatus::Converged;
                    
                    TemporaryHelper<RationalType, ImpreciseType>::swapSolutions(rationalX, temporaryRational, x, currentX, newX);
                } else {
                    // Increase the precision.
                    precision = precision / 10;
                }
            }
            
            if (status == SolverStatus::InProgress && overallIterations == maxIter) {
                status = SolverStatus::MaximalIterationsExceeded;
            }
            
            this->logIterations(status == SolverStatus::Converged, status == SolverStatus::TerminatedEarly, overallIterations);
            
            return status == SolverStatus::Converged || status == SolverStatus::TerminatedEarly;
        }
     
        template<typename ValueType>
        template<typename ImpreciseType>
        typename std::enable_if<std::is_same<ValueType, ImpreciseType>::value && !NumberTraits<ValueType>::IsExact, bool>::type NativeLinearEquationSolver<ValueType>::solveEquationsRationalSearchHelper(Environment const& env, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            // Version for when the overall value type is imprecise.
            
            // Create a rational representation of the input so we can check for a proper solution later.
            storm::storage::SparseMatrix<storm::RationalNumber> rationalA = this->A->template toValueType<storm::RationalNumber>();
            std::vector<storm::RationalNumber> rationalX(x.size());
            std::vector<storm::RationalNumber> rationalB = storm::utility::vector::convertNumericVector<storm::RationalNumber>(b);
                        
            if (!this->cachedRowVector) {
                this->cachedRowVector = std::make_unique<std::vector<ValueType>>(this->A->getRowCount());
            }
            if (!this->multiplier) {
                this->multiplier = storm::solver::MultiplierFactory<ValueType>().create(env, *A);
            }
            
            // Forward the call to the core rational search routine.
            bool converged = solveEquationsRationalSearchHelper<storm::RationalNumber, ImpreciseType>(env, *this, rationalA, rationalX, rationalB, *this->A, x, b, *this->cachedRowVector);
            
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
        typename std::enable_if<std::is_same<ValueType, ImpreciseType>::value && NumberTraits<ValueType>::IsExact, bool>::type NativeLinearEquationSolver<ValueType>::solveEquationsRationalSearchHelper(Environment const& env, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            // Version for when the overall value type is exact and the same type is to be used for the imprecise part.
            
            if (!this->cachedRowVector) {
                this->cachedRowVector = std::make_unique<std::vector<ValueType>>(this->A->getRowCount());
            }
            if (!this->multiplier) {
                this->multiplier = storm::solver::MultiplierFactory<ValueType>().create(env, *A);
            }
            
            // Forward the call to the core rational search routine.
            bool converged = solveEquationsRationalSearchHelper<ValueType, ImpreciseType>(env, *this, *this->A, x, b, *this->A, *this->cachedRowVector, b, x);
            
            if (!this->isCachingEnabled()) {
                this->clearCache();
            }
            
            return converged;
        }
        
        template<typename ValueType>
        template<typename ImpreciseType>
        typename std::enable_if<!std::is_same<ValueType, ImpreciseType>::value, bool>::type NativeLinearEquationSolver<ValueType>::solveEquationsRationalSearchHelper(Environment const& env, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
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
            NativeLinearEquationSolver<ImpreciseType> impreciseSolver;
            impreciseSolver.setMatrix(impreciseA);
            impreciseSolver.setCachingEnabled(true);
            impreciseSolver.multiplier = storm::solver::MultiplierFactory<ImpreciseType>().create(env, impreciseA);

            bool converged = false;
            try {
                // Forward the call to the core rational search routine.
                converged = solveEquationsRationalSearchHelper<ValueType, ImpreciseType>(env, impreciseSolver, *this->A, x, b, impreciseA, impreciseX, impreciseB, impreciseTmpX);
                impreciseSolver.clearCache();
            } catch (storm::exceptions::PrecisionExceededException const& e) {
                STORM_LOG_WARN("Precision of value type was exceeded, trying to recover by switching to rational arithmetic.");
                
                if (!this->cachedRowVector) {
                    this->cachedRowVector = std::make_unique<std::vector<ValueType>>(this->A->getRowGroupCount());
                }
                if (!this->multiplier) {
                    this->multiplier = storm::solver::MultiplierFactory<ValueType>().create(env, *A);
                }
                // Translate the imprecise value iteration result to the one we are going to use from now on.
                auto targetIt = this->cachedRowVector->begin();
                for (auto it = impreciseX.begin(), ite = impreciseX.end(); it != ite; ++it, ++targetIt) {
                    *targetIt = storm::utility::convertNumber<ValueType>(*it);
                }
                
                // Get rid of the superfluous data structures.
                impreciseX = std::vector<ImpreciseType>();
                impreciseTmpX = std::vector<ImpreciseType>();
                impreciseB = std::vector<ImpreciseType>();
                impreciseA = storm::storage::SparseMatrix<ImpreciseType>();
                
                // Forward the call to the core rational search routine, but now with our value type as the imprecise value type.
                converged = solveEquationsRationalSearchHelper<ValueType, ValueType>(env, *this, *this->A, x, b, *this->A, *this->cachedRowVector, b, x);
            }
            
            if (!this->isCachingEnabled()) {
                this->clearCache();
            }
            
            return converged;
        }

        template<typename ValueType>
        template<typename RationalType, typename ImpreciseType>
        bool NativeLinearEquationSolver<ValueType>::sharpen(uint64_t precision, storm::storage::SparseMatrix<RationalType> const& A, std::vector<ImpreciseType> const& x, std::vector<RationalType> const& b, std::vector<RationalType>& tmp) {
            for (uint64_t p = 1; p <= precision; ++p) {
                storm::utility::kwek_mehlhorn::sharpen(p, x, tmp);

                if (NativeLinearEquationSolver<RationalType>::isSolution(A, tmp, b)) {
                    return true;
                }
            }
            return false;
        }

        template<typename ValueType>
        bool NativeLinearEquationSolver<ValueType>::isSolution(storm::storage::SparseMatrix<ValueType> const& matrix, std::vector<ValueType> const& values, std::vector<ValueType> const& b) {
            storm::utility::ConstantsComparator<ValueType> comparator;
            
            auto valueIt = values.begin();
            auto bIt = b.begin();
            for (uint64_t row = 0; row < matrix.getRowCount(); ++row, ++valueIt, ++bIt) {
                ValueType rowValue = *bIt + matrix.multiplyRowWithVector(row, values);
                
                // If the value does not match the one in the values vector, the given vector is not a solution.
                if (!comparator.isEqual(rowValue, *valueIt)) {
                    return false;
                }
            }
            
            // Checked all values at this point.
            return true;
        }
        
        template<typename ValueType>
        void NativeLinearEquationSolver<ValueType>::logIterations(bool converged, bool terminate, uint64_t iterations) const {
            if (converged) {
                STORM_LOG_INFO("Iterative solver converged in " << iterations << " iterations.");
            } else if (terminate) {
                STORM_LOG_INFO("Iterative solver terminated after " << iterations << " iterations.");
            } else {
                STORM_LOG_WARN("Iterative solver did not converge in " << iterations << " iterations.");
            }
        }
        
        template<typename ValueType>
        NativeLinearEquationSolverMethod NativeLinearEquationSolver<ValueType>::getMethod(Environment const& env, bool isExactMode) const {
            // Adjust the method if none was specified and we want exact or sound computations
            auto method = env.solver().native().getMethod();
            
            if (isExactMode && method != NativeLinearEquationSolverMethod::RationalSearch) {
                if (env.solver().native().isMethodSetFromDefault()) {
                    method = NativeLinearEquationSolverMethod::RationalSearch;
                    STORM_LOG_INFO("Selecting '" + toString(method) + "' as the solution technique to guarantee exact results. If you want to override this, please explicitly specify a different method.");
                } else {
                    STORM_LOG_WARN("The selected solution method does not guarantee exact results.");
                }
            } else if (env.solver().isForceSoundness() && method != NativeLinearEquationSolverMethod::SoundPower && method != NativeLinearEquationSolverMethod::IntervalIteration && method != NativeLinearEquationSolverMethod::RationalSearch) {
                if (env.solver().native().isMethodSetFromDefault()) {
                    method = NativeLinearEquationSolverMethod::SoundPower;
                    STORM_LOG_INFO("Selecting '" + toString(method) + "' as the solution technique to guarantee sound results. If you want to override this, please explicitly specify a different method.");
                } else {
                    STORM_LOG_WARN("The selected solution method does not guarantee sound results.");
                }
            }
            return method;
        }

        
        template<typename ValueType>
        bool NativeLinearEquationSolver<ValueType>::internalSolveEquations(Environment const& env, std::vector<ValueType>& x, std::vector<ValueType> const& b) const {
            switch(getMethod(env, storm::NumberTraits<ValueType>::IsExact)) {
                case NativeLinearEquationSolverMethod::SOR:
                    return this->solveEquationsSOR(env, x, b, storm::utility::convertNumber<ValueType>(env.solver().native().getSorOmega()));
                case NativeLinearEquationSolverMethod::GaussSeidel:
                    return this->solveEquationsSOR(env, x, b, storm::utility::one<ValueType>());
                case NativeLinearEquationSolverMethod::Jacobi:
                    return this->solveEquationsJacobi(env, x, b);
                case NativeLinearEquationSolverMethod::WalkerChae:
                    return this->solveEquationsWalkerChae(env, x, b);
                case NativeLinearEquationSolverMethod::Power:
                    return this->solveEquationsPower(env, x, b);
                case NativeLinearEquationSolverMethod::SoundPower:
                    return this->solveEquationsSoundPower(env, x, b);
                case NativeLinearEquationSolverMethod::IntervalIteration:
                    return this->solveEquationsIntervalIteration(env, x, b);
                case NativeLinearEquationSolverMethod::RationalSearch:
                    return this->solveEquationsRationalSearch(env, x, b);
            }
            STORM_LOG_THROW(false, storm::exceptions::InvalidEnvironmentException, "Unknown solving technique.");
            return false;
        }
        
        template<typename ValueType>
        LinearEquationSolverProblemFormat NativeLinearEquationSolver<ValueType>::getEquationProblemFormat(Environment const& env) const {
            auto method = getMethod(env, storm::NumberTraits<ValueType>::IsExact);
            if (method == NativeLinearEquationSolverMethod::Power || method == NativeLinearEquationSolverMethod::SoundPower || method == NativeLinearEquationSolverMethod::RationalSearch || method == NativeLinearEquationSolverMethod::IntervalIteration) {
                return LinearEquationSolverProblemFormat::FixedPointSystem;
            } else {
                return LinearEquationSolverProblemFormat::EquationSystem;
            }
        }
        
        template<typename ValueType>
        LinearEquationSolverRequirements NativeLinearEquationSolver<ValueType>::getRequirements(Environment const& env) const {
            LinearEquationSolverRequirements requirements;
            if (env.solver().native().isForceBoundsSet()) {
                requirements.requireBounds();
            }
            auto method = getMethod(env, storm::NumberTraits<ValueType>::IsExact);
            if (method == NativeLinearEquationSolverMethod::IntervalIteration) {
                requirements.requireBounds();
            } else if (method == NativeLinearEquationSolverMethod::RationalSearch) {
                requirements.requireLowerBounds();
            }
            return requirements;
        }
        
        template<typename ValueType>
        void NativeLinearEquationSolver<ValueType>::clearCache() const {
            jacobiDecomposition.reset();
            cachedRowVector2.reset();
            walkerChaeData.reset();
            multiplier.reset();
            LinearEquationSolver<ValueType>::clearCache();
        }
        
        template<typename ValueType>
        uint64_t NativeLinearEquationSolver<ValueType>::getMatrixRowCount() const {
            return this->A->getRowCount();
        }
        
        template<typename ValueType>
        uint64_t NativeLinearEquationSolver<ValueType>::getMatrixColumnCount() const {
            return this->A->getColumnCount();
        }
        
        template<typename ValueType>
        std::unique_ptr<storm::solver::LinearEquationSolver<ValueType>> NativeLinearEquationSolverFactory<ValueType>::create(Environment const& env) const {
            return std::make_unique<storm::solver::NativeLinearEquationSolver<ValueType>>();
        }
        
        template<typename ValueType>
        std::unique_ptr<LinearEquationSolverFactory<ValueType>> NativeLinearEquationSolverFactory<ValueType>::clone() const {
            return std::make_unique<NativeLinearEquationSolverFactory<ValueType>>(*this);
        }
        
        // Explicitly instantiate the linear equation solver.
        template class NativeLinearEquationSolver<double>;
        template class NativeLinearEquationSolverFactory<double>;
        
#ifdef STORM_HAVE_CARL
        template class NativeLinearEquationSolver<storm::RationalNumber>;
        template class NativeLinearEquationSolverFactory<storm::RationalNumber>;

#endif
    }
}
