#ifndef STORM_UTILITY_GRAPH_H_
#define STORM_UTILITY_GRAPH_H_

#include <set>
#include <limits>

#include "utility/OsDetection.h"

#include "src/storage/sparse/StateType.h"
#include "src/models/symbolic/DeterministicModel.h"
#include "src/models/symbolic/NondeterministicModel.h"
#include "src/models/sparse/DeterministicModel.h"
#include "src/models/sparse/NondeterministicModel.h"
#include "src/utility/constants.h"
#include "src/exceptions/InvalidArgumentException.h"

#include "log4cplus/logger.h"
#include "log4cplus/loggingmacros.h"

extern log4cplus::Logger logger;

namespace storm {
    namespace utility {
        namespace graph {
            
            /*!
             * Performs a forward depth-first search through the underlying graph structure to identify the states that
             * are reachable from the given set only passing through a constrained set of states until some target
             * have been reached.
             *
             * @param transitionMatrix The transition relation of the graph structure to search.
             * @param initialStates The set of states from which to start the search.
             * @param constraintStates The set of states that must not be left.
             * @param targetStates The target states that may not be passed.
             */
            template<typename T>
            storm::storage::BitVector getReachableStates(storm::storage::SparseMatrix<T> const& transitionMatrix, storm::storage::BitVector const& initialStates, storm::storage::BitVector const& constraintStates, storm::storage::BitVector const& targetStates) {
                storm::storage::BitVector reachableStates(initialStates);
                
                // Initialize the stack used for the DFS with the states.
                std::vector<uint_fast64_t> stack(initialStates.begin(), initialStates.end());
                
                // Perform the actual DFS.
                uint_fast64_t currentState = 0;
                while (!stack.empty()) {
                    currentState = stack.back();
                    stack.pop_back();
                    
                    for (auto const& successor : transitionMatrix.getRowGroup(currentState)) {
                        // Only explore the state if the transition was actually there and the successor has not yet
                        // been visited.
                        if (successor.getValue() != storm::utility::zero<T>() && !reachableStates.get(successor.getColumn())) {
                            // If the successor is one of the target states, we need to include it, but must not explore
                            // it further.
                            if (targetStates.get(successor.getColumn())) {
                                reachableStates.set(successor.getColumn());
                            } else if (constraintStates.get(successor.getColumn())) {
                                // However, if the state is in the constrained set of states, we need to follow it.
                                reachableStates.set(successor.getColumn());
                                stack.push_back(successor.getColumn());
                            }
                        }
                    }
                }
                
                return reachableStates;
            }
            
            /*!
             * Performs a breadth-first search through the underlying graph structure to compute the distance from all
             * states to the starting states of the search.
             *
             * @param transitionMatrix The transition relation of the graph structure to search.
             * @param initialStates The set of states from which to start the search.
             * @return The distances of each state to the initial states of the sarch.
             */
            template<typename T>
            std::vector<std::size_t> getDistances(storm::storage::SparseMatrix<T> const& transitionMatrix, storm::storage::BitVector const& initialStates) {
                std::vector<std::size_t> distances(transitionMatrix.getRowGroupCount());
                
                std::vector<std::pair<storm::storage::sparse::state_type, std::size_t>> stateQueue;
                stateQueue.reserve(transitionMatrix.getRowGroupCount());
                storm::storage::BitVector statesInQueue(transitionMatrix.getRowGroupCount());
                
                storm::storage::sparse::state_type currentPosition = 0;
                for (auto const& initialState : initialStates) {
                    stateQueue.emplace_back(initialState, 0);
                    statesInQueue.set(initialState);
                }
                
                // Perform a BFS.
                while (currentPosition < stateQueue.size()) {
                    std::pair<storm::storage::sparse::state_type, std::size_t> const& stateDistancePair = stateQueue[currentPosition];
                    distances[stateDistancePair.first] = stateDistancePair.second;
                    
                    for (auto const& successorEntry : transitionMatrix.getRowGroup(stateDistancePair.first)) {
                        if (!statesInQueue.get(successorEntry.getColumn())) {
                            stateQueue.emplace_back(successorEntry.getColumn(), stateDistancePair.second + 1);
                            statesInQueue.set(successorEntry.getColumn());
                        }
                    }
                    ++currentPosition;
                }
                
                return distances;
            }
            
            /*!
             * Performs a backward depth-first search trough the underlying graph structure
             * of the given model to determine which states of the model have a positive probability
             * of satisfying phi until psi. The resulting states are written to the given bit vector.
             *
             * @param backwardTransitions The reversed transition relation of the graph structure to search.
             * @param phiStates A bit vector of all states satisfying phi.
             * @param psiStates A bit vector of all states satisfying psi.
             * @param useStepBound A flag that indicates whether or not to use the given number of maximal steps for the search.
             * @param maximalSteps The maximal number of steps to reach the psi states.
             * @return A bit vector with all indices of states that have a probability greater than 0.
             */
            template <typename T>
            storm::storage::BitVector performProbGreater0(storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates, bool useStepBound = false, uint_fast64_t maximalSteps = 0) {
                // Prepare the resulting bit vector.
                uint_fast64_t numberOfStates = phiStates.size();
                storm::storage::BitVector statesWithProbabilityGreater0(numberOfStates);
                
                // Add all psi states as they already satisfy the condition.
                statesWithProbabilityGreater0 |= psiStates;
                
                // Initialize the stack used for the DFS with the states.
                std::vector<uint_fast64_t> stack(psiStates.begin(), psiStates.end());
                
                // Initialize the stack for the step bound, if the number of steps is bounded.
                std::vector<uint_fast64_t> stepStack;
                std::vector<uint_fast64_t> remainingSteps;
                if (useStepBound) {
                    stepStack.reserve(numberOfStates);
                    stepStack.insert(stepStack.begin(), psiStates.getNumberOfSetBits(), maximalSteps);
                    remainingSteps.resize(numberOfStates);
                    for (auto state : psiStates) {
                        remainingSteps[state] = maximalSteps;
                    }
                }
                
                // Perform the actual DFS.
                uint_fast64_t currentState, currentStepBound;
                while (!stack.empty()) {
                    currentState = stack.back();
                    stack.pop_back();
                    
                    if (useStepBound) {
                        currentStepBound = stepStack.back();
                        stepStack.pop_back();
                    }
                    
                    for (typename storm::storage::SparseMatrix<T>::const_iterator entryIt = backwardTransitions.begin(currentState), entryIte = backwardTransitions.end(currentState); entryIt != entryIte; ++entryIt) {
                        if (phiStates[entryIt->getColumn()] && (!statesWithProbabilityGreater0.get(entryIt->getColumn()) || (useStepBound && remainingSteps[entryIt->getColumn()] < currentStepBound - 1))) {
                            // If we don't have a bound on the number of steps to take, just add the state to the stack.
                            if (!useStepBound) {
                                statesWithProbabilityGreater0.set(entryIt->getColumn(), true);
                                stack.push_back(entryIt->getColumn());
                            } else if (currentStepBound > 0) {
                                // If there is at least one more step to go, we need to push the state and the new number of steps.
                                remainingSteps[entryIt->getColumn()] = currentStepBound - 1;
                                statesWithProbabilityGreater0.set(entryIt->getColumn(), true);
                                stack.push_back(entryIt->getColumn());
                                stepStack.push_back(currentStepBound - 1);
                            }
                        }
                    }
                }
                
                // Return result.
                return statesWithProbabilityGreater0;
            }
            
            /*!
             * Computes the set of states of the given model for which all paths lead to
             * the given set of target states and only visit states from the filter set
             * before. In order to do this, it uses the given set of states that
             * characterizes the states that possess at least one path to a target state.
             * The results are written to the given bit vector.
             *
             * @param backwardTransitions The reversed transition relation of the graph structure to search.
             * @param phiStates A bit vector of all states satisfying phi.
             * @param psiStates A bit vector of all states satisfying psi.
             * @param statesWithProbabilityGreater0 A reference to a bit vector of states that possess a positive
             * probability mass of satisfying phi until psi.
             * @return A bit vector with all indices of states that have a probability greater than 1.
             */
            template <typename T>
            storm::storage::BitVector performProb1(storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates, storm::storage::BitVector const& statesWithProbabilityGreater0) {
                storm::storage::BitVector statesWithProbability1 = performProbGreater0(backwardTransitions, ~psiStates, ~statesWithProbabilityGreater0);
                statesWithProbability1.complement();
                return statesWithProbability1;
            }
            
            /*!
             * Computes the set of states of the given model for which all paths lead to
             * the given set of target states and only visit states from the filter set
             * before. In order to do this, it uses the given set of states that
             * characterizes the states that possess at least one path to a target state.
             * The results are written to the given bit vector.
             *
             * @param backwardTransitions The reversed transition relation of the graph structure to search.
             * @param phiStates A bit vector of all states satisfying phi.
             * @param psiStates A bit vector of all states satisfying psi.
             * @return A bit vector with all indices of states that have a probability greater than 1.
             */
            template <typename T>
            storm::storage::BitVector performProb1(storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                storm::storage::BitVector statesWithProbabilityGreater0 = performProbGreater0(backwardTransitions, phiStates, psiStates);
                storm::storage::BitVector statesWithProbability1 = performProbGreater0(backwardTransitions, ~psiStates, ~(statesWithProbabilityGreater0));
                statesWithProbability1.complement();
                return statesWithProbability1;
            }
            
            /*!
             * Computes the sets of states that have probability 0 or 1, respectively, of satisfying phi until psi in a
             * deterministic model.
             *
             * @param model The model whose graph structure to search.
             * @param phiStates The set of all states satisfying phi.
             * @param psiStates The set of all states satisfying psi.
             * @return A pair of bit vectors such that the first bit vector stores the indices of all states
             * with probability 0 and the second stores all indices of states with probability 1.
             */
            template <typename T>
            static std::pair<storm::storage::BitVector, storm::storage::BitVector> performProb01(storm::models::sparse::DeterministicModel<T> const& model, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                std::pair<storm::storage::BitVector, storm::storage::BitVector> result;
                storm::storage::SparseMatrix<T> backwardTransitions = model.getBackwardTransitions();
                result.first = performProbGreater0(backwardTransitions, phiStates, psiStates);
                result.second = performProb1(backwardTransitions, phiStates, psiStates, result.first);
                result.first.complement();
                return result;
            }
            
            /*!
             * Computes the sets of states that have probability 0 or 1, respectively, of satisfying phi until psi in a
             * deterministic model.
             *
             * @param backwardTransitions The backward transitions of the model whose graph structure to search.
             * @param phiStates The set of all states satisfying phi.
             * @param psiStates The set of all states satisfying psi.
             * @return A pair of bit vectors such that the first bit vector stores the indices of all states
             * with probability 0 and the second stores all indices of states with probability 1.
             */
            template <typename T>
            static std::pair<storm::storage::BitVector, storm::storage::BitVector> performProb01(storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                std::pair<storm::storage::BitVector, storm::storage::BitVector> result;
                result.first = performProbGreater0(backwardTransitions, phiStates, psiStates);
                result.second = performProb1(backwardTransitions, phiStates, psiStates, result.first);
                result.first.complement();
                return result;
            }
            
            /*!
             * Computes the set of states that has a positive probability of reaching psi states after only passing
             * through phi states before.
             *
             * @param model The (symbolic) model for which to compute the set of states.
             * @param transitionMatrix The transition matrix of the model as a BDD.
             * @param phiStates The BDD containing all phi states of the model.
             * @param psiStates The BDD containing all psi states of the model.
             * @param stepBound If given, this number indicates the maximal amount of steps allowed.
             * @return All states with positive probability.
             */
            template <storm::dd::DdType Type>
            storm::dd::Bdd<Type> performProbGreater0(storm::models::symbolic::Model<Type> const& model, storm::dd::Bdd<Type> const& transitionMatrix, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates, boost::optional<uint_fast64_t> const& stepBound = boost::optional<uint_fast64_t>()) {
                // Initialize environment for backward search.
                storm::dd::DdManager<Type> const& manager = model.getManager();
                storm::dd::Bdd<Type> lastIterationStates = manager.getBddZero();
                storm::dd::Bdd<Type> statesWithProbabilityGreater0 = psiStates;
                
                uint_fast64_t iterations = 0;
                while (lastIterationStates != statesWithProbabilityGreater0) {
                    if (stepBound && iterations >= stepBound.get()) {
                        break;
                    }
                    
                    lastIterationStates = statesWithProbabilityGreater0;
                    statesWithProbabilityGreater0 = statesWithProbabilityGreater0.swapVariables(model.getRowColumnMetaVariablePairs());
                    statesWithProbabilityGreater0 &= transitionMatrix;
                    statesWithProbabilityGreater0 = statesWithProbabilityGreater0.existsAbstract(model.getColumnVariables());
                    statesWithProbabilityGreater0 &= phiStates;
                    statesWithProbabilityGreater0 |= lastIterationStates;
                    ++iterations;
                }

                return statesWithProbabilityGreater0;
            }
            
            /*!
             * Computes the set of states that have a probability of one of reaching psi states after only passing
             * through phi states before.
             *
             * @param model The (symbolic) model for which to compute the set of states.
             * @param transitionMatrix The transition matrix of the model as a BDD.
             * @param phiStates The BDD containing all phi states of the model.
             * @param psiStates The BDD containing all psi states of the model.
             * @param statesWithProbabilityGreater0 The set of states with a positive probability of satisfying phi
             * until psi as a BDD.
             * @return All states with probability 1.
             */
            template <storm::dd::DdType Type>
            storm::dd::Bdd<Type> performProb1(storm::models::symbolic::Model<Type> const& model, storm::dd::Bdd<Type> const& transitionMatrix, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates, storm::dd::Bdd<Type> const& statesWithProbabilityGreater0) {
                storm::dd::Bdd<Type> statesWithProbability1 = performProbGreater0(model, transitionMatrix, !psiStates && model.getReachableStates(), !statesWithProbabilityGreater0 && model.getReachableStates());
                statesWithProbability1 = !statesWithProbability1 && model.getReachableStates();
                return statesWithProbability1;
            }
            
            /*!
             * Computes the set of states that have a probability of one of reaching psi states after only passing
             * through phi states before.
             *
             * @param model The (symbolic) model for which to compute the set of states.
             * @param transitionMatrix The transition matrix of the model as a BDD.
             * @param phiStates The BDD containing all phi states of the model.
             * @param psiStates The BDD containing all psi states of the model.
             * @return All states with probability 1.
             */
            template <storm::dd::DdType Type>
            storm::dd::Bdd<Type> performProb1(storm::models::symbolic::Model<Type> const& model, storm::dd::Bdd<Type> const& transitionMatrix, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates) {
                storm::dd::Bdd<Type> statesWithProbabilityGreater0 = performProbGreater0(model, transitionMatrix, phiStates, psiStates);
                return performProb1(model, transitionMatrix, phiStates, psiStates, statesWithProbabilityGreater0);
            }
            
            /*!
             * Computes the sets of states that have probability 0 or 1, respectively, of satisfying phi until psi in a
             * deterministic model.
             *
             * @param model The (symbolic) model for which to compute the set of states.
             * @param phiStates The BDD containing all  phi states of the model.
             * @param psiStates The BDD containing all psi states of the model.
             * @return A pair of BDDs that represent all states with probability 0 and 1, respectively.
             */
            template <storm::dd::DdType Type>
            static std::pair<storm::dd::Bdd<Type>, storm::dd::Bdd<Type>> performProb01(storm::models::symbolic::DeterministicModel<Type> const& model, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates) {
                std::pair<storm::dd::Bdd<Type>, storm::dd::Bdd<Type>> result;
                storm::dd::Bdd<Type> transitionMatrix = model.getTransitionMatrix().notZero();
                result.first = performProbGreater0(model, transitionMatrix, phiStates, psiStates);
                result.second = !performProbGreater0(model, transitionMatrix, !psiStates && model.getReachableStates(), !result.first && model.getReachableStates()) && model.getReachableStates();
                result.first = !result.first && model.getReachableStates();
                return result;
            }
            
            /*!
             * Computes the sets of states that have probability 0 or 1, respectively, of satisfying phi until psi in a
             * deterministic model.
             *
             * @param model The (symbolic) model for which to compute the set of states. This is used for retrieving the
             * manager and information about the meta variables.
             * @param phiStates The BDD containing all  phi states of the model.
             * @param psiStates The BDD containing all psi states of the model.
             * @return A pair of BDDs that represent all states with probability 0 and 1, respectively.
             */
            template <storm::dd::DdType Type>
            static std::pair<storm::dd::Bdd<Type>, storm::dd::Bdd<Type>> performProb01(storm::models::symbolic::Model<Type> const& model, storm::dd::Add<Type> const& transitionMatrix, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates) {
                std::pair<storm::dd::Bdd<Type>, storm::dd::Bdd<Type>> result;
                storm::dd::Bdd<Type> transitionMatrixBdd = transitionMatrix.notZero();
                result.first = performProbGreater0(model, transitionMatrixBdd, phiStates, psiStates);
                result.second = !performProbGreater0(model, transitionMatrixBdd, !psiStates && model.getReachableStates(), !result.first && model.getReachableStates()) && model.getReachableStates();
                result.first = !result.first && model.getReachableStates();
                return result;
            }
            
            /*!
             * Computes the sets of states that have probability greater 0 of satisfying phi until psi under at least
             * one possible resolution of non-determinism in a non-deterministic model. Stated differently,
             * this means that these states have a probability greater 0 of satisfying phi until psi if the
             * scheduler tries to minimize this probability.
             *
             * @param model The model whose graph structure to search.
             * @param backwardTransitions The reversed transition relation of the model.
             * @param phiStates The set of all states satisfying phi.
             * @param psiStates The set of all states satisfying psi.
             * @param useStepBound A flag that indicates whether or not to use the given number of maximal steps for the search.
             * @param maximalSteps The maximal number of steps to reach the psi states.
             * @return A bit vector that represents all states with probability 0.
             */
            template <typename T>
            storm::storage::BitVector performProbGreater0E(storm::storage::SparseMatrix<T> const& transitionMatrix, std::vector<uint_fast64_t> const& nondeterministicChoiceIndices, storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates, bool useStepBound = false, uint_fast64_t maximalSteps = 0) {
                size_t numberOfStates = phiStates.size();
                
                // Prepare resulting bit vector.
                storm::storage::BitVector statesWithProbabilityGreater0(numberOfStates);
                
                // Add all psi states as the already satisfy the condition.
                statesWithProbabilityGreater0 |= psiStates;
                
                // Initialize the stack used for the DFS with the states
                std::vector<uint_fast64_t> stack(psiStates.begin(), psiStates.end());
                
                // Initialize the stack for the step bound, if the number of steps is bounded.
                std::vector<uint_fast64_t> stepStack;
                std::vector<uint_fast64_t> remainingSteps;
                if (useStepBound) {
                    stepStack.reserve(numberOfStates);
                    stepStack.insert(stepStack.begin(), psiStates.getNumberOfSetBits(), maximalSteps);
                    remainingSteps.resize(numberOfStates);
                    for (auto state : psiStates) {
                        remainingSteps[state] = maximalSteps;
                    }
                }
                
                // Perform the actual DFS.
                uint_fast64_t currentState, currentStepBound;
                while (!stack.empty()) {
                    currentState = stack.back();
                    stack.pop_back();
                    
                    if (useStepBound) {
                        currentStepBound = stepStack.back();
                        stepStack.pop_back();
                    }
                    
                    for (typename storm::storage::SparseMatrix<T>::const_iterator entryIt = backwardTransitions.begin(currentState), entryIte = backwardTransitions.end(currentState); entryIt != entryIte; ++entryIt) {
                        if (phiStates.get(entryIt->getColumn()) && (!statesWithProbabilityGreater0.get(entryIt->getColumn()) || (useStepBound && remainingSteps[entryIt->getColumn()] < currentStepBound - 1))) {
                            // If we don't have a bound on the number of steps to take, just add the state to the stack.
                            if (!useStepBound) {
                                statesWithProbabilityGreater0.set(entryIt->getColumn(), true);
                                stack.push_back(entryIt->getColumn());
                            } else if (currentStepBound > 0) {
                                // If there is at least one more step to go, we need to push the state and the new number of steps.
                                remainingSteps[entryIt->getColumn()] = currentStepBound - 1;
                                statesWithProbabilityGreater0.set(entryIt->getColumn(), true);
                                stack.push_back(entryIt->getColumn());
                                stepStack.push_back(currentStepBound - 1);
                            }
                        }
                    }
                }
                
                return statesWithProbabilityGreater0;
            }
            
            template <typename T>
            storm::storage::BitVector performProb0A(storm::storage::SparseMatrix<T> const& transitionMatrix, std::vector<uint_fast64_t> const& nondeterministicChoiceIndices, storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                storm::storage::BitVector statesWithProbability0 = performProbGreater0E(transitionMatrix, nondeterministicChoiceIndices, backwardTransitions, phiStates, psiStates);
                statesWithProbability0.complement();
                return statesWithProbability0;
            }
            
            /*!
             * Computes the sets of states that have probability 0 of satisfying phi until psi under all
             * possible resolutions of non-determinism in a non-deterministic model. Stated differently,
             * this means that these states have probability 0 of satisfying phi until psi even if the
             * scheduler tries to maximize this probability.
             *
             * @param model The model whose graph structure to search.
             * @param backwardTransitions The reversed transition relation of the model.
             * @param phiStates The set of all states satisfying phi.
             * @param psiStates The set of all states satisfying psi.
             * @param useStepBound A flag that indicates whether or not to use the given number of maximal steps for the search.
             * @param maximalSteps The maximal number of steps to reach the psi states.
             * @return A bit vector that represents all states with probability 0.
             */
            template <typename T>
            storm::storage::BitVector performProb0A(storm::models::sparse::NondeterministicModel<T> const& model, storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                return performProb0A(model.getTransitionMatrix(), model.getNondeterministicChoiceIndices(), backwardTransitions, phiStates, psiStates);
            }

            /*!
             * Computes the sets of states that have probability 1 of satisfying phi until psi under at least
             * one possible resolution of non-determinism in a non-deterministic model. Stated differently,
             * this means that these states have probability 1 of satisfying phi until psi if the
             * scheduler tries to maximize this probability.
             *
             * @param model The model whose graph structure to search.
             * @param backwardTransitions The reversed transition relation of the model.
             * @param phiStates The set of all states satisfying phi.
             * @param psiStates The set of all states satisfying psi.
             * @return A bit vector that represents all states with probability 1.
             */
            template <typename T>
            storm::storage::BitVector performProb1E(storm::storage::SparseMatrix<T> const& transitionMatrix, std::vector<uint_fast64_t> const& nondeterministicChoiceIndices, storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                size_t numberOfStates = phiStates.size();
                
                // Initialize the environment for the iterative algorithm.
                storm::storage::BitVector currentStates(numberOfStates, true);
                std::vector<uint_fast64_t> stack;
                stack.reserve(numberOfStates);
                
                // Perform the loop as long as the set of states gets larger.
                bool done = false;
                uint_fast64_t currentState;
                while (!done) {
                    stack.clear();
                    storm::storage::BitVector nextStates(psiStates);
                    stack.insert(stack.end(), psiStates.begin(), psiStates.end());
                    
                    while (!stack.empty()) {
                        currentState = stack.back();
                        stack.pop_back();
                        
                        for (typename storm::storage::SparseMatrix<T>::const_iterator predecessorEntryIt = backwardTransitions.begin(currentState), predecessorEntryIte = backwardTransitions.end(currentState); predecessorEntryIt != predecessorEntryIte; ++predecessorEntryIt) {
                            if (phiStates.get(predecessorEntryIt->getColumn()) && !nextStates.get(predecessorEntryIt->getColumn())) {
                                // Check whether the predecessor has only successors in the current state set for one of the
                                // nondeterminstic choices.
                                for (uint_fast64_t row = nondeterministicChoiceIndices[predecessorEntryIt->getColumn()]; row < nondeterministicChoiceIndices[predecessorEntryIt->getColumn() + 1]; ++row) {
                                    bool allSuccessorsInCurrentStates = true;
                                    for (typename storm::storage::SparseMatrix<T>::const_iterator successorEntryIt = transitionMatrix.begin(row), successorEntryIte = transitionMatrix.end(row); successorEntryIt != successorEntryIte; ++successorEntryIt) {
                                        if (!currentStates.get(successorEntryIt->getColumn())) {
                                            allSuccessorsInCurrentStates = false;
                                            break;
                                        }
                                    }
                                    
                                    // If all successors for a given nondeterministic choice are in the current state set, we
                                    // add it to the set of states for the next iteration and perform a backward search from
                                    // that state.
                                    if (allSuccessorsInCurrentStates) {
                                        nextStates.set(predecessorEntryIt->getColumn(), true);
                                        stack.push_back(predecessorEntryIt->getColumn());
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    
                    // Check whether we need to perform an additional iteration.
                    if (currentStates == nextStates) {
                        done = true;
                    } else {
                        currentStates = std::move(nextStates);
                    }
                }
                
                return currentStates;
            }
            
            /*!
             * Computes the sets of states that have probability 1 of satisfying phi until psi under at least
             * one possible resolution of non-determinism in a non-deterministic model. Stated differently,
             * this means that these states have probability 1 of satisfying phi until psi if the
             * scheduler tries to maximize this probability.
             *
             * @param model The model whose graph structure to search.
             * @param backwardTransitions The reversed transition relation of the model.
             * @param phiStates The set of all states satisfying phi.
             * @param psiStates The set of all states satisfying psi.
             * @return A bit vector that represents all states with probability 1.
             */
            template <typename T>
            storm::storage::BitVector performProb1E(storm::models::sparse::NondeterministicModel<T> const& model, storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                return performProb1E(model.getTransitionMatrix(), model.getNondeterministicChoiceIndices(), backwardTransitions, phiStates, psiStates);
            }
            
            template <typename T>
            std::pair<storm::storage::BitVector, storm::storage::BitVector> performProb01Max(storm::storage::SparseMatrix<T> const& transitionMatrix, std::vector<uint_fast64_t> const& nondeterministicChoiceIndices, storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                std::pair<storm::storage::BitVector, storm::storage::BitVector> result;
                
                result.first = performProb0A(transitionMatrix, nondeterministicChoiceIndices, backwardTransitions, phiStates, psiStates);
                result.second = performProb1E(transitionMatrix, nondeterministicChoiceIndices, backwardTransitions, phiStates, psiStates);
                return result;
            }

            /*!
             * Computes the sets of states that have probability 0 or 1, respectively, of satisfying phi
             * until psi in a non-deterministic model in which all non-deterministic choices are resolved
             * such that the probability is maximized.
             *
             * @param model The model whose graph structure to search.
             * @param phiStates The set of all states satisfying phi.
             * @param psiStates The set of all states satisfying psi.
             * @return A pair of bit vectors that represent all states with probability 0 and 1, respectively.
             */
            template <typename T>
            std::pair<storm::storage::BitVector, storm::storage::BitVector> performProb01Max(storm::models::sparse::NondeterministicModel<T> const& model, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                return performProb01Max(model.getTransitionMatrix(), model.getTransitionMatrix().getRowGroupIndices(), model.getBackwardTransitions(), phiStates, psiStates);
            }
            
            /*!
             * Computes the sets of states that have probability greater 0 of satisfying phi until psi under any
             * possible resolution of non-determinism in a non-deterministic model. Stated differently,
             * this means that these states have a probability greater 0 of satisfying phi until psi if the
             * scheduler tries to maximize this probability.
             *
             * @param model The model whose graph structure to search.
             * @param backwardTransitions The reversed transition relation of the model.
             * @param phiStates The set of all states satisfying phi.
             * @param psiStates The set of all states satisfying psi.
             * @param useStepBound A flag that indicates whether or not to use the given number of maximal steps for the search.
             * @param maximalSteps The maximal number of steps to reach the psi states.
             * @return A bit vector that represents all states with probability 0.
             */
            template <typename T>
            storm::storage::BitVector performProbGreater0A(storm::storage::SparseMatrix<T> const& transitionMatrix, std::vector<uint_fast64_t> const& nondeterministicChoiceIndices, storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates, bool useStepBound = false, uint_fast64_t maximalSteps = 0) {
                size_t numberOfStates = phiStates.size();
                
                // Prepare resulting bit vector.
                storm::storage::BitVector statesWithProbabilityGreater0(numberOfStates);
                
                // Add all psi states as the already satisfy the condition.
                statesWithProbabilityGreater0 |= psiStates;
                
                // Initialize the stack used for the DFS with the states
                std::vector<uint_fast64_t> stack(psiStates.begin(), psiStates.end());
                
                // Initialize the stack for the step bound, if the number of steps is bounded.
                std::vector<uint_fast64_t> stepStack;
                std::vector<uint_fast64_t> remainingSteps;
                if (useStepBound) {
                    stepStack.reserve(numberOfStates);
                    stepStack.insert(stepStack.begin(), psiStates.getNumberOfSetBits(), maximalSteps);
                    remainingSteps.resize(numberOfStates);
                    for (auto state : psiStates) {
                        remainingSteps[state] = maximalSteps;
                    }
                }
                
                // Perform the actual DFS.
                uint_fast64_t currentState, currentStepBound;
                while(!stack.empty()) {
                    currentState = stack.back();
                    stack.pop_back();
                    
                    if (useStepBound) {
                        currentStepBound = stepStack.back();
                        stepStack.pop_back();
                    }
                    
                    for(typename storm::storage::SparseMatrix<T>::const_iterator predecessorEntryIt = backwardTransitions.begin(currentState), predecessorEntryIte = backwardTransitions.end(currentState); predecessorEntryIt != predecessorEntryIte; ++predecessorEntryIt) {
                        if (phiStates.get(predecessorEntryIt->getColumn()) && (!statesWithProbabilityGreater0.get(predecessorEntryIt->getColumn()) || (useStepBound && remainingSteps[predecessorEntryIt->getColumn()] < currentStepBound - 1))) {
                            // Check whether the predecessor has at least one successor in the current state set for every
                            // nondeterministic choice.
                            bool addToStatesWithProbabilityGreater0 = true;
                            for (uint_fast64_t row = nondeterministicChoiceIndices[predecessorEntryIt->getColumn()]; row < nondeterministicChoiceIndices[predecessorEntryIt->getColumn() + 1]; ++row) {
                                bool hasAtLeastOneSuccessorWithProbabilityGreater0 = false;
                                for (typename storm::storage::SparseMatrix<T>::const_iterator successorEntryIt = transitionMatrix.begin(row), successorEntryIte = transitionMatrix.end(row); successorEntryIt != successorEntryIte; ++successorEntryIt) {
                                    if (statesWithProbabilityGreater0.get(successorEntryIt->getColumn())) {
                                        hasAtLeastOneSuccessorWithProbabilityGreater0 = true;
                                        break;
                                    }
                                }
                                
                                if (!hasAtLeastOneSuccessorWithProbabilityGreater0) {
                                    addToStatesWithProbabilityGreater0 = false;
                                    break;
                                }
                            }
                            
                            // If we need to add the state, then actually add it and perform further search from the state.
                            if (addToStatesWithProbabilityGreater0) {
                                // If we don't have a bound on the number of steps to take, just add the state to the stack.
                                if (!useStepBound) {
                                    statesWithProbabilityGreater0.set(predecessorEntryIt->getColumn(), true);
                                    stack.push_back(predecessorEntryIt->getColumn());
                                } else if (currentStepBound > 0) {
                                    // If there is at least one more step to go, we need to push the state and the new number of steps.
                                    remainingSteps[predecessorEntryIt->getColumn()] = currentStepBound - 1;
                                    statesWithProbabilityGreater0.set(predecessorEntryIt->getColumn(), true);
                                    stack.push_back(predecessorEntryIt->getColumn());
                                    stepStack.push_back(currentStepBound - 1);
                                }
                            }
                        }
                    }
                }
                
                return statesWithProbabilityGreater0;
            }
            
            /*!
             * Computes the sets of states that have probability 0 of satisfying phi until psi under at least
             * one possible resolution of non-determinism in a non-deterministic model. Stated differently,
             * this means that these states have probability 0 of satisfying phi until psi if the
             * scheduler tries to minimize this probability.
             *
             * @param model The model whose graph structure to search.
             * @param backwardTransitions The reversed transition relation of the model.
             * @param phiStates The set of all states satisfying phi.
             * @param psiStates The set of all states satisfying psi.
             * @return A bit vector that represents all states with probability 0.
             */
            template <typename T>
            storm::storage::BitVector performProb0E(storm::models::sparse::NondeterministicModel<T> const& model, storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                storm::storage::BitVector statesWithProbability0 = performProbGreater0A(model.getTransitionMatrix(), model.getNondeterministicChoiceIndices(), backwardTransitions, phiStates, psiStates);
                statesWithProbability0.complement();
                return statesWithProbability0;
            }

            template <typename T>
            storm::storage::BitVector performProb0E(storm::storage::SparseMatrix<T> const& transitionMatrix, std::vector<uint_fast64_t> const& nondeterministicChoiceIndices,  storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                storm::storage::BitVector statesWithProbability0 = performProbGreater0A(transitionMatrix, nondeterministicChoiceIndices, backwardTransitions, phiStates, psiStates);
                statesWithProbability0.complement();
                return statesWithProbability0;
            }
            
            /*!
             * Computes the sets of states that have probability 1 of satisfying phi until psi under all
             * possible resolutions of non-determinism in a non-deterministic model. Stated differently,
             * this means that these states have probability 1 of satisfying phi until psi even if the
             * scheduler tries to minimize this probability.
             *
             * @param model The model whose graph structure to search.
             * @param backwardTransitions The reversed transition relation of the model.
             * @param phiStates The set of all states satisfying phi.
             * @param psiStates The set of all states satisfying psi.
             * @return A bit vector that represents all states with probability 0.
             */
            template <typename T>
            storm::storage::BitVector performProb1A( storm::storage::SparseMatrix<T> const& transitionMatrix, std::vector<uint_fast64_t> const& nondeterministicChoiceIndices, storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                size_t numberOfStates = phiStates.size();
                
                // Initialize the environment for the iterative algorithm.
                storm::storage::BitVector currentStates(numberOfStates, true);
                std::vector<uint_fast64_t> stack;
                stack.reserve(numberOfStates);
                
                // Perform the loop as long as the set of states gets smaller.
                bool done = false;
                uint_fast64_t currentState;
                while (!done) {
                    stack.clear();
                    storm::storage::BitVector nextStates(psiStates);
                    stack.insert(stack.end(), psiStates.begin(), psiStates.end());
                    
                    while (!stack.empty()) {
                        currentState = stack.back();
                        stack.pop_back();
                        
                        for(typename storm::storage::SparseMatrix<T>::const_iterator predecessorEntryIt = backwardTransitions.begin(currentState), predecessorEntryIte = backwardTransitions.end(currentState); predecessorEntryIt != predecessorEntryIte; ++predecessorEntryIt) {
                            if (phiStates.get(predecessorEntryIt->getColumn()) && !nextStates.get(predecessorEntryIt->getColumn())) {
                                // Check whether the predecessor has only successors in the current state set for all of the
                                // nondeterminstic choices.
                                bool allSuccessorsInCurrentStatesForAllChoices = true;
                                for (typename storm::storage::SparseMatrix<T>::const_iterator successorEntryIt = transitionMatrix.begin(nondeterministicChoiceIndices[predecessorEntryIt->getColumn()]), successorEntryIte = transitionMatrix.begin(nondeterministicChoiceIndices[predecessorEntryIt->getColumn() + 1]); successorEntryIt != successorEntryIte; ++successorEntryIt) {
                                    if (!currentStates.get(successorEntryIt->getColumn())) {
                                        allSuccessorsInCurrentStatesForAllChoices = false;
                                        goto afterCheckLoop;
                                    }
                                }
                                
                            afterCheckLoop:
                                // If all successors for all nondeterministic choices are in the current state set, we
                                // add it to the set of states for the next iteration and perform a backward search from
                                // that state.
                                if (allSuccessorsInCurrentStatesForAllChoices) {
                                    nextStates.set(predecessorEntryIt->getColumn(), true);
                                    stack.push_back(predecessorEntryIt->getColumn());
                                }
                            }
                        }
                    }
                    
                    // Check whether we need to perform an additional iteration.
                    if (currentStates == nextStates) {
                        done = true;
                    } else {
                        currentStates = std::move(nextStates);
                    }
                }
                return currentStates;
            }
            
            template <typename T>
            std::pair<storm::storage::BitVector, storm::storage::BitVector> performProb01Min(storm::storage::SparseMatrix<T> const& transitionMatrix, std::vector<uint_fast64_t> const& nondeterministicChoiceIndices, storm::storage::SparseMatrix<T> const& backwardTransitions, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                std::pair<storm::storage::BitVector, storm::storage::BitVector> result;
                result.first = performProb0E(transitionMatrix, nondeterministicChoiceIndices, backwardTransitions, phiStates, psiStates);
                result.second = performProb1A(transitionMatrix, nondeterministicChoiceIndices, backwardTransitions, phiStates, psiStates);
                return result;
            }

            /*!
             * Computes the sets of states that have probability 0 or 1, respectively, of satisfying phi
             * until psi in a non-deterministic model in which all non-deterministic choices are resolved
             * such that the probability is minimized.
             *
             * @param model The model whose graph structure to search.
             * @param phiStates The set of all states satisfying phi.
             * @param psiStates The set of all states satisfying psi.
             * @return A pair of bit vectors that represent all states with probability 0 and 1, respectively.
             */
            template <typename T>
            std::pair<storm::storage::BitVector, storm::storage::BitVector> performProb01Min(storm::models::sparse::NondeterministicModel<T> const& model, storm::storage::BitVector const& phiStates, storm::storage::BitVector const& psiStates) {
                return performProb01Min(model.getTransitionMatrix(), model.getTransitionMatrix().getRowGroupIndices(), model.getBackwardTransitions(), phiStates, psiStates);
            }
            
            /*!
             * Computes the set of states for which there exists a scheduler that achieves a probability greater than
             * zero of satisfying phi until psi.
             *
             * @param model The (symbolic) model for which to compute the set of states.
             * @param transitionMatrix The transition matrix of the model as a BDD.
             * @param phiStates The BDD containing all phi states of the model.
             * @param psiStates The BDD containing all psi states of the model.
             * @return A BDD representing all such states.
             */
            template <storm::dd::DdType Type>
            storm::dd::Bdd<Type> performProbGreater0E(storm::models::symbolic::NondeterministicModel<Type> const& model, storm::dd::Bdd<Type> const& transitionMatrix, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates) {
                // Initialize environment for backward search.
                storm::dd::DdManager<Type> const& manager = model.getManager();
                storm::dd::Bdd<Type> lastIterationStates = manager.getBddZero();
                storm::dd::Bdd<Type> statesWithProbabilityGreater0E = psiStates;

                uint_fast64_t iterations = 0;
                storm::dd::Bdd<Type> abstractedTransitionMatrix = transitionMatrix.existsAbstract(model.getNondeterminismVariables());
                while (lastIterationStates != statesWithProbabilityGreater0E) {
                    lastIterationStates = statesWithProbabilityGreater0E;
                    statesWithProbabilityGreater0E = statesWithProbabilityGreater0E.swapVariables(model.getRowColumnMetaVariablePairs());
                    statesWithProbabilityGreater0E = statesWithProbabilityGreater0E.andExists(abstractedTransitionMatrix, model.getColumnVariables());
                    statesWithProbabilityGreater0E &= phiStates;
                    statesWithProbabilityGreater0E |= lastIterationStates;
                    ++iterations;
                }
                                
                return statesWithProbabilityGreater0E;
            }
            
            /*!
             * Computes the set of states for which there does not exist a scheduler that achieves a probability greater
             * than zero of satisfying phi until psi.
             *
             * @param model The (symbolic) model for which to compute the set of states.
             * @param transitionMatrix The transition matrix of the model as a BDD.
             * @param phiStates The phi states of the model.
             * @param psiStates The psi states of the model.
             * @return A BDD representing all such states.
             */
            template <storm::dd::DdType Type>
            storm::dd::Bdd<Type> performProb0A(storm::models::symbolic::NondeterministicModel<Type> const& model, storm::dd::Bdd<Type> const& transitionMatrix, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates) {
                return !performProbGreater0E(model, transitionMatrix, phiStates, psiStates) && model.getReachableStates();
            }
            
            /*!
             * Computes the set of states for which all schedulers achieve a probability greater than zero of satisfying
             * phi until psi.
             *
             * @param model The (symbolic) model for which to compute the set of states.
             * @param transitionMatrix The transition matrix of the model as a BDD.
             * @param phiStates The BDD containing all phi states of the model.
             * @param psiStates The BDD containing all psi states of the model.
             * @return A BDD representing all such states.
             */
            template <storm::dd::DdType Type>
            storm::dd::Bdd<Type> performProbGreater0A(storm::models::symbolic::NondeterministicModel<Type> const& model, storm::dd::Bdd<Type> const& transitionMatrix, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates) {
                // Initialize environment for backward search.
                storm::dd::DdManager<Type> const& manager = model.getManager();
                storm::dd::Bdd<Type> lastIterationStates = manager.getBddZero();
                storm::dd::Bdd<Type> statesWithProbabilityGreater0A = psiStates;
                
                uint_fast64_t iterations = 0;
                while (lastIterationStates != statesWithProbabilityGreater0A) {
                    lastIterationStates = statesWithProbabilityGreater0A;
                    statesWithProbabilityGreater0A = statesWithProbabilityGreater0A.swapVariables(model.getRowColumnMetaVariablePairs());
                    statesWithProbabilityGreater0A = statesWithProbabilityGreater0A.andExists(transitionMatrix, model.getColumnVariables());
                    statesWithProbabilityGreater0A |= model.getIllegalMask();
                    statesWithProbabilityGreater0A = statesWithProbabilityGreater0A.universalAbstract(model.getNondeterminismVariables());
                    statesWithProbabilityGreater0A &= phiStates;
                    statesWithProbabilityGreater0A |= psiStates;
                    ++iterations;
                }
                
                return statesWithProbabilityGreater0A;
            }
            
            /*!
             * Computes the set of states for which there exists a scheduler that achieves probability zero of satisfying
             * phi until psi.
             *
             * @param model The (symbolic) model for which to compute the set of states.
             * @param transitionMatrix The transition matrix of the model as a BDD.
             * @param phiStates The BDD containing all phi states of the model.
             * @param psiStates The BDD containing all psi states of the model.
             * @return A BDD representing all such states.
             */
            template <storm::dd::DdType Type>
            storm::dd::Bdd<Type> performProb0E(storm::models::symbolic::NondeterministicModel<Type> const& model, storm::dd::Bdd<Type> const& transitionMatrix, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates) {
                return !performProbGreater0A(model, transitionMatrix, phiStates, psiStates) && model.getReachableStates();
            }
            
            /*!
             * Computes the set of states for which all schedulers achieve probability one of satisfying phi until psi.
             *
             * @param model The (symbolic) model for which to compute the set of states.
             * @param transitionMatrix The transition matrix of the model as a BDD.
             * @param phiStates The BDD containing all phi states of the model.
             * @param psiStates The BDD containing all psi states of the model.
             * @param statesWithProbabilityGreater0A The states of the model that have a probability greater zero under
             * all schedulers.
             * @return A BDD representing all such states.
             */
            template <storm::dd::DdType Type>
            storm::dd::Bdd<Type> performProb1A(storm::models::symbolic::NondeterministicModel<Type> const& model, storm::dd::Bdd<Type> const& transitionMatrix, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates, storm::dd::Bdd<Type> const& statesWithProbabilityGreater0A) {
                // Initialize environment for backward search.
                storm::dd::DdManager<Type> const& manager = model.getManager();
                storm::dd::Bdd<Type> lastIterationStates = manager.getBddZero();
                storm::dd::Bdd<Type> statesWithProbability1A = psiStates || statesWithProbabilityGreater0A;
                
                uint_fast64_t iterations = 0;
                while (lastIterationStates != statesWithProbability1A) {
                    lastIterationStates = statesWithProbability1A;
                    statesWithProbability1A = statesWithProbability1A.swapVariables(model.getRowColumnMetaVariablePairs());
                    statesWithProbability1A = transitionMatrix.implies(statesWithProbability1A).universalAbstract(model.getColumnVariables());
                    statesWithProbability1A |= model.getIllegalMask();
                    statesWithProbability1A = statesWithProbability1A.universalAbstract(model.getNondeterminismVariables());
                    statesWithProbability1A &= statesWithProbabilityGreater0A;
                    statesWithProbability1A |= psiStates;
                    ++iterations;
                }
                
                return statesWithProbability1A;
            }
            
            /*!
             * Computes the set of states for which there exists a scheduler that achieves probability one of satisfying
             * phi until psi.
             *
             * @param model The (symbolic) model for which to compute the set of states.
             * @param transitionMatrix The transition matrix of the model as a BDD.
             * @param phiStates The BDD containing all phi states of the model.
             * @param psiStates The BDD containing all psi states of the model.
             * @param statesWithProbabilityGreater0E The states of the model that have a scheduler that achieves a value
             * greater than zero.
             * @return A BDD representing all such states.
             */
            template <storm::dd::DdType Type>
            storm::dd::Bdd<Type> performProb1E(storm::models::symbolic::NondeterministicModel<Type> const& model, storm::dd::Bdd<Type> const& transitionMatrix, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates, storm::dd::Bdd<Type> const& statesWithProbabilityGreater0E) {
                // Initialize environment for backward search.
                storm::dd::DdManager<Type> const& manager = model.getManager();
                storm::dd::Bdd<Type> statesWithProbability1E = statesWithProbabilityGreater0E;

                uint_fast64_t iterations = 0;
                bool outerLoopDone = false;
                while (!outerLoopDone) {
                    storm::dd::Bdd<Type> innerStates = manager.getBddZero();
                    
                    bool innerLoopDone = false;
                    while (!innerLoopDone) {
                        storm::dd::Bdd<Type> temporary = statesWithProbability1E.swapVariables(model.getRowColumnMetaVariablePairs());
                        temporary = transitionMatrix.implies(temporary).universalAbstract(model.getColumnVariables());
                        
                        storm::dd::Bdd<Type> temporary2 = innerStates.swapVariables(model.getRowColumnMetaVariablePairs());
                        temporary2 = transitionMatrix.andExists(temporary2, model.getColumnVariables());
                        
                        temporary = temporary.andExists(temporary2, model.getNondeterminismVariables());
                        temporary &= phiStates;
                        temporary |= psiStates;
                        
                        if (innerStates == temporary) {
                            innerLoopDone = true;
                        } else {
                            innerStates = temporary;
                        }
                    }
                    
                    if (statesWithProbability1E == innerStates) {
                        outerLoopDone = true;
                    } else {
                        statesWithProbability1E = innerStates;
                    }
                    ++iterations;
                }

                return statesWithProbability1E;
            }
            
            template <storm::dd::DdType Type>
            std::pair<storm::dd::Bdd<Type>, storm::dd::Bdd<Type>> performProb01Max(storm::models::symbolic::NondeterministicModel<Type> const& model, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates) {
                std::pair<storm::dd::Bdd<Type>, storm::dd::Bdd<Type>> result;
                storm::dd::Bdd<Type> transitionMatrix = model.getTransitionMatrix().notZero();
                result.first = performProb0A(model, transitionMatrix, phiStates, psiStates);
                result.second = performProb1E(model, transitionMatrix, phiStates, psiStates, !result.first && model.getReachableStates());
                return result;
            }
            
            template <storm::dd::DdType Type>
            std::pair<storm::dd::Bdd<Type>, storm::dd::Bdd<Type>> performProb01Min(storm::models::symbolic::NondeterministicModel<Type> const& model, storm::dd::Bdd<Type> const& phiStates, storm::dd::Bdd<Type> const& psiStates) {
                std::pair<storm::dd::Bdd<Type>, storm::dd::Bdd<Type>> result;
                storm::dd::Bdd<Type> transitionMatrix = model.getTransitionMatrix().notZero();
                result.first = performProb0E(model, transitionMatrix, phiStates, psiStates);
                result.second = performProb1A(model, transitionMatrix, phiStates, psiStates, !result.first && model.getReachableStates());
                return result;
            }
            
            /*!
             * Performs a topological sort of the states of the system according to the given transitions.
             *
             * @param matrix A square matrix representing the transition relation of the system.
             * @return A vector of indices that is a topological sort of the states.
             */
            template <typename T>
            std::vector<uint_fast64_t> getTopologicalSort(storm::storage::SparseMatrix<T> const& matrix) {
                if (matrix.getRowCount() != matrix.getColumnCount()) {
                    LOG4CPLUS_ERROR(logger, "Provided matrix is required to be square.");
                    throw storm::exceptions::InvalidArgumentException() << "Provided matrix is required to be square.";
                }

                uint_fast64_t numberOfStates = matrix.getRowCount();
                
                // Prepare the result. This relies on the matrix being square.
                std::vector<uint_fast64_t> topologicalSort;
                topologicalSort.reserve(numberOfStates);
                
                // Prepare the stacks needed for recursion.
                std::vector<uint_fast64_t> recursionStack;
                recursionStack.reserve(matrix.getRowCount());
                std::vector<typename storm::storage::SparseMatrix<T>::const_iterator> iteratorRecursionStack;
                iteratorRecursionStack.reserve(numberOfStates);
                
                // Perform a depth-first search over the given transitions and record states in the reverse order they were visited.
                storm::storage::BitVector visitedStates(numberOfStates);
                for (uint_fast64_t state = 0; state < numberOfStates; ++state) {
                    if (!visitedStates.get(state)) {
                        recursionStack.push_back(state);
                        iteratorRecursionStack.push_back(matrix.begin(state));
                        
                    recursionStepForward:
                        while (!recursionStack.empty()) {
                            uint_fast64_t currentState = recursionStack.back();
                            typename storm::storage::SparseMatrix<T>::const_iterator successorIterator = iteratorRecursionStack.back();
                            
                            visitedStates.set(currentState, true);
                            
                        recursionStepBackward:
                            for (; successorIterator != matrix.end(currentState); ++successorIterator) {
                                if (!visitedStates.get(successorIterator->getColumn())) {
                                    // Put unvisited successor on top of our recursion stack and remember that.
									recursionStack.push_back(successorIterator->getColumn());
                                    
                                    // Also, put initial value for iterator on corresponding recursion stack.
									iteratorRecursionStack.push_back(matrix.begin(successorIterator->getColumn()));
                                    
                                    goto recursionStepForward;
                                }
                            }
                            
                            topologicalSort.push_back(currentState);
                            
                            // If we reach this point, we have completed the recursive descent for the current state.
                            // That is, we need to pop it from the recursion stacks.
                            recursionStack.pop_back();
                            iteratorRecursionStack.pop_back();
                            
                            // If there is at least one state under the current one in our recursion stack, we need
                            // to restore the topmost state as the current state and jump to the part after the
                            // original recursive call.
                            if (recursionStack.size() > 0) {
                                currentState = recursionStack.back();
                                successorIterator = iteratorRecursionStack.back();
                                
                                goto recursionStepBackward;
                            }
                        }
                    }
                }
                
                return topologicalSort;
            }
            
            /*!
             * A class needed to compare the distances for two states in the Dijkstra search.
             */
            template<typename T>
            struct DistanceCompare {
                bool operator()(std::pair<T, uint_fast64_t> const& lhs, std::pair<T, uint_fast64_t> const& rhs) const {
                    return lhs.first > rhs.first || (lhs.first == rhs.first && lhs.second > rhs.second);
                }
            };
            
            /*!
             * Performs a Dijkstra search from the given starting states to determine the most probable paths to all other states
             * by only passing through the given state set.
             *
             * @param model The model whose state space is to be searched.
             * @param transitions The transitions wrt to which to compute the most probable paths.
             * @param startingStates The starting states of the Dijkstra search.
             * @param filterStates A set of states that must not be left on any path.
             */
            template <typename T>
            std::pair<std::vector<T>, std::vector<uint_fast64_t>> performDijkstra(storm::models::sparse::Model<T> const& model,
                                                                                  storm::storage::SparseMatrix<T> const& transitions,
                                                                                  storm::storage::BitVector const& startingStates,
                                                                                  storm::storage::BitVector const* filterStates = nullptr) {
                
                LOG4CPLUS_INFO(logger, "Performing Dijkstra search.");
                
				const uint_fast64_t noPredecessorValue = storm::utility::zero<uint_fast64_t>();
                std::vector<T> probabilities(model.getNumberOfStates(), storm::utility::zero<T>());
                std::vector<uint_fast64_t> predecessors(model.getNumberOfStates(), noPredecessorValue);
                
                // Set the probability to 1 for all starting states.
                std::set<std::pair<T, uint_fast64_t>, DistanceCompare<T>> probabilityStateSet;

                for (auto state : startingStates) {
                    probabilityStateSet.emplace(storm::utility::one<T>(), state);
                    probabilities[state] = storm::utility::one<T>();
                }
                
                // As long as there is one reachable state, we need to consider it.
                while (!probabilityStateSet.empty()) {
                    // Get the state with the least distance from the set and remove it.
                    std::pair<T, uint_fast64_t> probabilityStatePair = *probabilityStateSet.begin();
                    probabilityStateSet.erase(probabilityStateSet.begin());
                    
                    // Now check the new distances for all successors of the current state.
                    typename storm::storage::SparseMatrix<T>::Rows row = transitions.getRow(probabilityStatePair.second);
                    for (auto const& transition : row) {
                        // Only follow the transition if it lies within the filtered states.
                        if (filterStates != nullptr && filterStates->get(transition.first)) {
                            // Calculate the distance we achieve when we take the path to the successor via the current state.
                            T newDistance = probabilityStatePair.first * transition.second;
                            
                            // We found a cheaper way to get to the target state of the transition.
                            if (newDistance > probabilities[transition.first]) {
                                // Remove the old distance.
                                if (probabilities[transition.first] != noPredecessorValue) {
                                    probabilityStateSet.erase(std::make_pair(probabilities[transition.first], transition.first));
                                }
                                
                                // Set and add the new distance.
                                probabilities[transition.first] = newDistance;
                                predecessors[transition.first] = probabilityStatePair.second;
                                probabilityStateSet.insert(std::make_pair(newDistance, transition.first));
                            }
                        }
                    }
                }
                
                // Move the values into the result and return it.
                std::pair<std::vector<T>, std::vector<uint_fast64_t>> result;
                result.first = std::move(probabilities);
                result.second = std::move(predecessors);
                LOG4CPLUS_INFO(logger, "Done performing Dijkstra search.");
                return result;
            }
            
        } // namespace graph
    } // namespace utility
} // namespace storm

#endif /* STORM_UTILITY_GRAPH_H_ */
