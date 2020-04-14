#include <vector>
#include <sstream>
#include "storm/storage/expressions/Expressions.h"
#include "storm/solver/SmtSolver.h"
#include "storm/models/sparse/Pomdp.h"
#include "storm/utility/solver.h"
#include "storm/utility/Stopwatch.h"
#include "storm/exceptions/UnexpectedException.h"

#include "storm-pomdp/analysis/WinningRegion.h"

namespace storm {
namespace pomdp {

    class MemlessSearchOptions {

    public:
        void setExportSATCalls(std::string const& path) {
            exportSATcalls = path;
        }

        std::string const& getExportSATCallsPath() const {
            return exportSATcalls;
        }

        bool isExportSATSet() const {
            return exportSATcalls != "";
        }

        void setDebugLevel(uint64_t level = 1) {
            debugLevel = level;
        }

        bool computeInfoOutput() const {
            return debugLevel > 0;
        }

        bool computeDebugOutput() const {
            return debugLevel > 1;
        }

        bool computeTraceOutput() const {
            return debugLevel > 2;
        }

        bool onlyDeterministicStrategies = false;
        bool lookaheadRequired = true;

    private:
        std::string exportSATcalls = "";
        uint64_t debugLevel = 0;


    };

    struct InternalObservationScheduler {
        std::vector<storm::storage::BitVector> actions;
        std::vector<uint64_t> schedulerRef;
        storm::storage::BitVector switchObservations;

        void reset(uint64_t nrObservations, uint64_t nrActions) {
            actions = std::vector<storm::storage::BitVector>(nrObservations, storm::storage::BitVector(nrActions));
            schedulerRef = std::vector<uint64_t>(nrObservations, 0);
            switchObservations.clear();
        }

        bool empty() const {
            return actions.empty();
        }

        void printForObservations(storm::storage::BitVector const& observations, storm::storage::BitVector const& observationsAfterSwitch) const {
            for (uint64_t obs = 0; obs < observations.size(); ++obs) {
                if (observations.get(obs) || observationsAfterSwitch.get(obs)) {
                    STORM_LOG_INFO("For observation: " << obs);
                }
                if (observations.get(obs)) {
                    std::stringstream ss;
                    ss << "actions:";
                    for (auto act : actions[obs]) {
                        ss << " " << act;
                    }
                    if (switchObservations.get(obs)) {
                       ss << " and switch.";
                    }
                    STORM_LOG_INFO(ss.str());
                }
                if (observationsAfterSwitch.get(obs)) {
                    STORM_LOG_INFO("scheduler ref: " << schedulerRef[obs]);
                }

            }
        }
    };

    template<typename ValueType>
    class MemlessStrategySearchQualitative {
    // Implements an extension to the Chatterjee, Chmelik, Davies (AAAI-16) paper.

    public:
        class Statistics {
            public:
                Statistics() = default;
                void print() const;

                storm::utility::Stopwatch totalTimer;
                storm::utility::Stopwatch smtCheckTimer;
                storm::utility::Stopwatch initializeSolverTimer;
                storm::utility::Stopwatch updateExtensionSolverTime;
                storm::utility::Stopwatch updateNewStrategySolverTime;

                storm::utility::Stopwatch winningRegionUpdatesTimer;

                void incrementOuterIterations() {
                    outerIterations++;
                }

            void incrementSmtChecks() {
                satCalls++;
            }
        private:
                uint64_t satCalls = 0;
                uint64_t outerIterations = 0;
        };

        MemlessStrategySearchQualitative(storm::models::sparse::Pomdp<ValueType> const& pomdp,
                                         std::set<uint32_t> const& targetObservationSet,
                                         storm::storage::BitVector const& targetStates,
                                         storm::storage::BitVector const& surelyReachSinkStates,

                                         std::shared_ptr<storm::utility::solver::SmtSolverFactory>& smtSolverFactory,
                                         MemlessSearchOptions const& options);

        void analyzeForInitialStates(uint64_t k) {
            stats.totalTimer.start();
            STORM_LOG_TRACE("Bad states: " << surelyReachSinkStates);
            STORM_LOG_TRACE("Target states: " << targetStates);
            STORM_LOG_TRACE("Questionmark states: " <<  (~surelyReachSinkStates & ~targetStates));
            bool result = analyze(k, ~surelyReachSinkStates & ~targetStates, pomdp.getInitialStates());
            stats.totalTimer.stop();
            if (result) {
                STORM_PRINT_AND_LOG("From initial state, one can almost-surely reach the target.");
            } else {
                if (k == pomdp.getNumberOfStates()) {
                    STORM_PRINT_AND_LOG("From initial state, one cannot almost-surely reach the target.");
                } else {
                    STORM_PRINT_AND_LOG("From initial state, one may not almost-surely reach the target.");
                }
            }
        }

        void findNewStrategyForSomeState(uint64_t k) {
            std::cout << surelyReachSinkStates << std::endl;
            std::cout << targetStates << std::endl;
            std::cout << (~surelyReachSinkStates & ~targetStates) << std::endl;
            stats.totalTimer.start();
            analyze(k, ~surelyReachSinkStates & ~targetStates);
            stats.totalTimer.stop();
        }

        bool analyze(uint64_t k, storm::storage::BitVector const& oneOfTheseStates, storm::storage::BitVector const& allOfTheseStates = storm::storage::BitVector());

        Statistics const& getStatistics() const;
        void finalizeStatistics();
    private:
        storm::expressions::Expression const& getDoneActionExpression(uint64_t obs) const;

        void printScheduler(std::vector<InternalObservationScheduler> const& );
        void printCoveredStates(storm::storage::BitVector const& remaining) const;

        void initialize(uint64_t k);

        bool smtCheck(uint64_t iteration, std::set<storm::expressions::Expression> const& assumptions = {});


        std::unique_ptr<storm::solver::SmtSolver> smtSolver;
        storm::models::sparse::Pomdp<ValueType> const& pomdp;
        std::shared_ptr<storm::expressions::ExpressionManager> expressionManager;
        uint64_t maxK = std::numeric_limits<uint64_t>::max();

        storm::storage::BitVector surelyReachSinkStates;
        std::set<uint32_t> targetObservations;
        storm::storage::BitVector targetStates;
        std::vector<std::vector<uint64_t>> statesPerObservation;

        std::vector<storm::expressions::Variable> schedulerVariables;
        std::vector<storm::expressions::Expression> schedulerVariableExpressions;
        std::vector<std::vector<storm::expressions::Expression>> actionSelectionVarExpressions; // A_{z,a}
        std::vector<std::vector<storm::expressions::Variable>> actionSelectionVars; // A_{z,a}

        std::vector<storm::expressions::Variable> reachVars;
        std::vector<storm::expressions::Expression> reachVarExpressions;
        std::vector<std::vector<storm::expressions::Expression>> reachVarExpressionsPerObservation;

        std::vector<storm::expressions::Variable> observationUpdatedVariables;
        std::vector<storm::expressions::Expression> observationUpdatedExpressions;

        std::vector<storm::expressions::Variable> switchVars;
        std::vector<storm::expressions::Expression> switchVarExpressions;
        std::vector<storm::expressions::Variable> continuationVars;
        std::vector<storm::expressions::Expression> continuationVarExpressions;
        std::vector<std::vector<storm::expressions::Expression>> pathVars;

        std::vector<InternalObservationScheduler> finalSchedulers;
        std::vector<std::vector<uint64_t>> schedulerForObs;
        WinningRegion winningRegion;

        MemlessSearchOptions options;
        Statistics stats;


    };
}
}
