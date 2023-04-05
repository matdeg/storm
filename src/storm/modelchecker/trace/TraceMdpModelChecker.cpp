#include "storm/modelchecker/trace/TraceMdpModelChecker.h"

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
TraceMdpModelChecker<SparseMdpModelType>::TraceMdpModelChecker(SparseMdpModelType const& model, storm::storage::EventLog const& eventLog) 
    : SparsePropositionalModelChecker<SparseMdpModelType>(model), eventLog(eventLog) {
    // Intentionally left empty.
}

template<typename SparseMdpModelType>
void TraceMdpModelChecker<SparseMdpModelType>::check2() {
    std::cout << "CA MARCHE \n";
}

template class TraceMdpModelChecker<storm::models::sparse::Mdp<double>>;

#ifdef STORM_HAVE_CARL
template class TraceMdpModelChecker<storm::models::sparse::Mdp<storm::RationalNumber>>;
#endif
}  // namespace modelchecker
}  // namespace storm
