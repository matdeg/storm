#ifndef STORM_MODELCHECKER_TRACEMDPMODELCHECKER_H_
#define STORM_MODELCHECKER_TRACEMDPMODELCHECKER_H_

#include "storm/modelchecker/propositional/SparsePropositionalModelChecker.h"
#include "storm/models/sparse/Mdp.h"
#include "storm/solver/MinMaxLinearEquationSolver.h"
#include "storm/storage/EventLog.h"

namespace storm {

class Environment;

namespace modelchecker {
template<class SparseMdpModelType>
class TraceMdpModelChecker : public SparsePropositionalModelChecker<SparseMdpModelType> {
   public:
    using StateType = uint_fast32_t;
    typedef typename SparseMdpModelType::ValueType ValueType;
    typedef typename SparseMdpModelType::RewardModelType RewardModelType;

    explicit TraceMdpModelChecker(SparseMdpModelType const& model);

    std::unique_ptr<CheckResult> check(Environment const& env, std::vector<uint_fast64_t> const trace);

};
}  // namespace modelchecker
}  // namespace storm

#endif /* STORM_MODELCHECKER_TRACEMDPMODELCHECKER_H_ */
