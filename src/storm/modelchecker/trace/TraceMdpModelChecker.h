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
    typedef typename SparseMdpModelType::ValueType ValueType;
    typedef typename SparseMdpModelType::RewardModelType RewardModelType;

    explicit TraceMdpModelChecker(SparseMdpModelType const& model, storm::storage::EventLog const& eventLog);

    void check2();

   private:
    storm::storage::EventLog eventLog;
};
}  // namespace modelchecker
}  // namespace storm

#endif /* STORM_MODELCHECKER_TRACEMDPMODELCHECKER_H_ */
