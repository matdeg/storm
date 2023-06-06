#ifndef STORM_MODELCHECKER_TRACEMDPMODELCHECKER_H_
#define STORM_MODELCHECKER_TRACEMDPMODELCHECKER_H_

#include "storm/modelchecker/prctl/SparseDtmcPrctlModelChecker.h"
#include "storm/modelchecker/propositional/SparsePropositionalModelChecker.h"
#include "storm/models/sparse/Mdp.h"
#include "storm/models/sparse/Ctmc.h"
#include "storm/solver/MinMaxLinearEquationSolver.h"
#include "storm/storage/EventLog.h"
#include "cpphoafparser/consumer/hoa_consumer.hh"

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

    std::shared_ptr<storm::logic::Formula> buildFormulaFromAcceptance(cpphoafparser::HOAConsumer::acceptance_expr& accExpr) ;
    std::shared_ptr<storm::models::sparse::Dtmc<ValueType>> buildProductAsDtmc(Environment const& env, std::vector<uint_fast64_t> const trace);
    std::shared_ptr<storm::models::sparse::Ctmc<ValueType>> buildProductAsCtmc(Environment const& env, std::vector<uint_fast64_t> const trace);
    std::shared_ptr<storm::models::sparse::Ctmc<ValueType>> buildAsCtmc(Environment const& env);
    void checkPsl(Environment const& env, std::string stringPsl);

};
}  // namespace modelchecker
}  // namespace storm

#endif /* STORM_MODELCHECKER_TRACEMDPMODELCHECKER_H_ */
