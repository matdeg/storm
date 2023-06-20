#ifndef STORM_MODELCHECKER_TRACEMDPMODELCHECKERPARS_H_
#define STORM_MODELCHECKER_TRACEMDPMODELCHECKERPARS_H_

#include "storm/modelchecker/propositional/SparsePropositionalModelChecker.h"
#include "storm/models/sparse/Mdp.h"
#include "storm/solver/MinMaxLinearEquationSolver.h"
#include "storm/storage/EventLog.h"
#include "storm/storage/StronglyConnectedComponentDecomposition.h"
#include "cpphoafparser/consumer/hoa_consumer.hh"
#include "storm/models/sparse/Dtmc.h"
#include "storm/models/sparse/Ctmc.h"

namespace storm {

class Environment;

namespace modelchecker {
template<class SparseMdpModelType>
class TraceMdpModelCheckerPars : public SparsePropositionalModelChecker<SparseMdpModelType> {
   public:
    using StateType = uint_fast32_t;
    typedef typename SparseMdpModelType::ValueType ValueType;
    typedef typename SparseMdpModelType::RewardModelType RewardModelType;

    explicit TraceMdpModelCheckerPars(SparseMdpModelType const& model);
    bool isAccepting(cpphoafparser::HOAConsumer::acceptance_expr& accExpr, storm::models::sparse::StateLabeling const& stateLabeling, const storm::storage::StateBlock& scc);
    std::shared_ptr<storm::logic::Formula> buildFormulaFromAcceptance(cpphoafparser::HOAConsumer::acceptance_expr& accExpr) ;
    std::shared_ptr<storm::models::sparse::Dtmc<ValueType>> buildProductAsDtmc(Environment const& env, std::vector<uint_fast64_t> const trace, std::vector<std::string> const& parameters);
    std::shared_ptr<storm::models::sparse::Ctmc<ValueType>> buildProductAsCtmc(Environment const& env, std::vector<uint_fast64_t> const trace, std::vector<std::string> const& parameters);
    std::pair<std::shared_ptr<storm::models::sparse::Dtmc<ValueType>>, std::shared_ptr<storm::logic::Formula>>  checkPsl(Environment const& env, std::string stringPsl, std::vector<std::string> parameters);

};
}  // namespace modelchecker
}  // namespace storm

#endif /* STORM_MODELCHECKER_TRACEMDPMODELCHECKERPARS_H_ */
