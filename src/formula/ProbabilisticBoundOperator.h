/*
 * ProbabilisticBoundOperator.h
 *
 *  Created on: 19.10.2012
 *      Author: Thomas Heinemann
 */

#ifndef STORM_FORMULA_PROBABILISTICBOUNDOPERATOR_H_
#define STORM_FORMULA_PROBABILISTICBOUNDOPERATOR_H_

#include "AbstractStateFormula.h"
#include "AbstractPathFormula.h"
#include "src/formula/PathBoundOperator.h"
#include "utility/ConstTemplates.h"

namespace storm {
namespace formula {

template <class T> class ProbabilisticBoundOperator;

/*!
 *  @brief Interface class for model checkers that support ProbabilisticBoundOperator.
 *
 *  All model checkers that support the formula class PathBoundOperator must inherit
 *  this pure virtual class.
 */
template <class T>
class IProbabilisticBoundOperatorModelChecker {
    public:
        virtual storm::storage::BitVector* checkProbabilisticBoundOperator(const ProbabilisticBoundOperator<T>& obj) const = 0;
};

/*!
 * @brief
 * Class for a Abstract formula tree with a P (probablistic) operator node over a probability interval
 * as root.
 *
 * Has one Abstract path formula as sub formula/tree.
 *
 * @par Semantics
 * 	  The formula holds iff the probability that the path formula holds is inside the bounds
 * 	  specified in this operator
 *
 * The subtree is seen as part of the object and deleted with it
 * (this behavior can be prevented by setting them to NULL before deletion)
 *
 *
 * @see AbstractStateFormula
 * @see AbstractPathFormula
 * @see ProbabilisticOperator
 * @see ProbabilisticNoBoundsOperator
 * @see AbstractFormula
 */
template<class T>
class ProbabilisticBoundOperator : public PathBoundOperator<T> {

public:
	/*!
	 * Empty constructor
	 */
	ProbabilisticBoundOperator() : PathBoundOperator<T>
		(PathBoundOperator<T>::LESS_EQUAL, storm::utility::constGetZero<T>(), nullptr) {
		// Intentionally left empty
	}


	/*!
	 * Constructor
	 *
	 * @param comparisonRelation The relation to compare the actual value and the bound
	 * @param bound The bound for the probability
	 * @param pathFormula The child node
	 */
	ProbabilisticBoundOperator(
			typename PathBoundOperator<T>::ComparisonType comparisonRelation, T bound, AbstractPathFormula<T>* pathFormula) :
				PathBoundOperator<T>(comparisonRelation, bound, pathFormula) {
		// Intentionally left empty
	}

	/*!
	 * @returns a string representation of the formula
	 */
	virtual std::string toString() const {
		std::string result = "P ";
		result += PathBoundOperator<T>::toString();
		return result;
	}

	/*!
	 * Clones the called object.
	 *
	 * Performs a "deep copy", i.e. the subtrees of the new object are clones of the original ones
	 *
	 * @returns a new AND-object that is identical the called object.
	 */
	virtual AbstractStateFormula<T>* clone() const {
		ProbabilisticBoundOperator<T>* result = new ProbabilisticBoundOperator<T>();
		result->setComparisonOperator(this->getComparisonOperator());
		result->setBound(this->getBound());
		result->setPathFormula(this->getPathFormula().clone());
		return result;
	}

	/*!
	 * Calls the model checker to check this formula.
	 * Needed to infer the correct type of formula class.
	 *
	 * @note This function should only be called in a generic check function of a model checker class. For other uses,
	 *       the methods of the model checker should be used.
	 *
	 * @returns A bit vector indicating all states that satisfy the formula represented by the called object.
	 */
	virtual storm::storage::BitVector* check(const storm::modelChecker::AbstractModelChecker<T>& modelChecker) const {
		return modelChecker.template as<IProbabilisticBoundOperatorModelChecker>()->checkProbabilisticBoundOperator(*this);
	}
};

} //namespace formula
} //namespace storm

#endif /* STORM_FORMULA_PROBABILISTICBOUNDOPERATOR_H_ */
