/* 
 * File:   AssignmentStatement.cpp
 * Author: Lukas Westhofen
 * 
 * Created on 11. April 2015, 17:42
 */

#include "src/storage/pgcl/AssignmentStatement.h"

namespace storm {
    namespace pgcl {
        AssignmentStatement::AssignmentStatement(storm::expressions::Variable const& variable, boost::variant<storm::expressions::Expression, storm::pgcl::UniformExpression> const& expression) :
            variable(variable), expression(expression) {
        }
        
        boost::variant<storm::expressions::Expression, storm::pgcl::UniformExpression> const& AssignmentStatement::getExpression() {
            return this->expression;
        }

        storm::expressions::Variable const& AssignmentStatement::getVariable() {
            return this->variable;
        }

        void AssignmentStatement::accept(storm::pgcl::AbstractStatementVisitor& visitor) {
            visitor.visit(*this);
        }

        std::size_t AssignmentStatement::getNumberOfOutgoingTransitions() {
            if(this->expression.which() == 1) {
                return boost::get<storm::pgcl::UniformExpression>(this->expression).getEnd() - boost::get<storm::pgcl::UniformExpression>(this->expression).getBegin() + 1;
            } else {
                return 1;
            }
        }
    }
}

