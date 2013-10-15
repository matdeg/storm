/*
 * BinaryBooleanFunctionExpression.cpp
 *
 *  Created on: 10.06.2013
 *      Author: Christian Dehnert
 */

#include <sstream>

#include "BinaryNumericalFunctionExpression.h"

namespace storm {
    namespace ir {
        namespace expressions {
            
            BinaryNumericalFunctionExpression::BinaryNumericalFunctionExpression(ReturnType type, std::unique_ptr<BaseExpression>&& left, std::unique_ptr<BaseExpression>&& right, FunctionType functionType)
            : BinaryExpression(type, std::move(left), std::move(right)), functionType(functionType) {
                // Nothing to do here.
            }
            
            BinaryNumericalFunctionExpression::BinaryNumericalFunctionExpression(BinaryNumericalFunctionExpression const& binaryNumericalFunctionExpression)
            : BinaryExpression(binaryNumericalFunctionExpression), functionType(binaryNumericalFunctionExpression.functionType) {
                // Nothing to do here.
            }

            std::unique_ptr<BaseExpression> BinaryNumericalFunctionExpression::clone() const {
                return std::unique_ptr<BaseExpression>(new BinaryNumericalFunctionExpression(this->getType(), this->getLeft()->clone(), this->getRight()->clone(), functionType));
            }

            std::unique_ptr<BaseExpression> BinaryNumericalFunctionExpression::clone(std::map<std::string, std::string> const& renaming, storm::parser::prism::VariableState const& variableState) const {
                return std::unique_ptr<BaseExpression>(new BinaryNumericalFunctionExpression(this->getType(), this->getLeft()->clone(renaming, variableState), this->getRight()->clone(renaming, variableState), this->functionType));
            }
            
            BinaryNumericalFunctionExpression::FunctionType BinaryNumericalFunctionExpression::getFunctionType() const {
                return functionType;
            }
            
            int_fast64_t BinaryNumericalFunctionExpression::getValueAsInt(std::pair<std::vector<bool>, std::vector<int_fast64_t>> const* variableValues) const {
                if (this->getType() != int_) {
                    BaseExpression::getValueAsInt(variableValues);
                }
                
                int_fast64_t resultLeft = this->getLeft()->getValueAsInt(variableValues);
                int_fast64_t resultRight = this->getRight()->getValueAsInt(variableValues);
                switch(functionType) {
                    case PLUS: return resultLeft + resultRight; break;
                    case MINUS: return resultLeft - resultRight; break;
                    case TIMES: return resultLeft * resultRight; break;
                    case DIVIDE: return resultLeft / resultRight; break;
                    case MIN: return std::min(resultLeft, resultRight); break;
                    case MAX: return std::max(resultLeft, resultRight); break;
                    default: throw storm::exceptions::ExpressionEvaluationException() << "Cannot evaluate expression: "
                        << "Unknown numeric binary operator: '" << functionType << "'.";
                }
            }
                        
            double BinaryNumericalFunctionExpression::getValueAsDouble(std::pair<std::vector<bool>, std::vector<int_fast64_t>> const* variableValues) const {
                if (this->getType() != double_ && this->getType() != int_) {
                    BaseExpression::getValueAsDouble(variableValues);
                }
                
                double resultLeft = this->getLeft()->getValueAsDouble(variableValues);
                double resultRight = this->getRight()->getValueAsDouble(variableValues);
                switch(functionType) {
                    case PLUS: return resultLeft + resultRight; break;
                    case MINUS: return resultLeft - resultRight; break;
                    case TIMES: return resultLeft * resultRight; break;
                    case DIVIDE: return resultLeft / resultRight; break;
                    case MIN: return std::min(resultLeft, resultRight); break;
                    case MAX: return std::max(resultLeft, resultRight); break;
                    default: throw storm::exceptions::ExpressionEvaluationException() << "Cannot evaluate expression: "
                        << "Unknown numeric binary operator: '" << functionType << "'.";
                }
            }
            
            void BinaryNumericalFunctionExpression::accept(ExpressionVisitor* visitor) {
                visitor->visit(this);
            }
            
            std::string BinaryNumericalFunctionExpression::toString() const {
                std::stringstream result;
                switch (functionType) {
                    case PLUS: result << "(" << this->getLeft()->toString() << " + " << this->getRight()->toString() << ")"; break;
                    case MINUS: result << "(" << this->getLeft()->toString() << " - " << this->getRight()->toString() << ")"; break;
                    case TIMES: result << "(" << this->getLeft()->toString() << " * " << this->getRight()->toString() << ")"; break;
                    case DIVIDE: result << "(" << this->getLeft()->toString() << " / " << this->getRight()->toString() << ")"; break;
                    case MIN: result << "min(" << this->getLeft()->toString() << ", " << this->getRight()->toString() << ")"; break;
                    case MAX: result << "max(" << this->getLeft()->toString() << ", " << this->getRight()->toString() << ")"; break;
                }
                return result.str();
            }
            
        } // namespace expressions
    } // namespace ir
} // namespace storm