#pragma once

#include <vector>
#include "storm/storage/jani/Property.h"
#include "storm/storage/jani/Model.h"
#include "storm/storage/expressions/ExpressionManager.h"

namespace storm {
namespace storage {
class Trace {
   public:

    Trace(storm::jani::Model model, uint_fast64_t id);
    Trace(std::vector<uint_fast64_t> const& trace);

    void addEvent(uint_fast64_t k);

    std::vector<uint_fast64_t> get();

    storm::jani::Model getModel();

    uint_fast64_t size();

    void updateModel();

    std::vector<storm::jani::Property> getProperty();

    std::shared_ptr<storm::expressions::ExpressionManager> getManager();

    void setEmptyActionIndex(uint_fast64_t k);
   private:
    std::vector<uint_fast64_t> trace; 
    storm::jani::Model model;
    uint_fast64_t emptyActionIndex;
    std::vector<storm::jani::Property> standardProperties;
    uint_fast64_t id;
};

 std::ostream& operator<<(std::ostream& stream, storm::storage::Trace trace) {
        stream << "[";
        for (uint_fast64_t event : trace.get()) {
            if (event == -1) {
                stream << "Undefined Action, ";
            } else {
                stream << trace.getModel().getAction(event).getName() << ", ";
            }
        }
        stream << "]";
        return stream;
    }
}  // namespace storage
}  // namespace storm