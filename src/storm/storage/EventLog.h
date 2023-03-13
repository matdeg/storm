#pragma once

#include <vector>
#include "storm/storage/jani/Property.h"
#include "storm/storage/jani/Model.h"
#include "storm/storage/Trace.h"
#include "storm/storage/expressions/ExpressionManager.h"

namespace storm {
namespace storage {
class EventLog {
   public:

    EventLog();
    EventLog(storm::jani::Model const& model);

    void addTrace(storm::storage::Trace trace);
    storm::storage::Trace getTrace(uint_fast64_t k);

    storm::jani::Model getModel();

    uint_fast64_t size();

    void updateModel();

    std::vector<storm::jani::Property> getProperties();

    std::shared_ptr<storm::expressions::ExpressionManager> getManager();

   private:
    std::vector<storm::storage::Trace> traces; 
    storm::jani::Model model;
    std::vector<storm::jani::Property> standardProperties;
};

}  // namespace storage
}  // namespace storm