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

    static bool smaller(storm::storage::Trace trace1, storm::storage::Trace trace2);
    void setCuts();
    void addTrace(storm::storage::Trace trace);
    storm::storage::Trace getTrace(uint_fast64_t k);

    storm::jani::Model const& getModel();

    uint_fast64_t size();
    void initialize();

    void updateModel();
    void updateModelUnion();
    void updateModelNot();

    std::vector<storm::jani::Property> getProperties();

    std::shared_ptr<storm::expressions::ExpressionManager> getManager();

   private:
    uint_fast64_t minAutomata = 100;
    std::vector<storm::storage::Trace> traces; 
    std::vector<uint_fast64_t> cuts;
    storm::jani::Model model;
    std::vector<storm::jani::Property> standardProperties;
};

}  // namespace storage
}  // namespace storm