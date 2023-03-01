#pragma once
#include <vector>
#include "storm/storage/jani/Model.h"

namespace storm {
namespace storage {
class Trace {
   public:
    Trace() {}
    Trace(std::vector<uint_fast64_t> const& trace) : trace(trace) {}

    void addEvent(uint_fast64_t k) {
        this->trace.emplace_back(k);
    }

    std::vector<uint_fast64_t> get() {
        return this->trace;
    }

    void setModel(storm::jani::Model model) {
        this->model = model;
    }

    storm::jani::Model getModel() {
        return this->model;
    }

   


   private:
    std::vector<uint_fast64_t> trace; 
    storm::jani::Model model;
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