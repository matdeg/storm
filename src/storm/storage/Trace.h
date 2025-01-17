#pragma once

#include <vector>
#include "storm/storage/jani/Property.h"
#include "storm/storage/jani/Model.h"
#include "storm/storage/expressions/ExpressionManager.h"

namespace storm {
namespace storage {
class Trace {
   public:

    Trace(uint_fast64_t id);
    void addEvent(uint_fast64_t k);
    std::vector<uint_fast64_t> get();

    uint_fast64_t getID();
    uint_fast64_t size();
    void setValid(bool valid);
    bool getValid() const;

   private:
    std::vector<uint_fast64_t> trace; 
    uint_fast64_t id;
    bool isValid = false;
};

std::ostream& operator<<(std::ostream& out, Trace trace);

}  // namespace storage
}  // namespace storm