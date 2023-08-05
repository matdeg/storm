#include <vector>
#include "storm/storage/jani/Model.h"
#include "storm/storage/Trace.h"

namespace storm {
namespace storage {

Trace::Trace(uint_fast64_t id) : id(id) {}

void Trace::addEvent(uint_fast64_t k) {
    this->trace.emplace_back(k);
}

std::vector<uint_fast64_t> Trace::get() {
    return this->trace;
}

uint_fast64_t Trace::getID() {
    return this->id;
}

uint_fast64_t Trace::size() {
    return get().size();
}

void Trace::setValid(bool valid) {
    isValid = valid;
}

bool Trace::getValid() const {
    return this->isValid;
}

std::ostream& operator<< (std::ostream& os, Trace ttrace) {
    std::vector<uint_fast64_t> trace = ttrace.get();
    for(uint_fast64_t j : trace) {
        os << j << ", ";
    }
    os << "\n";
    return os;
}

   

}  // namespace storage
}  // namespace storm