//
// Created by yalavrinenko on 22.05.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_EXCEPTIONS_HPP
#define TRASH_ROGAINE_SOLVER_EXCEPTIONS_HPP
#include <exception>
#include <stdexcept>

namespace trs{
class load_failure : public std::runtime_error{
public:
  explicit load_failure(char const *msg): std::runtime_error(msg) {}
};
}

#endif //TRASH_ROGAINE_SOLVER_EXCEPTIONS_HPP
