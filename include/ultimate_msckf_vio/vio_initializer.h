#ifndef VIO_INITIALIZER_H
#define VIO_INITIALIZER_H

#include <memory>

namespace ultimate_msckf_vio {

constexpr int kNumFramesToInitialize = 20;

class DataManager;

class VIOInitializer {
 public:
  VIOInitializer() = delete;

  VIOInitializer(DataManager* data_manager_ptr);

  bool is_initialized();

  bool TryInitialize();

 private:

  std::shared_ptr<DataManager> data_manager_;
  bool is_initialized_;
};

}


#endif // VIO_INITIALIZER_H
