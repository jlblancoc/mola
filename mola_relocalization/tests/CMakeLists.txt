# Unit tests:

if (mola_test_datasets_FOUND)
#message(STATUS "mola_test_datasets: ${mola_test_datasets_DIR}")

set(TEST_DATASETS_ROOT "${mola_test_datasets_DIR}/../datasets/")

mola_add_test(
  TARGET  test-relocalization-se2-kitti
  SOURCES test-relocalization-se2-kitti.cpp
  LINK_LIBRARIES
    mola::mola_relocalization
)
target_compile_definitions(test-relocalization-se2-kitti PRIVATE TEST_DATASETS_ROOT=\"${TEST_DATASETS_ROOT}\")
endif()
