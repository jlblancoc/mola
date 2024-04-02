# Unit tests:
mola_add_test(
  TARGET  test-relocalization-se2-kitti
  SOURCES test-relocalization-se2-kitti.cpp
  LINK_LIBRARIES
    mola::mola_relocalization
)
