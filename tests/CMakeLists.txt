add_executable(BulletPrimTest BulletPrimTest.cpp)
target_link_libraries(BulletPrimTest ${LIBRARY_NAME})

add_executable(BulletPrimParallelTest BulletPrimParallelTest.cpp)
target_link_libraries(BulletPrimParallelTest ${LIBRARY_NAME})

add_executable(CarPlannerRandomTests CarPlannerRandomTests.cpp)
target_link_libraries(CarPlannerRandomTests ${LIBRARY_NAME} )
