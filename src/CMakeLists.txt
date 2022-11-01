ADD_EXECUTABLE( main main.cpp )

# dependencies of PCL
find_package( PCL 1.10 REQUIRED COMPONENTS common io visualization filters)
set(PCL_INCLUDE_DIRS /usr/include/pcl-1.10)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# dependencies of OpenCV
find_package( OpenCV REQUIRED )

# head files and lib files
add_definitions( ${PCL_DEFINITIONS} )
include_directories( ${PCL_INCLUDE_DIRS} )
link_libraries( ${PCL_LIBRARY_DIRS} )

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# dependencies of g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( G2O_DIR /usr/local/include/g2o )
find_package( G2O REQUIRED)
# CSparse
find_package( CSparse REQUIRED)
include_directories( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

add_executable( generate_pointcloud generatePointCloud.cpp )
target_link_libraries( generate_pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library( slambase slamBase.cpp )
target_link_libraries( slambase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES} )

# add_executable( detect_features detectFeatures.cpp )
# target_link_libraries( detect_features slambase ${OpenCV_LIBS})

# add_executable( joint_pointcloud jointPointCloud.cpp )
# target_link_libraries( joint_pointcloud 
#     slambase 
#     ${OpenCV_LIBS} 
#     ${PCL_LIBRARIES} )

# add_executable( visual_odometry visualOdometry.cpp )
# target_link_libraries( visual_odometry 
#     slambase 
#     ${OpenCV_LIBS} 
#     ${PCL_LIBRARIES} )


# add_executable( slamEnd slamEnd.cpp )
# target_link_libraries( slamEnd 
#     slambase
#     ${OpenCV_LIBS}
#     ${PCL_LIBRARIES}
#     g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

add_executable( slam slam.cpp )
target_link_libraries( slam 
    slambase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})