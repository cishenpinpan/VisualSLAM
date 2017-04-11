SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")

MESSAGE(STATUS "THe MODULE SEARCH PATH IS" ${CMAKE_MODULE_PATH})
MESSAGE(STATUS "THE BINARY PATH IS" ${EXECUTABLE_OUTPUT_PATH})
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE( CSparse REQUIRED )
FIND_PACKAGE( Cholmod REQUIRED )
FIND_PACKAGE( Eigen3 REQUIRED )
FIND_PACKAGE( G2O REQUIRED )


INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${CSPARSE_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${CHOLMOD_INCLUDE_DIRS} )

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse g2o_solver_cholmod)

SET(OPENCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect opencv_contrib opencv_legacy opencv_gpu opencv_xfeatures2d)

SET(Cholmod_LIBS cholmod)

SET(LMMIN_LIBS lmfit)

file(GLOB codeVisualSLAM ../include/*.h *.cpp)

ADD_EXECUTABLE(VisualSLAM ${codeVisualSLAM} )
TARGET_LINK_LIBRARIES(VisualSLAM ${OPENCV_LIBS} ${G2O_LIBS}  ${LMMIN_LIBS} ${Cholmod_LIBS})

