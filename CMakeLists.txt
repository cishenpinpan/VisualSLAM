CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT(VisualSlam )

SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) 
SET(CMAKE_CXX_STANDARD 11) 
#Change if your g2o library is not here
#SET( G2O_INCLUDE_PATH /usr/local/include)
LINK_DIRECTORIES( /usr/local/lib )

#Change if your g2o library is not in this format
#SET( G2O_LIBS 
#			libg2o_cli.so
#			libg2o_core.so
#			libg2o_csparse_extension.so
#			libg2o_ext_freeglut_minimal.so
#			libg2o_incremental.so
#			libg2o_interactive.so
#			libg2o_interface.so
#			libg2o_opengl_helper.so
#			libg2o_parser.so
#			libg2o_simulator.so
#			libg2o_solver_cholmod.so
#			libg2o_solver_csparse.so
#			libg2o_solver_dense.so
#			libg2o_solver_eigen.so
#			libg2o_solver_pcg.so
#			libg2o_solver_slam2d_linear.so
#			libg2o_solver_structure_only.so
#			libg2o_stuff.so
#			libg2o_types_data.so
#			libg2o_types_icp.so
#			libg2o_types_sba.so
#			libg2o_types_sclam2d.so
#			libg2o_types_sim3.so
#			libg2o_types_slam2d.so
#			libg2o_types_slam2d_addons.so
#			libg2o_types_slam3d.so
#			libg2o_types_slam3d_addons.so
#			libcxsparse.so
#			libcholmod.so
#)

#LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)	      
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)

ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src)
