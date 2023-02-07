set (files
  base_link.STL
  hokuyo_link.STL
  head.dae
  head_camera.dae
  multisense_ks21.STL
  multisense_s21.STL
  multisense_s27.STL
  multisense_s7.STL
  multisense_s7s.STL
)
install(FILES ${files} DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/multisense_sl/meshes/)
