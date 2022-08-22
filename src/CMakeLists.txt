set(${PROJECT_NAME}_PYTHON __init__.py appli.py appli_dcmZmpControl_file.py
                           appli_online_walking.py)

foreach(file ${${PROJECT_NAME}_PYTHON})
  python_install_on_site("talos_integration_tests" ${file})
endforeach(file ${${PROJECT_NAME}_PYTHON})
