SET(${PROJECT_NAME}_PYTHON
  __init__.py
  appli.py
  appli_dcmZmpControl_file.py
  appli_online_walking.py
  )

FOREACH(file ${${PROJECT_NAME}_PYTHON})
  PYTHON_INSTALL_ON_SITE("talos_integration_tests" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON})

