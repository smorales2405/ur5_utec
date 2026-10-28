# Genera git_version.hpp con el commit y si el arbol estaba sucio, en CADA
# build (add_custom_target ALL en CMakeLists.txt). Antes el SHA se capturaba al
# CONFIGURAR, y colcon no reconfigura si CMakeLists.txt no cambia: el CSV podia
# llevar el SHA de hace varios commits. Solo reescribe la cabecera si cambia,
# para no recompilar en cada build.
#
# Entrada: -DSRC_DIR=<paquete> -DOUT=<fichero .hpp>
# "Sucio" = ficheros VERSIONADOS modificados dentro de este paquete (codigo,
# config, launch, mundos): lo que cambia el comportamiento del nodo. Otros
# paquetes del repositorio no cuentan.
execute_process(
  COMMAND git -C ${SRC_DIR} rev-parse --short HEAD
  OUTPUT_VARIABLE SHA OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT SHA)
  set(SHA "unknown")
  set(DIRTY "unknown")
else()
  execute_process(
    COMMAND git -C ${SRC_DIR} status --porcelain --untracked-files=no -- .
    OUTPUT_VARIABLE STATUS OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(STATUS)
    set(DIRTY "true")
  else()
    set(DIRTY "false")
  endif()
endif()
set(CONTENT "// Generado por cmake/git_version.cmake en cada build. No editar.
#pragma once
#define UR5_DYN_CONTROL_GIT_SHA \"${SHA}\"
#define UR5_DYN_CONTROL_GIT_DIRTY \"${DIRTY}\"
")
if(EXISTS ${OUT})
  file(READ ${OUT} OLD)
endif()
if(NOT "${OLD}" STREQUAL "${CONTENT}")
  file(WRITE ${OUT} "${CONTENT}")
endif()
