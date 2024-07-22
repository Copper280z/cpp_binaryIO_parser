project('serial_parser', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'cpp_std=c++23'])

add_project_arguments('-ggdb', language : 'cpp')

includes = []
sources = []

subdir('src')
subdir('include')

incdirs = include_directories('include')

exe = executable('serial_parser', sources+'src/binary_parse.cpp',
  include_directories : incdirs,
  install : true)

test('basic', exe)

#################################################################

subproject('pybind11')

py = import('python').find_installation(pure: false)
pybind11_dep = dependency('pybind11')

py_lib_deps = [pybind11_dep]

py.extension_module('pylib',sources+'src/pylib.cpp',
  include_directories : incdirs,
  dependencies : py_lib_deps)


#################################################################

# subproject('implot')
#
#
# exe = executable('serial_parser', sources+'src/gui/gui.cpp',
#   include_directories : incdirs,
#   install : true)
