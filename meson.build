project('serial_parser', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'cpp_std=c++20'])

add_project_arguments('-ggdb', language : 'cpp')

includes = []
sources = []

subdir('src')
subdir('include')

incdirs = include_directories('include')

exe = executable('serial_parser', sources,
  include_directories : incdirs,
  install : true)

test('basic', exe)
