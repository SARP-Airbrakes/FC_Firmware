project(
  'filter_test',
  'cpp',
  version : '0.1',
  meson_version : '>= 1.3.0',
  default_options : ['warning_level=3', 'cpp_std=c++14'],
)

dependencies = [
]

sources = [
  'filter_test.cpp',
  '../../Src/filter.cpp'
]

incdir = include_directories('../../Inc', '../../eigen')

exe = executable(
  'filter_test',
  sources,
  install : true,
  dependencies : dependencies,
  include_directories : incdir,
)

test('basic', exe)
