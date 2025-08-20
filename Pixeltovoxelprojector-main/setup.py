from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import pybind11
import numpy
import sys
import os

class BuildExt(build_ext):
    def build_extensions(self):
        ct = self.compiler.compiler_type
        
        if sys.platform == 'darwin':  # macOS
            opts = ['-std=c++14', '-O3', '-Xpreprocessor', '-fopenmp']
            link_opts = ['-lomp']
        elif ct == 'msvc':
            opts = ['/O2', '/openmp', '/std:c++14']
            link_opts = []
        else:  # Linux
            opts = ['-std=c++14', '-O3', '-fopenmp']
            link_opts = ['-fopenmp']
        
        for ext in self.extensions:
            ext.extra_compile_args = opts
            ext.extra_link_args = link_opts
            if sys.platform == 'darwin':
                ext.include_dirs.append('/opt/homebrew/include')
                ext.library_dirs = ['/opt/homebrew/lib']
        
        build_ext.build_extensions(self)

ext_modules = [
    Extension(
        'process_image_cpp',
        ['src/processing/process_image.cpp'],
        include_dirs=[
            pybind11.get_include(),
            numpy.get_include(),
        ],
        language='c++'
    ),
]

setup(
    name='process_image_cpp',
    version='0.0.1',
    ext_modules=ext_modules,
    cmdclass={'build_ext': BuildExt},
)
